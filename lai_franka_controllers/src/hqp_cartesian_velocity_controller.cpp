/// @file hqp_cartesian_velocity_controller.cpp
/// @brief Multi-priority hardware control loop implementation details.

#include "lai_franka_controllers/hqp_cartesian_velocity_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace lai_franka_controllers {

controller_interface::CallbackReturn HqpCartesianVelocityController::on_init() {
    auto node = get_node();
    joint_names_ = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    node->declare_parameter("joint_names", joint_names_);

    // Establish external parameter server infrastructure hooks
    auto declare_if_not_exists = [&node](const std::string & name, const auto & default_val) {
        if (!node->has_parameter(name)) {
            node->declare_parameter(name, default_val);
        }
    };
    declare_if_not_exists("mod_DH.a", std::vector<double>());
    declare_if_not_exists("mod_DH.alpha", std::vector<double>());
    declare_if_not_exists("mod_DH.d", std::vector<double>());
    declare_if_not_exists("mod_DH.theta", std::vector<double>());
    declare_if_not_exists("A7e", std::vector<double>());

    declare_if_not_exists("joint_limits.q_max", std::vector<double>({2.897, 1.832, 2.897, -0.122, 2.879, 4.625, 3.054}));
    declare_if_not_exists("joint_limits.q_min", std::vector<double>({-2.897, -1.832, -2.897, -3.071, -2.879, 0.436, -3.054}));
    declare_if_not_exists("joint_limits.dq_max", std::vector<double>({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}));
    
    declare_if_not_exists("self_collision.safe_points", std::vector<int64_t>({6}));
    declare_if_not_exists("self_collision.avoid_points", std::vector<int64_t>({0, 3}));
    declare_if_not_exists("self_collision.min_distance", 0.2);

    declare_if_not_exists("virtual_walls.joints_to_protect", std::vector<int64_t>({3, 7}));
    declare_if_not_exists("virtual_walls.margin", 0.05);
    declare_if_not_exists("virtual_walls.gain", 1.0);
    declare_if_not_exists("virtual_walls.priority", 3);
    declare_if_not_exists("virtual_walls.floor_z", 0.05);
    declare_if_not_exists("virtual_walls.ceiling_z", 1.0);
    declare_if_not_exists("virtual_walls.front_x", 0.7);
    declare_if_not_exists("virtual_walls.back_x", -0.5);
    declare_if_not_exists("virtual_walls.left_y", 0.3);
    declare_if_not_exists("virtual_walls.right_y", -0.3);

    declare_if_not_exists("primary_task.mode", std::string("cartesian"));
    declare_if_not_exists("primary_task.priority", 4);
    declare_if_not_exists("primary_task.gain", 5.0);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HqpCartesianVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = get_node();

    // Map parameterized joint saturation profiles directly onto local math fields
    auto q_max_vec = node->get_parameter("joint_limits.q_max").as_double_array();
    auto q_min_vec = node->get_parameter("joint_limits.q_min").as_double_array();
    auto dq_max_vec = node->get_parameter("joint_limits.dq_max").as_double_array();

    q_max_ = Eigen::Matrix<double, 7, 1>::Map(q_max_vec.data());
    q_min_ = Eigen::Matrix<double, 7, 1>::Map(q_min_vec.data());
    dq_limit_ = Eigen::Matrix<double, 7, 1>::Map(dq_max_vec.data());
    dq_cmd_.setZero();

    // Ingest analytical configuration arrays definitions
    auto a = node->get_parameter("mod_DH.a").as_double_array();
    auto alpha = node->get_parameter("mod_DH.alpha").as_double_array();
    auto d = node->get_parameter("mod_DH.d").as_double_array();
    auto theta = node->get_parameter("mod_DH.theta").as_double_array();
    auto a7e = node->get_parameter("A7e").as_double_array();

    std::vector<double> mDH;
    mDH.reserve(a.size() + alpha.size() + d.size() + theta.size());
    mDH.insert(mDH.end(), a.begin(), a.end());
    mDH.insert(mDH.end(), alpha.begin(), alpha.end());
    mDH.insert(mDH.end(), d.begin(), d.end());
    mDH.insert(mDH.end(), theta.begin(), theta.end());

    kinematics_ = std::make_shared<FrankaKinematics>();
    if (mDH.size() == 28 && a7e.size() == 16) {
        kinematics_->setParameters(mDH, a7e);
    } else {
        RCLCPP_ERROR(node->get_logger(), "DH kinematic parameter block size calculation mismatched.");
        return controller_interface::CallbackReturn::ERROR;
    }

    kinematics_->getSelectDOF()->assign(7, true);
    kinematics_->getSelectTask()->assign(6, true);

    // Initialize Optimization System Engine and Task Stack Vector Array
    solver_ = std::make_shared<HierarchicalQP>(7, GRB_CONTINUOUS);
    task_stack_.clear();

    // ==========================================
    // PRIORITY LEVEL 1: HARD SAFETY JOINT LIMITS
    // ==========================================
    q_upper_task_ = std::make_shared<JointsConfigurationLimits>(kinematics_.get(), q_max_, GRB_LESS_EQUAL, 1.0);
    q_lower_task_ = std::make_shared<JointsConfigurationLimits>(kinematics_.get(), q_min_, GRB_GREATER_EQUAL, 1.0);
    q_upper_task_->setPriorityLevel(1);
    q_lower_task_->setPriorityLevel(1);
    q_upper_task_->setSlacksState(false);
    q_lower_task_->setSlacksState(false);
    task_stack_.push_back(q_upper_task_);
    task_stack_.push_back(q_lower_task_);
    
    dq_upper_task_ = std::make_shared<JointsVelocityLimits>(kinematics_.get(), dq_limit_, GRB_LESS_EQUAL, 1.0);
    dq_lower_task_ = std::make_shared<JointsVelocityLimits>(kinematics_.get(), -dq_limit_, GRB_GREATER_EQUAL, 1.0);
    dq_upper_task_->setPriorityLevel(1);
    dq_lower_task_->setPriorityLevel(1);
    dq_upper_task_->setSlacksState(false);
    dq_lower_task_->setSlacksState(false);
    task_stack_.push_back(dq_upper_task_);
    task_stack_.push_back(dq_lower_task_);
    
    // ==========================================
    // PRIORITY LEVEL 2: SELF-COLLISION PROTECTION
    // ==========================================
    auto safe_pts_raw = node->get_parameter("self_collision.safe_points").as_integer_array();
    auto avoid_pts_raw = node->get_parameter("self_collision.avoid_points").as_integer_array();
    double selfhits_min_dist = node->get_parameter("self_collision.min_distance").as_double();

    Eigen::VectorXi sefhits_safe_points(safe_pts_raw.size());
    for (size_t i = 0; i < safe_pts_raw.size(); ++i) sefhits_safe_points(i) = static_cast<int>(safe_pts_raw[i]);

    Eigen::VectorXi sefhits_avoid_points(avoid_pts_raw.size());
    for (size_t i = 0; i < avoid_pts_raw.size(); ++i) sefhits_avoid_points(i) = static_cast<int>(avoid_pts_raw[i]);

    self_collision_task_ = std::make_shared<SelfHits>(kinematics_.get(), sefhits_safe_points, sefhits_avoid_points, selfhits_min_dist, GRB_GREATER_EQUAL, 1.0);
    self_collision_task_->setPriorityLevel(2);
    self_collision_task_->setSlacksState(false);
    task_stack_.push_back(self_collision_task_);

    // ==========================================
    // PRIORITY LEVEL 3: WORKSPACE VIRTUAL WALL BOX
    // ==========================================
    auto protect_joints_raw = node->get_parameter("virtual_walls.joints_to_protect").as_integer_array();
    Eigen::VectorXi joints_to_protect_from_walls(protect_joints_raw.size());
    for (size_t i = 0; i < protect_joints_raw.size(); ++i) joints_to_protect_from_walls(i) = static_cast<int>(protect_joints_raw[i]);

    double margin = node->get_parameter("virtual_walls.margin").as_double(); 
    double wall_gain = node->get_parameter("virtual_walls.gain").as_double();
    int wall_priority = static_cast<int>(node->get_parameter("virtual_walls.priority").as_int());

    double floor_z = node->get_parameter("virtual_walls.floor_z").as_double();
    double ceiling_z = node->get_parameter("virtual_walls.ceiling_z").as_double();
    double front_x = node->get_parameter("virtual_walls.front_x").as_double();
    double back_x = node->get_parameter("virtual_walls.back_x").as_double();
    double left_y = node->get_parameter("virtual_walls.left_y").as_double();
    double right_y = node->get_parameter("virtual_walls.right_y").as_double();

    // WALL 1: FLOOR BOUNDARY (Z = floor_z)
    Eigen::Vector3d f1(0,0,floor_z), f2(1,0,floor_z), f3(0,1,floor_z);
    virtual_wall_task_1_ = std::make_shared<VirtualWall>(kinematics_.get(), f1, f2, f3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 2: CEILING BOUNDARY (Z = ceiling_z)
    Eigen::Vector3d c1(0,0,ceiling_z), c2(0,1,ceiling_z), c3(1,0,ceiling_z);
    virtual_wall_task_2_ = std::make_shared<VirtualWall>(kinematics_.get(), c1, c2, c3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 3: FRONT BOUNDARY (X = front_x)
    Eigen::Vector3d fr1(front_x,0,0), fr2(front_x,0,1), fr3(front_x,1,0); 
    virtual_wall_task_3_ = std::make_shared<VirtualWall>(kinematics_.get(), fr1, fr2, fr3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 4: BACK BOUNDARY (X = back_x)
    Eigen::Vector3d bk1(back_x,0,0), bk2(back_x,1,0), bk3(back_x,0,1);
    virtual_wall_task_4_ = std::make_shared<VirtualWall>(kinematics_.get(), bk1, bk2, bk3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 5: LEFT BOUNDARY (Y = left_y)
    Eigen::Vector3d l1(0,left_y,0), l2(1,left_y,0), l3(0,left_y,1);
    virtual_wall_task_5_ = std::make_shared<VirtualWall>(kinematics_.get(), l1, l2, l3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 6: RIGHT BOUNDARY (Y = right_y)
    Eigen::Vector3d r1(0,right_y,0), r2(0,right_y,1), r3(1,right_y,0);
    virtual_wall_task_6_ = std::make_shared<VirtualWall>(kinematics_.get(), r1, r2, r3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);

    std::vector<std::shared_ptr<VirtualWall>> all_virtual_walls = {
        virtual_wall_task_1_, virtual_wall_task_2_, virtual_wall_task_3_, 
        virtual_wall_task_4_, virtual_wall_task_5_, virtual_wall_task_6_
    };
    for (auto& wall : all_virtual_walls) {
        wall->setPriorityLevel(wall_priority);
        wall->setSlacksState(false);
        task_stack_.push_back(wall);
    }

    // ==========================================
    // PRIORITY LEVEL 4: PRIMARY TRACKING TASK (CONDITIONAL)
    // ==========================================
    task_mode_ = node->get_parameter("primary_task.mode").as_string();
    int task_priority = static_cast<int>(node->get_parameter("primary_task.priority").as_int());
    double task_gain = node->get_parameter("primary_task.gain").as_double();

    if (task_mode_ == "cartesian") {
        pose_task_ = std::make_shared<Pose>(kinematics_.get(), GRB_EQUAL, Eigen::VectorXd::Ones(6), task_gain);
        pose_task_->setPriorityLevel(task_priority);
        pose_task_->setSlacksState(true);
        task_stack_.push_back(pose_task_);

        target_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                TargetPose target;
                target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
                target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).normalized();
                target.valid = true;
                rt_target_pose_ptr_.writeFromNonRT(target);
            });
            
        RCLCPP_INFO(node->get_logger(), "Primary Task Mode: Cartesian Pose Tracking");
    } 
    else if (task_mode_ == "joint") {
        joint_tracking_task_ = std::make_shared<JointTracking>(kinematics_.get(), GRB_EQUAL, task_gain);
        joint_tracking_task_->setPriorityLevel(task_priority);
        joint_tracking_task_->setSlacksState(true);
        task_stack_.push_back(joint_tracking_task_);

        target_joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>("~/target_joint", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                TargetJoint target;
                for (size_t i = 0; i < 7 && i < msg->position.size(); ++i) {
                    target.q(i) = msg->position[i];
                    target.dq(i) = (msg->velocity.empty()) ? 0.0 : msg->velocity[i];
                }
                target.valid = true;
                rt_target_joint_ptr_.writeFromNonRT(target);
            });

        RCLCPP_INFO(node->get_logger(), "Primary Task Mode: Joint Space Tracking");
    } 
    else {
        RCLCPP_ERROR(node->get_logger(), "Invalid primary_task.mode. Must be 'cartesian' or 'joint'.");
        return controller_interface::CallbackReturn::ERROR;
    }

    error_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>("~/tracking_error", rclcpp::SystemDefaultsQoS());
    rt_error_pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(error_pub_);

    dq_cmd_pub_ = node->create_publisher<sensor_msgs::msg::JointState>("~/dq_cmd", rclcpp::SystemDefaultsQoS());
    rt_dq_cmd_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(dq_cmd_pub_);
    rt_dq_cmd_pub_->msg_.velocity.resize(7);
    rt_dq_cmd_pub_->msg_.name = joint_names_;

    // Explicit dynamic output reservations protect internal allocation bounds from real-time leakage
    virtualwall_dist_pub_ = node->create_publisher<lai_franka_controllers::msg::HqpDistances>("~/virtual_wall_distances", rclcpp::SystemDefaultsQoS());
    rt_virtualwall_dist_pub_ = std::make_shared<realtime_tools::RealtimePublisher<lai_franka_controllers::msg::HqpDistances>>(virtualwall_dist_pub_);
    rt_virtualwall_dist_pub_->msg_.distances.resize(all_virtual_walls.size() * joints_to_protect_from_walls.size());

    selfhits_dist_pub_ = node->create_publisher<lai_franka_controllers::msg::HqpDistances>("~/self_hits_distances", rclcpp::SystemDefaultsQoS());
    rt_selfhits_dist_pub_ = std::make_shared<realtime_tools::RealtimePublisher<lai_franka_controllers::msg::HqpDistances>>(selfhits_dist_pub_);
    rt_selfhits_dist_pub_->msg_.distances.resize(sefhits_safe_points.size() * sefhits_avoid_points.size());

    RCLCPP_INFO(node->get_logger(), "CartesianVelocityController configured successfully.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration HqpCartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

controller_interface::InterfaceConfiguration HqpCartesianVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

controller_interface::CallbackReturn HqpCartesianVelocityController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    for (size_t i = 0; i < 7; ++i) { q_current_(i) = state_interfaces_[i].get_value(); }
    kinematics_->updateJointStates(q_current_);
    
    // Initialize boundary states to current actual positions to eliminate initialization errors
    x_target_ = kinematics_->getPosition();
    quat_target_ = kinematics_->getQuaternion();
    q_target_ = q_current_;
    dq_target_ = Eigen::VectorXd::Zero(7);

    last_target_time_ = get_node()->now();

    RCLCPP_INFO(get_node()->get_logger(), "Controller activated successfully.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HqpCartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    for (size_t i = 0; i < 7; ++i) { command_interfaces_[i].set_value(0.0); }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type HqpCartesianVelocityController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    
    // Acquire non-blocking reference configurations targets snapshot pointers
    TargetPose* target_ptr = rt_target_pose_ptr_.readFromRT(); // Cartesian
    if (target_ptr && target_ptr->valid) {
        x_target_ = target_ptr->position;
        quat_target_ = target_ptr->orientation;
        last_target_time_ = time;
        target_ptr->valid = false;
    }
    TargetJoint* joint_ptr = rt_target_joint_ptr_.readFromRT(); // Joint
    if (joint_ptr && joint_ptr->valid) {
        q_target_ = joint_ptr->q;
        dq_target_ = joint_ptr->dq;
        last_target_time_ = time;
        joint_ptr->valid = false;
    }
    
    // Read hardware inputs directly from peripheral interface links
    for (size_t i = 0; i < 7; ++i) { q_current_(i) = state_interfaces_[i].get_value(); }

    kinematics_->updateJointStates(q_current_);
    if (task_mode_ == "cartesian") {
        kinematics_->setDesiredPose(x_target_, quat_target_);
    } else if (task_mode_ == "joint") {
        joint_tracking_task_->set_desired_state(q_target_, dq_target_);
    }

    // Execute sequential matrix optimization loops checks steps
    try {
        for (auto& task : task_stack_) {
            if (task->isEnabled()) task->update();
            if (task->isEnabled()) solver_->addConstraints(task->get_A(), task->get_b(), task->getSlacksState(), task->getConstraintSense(), task->getPriorityLevel());
        }

        solver_->solve();
        dq_cmd_ = solver_->getVarsValue();
        solver_->reset();

    } catch (const std::exception& e) {
        std::cout << "Hardware Optimizer Critical Exception Fault Captured: " << e.what() << std::endl;
        dq_cmd_.setZero();
    }

    // Direct interface writes driving low-level motor drivers
    kinematics_->setPreviousVelocities(dq_cmd_);
    for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(dq_cmd_(i));
    }

    // ---- Lock-Free Non-blocking Diagnostic Transmission Publishing Loops ----
    Eigen::VectorXd error = kinematics_->getError();
    Eigen::Vector3d pos_error = error.head(3);
    Eigen::Vector3d ori_error = error.tail(3);
    
    if (rt_error_pub_ && rt_error_pub_->trylock()) {
        rt_error_pub_->msg_.header.stamp    = time;
        rt_error_pub_->msg_.twist.linear.x  = pos_error.x();
        rt_error_pub_->msg_.twist.linear.y  = pos_error.y();
        rt_error_pub_->msg_.twist.linear.z  = pos_error.z();
        rt_error_pub_->msg_.twist.angular.x = ori_error.x();
        rt_error_pub_->msg_.twist.angular.y = ori_error.y();
        rt_error_pub_->msg_.twist.angular.z = ori_error.z();
        rt_error_pub_->unlockAndPublish();
    }

    if (rt_dq_cmd_pub_ && rt_dq_cmd_pub_->trylock()) {
        rt_dq_cmd_pub_->msg_.header.stamp = time;
        for (size_t i = 0; i < 7; ++i) rt_dq_cmd_pub_->msg_.velocity[i] = dq_cmd_(i);
        rt_dq_cmd_pub_->unlockAndPublish();
    }

    if (rt_virtualwall_dist_pub_ && rt_virtualwall_dist_pub_->trylock()) {
        rt_virtualwall_dist_pub_->msg_.header.stamp = time;
        std::vector<std::shared_ptr<VirtualWall>> walls_to_pub = {
            virtual_wall_task_1_, virtual_wall_task_2_, virtual_wall_task_3_,
            virtual_wall_task_4_, virtual_wall_task_5_, virtual_wall_task_6_
        };
        int msg_idx = 0;
        for (const auto& wall : walls_to_pub) {
            Eigen::VectorXd dists = wall->get_distances_vector();
            for (int i = 0; i < dists.size(); ++i) {
                if (msg_idx < static_cast<int>(rt_virtualwall_dist_pub_->msg_.distances.size())) {
                    rt_virtualwall_dist_pub_->msg_.distances[msg_idx++] = dists(i);
                }
            }
        }
        rt_virtualwall_dist_pub_->unlockAndPublish();
    }

    if (self_collision_task_ && rt_selfhits_dist_pub_ && rt_selfhits_dist_pub_->trylock()) {
        rt_selfhits_dist_pub_->msg_.header.stamp = time;
        Eigen::VectorXd selfhits_dists = self_collision_task_->get_distances_vector();
        for (int i = 0; i < selfhits_dists.size(); ++i) {
            rt_selfhits_dist_pub_->msg_.distances[i] = selfhits_dists[i];
        }
        rt_selfhits_dist_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace lai_franka_controllers

PLUGINLIB_EXPORT_CLASS(lai_franka_controllers::HqpCartesianVelocityController, controller_interface::ControllerInterface)