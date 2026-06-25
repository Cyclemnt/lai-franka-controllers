#include "lai_franka_controllers/hqp_cartesian_velocity_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace lai_franka_controllers {

// -------------------------------------------------------------------------
// on_init
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_init() {
    auto node = get_node();
    joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    node->declare_parameter("joint_names", joint_names);

    // DH parameters
    auto declare_if_not_exists = [&](const std::string & name, const auto & default_val) {
        if (!node->has_parameter(name)) {
            node->declare_parameter(name, default_val);
        }
    };
    declare_if_not_exists("mod_DH.a", std::vector<double>());
    declare_if_not_exists("mod_DH.alpha", std::vector<double>());
    declare_if_not_exists("mod_DH.d", std::vector<double>());
    declare_if_not_exists("mod_DH.theta", std::vector<double>());
    declare_if_not_exists("A7e", std::vector<double>());

    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_configure
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = get_node();
    q_max <<    2.897,  1.832,  2.897, -0.122,  2.879,  4.625,  3.054;
    q_min <<   -2.897, -1.832, -2.897, -3.071, -2.879,  0.436, -3.054;
    // dq_limit << 2.617,  2.617,  2.617,  2.617,  5.253,  5.253,  5.253;
    dq_limit <<   0.5,    0.5,    0.5,    0.5,    0.5,    0.5,    0.5;
    dq_cmd.setZero();

    // Load DH Parameters
    auto a = node->get_parameter("mod_DH.a").as_double_array();
    auto alpha = node->get_parameter("mod_DH.alpha").as_double_array();
    auto d = node->get_parameter("mod_DH.d").as_double_array();
    auto theta = node->get_parameter("mod_DH.theta").as_double_array();
    auto a7e = node->get_parameter("A7e").as_double_array();

    std::vector<double> mDH;
    mDH.insert(mDH.end(), a.begin(), a.end());
    mDH.insert(mDH.end(), alpha.begin(), alpha.end());
    mDH.insert(mDH.end(), d.begin(), d.end());
    mDH.insert(mDH.end(), theta.begin(), theta.end());

    // Initialize Kinematics
    kinematics = std::make_shared<FrankaKinematics>();
    if (mDH.size() == 28 && a7e.size() == 16) {
        kinematics->setParameters(mDH, a7e);
    } else {
        RCLCPP_ERROR(node->get_logger(), "DH Parameter mismatch");
        return controller_interface::CallbackReturn::ERROR;
    }

    kinematics->getSelectDOF()->assign(7, true);
    kinematics->getSelectTask()->assign(6, true);

    // Initialize Solver and Task Stack
    solver = std::make_shared<HierarchicalQP>(7, GRB_CONTINUOUS);
    task_stack.clear();

    // Create Tasks
    // Joints configuration task
    q_upper_task = std::make_shared<JointsConfigurationLimits>(kinematics.get(), q_max, GRB_LESS_EQUAL, 1.0);
    q_lower_task = std::make_shared<JointsConfigurationLimits>(kinematics.get(), q_min, GRB_GREATER_EQUAL, 1.0);
    q_upper_task->setPriorityLevel(1);
    q_lower_task->setPriorityLevel(1);
    q_upper_task->setSlacksState(false);
    q_lower_task->setSlacksState(false);
    task_stack.push_back(q_upper_task);
    task_stack.push_back(q_lower_task);
    
    // Joints velocity task
    dq_upper_task = std::make_shared<JointsVelocityLimits>(kinematics.get(), dq_limit, GRB_LESS_EQUAL, 1.0);
    dq_lower_task = std::make_shared<JointsVelocityLimits>(kinematics.get(), -dq_limit, GRB_GREATER_EQUAL, 1.0);
    dq_upper_task->setPriorityLevel(1);
    dq_lower_task->setPriorityLevel(1);
    dq_upper_task->setSlacksState(false);
    dq_lower_task->setSlacksState(false);
    task_stack.push_back(dq_upper_task);
    task_stack.push_back(dq_lower_task);
    
    // Self hits task
    Eigen::VectorXi sefhits_safe_points(1);
    sefhits_safe_points << 6; // End-effector
    Eigen::VectorXi sefhits_avoid_points(2);
    sefhits_avoid_points << 0, 3; // Base and Elbow
    double selfhits_min_dist = 0.2; // Keep EE at least 40 cm away from the base
    self_collision_task = std::make_shared<SelfHits>(kinematics.get(), sefhits_safe_points, sefhits_avoid_points, selfhits_min_dist, GRB_GREATER_EQUAL, 1.0);
    self_collision_task->setPriorityLevel(2);
    self_collision_task->setSlacksState(false);
    task_stack.push_back(self_collision_task);

    // Virtual walls
    // Common settings
    Eigen::VectorXi joints_to_protect_from_walls(2);
    joints_to_protect_from_walls << 3, 7; 
    double margin = 0.05; // d_min
    double wall_gain = 1.0;
    int wall_priority = 3;

    // FLOOR (Z = 0.05)
    Eigen::Vector3d f1(0,0,0.05), f2(1,0,0.05), f3(0,1,0.05);
    virtual_wall_task_1 = std::make_shared<VirtualWall>(kinematics.get(), f1, f2, f3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // CEILING (Z = 1.0)
    Eigen::Vector3d c1(0,0,1.0), c2(0,1,1.0), c3(1,0,1.0);
    virtual_wall_task_2 = std::make_shared<VirtualWall>(kinematics.get(), c1, c2, c3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // FRONT (X = 0.7)
    Eigen::Vector3d fr1(0.7,0,0), fr2(0.7,0,1), fr3(0.7,1,0); 
    virtual_wall_task_3 = std::make_shared<VirtualWall>(kinematics.get(), fr1, fr2, fr3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // BACK (X = -0.5)
    Eigen::Vector3d bk1(-0.5,0,0), bk2(-0.5,1,0), bk3(-0.5,0,1);
    virtual_wall_task_4 = std::make_shared<VirtualWall>(kinematics.get(), bk1, bk2, bk3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // LEFT (Y = 0.3)
    Eigen::Vector3d l1(0,0.3,0), l2(1,0.3,0), l3(0,0.3,1);
    virtual_wall_task_5 = std::make_shared<VirtualWall>(kinematics.get(), l1, l2, l3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // RIGHT (Y = -0.3)
    Eigen::Vector3d r1(0,-0.3,0), r2(0,-0.3,1), r3(1,-0.3,0);
    virtual_wall_task_6 = std::make_shared<VirtualWall>(kinematics.get(), r1, r2, r3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);

    // Activate and add to stack
    std::vector<std::shared_ptr<VirtualWall>> all_virtual_walls = {
        virtual_wall_task_1, virtual_wall_task_2, virtual_wall_task_3, 
        virtual_wall_task_4, virtual_wall_task_5, virtual_wall_task_6
    };
    for (auto& wall : all_virtual_walls) {
        wall->setPriorityLevel(wall_priority);
        wall->setSlacksState(false);
        task_stack.push_back(wall);
    }

    // Pose task
    pose_task = std::make_shared<Pose>(kinematics.get(), GRB_EQUAL, Eigen::VectorXd::Ones(6), 5.0);
    pose_task->setPriorityLevel(4);
    pose_task->setSlacksState(true);
    task_stack.push_back(pose_task);

    // Sine task
    sine_task = std::make_shared<JointSineTask>();
    sine_task->setPriorityLevel(4);
    sine_task->setSlacksState(true);
    // task_stack.push_back(sine_task);
    
    // Target Subscription
    target_pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            TargetPose target;
            target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
            target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).normalized();
            target.valid = true;
            rt_target_pose_ptr.writeFromNonRT(target);
        });

    // Setup error publisher
    error_pub = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("~/tracking_error", rclcpp::SystemDefaultsQoS());
    rt_error_pub = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(error_pub);

    // Setup dq_cmd publisher
    dq_cmd_pub = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/dq_cmd", rclcpp::SystemDefaultsQoS());
    rt_dq_cmd_pub = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(dq_cmd_pub);
    rt_dq_cmd_pub->msg_.velocity.resize(7);
    rt_dq_cmd_pub->msg_.name = joint_names;

    // Setup joint_states publisher
    // joint_states_pub = get_node()->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SystemDefaultsQoS());
    // rt_joint_states_pub = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(joint_states_pub);
    // rt_joint_states_pub->msg_.position.resize(7);
    // rt_joint_states_pub->msg_.name = joint_names;

    // Virtual Wall Publisher
    virtualwall_dist_pub = get_node()->create_publisher<my_franka_msgs::msg::HqpDistances>("~/virtual_wall_distances", rclcpp::SystemDefaultsQoS());
    rt_virtualwall_dist_pub = std::make_shared<realtime_tools::RealtimePublisher<my_franka_msgs::msg::HqpDistances>>(virtualwall_dist_pub);
    rt_virtualwall_dist_pub->msg_.distances.resize(all_virtual_walls.size() * joints_to_protect_from_walls.size());

    // Self Hits Publisher
    selfhits_dist_pub = get_node()->create_publisher<my_franka_msgs::msg::HqpDistances>("~/self_hits_distances", rclcpp::SystemDefaultsQoS());
    rt_selfhits_dist_pub = std::make_shared<realtime_tools::RealtimePublisher<my_franka_msgs::msg::HqpDistances>>(selfhits_dist_pub);
    rt_selfhits_dist_pub->msg_.distances.resize(sefhits_safe_points.size() * sefhits_avoid_points.size());

    RCLCPP_INFO(node->get_logger(), "CartesianVelocityController configured successfully.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// command_interface_configuration
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration HqpCartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

// -------------------------------------------------------------------------
// state_interface_configuration
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration HqpCartesianVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

// -------------------------------------------------------------------------
// on_activate
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_activate(const rclcpp_lifecycle::State&) {
    for (size_t i = 0; i < 7; ++i) { q_current(i) = state_interfaces_[i].get_value(); }
    kinematics->updateJointStates(q_current);
    
    // to avoid jumps
    x_target = kinematics->getPosition();
    quat_target = kinematics->getQuaternion();

    last_target_time = get_node()->now();

    RCLCPP_INFO(get_node()->get_logger(), "Controller activated.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_deactivate
// -------------------------------------------------------------------------
controller_interface::CallbackReturn HqpCartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State&) {
    for (size_t i = 0; i < 7; ++i) { command_interfaces_[i].set_value(0.0); }
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// update: The 1000Hz Loop
// -------------------------------------------------------------------------
controller_interface::return_type HqpCartesianVelocityController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    // =========================================================
    // READ TARGET
    // =========================================================
    // Read target from subscriber
    TargetPose* target_ptr = rt_target_pose_ptr.readFromRT();
    if (target_ptr && target_ptr->valid) {
        x_target = target_ptr->position;
        quat_target = target_ptr->orientation;
        last_target_time = time;
        target_ptr->valid = false;
    }
    
    // =========================================================
    // READ JOINTS + UPDATE KINEMATICS
    // =========================================================
    // Read Current Joint States
    for (size_t i = 0; i < 7; ++i) { q_current(i) = state_interfaces_[i].get_value(); }
    // double dt = period.seconds();
    // q_current += dq_cmd * dt;

    kinematics->updateJointStates(q_current);
    
    // Pass the goal to the kinematics object so the Pose task can calculate 'b'
    kinematics->setDesiredPose(x_target, quat_target);

    // Update sine task
    // const double t = time.seconds();
    // sine_task->set_time(t);

    // =========================================================
    // HQP SOLVER
    // =========================================================
    try {
        // For all tasks
        for (auto& task : task_stack) {
            // Update
            if (task->isEnabled()) task->update();
            // Add to the solver (arguments: A, b, slack, sense, priorityLevel)
            if (task->isEnabled()) solver->addConstraints(task->get_A(), task->get_b(), task->getSlacksState(), task->getConstraintSense(), task->getPriorityLevel());
        }

        // Solve
        solver->solve();
        dq_cmd = solver->getVarsValue();
        
        // Reset the solver for the next control loop
        solver->reset();

    } catch (const std::exception& e) {
        // RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "HQP Solver Exception: %s", e.what());
        std::cout << "HQP Solver Exception: " << e.what() << std::endl;
        dq_cmd.setZero();
    }

    // =========================================================
    // WRITE IN HARDWARE INTERFACE
    // =========================================================
    kinematics->setPreviousVelocities(dq_cmd);
    for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(dq_cmd(i));
    }

    // =========================================================
    // DIAGNOSTICS PUBLISHING
    // =========================================================
    // try_lock() to not block the 1000Hz thread
    Eigen::VectorXd error = kinematics->getError();
    Eigen::Vector3d pos_error = error.head(3);
    Eigen::Vector3d ori_error = error.tail(3);
    if (rt_error_pub && rt_error_pub->trylock()) {
        rt_error_pub->msg_.header.stamp    = time;
        rt_error_pub->msg_.twist.linear.x  = pos_error.x();
        rt_error_pub->msg_.twist.linear.y  = pos_error.y();
        rt_error_pub->msg_.twist.linear.z  = pos_error.z();
        rt_error_pub->msg_.twist.angular.x = ori_error.x();
        rt_error_pub->msg_.twist.angular.y = ori_error.y();
        rt_error_pub->msg_.twist.angular.z = ori_error.z();
        rt_error_pub->unlockAndPublish();
    }

    if (rt_dq_cmd_pub && rt_dq_cmd_pub->trylock()) {
        rt_dq_cmd_pub->msg_.header.stamp = time;
        for (size_t i = 0; i < 7; ++i) rt_dq_cmd_pub->msg_.velocity[i] = dq_cmd(i);
        rt_dq_cmd_pub->unlockAndPublish();
    }

    // Publish Virtual Wall Distances
    if (rt_virtualwall_dist_pub && rt_virtualwall_dist_pub->trylock()) {
        rt_virtualwall_dist_pub->msg_.header.stamp = time;
        std::vector<std::shared_ptr<VirtualWall>> walls_to_pub = {
            virtual_wall_task_1, virtual_wall_task_2, virtual_wall_task_3,
            virtual_wall_task_4, virtual_wall_task_5, virtual_wall_task_6
        };
        int msg_idx = 0;
        for (const auto& wall : walls_to_pub) {
            Eigen::VectorXd dists = wall->get_distances_vector();
            for (int i = 0; i < dists.size(); ++i) {
                if (msg_idx < (int)rt_virtualwall_dist_pub->msg_.distances.size()) {
                    rt_virtualwall_dist_pub->msg_.distances[msg_idx++] = dists(i);
                }
            }
        }
        rt_virtualwall_dist_pub->unlockAndPublish();
    }

    // Publish Self Hits Distances
    if (self_collision_task && rt_selfhits_dist_pub && rt_selfhits_dist_pub->trylock()) {
        rt_selfhits_dist_pub->msg_.header.stamp = time;
        Eigen::VectorXd selfhits_dists = self_collision_task->get_distances_vector();
        for (int i = 0; i < selfhits_dists.size(); ++i) {
            rt_selfhits_dist_pub->msg_.distances[i] = selfhits_dists[i];
        }
        rt_selfhits_dist_pub->unlockAndPublish();
    }

    // if (rt_joint_states_pub && rt_joint_states_pub->trylock()) {
    //     rt_joint_states_pub->msg_.header.stamp = time;
    //     for (size_t i = 0; i < 7; ++i) {
    //         rt_joint_states_pub->msg_.position[i] = q_current(i);
    //     }
    //     rt_joint_states_pub->unlockAndPublish();
    // }

    return controller_interface::return_type::OK;
}

} // namespace lai_franka_controllers

PLUGINLIB_EXPORT_CLASS(lai_franka_controllers::HqpCartesianVelocityController, controller_interface::ControllerInterface)