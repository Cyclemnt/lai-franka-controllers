/// @file hqp_reference_generator_node.cpp
/// @brief Multi-priority task optimization loop execution details.

#include "lai_franka_controllers/hqp_reference_generator_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace lai_franka_controllers {

HqpReferenceGeneratorNode::HqpReferenceGeneratorNode() : Node("hqp_reference_generator_node") {
    
    joint_names_ = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    this->declare_parameter("joint_names", joint_names_);

    // Declare modifiable parameter hooks if not explicitly declared by external configuration profiles
    auto declare_if_not_exists = [this](const std::string & name, const auto & default_val) {
        if (!this->has_parameter(name)) {
            this->declare_parameter(name, default_val);
        }
    };
    declare_if_not_exists("mod_DH.a", std::vector<double>());
    declare_if_not_exists("mod_DH.alpha", std::vector<double>());
    declare_if_not_exists("mod_DH.d", std::vector<double>());
    declare_if_not_exists("mod_DH.theta", std::vector<double>());
    declare_if_not_exists("A7e", std::vector<double>());

    // Declare new configuration parameters matching yaml specifications
    declare_if_not_exists("joint_limits.enabled", true);
    declare_if_not_exists("joint_limits.q_max", std::vector<double>({2.897, 1.832, 2.897, -0.122, 2.879, 4.625, 3.054}));
    declare_if_not_exists("joint_limits.q_min", std::vector<double>({-2.897, -1.832, -2.897, -3.071, -2.879, 0.436, -3.054}));
    declare_if_not_exists("joint_limits.dq_max", std::vector<double>({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}));
    
    declare_if_not_exists("self_collision.enabled", true);
    declare_if_not_exists("self_collision.safe_points", std::vector<int64_t>({6}));
    declare_if_not_exists("self_collision.avoid_points", std::vector<int64_t>({0, 3}));
    declare_if_not_exists("self_collision.min_distance", 0.2);

    declare_if_not_exists("virtual_walls.enabled", true);
    declare_if_not_exists("virtual_walls.joints_to_protect", std::vector<int64_t>({3, 7}));
    declare_if_not_exists("virtual_walls.margin", 0.05);
    declare_if_not_exists("virtual_walls.gain", 1.0);
    declare_if_not_exists("virtual_walls.priority", 3);
    declare_if_not_exists("virtual_walls.floor_z", 0.05);
    declare_if_not_exists("virtual_walls.ceiling_z", 1.0);
    declare_if_not_exists("virtual_walls.front_x", 0.8);
    declare_if_not_exists("virtual_walls.back_x", -0.5);
    declare_if_not_exists("virtual_walls.left_y", 0.5);
    declare_if_not_exists("virtual_walls.right_y", -0.5);

    declare_if_not_exists("primary_task.enabled", true);
    declare_if_not_exists("primary_task.mode", std::string("cartesian"));
    declare_if_not_exists("primary_task.priority", 4);
    declare_if_not_exists("primary_task.gain", 5.0);

    // Retrieve and map active joint limits directly onto Eigen boundary structures
    auto q_max_vec = this->get_parameter("joint_limits.q_max").as_double_array();
    auto q_min_vec = this->get_parameter("joint_limits.q_min").as_double_array();
    auto dq_max_vec = this->get_parameter("joint_limits.dq_max").as_double_array();

    q_max_ = Eigen::Matrix<double, 7, 1>::Map(q_max_vec.data());
    q_min_ = Eigen::Matrix<double, 7, 1>::Map(q_min_vec.data());
    dq_limit_ = Eigen::Matrix<double, 7, 1>::Map(dq_max_vec.data());
    
    q_virtual_.setZero();
    dq_hqp_.setZero();

    // Ingest analytical DH configuration description parameters
    auto a = this->get_parameter("mod_DH.a").as_double_array();
    auto alpha = this->get_parameter("mod_DH.alpha").as_double_array();
    auto d = this->get_parameter("mod_DH.d").as_double_array();
    auto theta = this->get_parameter("mod_DH.theta").as_double_array();
    auto a7e = this->get_parameter("A7e").as_double_array();

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
        RCLCPP_ERROR(this->get_logger(), "DH Matrix dimension verification mismatch in YAML.");
        throw std::runtime_error("Invalid DH Parameters");
    }

    kinematics_->getSelectDOF()->assign(7, true);
    kinematics_->getSelectTask()->assign(6, true);

    // Initialize Optimization Engine and Clean Ingestion Vector Array Stack
    solver_ = std::make_shared<HierarchicalQP>(7, GRB_CONTINUOUS);
    task_stack_.clear();

    // ==========================================
    // PRIORITY LEVEL 1: SAFETY JOINT LIMITS
    // ==========================================
    bool joint_limits_enabled = this->get_parameter("joint_limits.enabled").as_bool();

    q_upper_task_ = std::make_shared<JointsConfigurationLimits>(kinematics_.get(), q_max_, GRB_LESS_EQUAL, 1.0);
    q_lower_task_ = std::make_shared<JointsConfigurationLimits>(kinematics_.get(), q_min_, GRB_GREATER_EQUAL, 1.0);
    q_upper_task_->setPriorityLevel(1);
    q_lower_task_->setPriorityLevel(1);
    q_upper_task_->setSlacksState(false);
    q_lower_task_->setSlacksState(false);
    
    dq_upper_task_ = std::make_shared<JointsVelocityLimits>(kinematics_.get(), dq_limit_, GRB_LESS_EQUAL, 1.0);
    dq_lower_task_ = std::make_shared<JointsVelocityLimits>(kinematics_.get(), -dq_limit_, GRB_GREATER_EQUAL, 1.0);
    dq_upper_task_->setPriorityLevel(1);
    dq_lower_task_->setPriorityLevel(1);
    dq_upper_task_->setSlacksState(false);
    dq_lower_task_->setSlacksState(false);

    if (joint_limits_enabled) {
        task_stack_.push_back(q_upper_task_);
        task_stack_.push_back(q_lower_task_);
        task_stack_.push_back(dq_upper_task_);
        task_stack_.push_back(dq_lower_task_);
    }

    // ==========================================
    // PRIORITY LEVEL 2: SELF-COLLISION PROTECTION
    // ==========================================
    bool selfhits_enabled = this->get_parameter("self_collision.enabled").as_bool();
    auto safe_pts_raw = this->get_parameter("self_collision.safe_points").as_integer_array();
    auto avoid_pts_raw = this->get_parameter("self_collision.avoid_points").as_integer_array();
    double selfhits_min_dist = this->get_parameter("self_collision.min_distance").as_double();

    Eigen::VectorXi sefhits_safe_points(safe_pts_raw.size());
    for (size_t i = 0; i < safe_pts_raw.size(); ++i) sefhits_safe_points(i) = static_cast<int>(safe_pts_raw[i]);

    Eigen::VectorXi sefhits_avoid_points(avoid_pts_raw.size());
    for (size_t i = 0; i < avoid_pts_raw.size(); ++i) sefhits_avoid_points(i) = static_cast<int>(avoid_pts_raw[i]);
    
    self_collision_task_ = std::make_shared<SelfHits>(kinematics_.get(), sefhits_safe_points, sefhits_avoid_points, selfhits_min_dist, GRB_GREATER_EQUAL, 1.0);
    self_collision_task_->setPriorityLevel(2);
    self_collision_task_->setSlacksState(false);
    if (selfhits_enabled) task_stack_.push_back(self_collision_task_);

    // ==========================================
    // PRIORITY LEVEL 3: WORKSPACE VIRTUAL WALL BOX
    // ==========================================
    bool virtual_walls_enabled = this->get_parameter("virtual_walls.enabled").as_bool();
    auto protect_joints_raw = this->get_parameter("virtual_walls.joints_to_protect").as_integer_array();
    Eigen::VectorXi joints_to_protect_from_walls(protect_joints_raw.size());
    for (size_t i = 0; i < protect_joints_raw.size(); ++i) joints_to_protect_from_walls(i) = static_cast<int>(protect_joints_raw[i]);

    double margin = this->get_parameter("virtual_walls.margin").as_double(); 
    double wall_gain = this->get_parameter("virtual_walls.gain").as_double();
    int wall_priority = static_cast<int>(this->get_parameter("virtual_walls.priority").as_int());

    double floor_z = this->get_parameter("virtual_walls.floor_z").as_double();
    double ceiling_z = this->get_parameter("virtual_walls.ceiling_z").as_double();
    double front_x = this->get_parameter("virtual_walls.front_x").as_double();
    double back_x = this->get_parameter("virtual_walls.back_x").as_double();
    double left_y = this->get_parameter("virtual_walls.left_y").as_double();
    double right_y = this->get_parameter("virtual_walls.right_y").as_double();

    // WALL 1: FLOOR BOUNDARY (Z = floor_z)
    Eigen::Vector3d f1(0, 0, floor_z), f2(1, 0, floor_z), f3(0, 1, floor_z);
    virtual_wall_task_1_ = std::make_shared<VirtualWall>(kinematics_.get(), f1, f2, f3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 2: CEILING BOUNDARY (Z = ceiling_z)
    Eigen::Vector3d c1(0, 0, ceiling_z), c2(0, 1, ceiling_z), c3(1, 0, ceiling_z);
    virtual_wall_task_2_ = std::make_shared<VirtualWall>(kinematics_.get(), c1, c2, c3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 3: FRONT BOUNDARY (X = front_x)
    Eigen::Vector3d fr1(front_x, 0, 0), fr2(front_x, 0, 1), fr3(front_x, 1, 0); 
    virtual_wall_task_3_ = std::make_shared<VirtualWall>(kinematics_.get(), fr1, fr2, fr3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 4: BACK BOUNDARY (X = back_x)
    Eigen::Vector3d bk1(back_x, 0, 0), bk2(back_x, 1, 0), bk3(back_x, 0, 1);
    virtual_wall_task_4_ = std::make_shared<VirtualWall>(kinematics_.get(), bk1, bk2, bk3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 5: LEFT BOUNDARY (Y = left_y)
    Eigen::Vector3d l1(0, left_y, 0), l2(1, left_y, 0), l3(0, left_y, 1);
    virtual_wall_task_5_ = std::make_shared<VirtualWall>(kinematics_.get(), l1, l2, l3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);
    // WALL 6: RIGHT BOUNDARY (Y = right_y)
    Eigen::Vector3d r1(0, right_y, 0), r2(0, right_y, 1), r3(1, right_y, 0);
    virtual_wall_task_6_ = std::make_shared<VirtualWall>(kinematics_.get(), r1, r2, r3, joints_to_protect_from_walls, margin, GRB_GREATER_EQUAL, wall_gain);

    std::vector<std::shared_ptr<VirtualWall>> all_virtual_walls = {
        virtual_wall_task_1_, virtual_wall_task_2_, virtual_wall_task_3_, 
        virtual_wall_task_4_, virtual_wall_task_5_, virtual_wall_task_6_
    };
    for (auto& wall : all_virtual_walls) {
        wall->setPriorityLevel(wall_priority);
        wall->setSlacksState(false);
        if (virtual_walls_enabled) task_stack_.push_back(wall);
    }

    // ==========================================
    // PRIORITY LEVEL 4: PRIMARY TRACKING TASK (CONDITIONAL)
    // ==========================================
    bool primary_task_enabled = this->get_parameter("primary_task.enabled").as_bool();
    task_mode_ = this->get_parameter("primary_task.mode").as_string();
    int task_priority = static_cast<int>(this->get_parameter("primary_task.priority").as_int());
    double task_gain = this->get_parameter("primary_task.gain").as_double();

    if (task_mode_ == "cartesian") {
        pose_task_ = std::make_shared<Pose>(kinematics_.get(), GRB_EQUAL, Eigen::VectorXd::Ones(6), task_gain);
        pose_task_->setPriorityLevel(task_priority);
        pose_task_->setSlacksState(true); 
        if (primary_task_enabled) task_stack_.push_back(pose_task_);

        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/target_pose", 10, std::bind(&HqpReferenceGeneratorNode::target_pose_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Primary Task Mode: Cartesian Pose Tracking");
    } 
    else if (task_mode_ == "joint") {
        joint_tracking_task_ = std::make_shared<task::JointTracking>(kinematics_.get(), GRB_EQUAL, task_gain);
        joint_tracking_task_->setPriorityLevel(task_priority);
        joint_tracking_task_->setSlacksState(true);
        if (primary_task_enabled) task_stack_.push_back(joint_tracking_task_);

        target_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "~/target_joint", 10, std::bind(&HqpReferenceGeneratorNode::target_joint_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Primary Task Mode: Joint Space Tracking");
    } 
    else {
        RCLCPP_ERROR(this->get_logger(), "Invalid primary_task.mode in YAML. Must be 'cartesian' or 'joint'.");
        throw std::runtime_error("Invalid task mode");
    }

    // Output and Diagnostic Pub Setup
    joint_cmd_pub_        = this->create_publisher<sensor_msgs::msg::JointState>("/joint_pd_velocity_controller/joint_commands", 10);
    error_pub_            = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/tracking_error", 10);
    virtualwall_dist_pub_ = this->create_publisher<lai_franka_controllers::msg::HqpDistances>("~/virtual_wall_distances", 10);
    selfhits_dist_pub_    = this->create_publisher<lai_franka_controllers::msg::HqpDistances>("~/self_hits_distances", 10);
    current_pose_pub_     = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10);

    target_pose_sub_      = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", 10, std::bind(&HqpReferenceGeneratorNode::target_pose_callback, this, std::placeholders::_1));

    // Listen once to real robot states to align model reference frames
    joint_state_sub_      = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&HqpReferenceGeneratorNode::joint_state_callback, this, std::placeholders::_1));

    // Execution step scheduled at 500 Hz
    timer_ = this->create_wall_timer(2ms, std::bind(&HqpReferenceGeneratorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Optimization Architecture Initialized. Awaiting frame alignment from /joint_states...");
}

void HqpReferenceGeneratorNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (is_initialized_) return;

    std::vector<double> initial_positions(7, 0.0);
    size_t matched_joints = 0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            if (msg->name[i] == joint_names_[j]) {
                initial_positions[j] = msg->position[i];
                matched_joints++;
            }
        }
    }

    if (matched_joints == joint_names_.size()) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        q_virtual_ = Eigen::VectorXd::Map(initial_positions.data(), 7);
        kinematics_->updateJointStates(q_virtual_);
        
        pose_target_.position = kinematics_->getPosition();
        pose_target_.orientation = kinematics_->getQuaternion();
        pose_target_.valid = true;

        target_q_ = q_virtual_;
        target_dq_ = Eigen::VectorXd::Zero(7);
        
        last_time_ = this->get_clock()->now();
        is_initialized_ = true;
        
        // Sever joint subscription dependencies to minimize execution thread overhead
        joint_state_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Internal Reference Map Aligned. HQP Active.");
    }
}

void HqpReferenceGeneratorNode::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pose_target_.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    pose_target_.orientation = Eigen::Quaterniond(msg->pose.orientation.w, 
                                                     msg->pose.orientation.x, 
                                                     msg->pose.orientation.y, 
                                                     msg->pose.orientation.z).normalized();
    pose_target_.valid = true;
}

void HqpReferenceGeneratorNode::target_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
        // Find which index this joint name corresponds to in our internal target array
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            if (msg->name[i] == joint_names_[j]) {
                
                if (i < msg->position.size()) {
                    target_q_(j) = msg->position[i];
                }
                
                if (i < msg->velocity.size()) {
                    target_dq_(j) = msg->velocity[i];
                } else {
                    target_dq_(j) = 0.0; 
                }
                
                break; // Move to the next joint in the incoming message
            }
        }
    }
}

void HqpReferenceGeneratorNode::timer_callback() {
    if (!is_initialized_) return;

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Secure local tracking target copies under mutex control protection bounds
    if (task_mode_ == "cartesian") {
        Eigen::Vector3d local_target_pos;
        Eigen::Quaterniond local_target_ori;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            local_target_pos = pose_target_.position;
            local_target_ori = pose_target_.orientation;
        }
        kinematics_->setDesiredPose(local_target_pos, local_target_ori);
    } 
    else if (task_mode_ == "joint") {
        std::lock_guard<std::mutex> lock(data_mutex_);
        joint_tracking_task_->set_desired_state(target_q_, target_dq_);
    }

    // Integrate internal virtual model configurations frame definitions
    q_virtual_ += dq_hqp_ * dt;
    kinematics_->updateJointStates(q_virtual_);

    // Solve multi-priority optimization problem
    try {
        for (auto& task : task_stack_) {
            if (task->isEnabled()) {
                task->update();
                solver_->addConstraints(task->get_A(), task->get_b(), task->getSlacksState(), task->getConstraintSense(), task->getPriorityLevel());
            }
        }
        solver_->solve();
        dq_hqp_ = solver_->getVarsValue();
        solver_->reset();
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Optimization Failure Intercepted: %s", e.what());
        dq_hqp_.setZero();
        solver_->reset();
    }

    kinematics_->setPreviousVelocities(dq_hqp_);

    // Pack reference tracking message out onto low-level hardware communication interfaces
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = current_time;
    joint_msg.name = joint_names_;
    joint_msg.position.resize(7);
    joint_msg.velocity.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        joint_msg.position[i] = q_virtual_(i);
        joint_msg.velocity[i] = dq_hqp_(i);
    }
    joint_cmd_pub_->publish(joint_msg);

    // ---- Execution Error Metrics Diagnostic Publishing ----
    Eigen::VectorXd error = kinematics_->getError();
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = current_time;
    twist_msg.twist.linear.x  = error(0);
    twist_msg.twist.linear.y  = error(1);
    twist_msg.twist.linear.z  = error(2);
    twist_msg.twist.angular.x = error(3);
    twist_msg.twist.angular.y = error(4);
    twist_msg.twist.angular.z = error(5);
    error_pub_->publish(twist_msg);

    // ---- Virtual Wall Proximity Diagnostic Publishing ----
    std::vector<std::shared_ptr<VirtualWall>> walls_to_pub = {
        virtual_wall_task_1_, virtual_wall_task_2_, virtual_wall_task_3_,
        virtual_wall_task_4_, virtual_wall_task_5_, virtual_wall_task_6_
    };
    auto wall_msg = lai_franka_controllers::msg::HqpDistances();
    wall_msg.header.stamp = current_time;
    wall_msg.distances.resize(12); 
    size_t msg_idx = 0;
    for (const auto& wall : walls_to_pub) {
        Eigen::VectorXd dists = wall->get_distances_vector();
        for (int i = 0; i < dists.size(); ++i) {
            if (msg_idx < wall_msg.distances.size()) {
                wall_msg.distances[msg_idx++] = dists(i);
            }
        }
    }
    virtualwall_dist_pub_->publish(wall_msg);

    // ---- Self-Collision Distance Diagnostic Publishing ----
    if (self_collision_task_) {
        Eigen::VectorXd selfhits_dists = self_collision_task_->get_distances_vector();
        auto self_msg = lai_franka_controllers::msg::HqpDistances();
        self_msg.header.stamp = current_time;
        self_msg.distances.resize(selfhits_dists.size());
        for (int i = 0; i < selfhits_dists.size(); ++i) {
            self_msg.distances[i] = selfhits_dists[i];
        }
        selfhits_dist_pub_->publish(self_msg);
    }

    // ---- Stream Virtual Internal Model Tracking Pose Frames ----
    auto current_pose_msg = geometry_msgs::msg::PoseStamped();
    current_pose_msg.header.stamp = current_time;
    current_pose_msg.header.frame_id = "fr3_link0";

    Eigen::Vector3d current_p = kinematics_->getPosition();
    Eigen::Quaterniond current_q = kinematics_->getQuaternion();

    current_pose_msg.pose.position.x = current_p.x();
    current_pose_msg.pose.position.y = current_p.y();
    current_pose_msg.pose.position.z = current_p.z();
    current_pose_msg.pose.orientation.w = current_q.w();
    current_pose_msg.pose.orientation.x = current_q.x();
    current_pose_msg.pose.orientation.y = current_q.y();
    current_pose_msg.pose.orientation.z = current_q.z();

    current_pose_pub_->publish(current_pose_msg);
}

} // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::HqpReferenceGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}