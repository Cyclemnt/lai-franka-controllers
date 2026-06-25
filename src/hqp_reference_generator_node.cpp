#include "lai_franka_controllers/hqp_reference_generator_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace lai_franka_controllers {

HqpReferenceGeneratorNode::HqpReferenceGeneratorNode() : Node("hqp_reference_generator_node") {
    
    joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    this->declare_parameter("joint_names", joint_names);

    // DH parameters
    auto declare_if_not_exists = [&](const std::string & name, const auto & default_val) {
        if (!this->has_parameter(name)) {
            this->declare_parameter(name, default_val);
        }
    };
    declare_if_not_exists("mod_DH.a", std::vector<double>());
    declare_if_not_exists("mod_DH.alpha", std::vector<double>());
    declare_if_not_exists("mod_DH.d", std::vector<double>());
    declare_if_not_exists("mod_DH.theta", std::vector<double>());
    declare_if_not_exists("A7e", std::vector<double>());

    // Initialize Bounds
    q_max <<    2.897,  1.832,  2.897, -0.122,  2.879,  4.625,  3.054;
    q_min <<   -2.897, -1.832, -2.897, -3.071, -2.879,  0.436, -3.054;
    dq_limit <<   0.5,    0.5,    0.5,    0.5,    0.5,    0.5,    0.5;
    
    q_virtual.setZero();
    dq_hqp.setZero();

    // Load DH Parameters
    auto a = this->get_parameter("mod_DH.a").as_double_array();
    auto alpha = this->get_parameter("mod_DH.alpha").as_double_array();
    auto d = this->get_parameter("mod_DH.d").as_double_array();
    auto theta = this->get_parameter("mod_DH.theta").as_double_array();
    auto a7e = this->get_parameter("A7e").as_double_array();

    std::vector<double> mDH;
    mDH.insert(mDH.end(), a.begin(), a.end());
    mDH.insert(mDH.end(), alpha.begin(), alpha.end());
    mDH.insert(mDH.end(), d.begin(), d.end());
    mDH.insert(mDH.end(), theta.begin(), theta.end());

    kinematics = std::make_shared<FrankaKinematics>();
    if (mDH.size() == 28 && a7e.size() == 16) {
        kinematics->setParameters(mDH, a7e);
    } else {
        RCLCPP_ERROR(this->get_logger(), "DH Parameter mismatch. Check YAML configuration.");
        throw std::runtime_error("Invalid DH Parameters");
    }

    kinematics->getSelectDOF()->assign(7, true);
    kinematics->getSelectTask()->assign(6, true);

    solver = std::make_shared<HierarchicalQP>(7, GRB_CONTINUOUS);
    
    // Task Creation
    q_upper_task = std::make_shared<JointsConfigurationLimits>(kinematics.get(), q_max, GRB_LESS_EQUAL, 1.0);
    q_lower_task = std::make_shared<JointsConfigurationLimits>(kinematics.get(), q_min, GRB_GREATER_EQUAL, 1.0);
    q_upper_task->setPriorityLevel(1); q_upper_task->setSlacksState(false);
    q_lower_task->setPriorityLevel(1); q_lower_task->setSlacksState(false);
    task_stack.push_back(q_upper_task); task_stack.push_back(q_lower_task);
    
    dq_upper_task = std::make_shared<JointsVelocityLimits>(kinematics.get(), dq_limit, GRB_LESS_EQUAL, 1.0);
    dq_lower_task = std::make_shared<JointsVelocityLimits>(kinematics.get(), -dq_limit, GRB_GREATER_EQUAL, 1.0);
    dq_upper_task->setPriorityLevel(1); dq_upper_task->setSlacksState(false);
    dq_lower_task->setPriorityLevel(1); dq_lower_task->setSlacksState(false);
    task_stack.push_back(dq_upper_task); task_stack.push_back(dq_lower_task);
    
    Eigen::VectorXi sefhits_safe_points(1); sefhits_safe_points << 6;
    Eigen::VectorXi sefhits_avoid_points(2); sefhits_avoid_points << 0, 3;
    self_collision_task = std::make_shared<SelfHits>(kinematics.get(), sefhits_safe_points, sefhits_avoid_points, 0.35, GRB_GREATER_EQUAL, 1.0);
    self_collision_task->setPriorityLevel(2); self_collision_task->setSlacksState(false);
    task_stack.push_back(self_collision_task);

    Eigen::VectorXi joints_to_protect_from_walls(2); joints_to_protect_from_walls << 3, 7; 
    Eigen::Vector3d f1(0,0,0.05), f2(1,0,0.05), f3(0,1,0.05);
    virtual_wall_task_1 = std::make_shared<VirtualWall>(kinematics.get(), f1, f2, f3, joints_to_protect_from_walls, 0.05, GRB_GREATER_EQUAL, 1.0);
    Eigen::Vector3d c1(0,0,1.0), c2(0,1,1.0), c3(1,0,1.0);
    virtual_wall_task_2 = std::make_shared<VirtualWall>(kinematics.get(), c1, c2, c3, joints_to_protect_from_walls, 0.05, GRB_GREATER_EQUAL, 1.0);
    Eigen::Vector3d fr1(0.7,0,0), fr2(0.7,0,1), fr3(0.7,1,0); 
    virtual_wall_task_3 = std::make_shared<VirtualWall>(kinematics.get(), fr1, fr2, fr3, joints_to_protect_from_walls, 0.05, GRB_GREATER_EQUAL, 1.0);
    Eigen::Vector3d bk1(-0.5,0,0), bk2(-0.5,1,0), bk3(-0.5,0,1);
    virtual_wall_task_4 = std::make_shared<VirtualWall>(kinematics.get(), bk1, bk2, bk3, joints_to_protect_from_walls, 0.05, GRB_GREATER_EQUAL, 1.0);
    Eigen::Vector3d l1(0,0.3,0), l2(1,0.3,0), l3(0,0.3,1);
    virtual_wall_task_5 = std::make_shared<VirtualWall>(kinematics.get(), l1, l2, l3, joints_to_protect_from_walls, 0.05, GRB_GREATER_EQUAL, 1.0);
    Eigen::Vector3d r1(0,-0.3,0), r2(0,-0.3,1), r3(1,-0.3,0);
    virtual_wall_task_6 = std::make_shared<VirtualWall>(kinematics.get(), r1, r2, r3, joints_to_protect_from_walls, 0.05, GRB_GREATER_EQUAL, 1.0);

    std::vector<std::shared_ptr<VirtualWall>> all_virtual_walls = {
        virtual_wall_task_1, virtual_wall_task_2, virtual_wall_task_3, 
        virtual_wall_task_4, virtual_wall_task_5, virtual_wall_task_6
    };
    for (auto& wall : all_virtual_walls) {
        wall->setPriorityLevel(3); wall->setSlacksState(false);
        task_stack.push_back(wall);
    }

    pose_task = std::make_shared<Pose>(kinematics.get(), GRB_EQUAL, Eigen::VectorXd::Ones(6), 5.0);
    pose_task->setPriorityLevel(4); pose_task->setSlacksState(true);
    task_stack.push_back(pose_task);

    // Communication Setup
    joint_cmd_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_pd_velocity_controller/joint_commands", 10);
    error_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/tracking_error", 10);
    virtualwall_dist_pub = this->create_publisher<my_franka_msgs::msg::HqpDistances>("~/virtual_wall_distances", 10);
    selfhits_dist_pub = this->create_publisher<my_franka_msgs::msg::HqpDistances>("~/self_hits_distances", 10);

    target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose", 10, std::bind(&HqpReferenceGeneratorNode::target_pose_callback, this, std::placeholders::_1));

    // Subscribe to joint states just to initialize the virtual robot
    joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&HqpReferenceGeneratorNode::joint_state_callback, this, std::placeholders::_1));

    // 500 Hz Timer
    timer = this->create_wall_timer(2ms, std::bind(&HqpReferenceGeneratorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for initial /joint_states to align the virtual model...");
}

void HqpReferenceGeneratorNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (is_initialized) return;

    // We must map the incoming joint states carefully because robot_state_publisher does not always guarantee alphabetical order
    std::vector<double> initial_positions(7, 0.0);
    int matched_joints = 0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < joint_names.size(); ++j) {
            if (msg->name[i] == joint_names[j]) {
                initial_positions[j] = msg->position[i];
                matched_joints++;
            }
        }
    }

    if (matched_joints == 7) {
        std::lock_guard<std::mutex> lock(data_mutex);
        q_virtual = Eigen::VectorXd::Map(initial_positions.data(), 7);
        kinematics->updateJointStates(q_virtual);
        
        current_target.position = kinematics->getPosition();
        current_target.orientation = kinematics->getQuaternion();
        current_target.valid = true;
        
        last_time = this->get_clock()->now();
        is_initialized = true;
        
        // We can safely release the subscriber now to save overhead
        joint_state_sub.reset();
        RCLCPP_INFO(this->get_logger(), "Virtual Model Aligned. HQP Loop Active at 500 Hz.");
    }
}

void HqpReferenceGeneratorNode::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex);
    current_target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    current_target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).normalized();
    current_target.valid = true;
}

void HqpReferenceGeneratorNode::timer_callback() {
    if (!is_initialized) return;

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;

    // Thread-safe copy of the target
    Eigen::Vector3d local_target_pos;
    Eigen::Quaterniond local_target_ori;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        local_target_pos = current_target.position;
        local_target_ori = current_target.orientation;
    }

    // Integrate Virtual Model
    q_virtual += dq_hqp * dt;
    kinematics->updateJointStates(q_virtual);
    kinematics->setDesiredPose(local_target_pos, local_target_ori);

    // HQP SOLVER
    try {
        for (auto& task : task_stack) {
            if (task->isEnabled()) {
                task->update();
                solver->addConstraints(task->get_A(), task->get_b(), task->getSlacksState(), task->getConstraintSense(), task->getPriorityLevel());
            }
        }
        solver->solve();
        dq_hqp = solver->getVarsValue();
        solver->reset();
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "HQP Solver Exception: %s", e.what());
        dq_hqp.setZero();
        solver->reset();
    }

    kinematics->setPreviousVelocities(dq_hqp);

    // Publish to PD Controller
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = current_time; // Current time (unix) is different from original HQP (starting at 0), must change for plotjuggler
    joint_msg.name = joint_names;
    joint_msg.position.resize(7);
    joint_msg.velocity.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        joint_msg.position[i] = q_virtual(i);
        joint_msg.velocity[i] = dq_hqp(i);
    }
    joint_cmd_pub->publish(joint_msg);

    // Diagnostics
    // Tracking Error
    Eigen::VectorXd error = kinematics->getError();
    auto twist_msg = geometry_msgs::msg::TwistStamped();
    twist_msg.header.stamp = current_time;
    twist_msg.twist.linear.x = error(0);
    twist_msg.twist.linear.y = error(1);
    twist_msg.twist.linear.z = error(2);
    twist_msg.twist.angular.x = error(3);
    twist_msg.twist.angular.y = error(4);
    twist_msg.twist.angular.z = error(5);
    error_pub->publish(twist_msg);

    // Virtual Wall Distances
    std::vector<std::shared_ptr<VirtualWall>> walls_to_pub = {
        virtual_wall_task_1, virtual_wall_task_2, virtual_wall_task_3,
        virtual_wall_task_4, virtual_wall_task_5, virtual_wall_task_6
    };
    auto wall_msg = my_franka_msgs::msg::HqpDistances();
    wall_msg.header.stamp = current_time;
    wall_msg.distances.resize(12); 
    int msg_idx = 0;
    for (const auto& wall : walls_to_pub) {
        Eigen::VectorXd dists = wall->get_distances_vector();
        for (int i = 0; i < dists.size(); ++i) {
            if (msg_idx < (int)wall_msg.distances.size()) {
                wall_msg.distances[msg_idx++] = dists(i);
            }
        }
    }
    virtualwall_dist_pub->publish(wall_msg);

    // Self Hits Distances
    if (self_collision_task) {
        Eigen::VectorXd selfhits_dists = self_collision_task->get_distances_vector();
        auto self_msg = my_franka_msgs::msg::HqpDistances();
        self_msg.header.stamp = current_time;
        self_msg.distances.resize(selfhits_dists.size());
        for (int i = 0; i < selfhits_dists.size(); ++i) {
            self_msg.distances[i] = selfhits_dists[i];
        }
        selfhits_dist_pub->publish(self_msg);
    }
}

} // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::HqpReferenceGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}