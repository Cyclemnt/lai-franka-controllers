#include "lai_franka_controllers/joint_trajectory_test_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace lai_franka_controllers {

JointTrajectoryTestNode::JointTrajectoryTestNode() : Node("joint_trajectory_test_node"), is_initialized_(false) {
    
    // Configurable parameters with safe defaults
    this->declare_parameter("target_joint", "fr3_joint1");
    this->declare_parameter("amplitude", 0.1); // Moves +/- 0.1 rad from start
    this->declare_parameter("frequency", 0.2); // 0.2 Hz (5-second full cycle)

    target_joint_ = this->get_parameter("target_joint").as_string();
    amplitude_ = this->get_parameter("amplitude").as_double();
    frequency_ = this->get_parameter("frequency").as_double();

    // Publisher matching your HQP node's subscriber topic
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/hqp_reference_generator_node/target_joint", 10);

    // Subscribe to physical states to get the starting position
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&JointTrajectoryTestNode::joint_state_callback, this, std::placeholders::_1));

    // Run the trajectory stream at 100 Hz
    timer_ = this->create_wall_timer(10ms, std::bind(&JointTrajectoryTestNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory test node started. Waiting for initial joint state...");
}

void JointTrajectoryTestNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (is_initialized_) return;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == target_joint_) {
            start_q_ = msg->position[i];
            start_time_ = this->get_clock()->now();
            is_initialized_ = true;
            
            // Sever the subscription once we lock in the starting position
            joint_sub_.reset(); 
            RCLCPP_INFO(this->get_logger(), "Initial position for %s acquired: %f rad. Executing smooth trajectory.", target_joint_.c_str(), start_q_);
            break;
        }
    }
}

void JointTrajectoryTestNode::timer_callback() {
    if (!is_initialized_) return;

    rclcpp::Time current_time = this->get_clock()->now();
    double t = (current_time - start_time_).seconds();

    // Smooth Cosine Trajectory ensuring zero velocity at t=0
    double omega = 2.0 * M_PI * frequency_;
    double q_desired = start_q_ + amplitude_ - amplitude_ * std::cos(omega * t);
    double dq_desired = amplitude_ * omega * std::sin(omega * t);

    // Pack both position and feedforward velocity
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = current_time;
    msg.name.push_back(target_joint_);
    msg.position.push_back(q_desired);
    msg.velocity.push_back(dq_desired);

    joint_pub_->publish(msg);
}

} // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::JointTrajectoryTestNode>());
    rclcpp::shutdown();
    return 0;
}