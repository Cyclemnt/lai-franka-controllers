/// @file hqp_joint_trajectory_test_node.cpp
/// @brief Implementation of the HQP-targeted multi-joint trajectory generator.

#include "lai_franka_controllers/hqp_joint_trajectory_test_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace lai_franka_controllers {

HqpJointTrajectoryTestNode::HqpJointTrajectoryTestNode() : Node("hqp_joint_trajectory_test_node") {
    
    // Configurable parameters - Default targeted at HQP Reference Generator
    this->declare_parameter<std::string>("target_topic", "/hqp_reference_generator_node/target_joint");
    
    // Set default amplitudes (rad) and frequencies (Hz) for all 7 joints
    this->declare_parameter<std::vector<double>>("amplitudes", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    this->declare_parameter<std::vector<double>>("frequencies", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    std::string target_topic = this->get_parameter("target_topic").as_string();
    amplitudes_ = this->get_parameter("amplitudes").as_double_array();
    frequencies_ = this->get_parameter("frequencies").as_double_array();

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(target_topic, 10);
    
    // Listen to actual hardware exactly once to anchor the trajectory safely
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&HqpJointTrajectoryTestNode::joint_state_callback, this, std::placeholders::_1));

    joint_names_ = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};

    const size_t num_joints = joint_names_.size();
    initial_positions_.resize(num_joints, 0.0);

    if (amplitudes_.size() != num_joints || frequencies_.size() != num_joints) {
        RCLCPP_ERROR(this->get_logger(), "Waveform profile array sizes must strictly match the 7 joint dimensions.");
    }

    // 100 Hz tracking calculation loop
    timer_ = this->create_wall_timer(10ms, std::bind(&HqpJointTrajectoryTestNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "HQP Trajectory Generator Started. Target Topic: %s", target_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Waiting for hardware state initialization via /joint_states...");
}

void HqpJointTrajectoryTestNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (is_initialized_) return;

    size_t matched_joints = 0;
    const size_t num_joints = joint_names_.size();
    std::vector<double> temp_positions(num_joints, 0.0);

    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < num_joints; ++j) {
            if (msg->name[i] == joint_names_[j]) {
                temp_positions[j] = msg->position[i];
                matched_joints++;
            }
        }
    }

    // Once all 7 joints are located, lock the initial positions and sever the subscription
    if (matched_joints == num_joints) {
        initial_positions_ = temp_positions;
        start_time_ = this->get_clock()->now();
        is_initialized_ = true;
        
        // Disconnect to save CPU overhead—HQP will handle closed-loop error diagnostics
        joint_sub_.reset(); 
        
        RCLCPP_INFO(this->get_logger(), "Initial hardware positions mapped. Streaming trajectory to HQP Solver...");
    }
}

void HqpJointTrajectoryTestNode::timer_callback() {
    if (!is_initialized_) return;

    auto msg = sensor_msgs::msg::JointState();
    rclcpp::Time current_time = this->get_clock()->now();
    const size_t num_joints = joint_names_.size();

    msg.header.stamp = current_time;
    msg.name = joint_names_;
    msg.position.resize(num_joints);
    msg.velocity.resize(num_joints);

    double t = (current_time - start_time_).seconds();
    
    for (size_t i = 0; i < num_joints; ++i) {
        double omega = 2.0 * M_PI * frequencies_[i];
        
        // Smooth Cosine Trajectory: Velocity is mathematically 0 at t=0
        // q = q0 + A * (1 - cos(w * t))
        msg.position[i] = initial_positions_[i] + amplitudes_[i] * (1.0 - std::cos(omega * t));
        
        // Feedforward Velocity derivative: dq = A * w * sin(w * t)
        msg.velocity[i] = amplitudes_[i] * omega * std::sin(omega * t);
    }

    publisher_->publish(msg);
}

} // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::HqpJointTrajectoryTestNode>());
    rclcpp::shutdown();
    return 0;
}