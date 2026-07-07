/// @file joint_sine_publisher_node.cpp
/// @brief Diagnostic trajectory calculation and system tracking loop details.

#include "lai_franka_controllers/joint_sine_publisher_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace lai_franka_controllers {

JointSinePublisherNode::JointSinePublisherNode() : Node("joint_sine_publisher") {
    
    // Declare dynamic parameters with baseline tracking fallbacks
    this->declare_parameter<std::string>("target_topic", "/joint_pd_velocity_controller/joint_commands");
    this->declare_parameter<std::vector<double>>("amplitudes", {0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10});
    this->declare_parameter<std::vector<double>>("frequencies", {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05});

    std::string target_topic = this->get_parameter("target_topic").as_string();
    amplitudes_ = this->get_parameter("amplitudes").as_double_array();
    frequencies_ = this->get_parameter("frequencies").as_double_array();

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(target_topic, 10);
    
    // Subscribe to track initial hardware home configurations and error properties
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&JointSinePublisherNode::joint_state_callback, this, std::placeholders::_1));

    joint_names_ = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};

    const size_t num_joints = joint_names_.size();
    initial_positions_.resize(num_joints, 0.0);
    current_positions_.resize(num_joints, 0.0);

    if (amplitudes_.size() != num_joints || frequencies_.size() != num_joints) {
        RCLCPP_ERROR(this->get_logger(), "Waveform profile array sizes must strictly match joint dimensions.");
    }

    // 100 Hz tracking calculations update loop
    timer_ = this->create_wall_timer(10ms, std::bind(&JointSinePublisherNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for hardware state initialization via /joint_states...");
}

void JointSinePublisherNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
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

    if (matched_joints == num_joints) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        current_positions_ = temp_positions;
        
        if (!is_initialized_) {
            initial_positions_ = temp_positions;
            start_time_ = this->get_clock()->now();
            is_initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Initial hardware positions mapped. Beginning profile tracking output execution.");
        }
    }
}

void JointSinePublisherNode::timer_callback() {
    if (!is_initialized_) return;

    auto msg = sensor_msgs::msg::JointState();
    rclcpp::Time current_time = this->get_clock()->now();
    const size_t num_joints = joint_names_.size();

    msg.header.stamp = current_time;
    msg.name = joint_names_;
    msg.position.resize(num_joints);
    msg.velocity.resize(num_joints);

    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Analytical Sinusoidal trajectory generations calculation
    double t = (current_time - start_time_).seconds();
    for (size_t i = 0; i < num_joints; ++i) {
        double omega = 2.0 * M_PI * frequencies_[i];
        
        // Formulated offset protects system limits by starting calculation tracking with zero acceleration:
        // q = q0 + A * (1 - cos(w * t))
        msg.position[i] = initial_positions_[i] + amplitudes_[i] * (1.0 - std::cos(omega * t));
        
        // dq = A * w * sin(w * t)
        msg.velocity[i] = amplitudes_[i] * omega * std::sin(omega * t);
    }

    // Capture closed-loop tracking diagnostic updates
    double max_tracking_error = 0.0;
    for (size_t i = 0; i < num_joints; ++i) {
        double err = std::abs(current_positions_[i] - msg.position[i]);
        if (err > max_tracking_error) {
            max_tracking_error = err;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Max closed-loop controller tracking error: %.5f rad", max_tracking_error);

    publisher_->publish(msg);
}

} // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::JointSinePublisherNode>());
    rclcpp::shutdown();
    return 0;
}