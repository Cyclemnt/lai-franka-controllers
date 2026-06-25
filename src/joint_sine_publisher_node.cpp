#include "my_franka_controllers/joint_sine_publisher_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace my_franka_controllers {

JointSinePublisherNode::JointSinePublisherNode() : Node("joint_sine_publisher") {
    
    // Declare and get the target topic name
    this->declare_parameter<std::string>("target_topic", "/joint_pd_velocity_controller/joint_commands");
    std::string target_topic = this->get_parameter("target_topic").as_string();

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(target_topic, 10);
    
    // Subscribe to capture the initial position safely
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&JointSinePublisherNode::joint_state_callback, this, std::placeholders::_1));

    joint_names_ = {
        "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", 
        "fr3_joint5", "fr3_joint6", "fr3_joint7"
    };

    initial_positions_.resize(7, 0.0);

    // Safe, gentle amplitudes and identical frequencies for a smooth test dance
    amplitudes_  = {0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10}; // Radians
    frequencies_ = {0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15}; // Hz

    // 100 Hz Loop
    timer_ = this->create_wall_timer(
        10ms, std::bind(&JointSinePublisherNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states to capture initial hardware position...");
}

void JointSinePublisherNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (is_initialized_) return;

    int matched_joints = 0;
    std::vector<double> temp_positions(7, 0.0);

    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < joint_names_.size(); ++j) {
            if (msg->name[i] == joint_names_[j]) {
                temp_positions[j] = msg->position[i];
                matched_joints++;
            }
        }
    }

    if (matched_joints == 7) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        initial_positions_ = temp_positions;
        start_time_ = this->get_clock()->now();
        is_initialized_ = true;
        
        // We only need the initial position once, so we can destroy the subscriber
        joint_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Initial positions captured! Starting safe sine trajectory.");
    }
}

void JointSinePublisherNode::timer_callback() {
    if (!is_initialized_) return;

    auto msg = sensor_msgs::msg::JointState();
    rclcpp::Time current_time = this->get_clock()->now();
    double t = (current_time - start_time_).seconds();

    msg.header.stamp = current_time;
    msg.name = joint_names_;
    
    msg.position.resize(7);
    msg.velocity.resize(7);

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < 7; ++i) {
        double omega = 2.0 * M_PI * frequencies_[i];
        
        // SAFE START MATH: q = q0 + A * (1 - cos(wt))
        // This ensures that at t=0, q = q0 and dq = 0. 
        msg.position[i] = initial_positions_[i] + amplitudes_[i] * (1.0 - std::cos(omega * t));
        
        // dq = A * w * sin(wt)
        msg.velocity[i] = amplitudes_[i] * omega * std::sin(omega * t);
    }

    publisher_->publish(msg);
}

} // namespace my_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<my_franka_controllers::JointSinePublisherNode>());
    rclcpp::shutdown();
    return 0;
}