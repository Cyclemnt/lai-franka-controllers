#include "my_franka_controllers/joint_sine_publisher_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace my_franka_controllers {

JointSinePublisherNode::JointSinePublisherNode() : Node("joint_sine_publisher") {
    
    // Declare and get the target topic name
    this->declare_parameter<std::string>("target_topic", "/joint_pd_velocity_controller/joint_commands");
    std::string target_topic = this->get_parameter("target_topic").as_string();

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(target_topic, 10);

    // Standard Franka Joint Names
    joint_names_ = {
        "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", 
        "fr3_joint5", "fr3_joint6", "fr3_joint7"
    };

    // Safe, complex multi-frequency parameters based on your previous task
    amplitudes_  = {0.35, 0.30, 0.25, 0.20, 0.15, 0.12, 0.10};
    frequencies_ = {0.03, 0.07, 0.11, 0.17, 0.23, 0.29, 0.35};
    phases_      = {0.1,  0.2,  0.3,  0.4,  0.5,  0.6,  0.7};

    start_time_ = this->get_clock()->now();

    // 100 Hz Loop (10ms period)
    timer_ = this->create_wall_timer(
        10ms, std::bind(&JointSinePublisherNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Joint Sine Publisher started at 100Hz on topic: %s", target_topic.c_str());
}

void JointSinePublisherNode::timer_callback() {
    auto msg = sensor_msgs::msg::JointState();
    
    rclcpp::Time current_time = this->get_clock()->now();
    double t = (current_time - start_time_).seconds();

    msg.header.stamp = current_time;
    msg.name = joint_names_;
    
    msg.position.resize(7);
    msg.velocity.resize(7);

    for (size_t i = 0; i < 7; ++i) {
        double omega = 2.0 * M_PI * frequencies_[i];
        
        // q_D = A * sin(omega * t + phase)
        msg.position[i] = amplitudes_[i] * std::sin(omega * t + phases_[i]);
        
        // dq_D = A * omega * cos(omega * t + phase)
        msg.velocity[i] = amplitudes_[i] * omega * std::cos(omega * t + phases_[i]);
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