#ifndef LAI_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>

namespace lai_franka_controllers {

class JointSinePublisherNode : public rclcpp::Node {
public:
    JointSinePublisherNode();
    virtual ~JointSinePublisherNode() = default;

private:
    void timer_callback();
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time start_time_;
    bool is_initialized_{false};
    std::mutex data_mutex_;

    // Delay variables
    int iteration_count_{0};
    int n_delay_iterations_{1}; // Wait for n iterations before starting the sine wave

    std::vector<std::string> joint_names_;
    std::vector<double> initial_positions_;
    std::vector<double> current_positions_; // Added to track live hardware state
    std::vector<double> amplitudes_;
    std::vector<double> frequencies_;
};

} // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_