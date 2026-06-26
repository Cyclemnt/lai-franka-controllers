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

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    rclcpp::TimerBase::SharedPtr timer;
    
    rclcpp::Time start_time;
    bool is_initialized{false};
    std::mutex data_mutex;

    std::vector<std::string> joint_names;
    std::vector<double> initial_positions;
    std::vector<double> current_positions; // Added to track live hardware state
    std::vector<double> amplitudes;
    std::vector<double> frequencies;
};

} // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_