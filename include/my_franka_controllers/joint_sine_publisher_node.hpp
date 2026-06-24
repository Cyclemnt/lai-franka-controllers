#ifndef MY_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_
#define MY_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <chrono>

namespace my_franka_controllers {

class JointSinePublisherNode : public rclcpp::Node {
public:
    JointSinePublisherNode();
    virtual ~JointSinePublisherNode() = default;

private:
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    std::vector<std::string> joint_names_;
    std::vector<double> amplitudes_;
    std::vector<double> frequencies_;
    std::vector<double> phases_;
};

} // namespace my_franka_controllers

#endif // MY_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_