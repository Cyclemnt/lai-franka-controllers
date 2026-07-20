#ifndef LAI_FRANKA_CONTROLLERS_JOINT_TRAJECTORY_TEST_NODE_HPP
#define LAI_FRANKA_CONTROLLERS_JOINT_TRAJECTORY_TEST_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace lai_franka_controllers {

class JointTrajectoryTestNode : public rclcpp::Node {
public:
    JointTrajectoryTestNode();

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string target_joint_;
    double amplitude_;
    double frequency_;
    
    bool is_initialized_;
    double start_q_;
    rclcpp::Time start_time_;
};

} // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS_JOINT_TRAJECTORY_TEST_NODE_HPP