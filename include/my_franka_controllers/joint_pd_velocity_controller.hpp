#ifndef MY_FRANKA_CONTROLLERS__JOINT_PD_VELOCITY_CONTROLLER_HPP_
#define MY_FRANKA_CONTROLLERS__JOINT_PD_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS 2 Control
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

// ROS 2 Messages
#include "sensor_msgs/msg/joint_state.hpp"

// Eigen
#include <Eigen/Dense>

namespace my_franka_controllers {

class JointPdVelocityController : public controller_interface::ControllerInterface {
public:
    JointPdVelocityController() = default;
    ~JointPdVelocityController() = default;

    // controller_interface::ControllerInterface Overrides
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Custom structure to hold target state in the realtime buffer
    struct TargetJointState {
        Eigen::VectorXd q_d;
        Eigen::VectorXd dq_d;
        rclcpp::Time timestamp;
        bool valid{false};
    };

    // ROS 2 Parameters & Communication
    std::vector<std::string> joint_names;
    size_t num_joints;
    
    // Proportional Gains (K)
    Eigen::VectorXd k_gains;

    // Safety Timeout
    double timeout_sec{0.1};

    // Subscription and Realtime Buffer
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub;
    realtime_tools::RealtimeBuffer<TargetJointState> rt_command_ptr;

    // Math Variables
    Eigen::VectorXd q_current;
    Eigen::VectorXd dq_cmd;
};

}  // namespace my_franka_controllers

#endif // MY_FRANKA_CONTROLLERS__JOINT_PD_VELOCITY_CONTROLLER_HPP_