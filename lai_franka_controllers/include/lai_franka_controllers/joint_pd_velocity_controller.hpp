/// @file joint_pd_velocity_controller.hpp
/// @brief Low-level real-time joint space PD velocity tracking controller for Franka FR3.
/// @author Clement Lamouller
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__JOINT_PD_VELOCITY_CONTROLLER_HPP_
#define LAI_FRANKA_CONTROLLERS__JOINT_PD_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS 2 Control Core Lifecycle Interfaces
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

// ROS 2 Communication Messages
#include "sensor_msgs/msg/joint_state.hpp"

// Eigen Matrix Core Math
#include <Eigen/Dense>

namespace lai_franka_controllers {

/// @class JointPdVelocityController
/// @brief Serves as a low-level joint tracking interface within the ros2_control framework.
///
/// Implements a real-time safe proportional position feedback control loop with velocity feedforward:
/// \f$ \dot{q}_{cmd} = K \cdot (q_d - q) + \dot{q}_d \f$
/// Communications rely entirely on lock-free non-blocking real-time buffers and publishers.
class JointPdVelocityController : public controller_interface::ControllerInterface {
public:
    /// @brief Default constructor.
    JointPdVelocityController() = default;
    
    /// @brief Default Destructor.
    virtual ~JointPdVelocityController() = default;

    // ---- ros2_control Lifecycle Interface Overrides ----
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    /// @struct TargetJointState
    /// @brief Encapsulates setpoint targets securely for thread-safe cross-domain passing.
    struct TargetJointState {
        Eigen::VectorXd q_d;
        Eigen::VectorXd dq_d;
        rclcpp::Time timestamp;
        bool valid{false};
    };

    // ---- Configuration Properties ----
    std::vector<std::string> joint_names_;
    size_t num_joints_{0};

    // ---- Lock-Free Diagnostics Transmission Interfaces ----
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> dq_cmd_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_dq_cmd_pub_;
    
    // ---- Control Law Gains and Watchdogs ----
    Eigen::VectorXd k_gains_;
    double timeout_sec_{0.1};

    // ---- Subscriber Reference Ingestion Buffers ----
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    realtime_tools::RealtimeBuffer<TargetJointState> rt_command_ptr_;

    // ---- Real-time Math Cache Vectors ----
    Eigen::VectorXd q_current_;
    Eigen::VectorXd dq_cmd_;
};

}  // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__JOINT_PD_VELOCITY_CONTROLLER_HPP_