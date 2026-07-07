/// @file joy_teleop_node.hpp
/// @brief Gamepad teleoperation node for Franka FR3 tracking Cartesian pose targets via an HQP solver.
/// @author Clement Lamouller
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__JOY_TELEOP_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__JOY_TELEOP_NODE_HPP_

#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// ROS 2 Actions for Franka Gripper
#include "rclcpp_action/rclcpp_action.hpp"
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"

// TF2 Math Libraries
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"

namespace lai_franka_controllers {

/// @class JoyTeleopNode
/// @brief Processes gamepad inputs to smoothly integrate and stream Cartesian pose references.
///
/// This node bypasses TF tracking by subscribing directly to the virtual model's internal pose.
/// It features acceleration limiting, velocity tracking, an anti-windup "leash" mechanism,
/// live speed percentage scaling via the D-pad, and asynchronous Franka gripper action handling.
class JoyTeleopNode : public rclcpp::Node {
public:
    /// @brief Constructor for the Gamepad Teleoperation Node.
    /// @param options Lifecycle node configurations options.
    explicit JoyTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    /// @brief Virtual Default Destructor.
    virtual ~JoyTeleopNode() = default;

private:
    /// @brief Callback processing raw controller axis and button arrays.
    /// @param msg Shared pointer to the incoming sensor_msgs::msg::Joy message.
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /// @brief High-frequency timer loop integrating velocity commands into pose targets.
    void timer_callback();

    /// @brief Safely extracts the current tracking frame of the internal HQP virtual model.
    /// @param[out] x Current X position meters.
    /// @param[out] y Current Y position meters.
    /// @param[out] z Current Z position meters.
    /// @param[out] q Current orientation quaternion.
    /// @return true If a valid solver pose has been received, false otherwise.
    bool get_current_pose(double &x, double &y, double &z, tf2::Quaternion &q);

    // ---- ROS 2 Communication Interfaces ----
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    geometry_msgs::msg::PoseStamped latest_solver_pose_;
    bool has_latest_pose_{false};

    // ---- Franka Gripper Action Clients ----
    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_;
    rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr move_client_;

    // ---- Kinematic Configuration and Time Steps ----
    std::string base_frame_;
    std::string ee_frame_;
    double publish_rate_;
    double dt_;

    // ---- Shared Gamepad Commands ----
    double cmd_x_{0.0},     cmd_y_{0.0},     cmd_z_{0.0};
    double cmd_roll_{0.0},  cmd_pitch_{0.0}, cmd_yaw_{0.0};

    // ---- Internal Smooth State Velocities ----
    double current_vel_x_{0.0},    current_vel_y_{0.0},    current_vel_z_{0.0};
    double current_vel_roll_{0.0}, current_vel_pitch_{0.0}, current_vel_yaw_{0.0};

    // ---- Integrated Reference Pose Targets ----
    double target_x_{0.0}, target_y_{0.0}, target_z_{0.0};
    tf2::Quaternion target_q_{0.0, 0.0, 0.0, 1.0};

    bool is_initialized_{false};
    
    // ---- End-Effector Gripper States ----
    bool gripper_closed_{false};
    bool button_a_prev_{false};

    // ---- Live Speed Profile Scaling ----
    int speed_percentage_{50}; 
    bool dpad_left_prev_{false};
    bool dpad_right_prev_{false};
};

}  // namespace lai_franka_controllers

#endif  // LAI_FRANKA_CONTROLLERS__JOY_TELEOP_NODE_HPP_