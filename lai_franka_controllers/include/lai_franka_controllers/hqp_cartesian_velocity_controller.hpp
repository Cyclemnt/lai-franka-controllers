/// @file hqp_cartesian_velocity_controller.hpp
/// @brief Legacy low-level real-time HQP task-space velocity controller for Franka FR3.
/// @author Clement Lamouller
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define LAI_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS 2 Control Core Lifecycle Architecture
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

// ROS 2 Standard Communication Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lai_franka_controllers/msg/hqp_distances.hpp"

// Eigen and Structural Optimization Architecture
#include <Eigen/Dense>
#include <task/allTasks.hpp>
#include <hierarchical_qp/hierarchicalQP.h>
#include <robot_kinematics/FrankaKinematics.hpp>

using namespace task;
using namespace robot_kinematics;

namespace lai_franka_controllers {

/// @class HqpCartesianVelocityController
/// @brief Direct task-space controller running an active prioritization optimizer directly in the RT loop.
///
/// Bypasses the independent virtual internal model node layout to evaluate multi-priority safety tasks 
/// and calculate command output variables during a single 1kHz cycle execution pass.
class HqpCartesianVelocityController : public controller_interface::ControllerInterface {
public:
    /// @brief Default constructor.
    HqpCartesianVelocityController() = default;
    
    /// @brief Default Destructor.
    virtual ~HqpCartesianVelocityController() = default;

    // ---- ros2_control Lifecycle Interface Overrides ----
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    /// @struct TargetPose
    /// @brief Thread-safe encapsulation mapping input cartesian tracking targets across real-time loop cycles.
    struct TargetPose {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        bool valid{false};
    };

    /// @struct TargetJoint
    /// @brief Thread-safe encapsulation mapping input joint tracking targets across real-time loop cycles.
    struct TargetJoint {
        Eigen::Matrix<double, 7, 1> q{Eigen::Matrix<double, 7, 1>::Zero()};
        Eigen::Matrix<double, 7, 1> dq{Eigen::Matrix<double, 7, 1>::Zero()};
        bool valid{false};
    };

    // ---- Configuration Property Buffers ----
    std::vector<std::string> joint_names_;
    std::string task_mode_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    realtime_tools::RealtimeBuffer<TargetPose> rt_target_pose_ptr_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_sub_;
    realtime_tools::RealtimeBuffer<TargetJoint> rt_target_joint_ptr_;

    // ---- Lock-Free Diagnostics Transmission Interfaces ----
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> error_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> rt_error_pub_;
    
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> dq_cmd_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_dq_cmd_pub_;

    rclcpp::Publisher<lai_franka_controllers::msg::HqpDistances>::SharedPtr virtualwall_dist_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<lai_franka_controllers::msg::HqpDistances>> rt_virtualwall_dist_pub_;

    rclcpp::Publisher<lai_franka_controllers::msg::HqpDistances>::SharedPtr selfhits_dist_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<lai_franka_controllers::msg::HqpDistances>> rt_selfhits_dist_pub_;

    // ---- Core HQP Optimization Variables ----
    std::shared_ptr<FrankaKinematics> kinematics_;
    std::shared_ptr<HierarchicalQP> solver_;
    
    std::vector<std::shared_ptr<Task>> task_stack_;
    std::shared_ptr<JointsConfigurationLimits> q_upper_task_;
    std::shared_ptr<JointsConfigurationLimits> q_lower_task_;
    std::shared_ptr<JointsVelocityLimits> dq_upper_task_;
    std::shared_ptr<JointsVelocityLimits> dq_lower_task_;
    std::shared_ptr<SelfHits> self_collision_task_;
    std::shared_ptr<VirtualWall> virtual_wall_task_1_;
    std::shared_ptr<VirtualWall> virtual_wall_task_2_;
    std::shared_ptr<VirtualWall> virtual_wall_task_3_;
    std::shared_ptr<VirtualWall> virtual_wall_task_4_;
    std::shared_ptr<VirtualWall> virtual_wall_task_5_;
    std::shared_ptr<VirtualWall> virtual_wall_task_6_;
    std::shared_ptr<Pose> pose_task_;
    std::shared_ptr<JointTracking> joint_tracking_task_;

    // ---- Real-time Math State Vectors (7-DOF Space Requirements) ----
    Eigen::Matrix<double, 7, 1> q_current_;
    Eigen::Matrix<double, 7, 1> q_max_;
    Eigen::Matrix<double, 7, 1> q_min_;
    Eigen::Matrix<double, 7, 1> dq_cmd_;
    Eigen::Matrix<double, 7, 1> dq_limit_;
    
    Eigen::Vector3d x_target_;
    Eigen::Quaterniond quat_target_;
    Eigen::Matrix<double, 7, 1> q_target_;
    Eigen::Matrix<double, 7, 1> dq_target_;

    // ---- Timing Watchdogs and Guard Gates ----
    rclcpp::Time last_target_time_{0};
};

}  // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_