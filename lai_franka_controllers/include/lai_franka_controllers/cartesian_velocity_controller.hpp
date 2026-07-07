/// @file cartesian_velocity_controller.hpp
/// @brief Analytical Pinocchio-driven Cartesian task-space velocity controller for Franka FR3.
/// @author Clement Lamouller
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define LAI_FRANKA_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <atomic>

// ROS 2 Control Core Lifecycle Framework
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

// ROS 2 Core Interface Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// Eigen Matrix Core and Pinocchio Kinematics Solvers
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace lai_franka_controllers {

/// @class CartesianVelocityController
/// @brief Direct Task-Space Jacobian Inverse Controller featuring analytical nullspace optimization.
///
/// This component applies a Levenberg-Marquardt style adaptive damping proxy over the calculated 
/// robot Jacobian to stabilize transitions inside singularity neighborhoods. Redundancy control maps
/// joint-limit repulsion and manipulability gradients inside the remaining nullspace projector.
class CartesianVelocityController : public controller_interface::ControllerInterface {
public:
    /// @brief Default constructor.
    CartesianVelocityController() = default;
    
    /// @brief Default Destructor.
    virtual ~CartesianVelocityController() = default;

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
    /// @brief Thread-safe tracker ensuring atomic consumption of incoming Cartesian reference states.
    struct TargetPose {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        bool valid{false};
    };

    // ---- Communications and Configuration Settings ----
    std::vector<std::string> joint_names_;
    std::string end_effector_frame_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    realtime_tools::RealtimeBuffer<TargetPose> rt_target_pose_ptr_;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> error_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> rt_error_pub_;

    // ---- Pinocchio Structural Model Context Templates ----
    pinocchio::Model model_;
    std::shared_ptr<pinocchio::Data> data_;
    std::shared_ptr<pinocchio::Data> data_tmp_;
    pinocchio::FrameIndex ee_frame_id_;

    // ---- Real-time Math Core Cache Vectors and Matrices ----
    Eigen::Matrix<double, 7, 1> q_current_;
    Eigen::Matrix<double, 7, 1> dq_cmd_;
    Eigen::Matrix<double, 7, 1> dq_max_;
    
    Eigen::Vector3d x_current_;
    Eigen::Quaterniond quat_current_;
    
    Eigen::Vector3d x_target_;
    Eigen::Quaterniond quat_target_;

    Eigen::Vector3d pos_error_;
    Eigen::Quaterniond quat_error_;
    Eigen::Vector3d ori_error_;

    Eigen::Matrix<double, 6, 7> jacobian_;
    Eigen::Matrix<double, 6, 1> twist_error_;
    Eigen::Matrix<double, 6, 6> K_gain_;

    // ---- Analytical Secondary Redundancy Nullspace Configurations ----
    double K_null_{0.8};
    Eigen::Matrix<double, 7, 1> q_mid_;
    Eigen::Matrix<double, 7, 1> dq_limit_;
    Eigen::Matrix<double, 7, 1> dq_null_;
    Eigen::Matrix<double, 7, 1> dq_task_;
    Eigen::Matrix<double, 7, 1> q_perturbed_;
    Eigen::Matrix<double, 7, 7> I7_;
    Eigen::Matrix<double, 7, 7> N_;
    Eigen::Matrix<double, 6, 7> J_tmp_;

    Eigen::Matrix<double, 7, 1> grad_w_cached_;
    std::atomic<int> grad_cycle_counter_{0};
    static constexpr int GRAD_CYCLE_PERIOD = 50; 

    const double w_manip_weight_{0.3};
    const double w_limit_weight_{0.7};
    
    // ---- Damped Singular Value Pseudo-Inverse Schedulers ----
    Eigen::Matrix<double, 6, 6> I6_;
    Eigen::Matrix<double, 6, 6> JJt_;
    Eigen::Matrix<double, 6, 6> JJt_inv_;
    Eigen::Matrix<double, 7, 6> J_pinv_;
    double lambda_damping_{1e-4};
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt_solver_;

    // ---- Operational Continuity Safeties ----
    rclcpp::Time last_target_time_{0};
    double activation_ramp_{0.0};
    Eigen::Matrix<double, 7, 1> dq_cmd_prev_;
};

}  // namespace lai_franka_controllers

#endif  // LAI_FRANKA_CONTROLLERS__CARTESIAN_VELOCITY_CONTROLLER_HPP_