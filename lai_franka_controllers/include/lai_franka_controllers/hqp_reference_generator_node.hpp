/// @file hqp_reference_generator_node.hpp
/// @brief Hierarchical Quadratic Programming (HQP) Task-Space Controller Brain for Franka FR3.
/// @author Clement Lamouller
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__HQP_REFERENCE_GENERATOR_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__HQP_REFERENCE_GENERATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

// ROS 2 Core Communication Modules
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lai_franka_controllers/msg/hqp_distances.hpp"

// Eigen Matrix Math and Core HQP Solver Architecture
#include <Eigen/Dense>
#include <task/allTasks.hpp>
#include <hierarchical_qp/hierarchicalQP.h>
#include <robot_kinematics/FrankaKinematics.hpp>

using namespace task;
using namespace robot_kinematics;

namespace lai_franka_controllers {

/// @class HqpReferenceGeneratorNode
/// @brief Orchestrates a cascaded optimization priority task stack over a virtual integration model.
///
/// Subscribes to input waypoint tracks and processes task-space goals against joint limits,
/// velocity saturation ceilings, self-collisions, and an enclosed 6-sided virtual workspace boundary box.
/// Outputs are optimized joint positions and velocities driven smoothly at 500 Hz.
class HqpReferenceGeneratorNode : public rclcpp::Node {
public:
    /// @brief Constructor for the HQP Task Architecture Optimization Node.
    HqpReferenceGeneratorNode();
    
    /// @brief Default Virtual Destructor.
    virtual ~HqpReferenceGeneratorNode() = default;

private:
    /// @struct TargetPose
    /// @brief Storage capsule ensuring thread-safe synchronization of incoming waypoint tracks.
    struct TargetPose {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        bool valid{false};
    };

    /// @brief High-frequency loop advancing the internal integration model and executing the solver stack.
    void timer_callback();

    /// @brief Single-shot hook capturing initial physical hardware encoder frames to align the virtual model.
    /// @param msg Shared pointer to incoming sensor_msgs::msg::JointState telemetry.
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /// @brief Thread-safe intake wrapper caching targeted track waypoints.
    /// @param msg Shared pointer to target geometry_msgs::msg::PoseStamped data.
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /// @brief 
    /// @param msg 
    void target_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ---- ROS 2 Interface Variables ----
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr error_pub_;
    rclcpp::Publisher<lai_franka_controllers::msg::HqpDistances>::SharedPtr virtualwall_dist_pub_;
    rclcpp::Publisher<lai_franka_controllers::msg::HqpDistances>::SharedPtr selfhits_dist_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;

    // ---- Synchronization and Operational Safeguards ----
    std::vector<std::string> joint_names_;
    std::mutex data_mutex_;
    std::string task_mode_;
    TargetPose pose_target_;
    Eigen::VectorXd target_q_;
    Eigen::VectorXd target_dq_;
    bool is_initialized_{false};
    rclcpp::Time last_time_;

    // ---- Core HQP Optimization Objects ----
    std::shared_ptr<FrankaKinematics> kinematics_;
    std::shared_ptr<HierarchicalQP> solver_;
    std::vector<std::shared_ptr<Task>> task_stack_;
    
    // ---- Layered Task Definitions Stack Interfaces ----
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
    std::shared_ptr<task::JointTracking> joint_tracking_task_;

    // ---- Virtual Internal Model State Vectors (7-DOF Constraints) ----
    Eigen::Matrix<double, 7, 1> q_virtual_;
    Eigen::Matrix<double, 7, 1> dq_hqp_;
    Eigen::Matrix<double, 7, 1> q_max_;
    Eigen::Matrix<double, 7, 1> q_min_;
    Eigen::Matrix<double, 7, 1> dq_limit_;
};

}  // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__HQP_REFERENCE_GENERATOR_NODE_HPP_