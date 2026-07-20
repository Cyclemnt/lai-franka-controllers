/// @file hqp_joint_trajectory_test_node.hpp
/// @brief High-level HQP task diagnostic sine-wave trajectory generator.
/// @author Clement
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__HQP_JOINT_TRAJECTORY_TEST_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__HQP_JOINT_TRAJECTORY_TEST_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>

namespace lai_franka_controllers {

/// @class HqpJointTrajectoryTestNode
/// @brief Generates synchronous multi-joint trajectories targeted at the HQP Solver.
///
/// **Differentiation from JointSinePublisherNode:**
/// - `JointSinePublisherNode`: Bypasses the HQP and sends commands directly to the low-level 
///   PD controller (`/joint_pd_velocity_controller/joint_commands`) to test physical tracking.
/// - `HqpJointTrajectoryTestNode` (This node): Sends trajectories to the optimization solver 
///   (`/hqp_reference_generator_node/target_joint`) to test the solver's `JointTracking` task, 
///   feedforward velocity integration, and constraint prioritization.
class HqpJointTrajectoryTestNode : public rclcpp::Node {
public:
    /// @brief Constructor configuring the HQP trajectory generator.
    HqpJointTrajectoryTestNode();
    
    virtual ~HqpJointTrajectoryTestNode() = default;

private:
    /// @brief High-frequency loop calculating and publishing the smooth waveform trajectory to the HQP.
    void timer_callback();

    /// @brief Single-shot callback to capture the physical resting state of the robot.
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ---- ROS 2 Communication Interfaces ----
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // ---- Synchronous Context ----
    rclcpp::Time start_time_;
    bool is_initialized_{false};

    // ---- Trajectory Profile Definitions ----
    std::vector<std::string> joint_names_;
    std::vector<double> initial_positions_;
    std::vector<double> amplitudes_;
    std::vector<double> frequencies_;
};

} // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__HQP_JOINT_TRAJECTORY_TEST_NODE_HPP_