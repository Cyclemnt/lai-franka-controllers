/// @file joint_sine_publisher_node.hpp
/// @brief Joint space diagnostic sine-wave command publisher node for Franka FR3.
/// @author Clement Lamouller
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <chrono>
#include <mutex>

namespace lai_franka_controllers {

/// @class JointSinePublisherNode
/// @brief Generates synchronous smooth multi-joint sinusoidal position and velocity profiles.
///
/// This node captures initial hardware configurations to generate zero-starting offset
/// waveforms, tracks real-time physical error metrics, and feeds reference streams to low-level controllers.
class JointSinePublisherNode : public rclcpp::Node {
public:
    /// @brief Constructor for the Joint Sine Publisher Node.
    JointSinePublisherNode();
    
    /// @brief Default Virtual Destructor.
    virtual ~JointSinePublisherNode() = default;

private:
    /// @brief High-frequency loop calculating and publishing the smooth waveform trajectory slices.
    void timer_callback();

    /// @brief Ingestion callback monitoring feedback states for error calculations.
    /// @param msg Shared pointer to incoming sensor_msgs::msg::JointState state frames.
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ---- ROS 2 Communication Interfaces ----
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // ---- Synchronous Context Timestamps and Watchdogs ----
    rclcpp::Time start_time_;
    bool is_initialized_{false};
    std::mutex data_mutex_;

    // ---- Trajectory Profile Definition Configuration States ----
    std::vector<std::string> joint_names_;
    std::vector<double> initial_positions_;
    std::vector<double> current_positions_;
    std::vector<double> amplitudes_;
    std::vector<double> frequencies_;
};

} // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__JOINT_SINE_PUBLISHER_NODE_HPP_