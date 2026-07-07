/// @file trajectory_generator_node.hpp
/// @brief 5th-order polynomial trajectory generator node for the Franka FR3 robot arm.
/// @author Clement Lamouller
/// @date 2026

#ifndef LAI_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>

namespace lai_franka_controllers {

/// @class TrajectoryGenerator
/// @brief Interpolates smooth 5th-order minimum-jerk profile segments between Cartesian waypoints.
///
/// This node maps references based natively on the HQP virtual integration model to bypass
/// any initialization jerks caused by hardware feedback latency.
class TrajectoryGenerator : public rclcpp::Node {
public:
    /// @brief Constructor for the Trajectory Generator Node.
    TrajectoryGenerator();
    
    /// @brief Default Virtual Destructor.
    virtual ~TrajectoryGenerator() = default;

private:
    /// @struct Waypoint
    /// @brief Internal structure mapping a Cartesian coordinate point and orientation target.
    struct Waypoint {
        Eigen::Vector3d pos;
        Eigen::Quaterniond ori;
        double duration{-1.0};
    };

    /// @enum Mode
    /// @brief Operating state enumeration for track selection profiles.
    enum class Mode { IDLE, MANUAL, STRESS_TEST };

    /// @brief Callback processing individual Cartesian input target goals.
    /// @param msg Shared pointer to incoming geometry_msgs::msg::PoseStamped data.
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /// @brief High-frequency loop tracking interpolation profiles step-by-step.
    void timer_callback();

    /// @brief Generates automated operational verification loops.
    void start_stress_test();

    /// @brief Sets boundary limits and solves quintic profile durations.
    /// @param target_p Goal translation coordinates vector.
    /// @param target_q Goal orientation quaternion.
    /// @param manual_duration Forced timing tracking overriding analytical calculation if > 0.
    void prepare_segment(const Eigen::Vector3d& target_p, const Eigen::Quaterniond& target_q, double manual_duration = -1.0);

    // ---- ROS 2 Communication Interfaces ----
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub_;
    geometry_msgs::msg::PoseStamped latest_solver_pose_;
    bool has_latest_pose_{false};

    // ---- Operational Track Control States ----
    Mode current_mode_{Mode::IDLE};
    bool trajectory_active_{false};
    rclcpp::Time t_start_;
    double duration_{0.0};
    
    size_t current_waypoint_idx_{0};
    std::vector<Waypoint> waypoints_;

    // ---- Kinematic Limits Constraints ----
    static constexpr double SAFETY_FACTOR = 0.05;
    const double max_linear_vel_{2.00 * SAFETY_FACTOR};
    const double max_angular_vel_{2.62 * SAFETY_FACTOR};
    const double max_linear_acc_{1.0 * SAFETY_FACTOR};
    const double max_angular_acc_{1.5 * SAFETY_FACTOR};
    const double min_duration_{0.5};

    // ---- Boundary Condition Vectors ----
    Eigen::Vector3d p_start_, p_goal_;
    Eigen::Quaterniond q_start_, q_goal_;
};

} // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_