#ifndef LAI_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <vector>

namespace lai_franka_controllers {

class TrajectoryGenerator : public rclcpp::Node {
public:
    TrajectoryGenerator();

private:
    struct Waypoint {
        Eigen::Vector3d pos;
        Eigen::Quaterniond ori;
        double duration{-1.0};
    };

    enum class Mode { IDLE, MANUAL, STRESS_TEST };

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void timer_callback();

    void start_stress_test();
    void prepare_segment(const Eigen::Vector3d& target_p, const Eigen::Quaterniond& target_q, double manual_duration = -1.0);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_sub;
    geometry_msgs::msg::PoseStamped latest_solver_pose;
    bool has_latest_pose{false};

    // State
    Mode current_mode{Mode::IDLE};
    bool trajectory_active{false};
    rclcpp::Time t_start;
    double duration{0.0};
    
    size_t current_waypoint_idx{0};
    std::vector<Waypoint> waypoints;

// Constraints
    static constexpr double SAFETY_FACTOR = 0.05;
    const double max_linear_vel{2.00 * SAFETY_FACTOR};
    const double max_angular_vel{2.62 * SAFETY_FACTOR};
    const double max_linear_acc{1.0 * SAFETY_FACTOR};
    const double max_angular_acc{1.5 * SAFETY_FACTOR};
    const double min_duration{0.5};

    Eigen::Vector3d p_start, p_goal;
    Eigen::Quaterniond q_start, q_goal;
};

} // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__TRAJECTORY_GENERATOR_NODE_HPP_