#ifndef LAI_FRANKA_CONTROLLERS__CARTESIAN_REFERENCE_GENERATOR_NODE_HPP_
#define LAI_FRANKA_CONTROLLERS__CARTESIAN_REFERENCE_GENERATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// Eigen & Pinocchio
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace lai_franka_controllers {

class CartesianReferenceGeneratorNode : public rclcpp::Node {
public:
    CartesianReferenceGeneratorNode();
    ~CartesianReferenceGeneratorNode() = default;

private:
    struct TargetPose {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        bool valid{false};
    };

    // Callbacks
    void timer_callback();
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // ROS 2 Communication
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr error_pub_;

    // Variables & Thread Safety
    std::vector<std::string> joint_names;
    std::string end_effector_frame;
    double control_frequency_{100.0};
    
    std::mutex data_mutex_;
    TargetPose current_target_;
    bool is_initialized_{false};
    rclcpp::Time last_time_;
    rclcpp::Time last_target_time_;

    // Pinocchio Kinematics
    pinocchio::Model model;
    std::shared_ptr<pinocchio::Data> data;
    std::shared_ptr<pinocchio::Data> data_tmp;
    pinocchio::FrameIndex ee_frame_id;

    // Pre-allocated Math Variables for Virtual Model
    Eigen::Matrix<double, 7, 1> q_virtual;
    Eigen::Matrix<double, 7, 1> dq_cmd;
    Eigen::Matrix<double, 7, 1> dq_max;
    
    Eigen::Vector3d x_current;
    Eigen::Quaterniond quat_current;
    
    Eigen::Vector3d pos_error;
    Eigen::Quaterniond quat_error;
    Eigen::Vector3d ori_error;

    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 6, 1> twist_error;
    Eigen::Matrix<double, 6, 6> K_gain;

    // Nullspace control variables
    double K_null;
    Eigen::Matrix<double, 7, 1> q_mid;
    Eigen::Matrix<double, 7, 1> dq_limit;
    Eigen::Matrix<double, 7, 1> dq_null;
    Eigen::Matrix<double, 7, 1> dq_task;
    Eigen::Matrix<double, 7, 1> q_perturbed;
    Eigen::Matrix<double, 7, 7> I7;
    Eigen::Matrix<double, 7, 7> N;
    Eigen::Matrix<double, 6, 7> J_tmp;

    Eigen::Matrix<double, 7, 1> grad_w_cached;
    int grad_cycle_counter{0};
    int grad_cycle_period{5}; // Scaled down for 100Hz instead of 1000Hz

    const double w_manip_weight{0.3};
    const double w_limit_weight{0.7};
    
    // Damped pseudo-inverse variables
    Eigen::Matrix<double, 6, 6> I6;
    Eigen::Matrix<double, 6, 6> JJt;
    Eigen::Matrix<double, 6, 6> JJt_inv;
    Eigen::Matrix<double, 7, 6> J_pinv;
    double lambda_damping{1e-4};
    Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt_solver;

    double activation_ramp{0.0};
    Eigen::Matrix<double, 7, 1> dq_cmd_prev;
};

}  // namespace lai_franka_controllers

#endif  // LAI_FRANKA_CONTROLLERS__CARTESIAN_REFERENCE_GENERATOR_NODE_HPP_