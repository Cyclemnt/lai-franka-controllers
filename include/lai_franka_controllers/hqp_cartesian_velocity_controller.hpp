#ifndef LAI_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_
#define LAI_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS 2 Control
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

// ROS 2 Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "my_franka_msgs/msg/hqp_distances.hpp"

// Eigen & HQP Architecture
#include <Eigen/Dense>
#include <task/allTasks.hpp>
#include <hierarchical_qp/hierarchicalQP.h>
#include <robot_kinematics/FrankaKinematics.hpp>

namespace lai_franka_controllers {

class HqpCartesianVelocityController : public controller_interface::ControllerInterface {
public:
    HqpCartesianVelocityController() = default;
    ~HqpCartesianVelocityController() = default;

    // controller_interface::ControllerInterface Overrides
    controller_interface::CallbackReturn on_init() override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    // Custom structure to avoid shared_ptr<PoseStamped>
    struct TargetPose {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        bool valid{false};
    };

    // ROS 2 Parameters & Communication
    std::vector<std::string> joint_names;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
    realtime_tools::RealtimeBuffer<TargetPose> rt_target_pose_ptr;

    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> error_pub;
    std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> rt_error_pub;
    
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> dq_cmd_pub;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_dq_cmd_pub;

    rclcpp::Publisher<my_franka_msgs::msg::HqpDistances>::SharedPtr virtualwall_dist_pub;
    std::shared_ptr<realtime_tools::RealtimePublisher<my_franka_msgs::msg::HqpDistances>> rt_virtualwall_dist_pub;

    rclcpp::Publisher<my_franka_msgs::msg::HqpDistances>::SharedPtr selfhits_dist_pub;
    std::shared_ptr<realtime_tools::RealtimePublisher<my_franka_msgs::msg::HqpDistances>> rt_selfhits_dist_pub;

    // std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_states_pub;
    // std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_joint_states_pub;

    // HQP Components
    std::shared_ptr<FrankaKinematics> kinematics;
    std::shared_ptr<HierarchicalQP> solver;
    
    // The task stack holds pointers to the base Task class
    std::vector<std::shared_ptr<Task>> task_stack;
    std::shared_ptr<JointsConfigurationLimits> q_upper_task;
    std::shared_ptr<JointsConfigurationLimits> q_lower_task;
    std::shared_ptr<JointsVelocityLimits> dq_upper_task;
    std::shared_ptr<JointsVelocityLimits> dq_lower_task;
    std::shared_ptr<SelfHits> self_collision_task;
    std::shared_ptr<VirtualWall> virtual_wall_task_1;
    std::shared_ptr<VirtualWall> virtual_wall_task_2;
    std::shared_ptr<VirtualWall> virtual_wall_task_3;
    std::shared_ptr<VirtualWall> virtual_wall_task_4;
    std::shared_ptr<VirtualWall> virtual_wall_task_5;
    std::shared_ptr<VirtualWall> virtual_wall_task_6;
    std::shared_ptr<Pose> pose_task;
    std::shared_ptr<JointSineTask> sine_task;

    // Math Variables
    Eigen::Matrix<double, 7, 1> q_current;
    Eigen::Matrix<double, 7, 1> q_max;
    Eigen::Matrix<double, 7, 1> q_min;
    Eigen::Matrix<double, 7, 1> dq_cmd;
    Eigen::Matrix<double, 7, 1> dq_limit;
    Eigen::Matrix<double, 7, 1> ddq_limit;
    
    Eigen::Vector3d x_target;
    Eigen::Quaterniond quat_target;

    // Security
    rclcpp::Time last_target_time{0};
};

}  // namespace lai_franka_controllers

#endif // LAI_FRANKA_CONTROLLERS__HQP_CARTESIAN_VELOCITY_CONTROLLER_HPP_