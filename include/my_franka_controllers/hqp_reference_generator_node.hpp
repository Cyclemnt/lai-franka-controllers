#ifndef MY_FRANKA_CONTROLLERS__HQP_REFERENCE_GENERATOR_NODE_HPP_
#define MY_FRANKA_CONTROLLERS__HQP_REFERENCE_GENERATOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "my_franka_msgs/msg/hqp_distances.hpp"

// Eigen & HQP Architecture
#include <Eigen/Dense>
#include <task/allTasks.hpp>
#include <hierarchical_qp/hierarchicalQP.h>
#include <robot_kinematics/FrankaKinematics.hpp>

namespace my_franka_controllers {

class HqpReferenceGeneratorNode : public rclcpp::Node {
public:
    HqpReferenceGeneratorNode();
    ~HqpReferenceGeneratorNode() = default;

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
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr error_pub;
    rclcpp::Publisher<my_franka_msgs::msg::HqpDistances>::SharedPtr virtualwall_dist_pub;
    rclcpp::Publisher<my_franka_msgs::msg::HqpDistances>::SharedPtr selfhits_dist_pub;

    // Variables & Thread Safety
    std::vector<std::string> joint_names;
    std::mutex data_mutex;
    TargetPose current_target;
    bool is_initialized{false};
    rclcpp::Time last_time;

    // HQP Components
    std::shared_ptr<FrankaKinematics> kinematics;
    std::shared_ptr<HierarchicalQP> solver;
    
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

    // Math Variables for Virtual Internal Model
    Eigen::Matrix<double, 7, 1> q_virtual;
    Eigen::Matrix<double, 7, 1> dq_hqp;
    Eigen::Matrix<double, 7, 1> q_max;
    Eigen::Matrix<double, 7, 1> q_min;
    Eigen::Matrix<double, 7, 1> dq_limit;
};

}  // namespace my_franka_controllers

#endif // MY_FRANKA_CONTROLLERS__HQP_REFERENCE_GENERATOR_NODE_HPP_