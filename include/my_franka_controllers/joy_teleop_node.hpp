#ifndef MY_FRANKA_CONTROLLERS__JOY_TELEOP_NODE_HPP_
#define MY_FRANKA_CONTROLLERS__JOY_TELEOP_NODE_HPP_

#include <memory>
#include <string>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"

namespace my_franka_controllers {

class JoyTeleopNode : public rclcpp::Node {
public:
    explicit JoyTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~JoyTeleopNode() = default;

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();
    bool get_current_pose(double &x, double &y, double &z, tf2::Quaternion &q);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::TimerBase::SharedPtr timer;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::string base_frame;
    std::string ee_frame;
    double publish_rate;
    double dt;

    double cmd_x{0.0}, cmd_y{0.0}, cmd_z{0.0};
    double cmd_roll{0.0}, cmd_pitch{0.0}, cmd_yaw{0.0};

    double current_vel_x{0.0}, current_vel_y{0.0}, current_vel_z{0.0};
    double current_vel_roll{0.0}, current_vel_pitch{0.0}, current_vel_yaw{0.0};

    double target_x{0.0}, target_y{0.0}, target_z{0.0};
    tf2::Quaternion target_q{0.0, 0.0, 0.0, 1.0};

    bool is_initialized{false};
};

}  // namespace my_franka_controllers

#endif  // MY_FRANKA_CONTROLLERS__JOY_TELEOP_NODE_HPP_