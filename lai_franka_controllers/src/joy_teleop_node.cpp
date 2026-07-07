/// @file joy_teleop_node.cpp
/// @brief Implementation details for smoothed, parameter-driven gamepad Cartesian teleoperation tracking.

#include "lai_franka_controllers/joy_teleop_node.hpp"

namespace lai_franka_controllers {

JoyTeleopNode::JoyTeleopNode(const rclcpp::NodeOptions & options)
    : Node("joy_teleop_node", options)
{
    // Declare parameter interfaces with safe defaults matching physical limits configurations
    this->declare_parameter<std::string>("base_frame", "fr3_link0");
    this->declare_parameter<std::string>("ee_frame", "fr3_link8");
    this->declare_parameter<double>("publish_rate", 1000.0);
    this->declare_parameter<double>("v_max", 0.20);
    this->declare_parameter<double>("omega_max", 0.20);
    this->declare_parameter<double>("accel_limit_trans", 0.1);
    this->declare_parameter<double>("accel_limit_rot", 0.5);
    this->declare_parameter<double>("joystick_deadzone", 0.05);
    this->declare_parameter<double>("max_translation_lead", 0.01);
    this->declare_parameter<double>("max_rotation_lead", 0.1);
    this->declare_parameter<double>("gripper_grasp_speed", 0.1);
    this->declare_parameter<double>("gripper_grasp_force", 40.0);
    this->declare_parameter<double>("gripper_open_width", 0.08);

    // Cache initial parameters to initialize core timing properties
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("ee_frame", ee_frame_);
    this->get_parameter("publish_rate", publish_rate_);

    dt_ = 1.0 / publish_rate_;

    // Synchronize to internal HQP virtual model to resolve frame mismatches and initialization jerk
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/hqp_reference_generator_node/current_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            latest_solver_pose_ = *msg;
            has_latest_pose_ = true;
        });

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { this->joy_callback(msg); });

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/hqp_reference_generator_node/target_pose", 10);
    
    // Asynchronous Action Client initialization for the hardware Franka Hand
    grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(this, "/franka_gripper/grasp");
    move_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(this, "/franka_gripper/move");

    auto period = std::chrono::duration<double>(dt_);
    timer_ = this->create_wall_timer(period, std::bind(&JoyTeleopNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Teleop Engine Running. Initial speed scaling: %d%%.", speed_percentage_);
}

bool JoyTeleopNode::get_current_pose(double &x, double &y, double &z, tf2::Quaternion &q) {
    if (!has_latest_pose_) {
        return false; 
    }

    x = latest_solver_pose_.pose.position.x;
    y = latest_solver_pose_.pose.position.y;
    z = latest_solver_pose_.pose.position.z;

    q.setValue(latest_solver_pose_.pose.orientation.x, 
               latest_solver_pose_.pose.orientation.y, 
               latest_solver_pose_.pose.orientation.z, 
               latest_solver_pose_.pose.orientation.w);
               
    return true;
}

void JoyTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Reset inputs on incoming frame to prevent drifting stuck commands
    cmd_x_ = 0.0;    cmd_y_ = 0.0;     cmd_z_ = 0.0;
    cmd_roll_ = 0.0; cmd_pitch_ = 0.0; cmd_yaw_ = 0.0;

    // Parse sticks and triggers
    if (msg->axes.size() >= 7) {
        cmd_x_ = msg->axes[1]; 
        cmd_y_ = msg->axes[0];
        
        cmd_roll_  = msg->axes[3]; 
        cmd_pitch_ = msg->axes[4];

        double lt_axis = msg->axes[2];
        double rt_axis = msg->axes[5];

        // Trigger mapping logic for Z axis translations
        if (lt_axis < 0.9) {
            cmd_z_ = (1.0 - lt_axis) / 2.0;
        } else if (rt_axis < 0.9) {
            cmd_z_ = -(1.0 - rt_axis) / 2.0;
        }

        // Live D-PAD Speed Control
        bool dpad_left  = (msg->axes[6] > 0.5);
        bool dpad_right = (msg->axes[6] < -0.5);

        // Incremental modifications (Floor constraint 5%)
        if (dpad_left && !dpad_left_prev_) {
            speed_percentage_ = std::max(5, speed_percentage_ - 5);
            RCLCPP_INFO(this->get_logger(), "Speed Limit Scaled Down: %d%%", speed_percentage_);
        }
        
        // Incremental modifications (Ceil constraint 100%)
        if (dpad_right && !dpad_right_prev_) {
            speed_percentage_ = std::min(100, speed_percentage_ + 5);
            RCLCPP_INFO(this->get_logger(), "Speed Limit Scaled Up: %d%%", speed_percentage_);
        }

        dpad_left_prev_  = dpad_left;
        dpad_right_prev_ = dpad_right;
    }

    // Parse button array assignments
    if (msg->buttons.size() >= 6) {
        if (msg->buttons[4] == 1)      cmd_yaw_ = 1.0;
        else if (msg->buttons[5] == 1) cmd_yaw_ = -1.0;
        
        // Non-blocking asynchronous falling edge detection toggle logic for gripper
        bool button_a_pressed = (msg->buttons[0] == 1);
        if (button_a_pressed && !button_a_prev_) {
            gripper_closed_ = !gripper_closed_;
            
            if (gripper_closed_) {
                if (!grasp_client_->action_server_is_ready()) {
                    RCLCPP_WARN(this->get_logger(), "Franka Grasp action server not available.");
                } else {
                    auto goal = franka_msgs::action::Grasp::Goal();
                    goal.width = 0.0;        
                    goal.speed = this->get_parameter("gripper_grasp_speed").as_double();        
                    goal.force = this->get_parameter("gripper_grasp_force").as_double();       
                    goal.epsilon.inner = 0.08; 
                    goal.epsilon.outer = 0.08;
                    grasp_client_->async_send_goal(goal);
                    RCLCPP_INFO(this->get_logger(), "Gripper Command Sent: GRASP");
                }
            } else {
                if (!move_client_->action_server_is_ready()) {
                    RCLCPP_WARN(this->get_logger(), "Franka Move action server not available.");
                } else {
                    auto goal = franka_msgs::action::Move::Goal();
                    goal.width = this->get_parameter("gripper_open_width").as_double();       
                    goal.speed = this->get_parameter("gripper_grasp_speed").as_double();        
                    move_client_->async_send_goal(goal);
                    RCLCPP_INFO(this->get_logger(), "Gripper Command Sent: OPEN");
                }
            }
        }
        button_a_prev_ = button_a_pressed; 
    }
}

void JoyTeleopNode::timer_callback() {
    double current_x, current_y, current_z;
    tf2::Quaternion current_q;
    
    if (!get_current_pose(current_x, current_y, current_z, current_q)) {
        return; 
    }

    // Lock starting configuration variables relative to the solver position
    if (!is_initialized_) {
        target_x_ = current_x; target_y_ = current_y; target_z_ = current_z;
        target_q_ = current_q;
        is_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "HQP Virtual Model Frame Synchronized. Teleoperation Active.");
    }

    // Retrieve runtime configurable limits from parameter servers
    double base_v_max = this->get_parameter("v_max").as_double();         
    double base_omega_max = this->get_parameter("omega_max").as_double(); 
    double accel_limit_trans = this->get_parameter("accel_limit_trans").as_double();
    double accel_limit_rot   = this->get_parameter("accel_limit_rot").as_double();
    double joystick_deadzone  = this->get_parameter("joystick_deadzone").as_double();
    double max_translation_lead = this->get_parameter("max_translation_lead").as_double();
    double max_rotation_lead    = this->get_parameter("max_rotation_lead").as_double();

    // Scale operational envelope velocities via active percentage limits
    double v_max = base_v_max * (speed_percentage_ / 100.0);
    double omega_max = base_omega_max * (speed_percentage_ / 100.0);

    // Local Lambda tracking function to shape smooth velocity ramps
    auto update_axis_velocity = [this](double command, double &current_vel, double max_vel, double accel_lim, double deadzone) {
        if (std::abs(command) < deadzone) {
            current_vel = 0.0; 
            return;
        }
        double desired_vel = command * max_vel;
        double error = desired_vel - current_vel;
        double max_step = accel_lim * dt_;
        current_vel += std::clamp(error, -max_step, max_step);
    };

    update_axis_velocity(cmd_x_, cmd_y_ = static_cast<double>(cmd_y_), v_max, accel_limit_trans, joystick_deadzone);
    update_axis_velocity(cmd_y_, current_vel_y_, v_max, accel_limit_trans, joystick_deadzone);
    update_axis_velocity(cmd_z_, current_vel_z_, v_max, accel_limit_trans, joystick_deadzone);

    update_axis_velocity(cmd_roll_,  current_vel_roll_,  omega_max, accel_limit_rot, joystick_deadzone);
    update_axis_velocity(cmd_pitch_, current_vel_pitch_, omega_max, accel_limit_rot, joystick_deadzone);
    update_axis_velocity(cmd_yaw_,   current_vel_yaw_,   omega_max, accel_limit_rot, joystick_deadzone);

    // Position Integration Steps
    target_x_ += current_vel_x_ * dt_;
    target_y_ += current_vel_y_ * dt_;
    target_z_ += current_vel_z_ * dt_;

    // Angular Velocity Integration Steps
    tf2::Vector3 omega(current_vel_roll_, current_vel_pitch_, current_vel_yaw_);
    if (omega.length2() > 1e-8) {
        tf2::Quaternion dq;
        dq.setRotation(omega.normalized(), omega.length() * dt_);
        target_q_ = target_q_ * dq;
        target_q_.normalize();
    }

    // Leash constraints profile prevents integrator windup bounding errors against hard virtual walls
    target_x_ = std::clamp(target_x_, current_x - max_translation_lead, current_x + max_translation_lead);
    target_y_ = std::clamp(target_y_, current_y - max_translation_lead, current_y + max_translation_lead);
    target_z_ = std::clamp(target_z_, current_z - max_translation_lead, current_z + max_translation_lead);

    tf2::Quaternion q_error = current_q.inverse() * target_q_;
    q_error.normalize();
    double angle = q_error.getAngleShortestPath();
    if (angle > max_rotation_lead) {
        double t = max_rotation_lead / angle;
        target_q_ = current_q.slerp(target_q_, t);
        target_q_.normalize();
    }

    // Assemble and stream geometry structures
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = base_frame_;

    pose_msg.pose.position.x = target_x_;
    pose_msg.pose.position.y = target_y_;
    pose_msg.pose.position.z = target_z_;

    pose_msg.pose.orientation.x = target_q_.x();
    pose_msg.pose.orientation.y = target_q_.y();
    pose_msg.pose.orientation.z = target_q_.z();
    pose_msg.pose.orientation.w = target_q_.w();

    pose_pub_->publish(pose_msg);
}

}  // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lai_franka_controllers::JoyTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}