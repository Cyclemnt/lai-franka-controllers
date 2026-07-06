#include "lai_franka_controllers/joy_teleop_node.hpp"

namespace lai_franka_controllers {

JoyTeleopNode::JoyTeleopNode(const rclcpp::NodeOptions & options)
    : Node("joy_teleop_node", options)
{
    this->declare_parameter<std::string>("base_frame", "fr3_link0");
    this->declare_parameter<std::string>("ee_frame", "fr3_link8");
    this->declare_parameter<double>("publish_rate", 1000.0);
    this->declare_parameter<double>("v_max", 0.10);
    this->declare_parameter<double>("omega_max", 0.10);

    this->get_parameter("base_frame", base_frame);
    this->get_parameter("ee_frame", ee_frame);
    this->get_parameter("publish_rate", publish_rate);

    dt = 1.0 / publish_rate;

    current_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/hqp_reference_generator_node/current_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            latest_solver_pose = *msg;
            has_latest_pose = true;
        });

    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { this->joy_callback(msg); });

    pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/hqp_reference_generator_node/target_pose", 10);
    
    // Initialize Gripper Publisher
    gripper_pub = this->create_publisher<std_msgs::msg::Bool>("~/gripper_cmd", 10);

    auto period = std::chrono::duration<double>(dt);
    timer = this->create_wall_timer(period, std::bind(&JoyTeleopNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Teleop Engine Running. Waiting for HQP pose...");
}

bool JoyTeleopNode::get_current_pose(double &x, double &y, double &z, tf2::Quaternion &q) {
    if (!has_latest_pose) {
        // Return false until the HQP solver starts broadcasting
        return false; 
    }

    x = latest_solver_pose.pose.position.x;
    y = latest_solver_pose.pose.position.y;
    z = latest_solver_pose.pose.position.z;

    q.setValue(latest_solver_pose.pose.orientation.x, latest_solver_pose.pose.orientation.y, latest_solver_pose.pose.orientation.z, latest_solver_pose.pose.orientation.w);
               
    return true;
}

void JoyTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    cmd_x = 0.0; cmd_y = 0.0; cmd_z = 0.0;
    cmd_roll = 0.0; cmd_pitch = 0.0; cmd_yaw = 0.0;

    if (msg->axes.size() >= 6) {
        cmd_x = msg->axes[1]; 
        cmd_y = -msg->axes[0];
        
        cmd_roll = msg->axes[3]; 
        cmd_pitch = msg->axes[4];

        double lt_axis = msg->axes[2];
        double rt_axis = msg->axes[5];

        if (lt_axis < 0.9) {
            cmd_z = (1.0 - lt_axis) / 2.0;
        } else if (rt_axis < 0.9) {
            cmd_z = -(1.0 - rt_axis) / 2.0;
        }
    }

    if (msg->buttons.size() >= 6) {
        if (msg->buttons[4] == 1) cmd_yaw = 1.0;
        else if (msg->buttons[5] == 1) cmd_yaw = -1.0;
        
        // Gripper Toggle Logic
        bool button_a_pressed = (msg->buttons[0] == 1);
        if (button_a_pressed && !button_a_prev) {
            gripper_closed = !gripper_closed; // Toggle state
            
            std_msgs::msg::Bool gripper_msg;
            gripper_msg.data = gripper_closed;
            gripper_pub->publish(gripper_msg);
            
            RCLCPP_INFO(this->get_logger(), "Gripper Toggled: %s", gripper_closed ? "CLOSED" : "OPEN");
        }
        button_a_prev = button_a_pressed; // Store state for the next callback
    }
}

void JoyTeleopNode::timer_callback() {
    double current_x, current_y, current_z;
    tf2::Quaternion current_q;
    
    // Block integration until HQP broadcasts its state
    if (!get_current_pose(current_x, current_y, current_z, current_q)) {
        return; 
    }

    if (!is_initialized) {
        target_x = current_x; target_y = current_y; target_z = current_z;
        target_q = current_q;
        is_initialized = true;
        RCLCPP_INFO(this->get_logger(), "HQP Pose Locked. Teleop active.");
    }

    double v_max = this->get_parameter("v_max").as_double();         
    double omega_max = this->get_parameter("omega_max").as_double(); 
    
    const double ACCEL_LIMIT_TRANS = 0.1;  
    const double ACCEL_LIMIT_ROT   = 0.5;  

    auto update_axis_velocity = [this](double command, double &current_vel, double max_vel, double accel_lim) {
        if (std::abs(command) < 0.05) {
            current_vel = 0.0; 
            return;
        }
        double desired_vel = command * max_vel;
        double error = desired_vel - current_vel;
        double max_step = accel_lim * dt;
        current_vel += std::clamp(error, -max_step, max_step);
    };

    update_axis_velocity(cmd_x, current_vel_x, v_max, ACCEL_LIMIT_TRANS);
    update_axis_velocity(cmd_y, current_vel_y, v_max, ACCEL_LIMIT_TRANS);
    update_axis_velocity(cmd_z, current_vel_z, v_max, ACCEL_LIMIT_TRANS);

    update_axis_velocity(cmd_roll,  current_vel_roll,  omega_max, ACCEL_LIMIT_ROT);
    update_axis_velocity(cmd_pitch, current_vel_pitch, omega_max, ACCEL_LIMIT_ROT);
    update_axis_velocity(cmd_yaw,   current_vel_yaw,   omega_max, ACCEL_LIMIT_ROT);

    // Integrate Position Target
    target_x += current_vel_x * dt;
    target_y += current_vel_y * dt;
    target_z += current_vel_z * dt;

    // Integrate Orientation Target
    tf2::Vector3 omega(current_vel_roll, current_vel_pitch, current_vel_yaw);
    // Rotation in EE Frame:
    if (omega.length2() > 1e-8) {
        tf2::Quaternion dq;
        dq.setRotation(omega.normalized(), omega.length() * dt);
        target_q = target_q * dq;
        target_q.normalize();
    }

    // Clamp translation
    const double Max_Translation_Lead = 0.005; 
    target_x = std::clamp(target_x, current_x - Max_Translation_Lead, current_x + Max_Translation_Lead);
    target_y = std::clamp(target_y, current_y - Max_Translation_Lead, current_y + Max_Translation_Lead);
    target_z = std::clamp(target_z, current_z - Max_Translation_Lead, current_z + Max_Translation_Lead);

    // Clamp rotations
    const double Max_Rotation_Lead = 0.05; // Radians
    tf2::Quaternion q_error = current_q.inverse() * target_q;
    q_error.normalize();
    double angle = q_error.getAngleShortestPath();
    if (angle > Max_Rotation_Lead) {
        double t = Max_Rotation_Lead / angle;
        target_q = current_q.slerp(target_q, t);
        target_q.normalize();
    }

    // Publish Pose Target
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = base_frame;

    pose_msg.pose.position.x = target_x;
    pose_msg.pose.position.y = target_y;
    pose_msg.pose.position.z = target_z;

    pose_msg.pose.orientation.x = target_q.x();
    pose_msg.pose.orientation.y = target_q.y();
    pose_msg.pose.orientation.z = target_q.z();
    pose_msg.pose.orientation.w = target_q.w();

    pose_pub->publish(pose_msg);
}

}  // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lai_franka_controllers::JoyTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}