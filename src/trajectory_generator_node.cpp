#include "lai_franka_controllers/trajectory_generator_node.hpp"

using namespace std::chrono_literals;

namespace lai_franka_controllers {

TrajectoryGenerator::TrajectoryGenerator() : Node("trajectory_generator") {
    cmd_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/hqp_reference_generator_node/target_pose", 10);
    goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&TrajectoryGenerator::goal_callback, this, std::placeholders::_1));

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 100 Hz
    timer = this->create_wall_timer(1ms, std::bind(&TrajectoryGenerator::timer_callback, this)); // is 100Hz enough?
    
    RCLCPP_INFO(this->get_logger(), "Trajectory Generator Ready.");
}

void TrajectoryGenerator::prepare_segment(const Eigen::Vector3d& target_p, const Eigen::Quaterniond& target_q, double manual_duration) {
    try {
        auto transform = tf_buffer->lookupTransform("fr3_link0", "fr3_link8", tf2::TimePointZero);
        p_start << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z;
        q_start = Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
        
        p_goal = target_p;
        q_goal = target_q;
        q_start.normalize();
        q_goal.normalize();

        if (q_start.dot(q_goal) < 0.0) q_goal.coeffs() *= -1.0;

        if (manual_duration > 0) {
            duration = manual_duration;
        } else {
            double linear_distance = (p_goal - p_start).norm();
            double angular_distance = q_start.angularDistance(q_goal);
            double t_vel = 1.875 * std::max(linear_distance / max_linear_vel, angular_distance / max_angular_vel);
            double t_accel = std::sqrt(5.77 * std::max(linear_distance / max_linear_acc, angular_distance / max_angular_acc));
            duration = std::max({t_vel, t_accel, min_duration});
        }

        t_start = this->get_clock()->now();
        trajectory_active = true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "TF Error: %s", ex.what());
    }
    RCLCPP_INFO(this->get_logger(), "Starting new segment. Duration: %.2fs.", duration);
}

void TrajectoryGenerator::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Trigger sequence if Z is negative (impossible workspace)
    if (msg->pose.position.z < 0.0) {
        start_stress_test();
        return;
    }

    current_mode = Mode::MANUAL;
    Eigen::Vector3d target_p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond target_q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    RCLCPP_INFO(this->get_logger(), "Manual Goal Accepted.");
    prepare_segment(target_p, target_q);
}

void TrajectoryGenerator::start_stress_test() {
    RCLCPP_INFO(this->get_logger(), "Starting Stress Test Sequence.");
    waypoints.clear();
    Eigen::Quaterniond static_q(0.0, 1.0, 0.0, 0.0);

    waypoints.push_back({Eigen::Vector3d(-0.8,  0.0,  0.4), static_q});
    waypoints.push_back({Eigen::Vector3d( 0.0, -0.9,  0.4), static_q});
    waypoints.push_back({Eigen::Vector3d( 1.1,  0.0,  0.4), static_q});

    // waypoints.push_back({Eigen::Vector3d(-0.8,  0.0, 0.4), static_q});
    // waypoints.push_back({Eigen::Vector3d(-0.8, -0.8, 0.4), static_q});
    // waypoints.push_back({Eigen::Vector3d( 0.0, -0.8, 0.4), static_q});
    // waypoints.push_back({Eigen::Vector3d( 1.0, -0.8, 0.4), static_q});
    // waypoints.push_back({Eigen::Vector3d( 1.0,  0.0, 0.4), static_q});

    current_mode = Mode::STRESS_TEST;
    current_waypoint_idx = 0;
    prepare_segment(waypoints[0].pos, waypoints[0].ori, waypoints[0].duration);
}

void TrajectoryGenerator::timer_callback() {
    if (!trajectory_active) return;

    double t = (this->get_clock()->now() - t_start).seconds();
    
    geometry_msgs::msg::PoseStamped cmd_msg;
    cmd_msg.header.stamp = this->get_clock()->now();
    cmd_msg.header.frame_id = "fr3_link0";

    if (t >= duration) {
        if (current_mode == Mode::STRESS_TEST && ++current_waypoint_idx < waypoints.size()) {
            prepare_segment(waypoints[current_waypoint_idx].pos, waypoints[current_waypoint_idx].ori, waypoints[current_waypoint_idx].duration);
        } else {
            trajectory_active = false;
            current_mode = Mode::IDLE;
            RCLCPP_INFO(this->get_logger(), "Trajectory sequence finished.");
        }
    } else {
        double tau = t / duration;
        double s = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
        // double ds = (30.0 * std::pow(tau, 2) - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / duration;

        Eigen::Vector3d p_current = p_start + s * (p_goal - p_start);
        Eigen::Quaterniond q_current = q_start.slerp(s, q_goal);

        cmd_msg.pose.position.x = p_current.x();
        cmd_msg.pose.position.y = p_current.y();
        cmd_msg.pose.position.z = p_current.z();
        cmd_msg.pose.orientation.w = q_current.w();
        cmd_msg.pose.orientation.x = q_current.x();
        cmd_msg.pose.orientation.y = q_current.y();
        cmd_msg.pose.orientation.z = q_current.z();
        cmd_pub->publish(cmd_msg);
    }
}

} // namespace lai_franka_controllers

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::TrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}