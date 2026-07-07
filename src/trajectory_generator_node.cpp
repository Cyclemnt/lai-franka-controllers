/// @file trajectory_generator_node.cpp
/// @brief Quintic polynomial trajectory tracking loop implementation.

#include "lai_franka_controllers/trajectory_generator_node.hpp"

using namespace std::chrono_literals;

namespace lai_franka_controllers {

TrajectoryGenerator::TrajectoryGenerator() : Node("trajectory_generator") {
    // Set communication interface mapping
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/hqp_reference_generator_node/target_pose", 10); // TODO: the topic should be a parameter, to use this node with different controllers
        
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&TrajectoryGenerator::goal_callback, this, std::placeholders::_1));

    // Internal virtual model synchronization hookup resolving network lag jerks
    current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/hqp_reference_generator_node/current_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            latest_solver_pose_ = *msg;
            has_latest_pose_ = true;
        });

    // 1000 Hz loop scheduling step calculation
    timer_ = this->create_wall_timer(1ms, std::bind(&TrajectoryGenerator::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Trajectory Generator Node Successfully Active.");
}

void TrajectoryGenerator::prepare_segment(const Eigen::Vector3d& target_p, const Eigen::Quaterniond& target_q, double manual_duration) {
    if (!has_latest_pose_) {
        RCLCPP_ERROR(this->get_logger(), "Cannot calculate tracking profile: No feedback frame from HQP yet.");
        return;
    }

    // Capture boundary conditions precisely from the solver framework
    p_start_ << latest_solver_pose_.pose.position.x, 
                latest_solver_pose_.pose.position.y, 
                latest_solver_pose_.pose.position.z;
               
    q_start_ = Eigen::Quaterniond(latest_solver_pose_.pose.orientation.w, 
                                  latest_solver_pose_.pose.orientation.x, 
                                  latest_solver_pose_.pose.orientation.y, 
                                  latest_solver_pose_.pose.orientation.z);
    
    p_goal_ = target_p;
    q_goal_ = target_q;
    q_start_.normalize();
    q_goal_.normalize();

    // Prevent double wrapping flipping errors over quaternion arcs
    if (q_start_.dot(q_goal_) < 0.0) {
        q_goal_.coeffs() *= -1.0;
    }

    if (manual_duration > 0) {
        duration_ = manual_duration;
    } else {
        // Solve analytically using peak kinematic metrics limits constraints
        double linear_distance = (p_goal_ - p_start_).norm();
        double angular_distance = q_start_.angularDistance(q_goal_);
        
        double t_vel = 1.875 * std::max(linear_distance / max_linear_vel_, angular_distance / max_angular_vel_);
        double t_accel = std::sqrt(5.77 * std::max(linear_distance / max_linear_acc_, angular_distance / max_angular_acc_));
        
        duration_ = std::max({t_vel, t_accel, min_duration_});
    }

    t_start_ = this->get_clock()->now();
    trajectory_active_ = true;
    
    RCLCPP_INFO(this->get_logger(), "New Segment Executed. Total Profile Time Calculation: %.2fs.", duration_);
}

void TrajectoryGenerator::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Negative Z bounds filter captures requests designed to test loop stress profiles
    if (msg->pose.position.z < 0.0) {
        start_stress_test();
        return;
    }

    current_mode_ = Mode::MANUAL;
    Eigen::Vector3d target_p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Quaterniond target_q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
    RCLCPP_INFO(this->get_logger(), "External Goal Track Profile Received.");
    prepare_segment(target_p, target_q);
}

void TrajectoryGenerator::start_stress_test() {
    RCLCPP_INFO(this->get_logger(), "Executing Automated Verification Sequence.");
    waypoints_.clear();
    
    // Default safe target orientation (flange pointing vertically downward)
    Eigen::Quaterniond static_q(0.0, 1.0, 0.0, 0.0);

    // Populate automated spatial check routes positions
    waypoints_.push_back({Eigen::Vector3d(0.4,  0.1, 0.4), static_q, 2.0});
    waypoints_.push_back({Eigen::Vector3d(0.4, -0.1, 0.4), static_q, 2.0});
    waypoints_.push_back({Eigen::Vector3d(0.4,  0.0, 0.5), static_q, 2.0});

    current_mode_ = Mode::STRESS_TEST;
    current_waypoint_idx_ = 0;
    prepare_segment(waypoints_[0].pos, waypoints_[0].ori, waypoints_[0].duration);
}

void TrajectoryGenerator::timer_callback() {
    if (!trajectory_active_) return;

    double t = (this->get_clock()->now() - t_start_).seconds();
    
    geometry_msgs::msg::PoseStamped cmd_msg;
    cmd_msg.header.stamp = this->get_clock()->now();
    cmd_msg.header.frame_id = "fr3_link0";

    if (t >= duration_) {
        // Transition tracking evaluations to alternative legs if evaluating arrays
        if (current_mode_ == Mode::STRESS_TEST && ++current_waypoint_idx_ < waypoints_.size()) {
            prepare_segment(waypoints_[current_waypoint_idx_].pos, 
                            waypoints_[current_waypoint_idx_].ori, 
                            waypoints_[current_waypoint_idx_].duration);
        } else {
            trajectory_active_ = false;
            current_mode_ = Mode::IDLE;
            RCLCPP_INFO(this->get_logger(), "Trajectory Path Execution Tracking Completed.");
        }
    } else {
        // Solve structural tracking variables via 5th-order scaling polynomial calculation
        double tau = t / duration_;
        double s = 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) + 6.0 * std::pow(tau, 5);
        // double ds = (30.0 * std::pow(tau, 2) - 60.0 * std::pow(tau, 3) + 30.0 * std::pow(tau, 4)) / duration;

        Eigen::Vector3d p_current = p_start_ + s * (p_goal_ - p_start_);
        Eigen::Quaterniond q_current = q_start_.slerp(s, q_goal_);

        cmd_msg.pose.position.x = p_current.x();
        cmd_msg.pose.position.y = p_current.y();
        cmd_msg.pose.position.z = p_current.z();
        
        cmd_msg.pose.orientation.w = q_current.w();
        cmd_msg.pose.orientation.x = q_current.x();
        cmd_msg.pose.orientation.y = q_current.y();
        cmd_msg.pose.orientation.z = q_current.z();
        
        cmd_pub_->publish(cmd_msg);
    }
}

} // namespace lai_franka_controllers

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::TrajectoryGenerator>());
    rclcpp::shutdown();
    return 0;
}