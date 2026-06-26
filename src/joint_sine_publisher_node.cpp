#include "lai_franka_controllers/joint_sine_publisher_node.hpp"
#include <cmath>

using namespace std::chrono_literals;

namespace lai_franka_controllers {

JointSinePublisherNode::JointSinePublisherNode() : Node("joint_sine_publisher") {
    
    // Declare and get the target topic name
    this->declare_parameter<std::string>("target_topic", "/joint_pd_velocity_controller/joint_commands");
    std::string target_topic = this->get_parameter("target_topic").as_string();

    publisher = this->create_publisher<sensor_msgs::msg::JointState>(target_topic, 10);
    
    // Subscribe to capture the initial position and track current position
    joint_sub = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JointSinePublisherNode::joint_state_callback, this, std::placeholders::_1));

    joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};

    initial_positions.resize(7, 0.0);
    current_positions.resize(7, 0.0);

    amplitudes  = {0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10}; // Radians
    frequencies = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05}; // Hz

    // 100 Hz Loop
    timer = this->create_wall_timer(10ms, std::bind(&JointSinePublisherNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states to capture initial hardware position...");
}

void JointSinePublisherNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    int matched_joints = 0;
    std::vector<double> temp_positions(7, 0.0);

    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < joint_names.size(); ++j) {
            if (msg->name[i] == joint_names[j]) {
                temp_positions[j] = msg->position[i];
                matched_joints++;
            }
        }
    }

    if (matched_joints == 7) {
        std::lock_guard<std::mutex> lock(data_mutex);
        
        // Always update current position for the error calculation
        current_positions = temp_positions;
        
        if (!is_initialized) {
            initial_positions = temp_positions;
            start_time = this->get_clock()->now();
            is_initialized = true;
            
            RCLCPP_INFO(this->get_logger(), "Initial positions captured. Waiting for PD controller subscription.");
        }
    }
}

void JointSinePublisherNode::timer_callback() {
    if (!is_initialized) return;

    auto msg = sensor_msgs::msg::JointState();
    rclcpp::Time current_time = this->get_clock()->now();

    msg.header.stamp = current_time;
    msg.name = joint_names;
    
    msg.position.resize(7);
    msg.velocity.resize(7);

    std::lock_guard<std::mutex> lock(data_mutex);
    
    // Wait for subscription
    // if (publisher->get_subscription_count() == 0) {
    //     for (size_t i = 0; i < 7; ++i) {
    //         msg.position[i] = initial_positions[i];
    //         msg.velocity[i] = 0.0;
    //     }
        
    //     // Reset start time continuously during the delay so that t=0 when the sine finally starts
    //     start_time = current_time;
    //     RCLCPP_INFO(this->get_logger(), "Waiting.");
    // } 
    // else {
        // Standard Sine trajectory computation
        double t = (current_time - start_time).seconds();
        for (size_t i = 0; i < 7; ++i) {
            double omega = 2.0 * M_PI * frequencies[i];
            
            // q = q0 + A * (1 - cos(wt))
            msg.position[i] = initial_positions[i] + amplitudes[i] * (1.0 - std::cos(omega * t));
            
            // dq = A * w * sin(wt)
            msg.velocity[i] = amplitudes[i] * omega * std::sin(omega * t);
        }

        // Compute and print tracking error
        double max_tracking_error = 0.0;
        for (size_t i = 0; i < 7; ++i) {
            double err = std::abs(current_positions[i] - msg.position[i]);
            if (err > max_tracking_error) {
                max_tracking_error = err;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Max tracking error: %.5f rad", max_tracking_error);
    // }

    publisher->publish(msg);
}

} // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::JointSinePublisherNode>());
    rclcpp::shutdown();
    return 0;
}