#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class KinematicIntegrator : public rclcpp::Node {
public:
    KinematicIntegrator() : Node("kinematic_integrator") {
        // MATCHING THE HQP OUTPUT: Changed from Float64MultiArray to JointState
        velocity_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/my_hqp_cartesian_controller/dq_cmd", 10, 
            std::bind(&KinematicIntegrator::velocity_callback, this, std::placeholders::_1));

        // Standard ROS 2 topic for RViz
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        joint_names_ = {
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7"
        };
        
        // Franka Home Position
        joint_positions_ = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
        joint_velocities_.assign(7, 0.0);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&KinematicIntegrator::update_and_publish, this));

        last_update_time_ = this->get_clock()->now();
    }

private:
    void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Extract velocity from the JointState message instead of data array
        if (msg->velocity.size() >= 7) {
            for (size_t i = 0; i < 7; ++i) {
                joint_velocities_[i] = msg->velocity[i];
            }
        }
    }

    void update_and_publish() {
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_update_time_).seconds();
        last_update_time_ = now;

        // Simple Euler integration
        for (size_t i = 0; i < 7; ++i) {
            joint_positions_[i] += joint_velocities_[i] * dt;
        }

        auto state_msg = sensor_msgs::msg::JointState();
        state_msg.header.stamp = now;
        state_msg.name = joint_names_;
        state_msg.position = joint_positions_;
        state_msg.velocity = joint_velocities_;

        joint_state_pub_->publish(state_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velocity_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    rclcpp::Time last_update_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicIntegrator>());
    rclcpp::shutdown();
    return 0;
}