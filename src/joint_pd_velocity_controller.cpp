/// @file joint_pd_velocity_controller.cpp
/// @brief Low-level real-time control law tracking loop implementation details.

#include "lai_franka_controllers/joint_pd_velocity_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace lai_franka_controllers {

controller_interface::CallbackReturn JointPdVelocityController::on_init() {
    auto node = get_node();

    // Default 7-DOF configurations matching Franka specification bounds
    joint_names_ = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    node->declare_parameter("joint_names", joint_names_);

    // Baseline structural gains matching tracking requirements
    std::vector<double> default_k_gains(7, 0.9);
    node->declare_parameter("k_gains", default_k_gains);
    node->declare_parameter("timeout_sec", 0.1);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPdVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = get_node();

    joint_names_ = node->get_parameter("joint_names").as_string_array();
    num_joints_ = joint_names_.size();

    auto k_gains_std = node->get_parameter("k_gains").as_double_array();
    if (k_gains_std.size() != num_joints_) {
        RCLCPP_ERROR(node->get_logger(), "Proportional feedback array tracking dimension mismatch.");
        return controller_interface::CallbackReturn::ERROR;
    }
    
    // Explicit dynamic allocations are fully mapped during configuration step to protect real-time constraints
    k_gains_ = Eigen::VectorXd::Map(k_gains_std.data(), num_joints_);
    timeout_sec_ = node->get_parameter("timeout_sec").as_double();

    q_current_.resize(num_joints_);
    dq_cmd_.resize(num_joints_);
    q_current_.setZero();
    dq_cmd_.setZero();

    // Setup non-blocking diagnostics logging pipes
    dq_cmd_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/output_dq_cmd", rclcpp::SystemDefaultsQoS());
    rt_dq_cmd_pub_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(dq_cmd_pub_);
    rt_dq_cmd_pub_->msg_.velocity.resize(num_joints_);
    rt_dq_cmd_pub_->msg_.name = joint_names_;

    // Set real-time initial target state variables
    TargetJointState initial_target;
    initial_target.q_d = Eigen::VectorXd::Zero(num_joints_);
    initial_target.dq_d = Eigen::VectorXd::Zero(num_joints_);
    initial_target.timestamp = node->get_clock()->now();
    initial_target.valid = false;
    rt_command_ptr_.initRT(initial_target);

    // Thread-safe lock-free reference intake subscriber pipeline
    joint_command_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "~/joint_commands", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->position.size() != num_joints_ || msg->velocity.size() != num_joints_) return;

            TargetJointState target;
            target.q_d = Eigen::VectorXd::Map(msg->position.data(), num_joints_);
            target.dq_d = Eigen::VectorXd::Map(msg->velocity.data(), num_joints_);
            target.timestamp = rclcpp::Time(msg->header.stamp);
            target.valid = true;

            rt_command_ptr_.writeFromNonRT(target);
        });

    RCLCPP_INFO(node->get_logger(), "JointPdVelocityController configured successfully.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointPdVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

controller_interface::InterfaceConfiguration JointPdVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

controller_interface::CallbackReturn JointPdVelocityController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Collect direct actual encoder measurements on frame start
    for (size_t i = 0; i < num_joints_; ++i) { 
        q_current_(i) = state_interfaces_[i].get_value(); 
    }

    // Force zero-velocity tracking state from current positions to avoid baseline jumps
    TargetJointState init_state;
    init_state.q_d = q_current_;
    init_state.dq_d = Eigen::VectorXd::Zero(num_joints_);
    init_state.timestamp = get_node()->now();
    init_state.valid = true; 
    rt_command_ptr_.writeFromNonRT(init_state);

    RCLCPP_INFO(get_node()->get_logger(), "Controller activated. Holding initial hardware positions.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointPdVelocityController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Assert hard zero commands tracking safety check during context adjustments
    for (size_t i = 0; i < num_joints_; ++i) { 
        command_interfaces_[i].set_value(0.0); 
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPdVelocityController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    // Atomic lock-free single-cycle frame pointer acquisition
    TargetJointState* target = rt_command_ptr_.readFromRT();

    // Verify channel responsiveness
    double time_since_last_cmd = (time - target->timestamp).seconds();
    
    if (!target->valid || time_since_last_cmd >= timeout_sec_) {
        // 0 velocity if message drops occur
        dq_cmd_.setZero();
    } else {
        for (size_t i = 0; i < num_joints_; ++i) { 
            q_current_(i) = state_interfaces_[i].get_value(); 
        }

        // Apply feedforward PD control calculation
        dq_cmd_ = k_gains_.cwiseProduct(target->q_d - q_current_) + target->dq_d;
    }

    // Assign calculated references to internal command buffers
    for (size_t i = 0; i < num_joints_; ++i) {
        command_interfaces_[i].set_value(dq_cmd_(i));
    }

    // Try-lock check avoids blocking internal threads during asynchronous trace operations
    if (rt_dq_cmd_pub_ && rt_dq_cmd_pub_->trylock()) {
        rt_dq_cmd_pub_->msg_.header.stamp = get_node()->get_clock()->now();
        for (size_t i = 0; i < num_joints_; ++i) {
            rt_dq_cmd_pub_->msg_.velocity[i] = dq_cmd_(i);
        }
        rt_dq_cmd_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace lai_franka_controllers

PLUGINLIB_EXPORT_CLASS(lai_franka_controllers::JointPdVelocityController, controller_interface::ControllerInterface)