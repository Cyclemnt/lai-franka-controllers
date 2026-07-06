#include "lai_franka_controllers/joint_pd_velocity_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace lai_franka_controllers {

// -------------------------------------------------------------------------
// on_init
// -------------------------------------------------------------------------
controller_interface::CallbackReturn JointPdVelocityController::on_init() {
    auto node = get_node();

    // Default to 7 joints for Franka
    joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    node->declare_parameter("joint_names", joint_names);

    // Default K gains (yaml possible)
    std::vector<double> default_k_gains(7, 0.9);
    node->declare_parameter("k_gains", default_k_gains);

    // Timeout and Smoothing parameters
    node->declare_parameter("timeout_sec", 0.1);
    node->declare_parameter("smoothing_iterations", 10);
    node->declare_parameter("max_allowed_dv", 0.001);

    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_configure
// -------------------------------------------------------------------------
controller_interface::CallbackReturn JointPdVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = get_node();

    // Read Parameters
    joint_names = node->get_parameter("joint_names").as_string_array();
    num_joints = joint_names.size();

    auto k_gains_std = node->get_parameter("k_gains").as_double_array();
    if (k_gains_std.size() != num_joints) {
        RCLCPP_ERROR(node->get_logger(), "Size of k_gains must match size of joint_names.");
        return controller_interface::CallbackReturn::ERROR;
    }
    
    k_gains = Eigen::VectorXd::Map(k_gains_std.data(), num_joints);
    timeout_sec = node->get_parameter("timeout_sec").as_double();
    // max_allowed_dv = node->get_parameter("max_allowed_dv").as_double();

    // Resize Math Vectors
    q_current.resize(num_joints);
    dq_cmd.resize(num_joints);
    // prev_dq_cmd.resize(num_joints);
    q_current.setZero();
    dq_cmd.setZero();
    // prev_dq_cmd.setZero();

    // Setup dq_cmd publisher
    dq_cmd_pub = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/output_dq_cmd", rclcpp::SystemDefaultsQoS());
    rt_dq_cmd_pub = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(dq_cmd_pub);
    rt_dq_cmd_pub->msg_.velocity.resize(7);
    rt_dq_cmd_pub->msg_.name = joint_names;

    // Setup Target Buffer Initialization
    TargetJointState initial_target;
    initial_target.q_d = Eigen::VectorXd::Zero(num_joints);
    initial_target.dq_d = Eigen::VectorXd::Zero(num_joints);
    initial_target.timestamp = node->get_clock()->now();
    initial_target.valid = false;
    rt_command_ptr.initRT(initial_target);

    // Setup Subscriber
    joint_command_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "~/joint_commands", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->position.size() != num_joints || msg->velocity.size() != num_joints) return;

            TargetJointState target;
            target.q_d = Eigen::VectorXd::Map(msg->position.data(), num_joints);
            target.dq_d = Eigen::VectorXd::Map(msg->velocity.data(), num_joints);
            target.timestamp = rclcpp::Time(msg->header.stamp);
            target.valid = true;

            rt_command_ptr.writeFromNonRT(target);
        });

    RCLCPP_INFO(node->get_logger(), "JointPdVelocityController configured successfully.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// command_interface_configuration
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration JointPdVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

// -------------------------------------------------------------------------
// state_interface_configuration
// -------------------------------------------------------------------------
controller_interface::InterfaceConfiguration JointPdVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

// -------------------------------------------------------------------------
// on_activate
// -------------------------------------------------------------------------
controller_interface::CallbackReturn JointPdVelocityController::on_activate(const rclcpp_lifecycle::State&) {
    // Read current actual position
    for (size_t i = 0; i < num_joints; ++i) { 
        q_current(i) = state_interfaces_[i].get_value(); 
    }

    // Initialize the buffer to the current state.
    TargetJointState init_state;
    init_state.q_d = q_current;
    init_state.dq_d = Eigen::VectorXd::Zero(num_joints);
    init_state.timestamp = get_node()->now();
    init_state.valid = true; // Actively hold position
    rt_command_ptr.writeFromNonRT(init_state);

    RCLCPP_INFO(get_node()->get_logger(), "Controller activated. Holding initial position.");
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// on_deactivate
// -------------------------------------------------------------------------
controller_interface::CallbackReturn JointPdVelocityController::on_deactivate(const rclcpp_lifecycle::State&) {
    // Zero out commands on shutdown
    for (size_t i = 0; i < num_joints; ++i) { 
        command_interfaces_[i].set_value(0.0); 
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------------------------
// update: 1000Hz loop
// -------------------------------------------------------------------------
controller_interface::return_type JointPdVelocityController::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
    
    // Fetch latest command via Zero-Order Hold (ZOH) buffer
    TargetJointState* target = rt_command_ptr.readFromRT();

    // Timeout Safety Check
    double time_since_last_cmd = (time - target->timestamp).seconds();
    
    if (!target->valid || time_since_last_cmd >= timeout_sec) {
        // TIMEOUT: Commanded velocity goes to zero
        dq_cmd.setZero();
    } else {
        // Read current joint states
        for (size_t i = 0; i < num_joints; ++i) { 
            q_current(i) = state_interfaces_[i].get_value(); 
        }

        // Compute Control Law: dq = K * (q_d - q) + dq_d
        dq_cmd = k_gains.cwiseProduct(target->q_d - q_current) + target->dq_d;
    }

    // // If difference between samples is too high, clamp
    // for (size_t i = 0; i < num_joints; ++i) {
    //     double delta = dq_cmd(i) - prev_dq_cmd(i);
    //     if (std::abs(delta) > max_allowed_dv) {
    //         if (delta > 0) {
    //             dq_cmd(i) = prev_dq_cmd(i) + 0.003;
    //         } else {
    //             dq_cmd(i) = prev_dq_cmd(i) - 0.003;
    //         }
    //     }
    // }
    // prev_dq_cmd = dq_cmd;

    // Write to Hardware Interfaces
    for (size_t i = 0; i < num_joints; ++i) {
        command_interfaces_[i].set_value(dq_cmd(i));
    }

    // Publish commands for debugging
    if (rt_dq_cmd_pub && rt_dq_cmd_pub->trylock()) {
        rt_dq_cmd_pub->msg_.header.stamp = get_node()->get_clock()->now();
        for (size_t i = 0; i < 7; ++i) rt_dq_cmd_pub->msg_.velocity[i] = dq_cmd(i);
        rt_dq_cmd_pub->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace lai_franka_controllers

PLUGINLIB_EXPORT_CLASS(lai_franka_controllers::JointPdVelocityController, controller_interface::ControllerInterface)