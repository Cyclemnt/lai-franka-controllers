/// @file cartesian_velocity_controller.cpp
/// @brief Analytical Jacobian pseudo-inverse real-time execution loop implementation.

#include "lai_franka_controllers/cartesian_velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace lai_franka_controllers {

controller_interface::CallbackReturn CartesianVelocityController::on_init() {
    auto node = get_node();

    joint_names_ = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    node->declare_parameter("joint_names", joint_names_);
    node->declare_parameter("end_effector_frame", "fr3_link8");

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    auto node = get_node();

    joint_names_ = node->get_parameter("joint_names").as_string_array();
    end_effector_frame_ = node->get_parameter("end_effector_frame").as_string();
    std::string urdf_string = node->get_parameter("robot_description").as_string();

    // Parse geometry tree mappings using URDF stream
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    data_ = std::make_shared<pinocchio::Data>(model_);
    data_tmp_ = std::make_shared<pinocchio::Data>(model_);
    ee_frame_id_ = model_.getFrameId(end_effector_frame_);

    // Set structure limits maps sizes and properties
    dq_max_ << 2.62, 2.62, 2.62, 2.62, 5.26, 5.26, 5.26; 
    dq_cmd_.setZero();
    dq_cmd_prev_.setZero();
    
    // Core Operational Gains Configuration 
    K_gain_.setZero();
    K_gain_.diagonal() << 10.0, 10.0, 10.0, 5.0, 5.0, 5.0; 
    jacobian_.setZero();
    I6_.setIdentity();
    JJt_.setZero();
    JJt_inv_.setZero();
    llt_solver_.compute(Eigen::Matrix<double, 6, 6>::Identity());
    
    K_null_ = 0.8;
    I7_.setIdentity();
    grad_w_cached_.setZero();
    
    // Solve center bounds positions for joint configuration spaces
    const size_t num_joints = joint_names_.size();
    for (size_t i = 0; i < num_joints; ++i) {
        double upper = model_.upperPositionLimit[i];
        double lower = model_.lowerPositionLimit[i];
        if (upper > 1e3 || lower < -1e3) {
            q_mid_(i) = 0.0; 
        } else {
            q_mid_(i) = (upper + lower) / 2.0; 
        }
    }

    TargetPose init_target;
    init_target.valid = false;
    rt_target_pose_ptr_.initRT(init_target);

    // Context wrapper capturing reference stream snapshots
    target_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            TargetPose target;
            target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
            target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
            target.orientation.normalize();
            target.valid = true;
            rt_target_pose_ptr_.writeFromNonRT(target);
        });

    error_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("~/tracking_error", rclcpp::SystemDefaultsQoS());
    rt_error_pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(error_pub_);

    RCLCPP_INFO(node->get_logger(), "CartesianVelocityController configured successfully.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
}

controller_interface::InterfaceConfiguration CartesianVelocityController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint : joint_names_) {
        config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    }
    return config;
}

controller_interface::CallbackReturn CartesianVelocityController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)  {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q_current_(i) = state_interfaces_[i].get_value();
    }

    // Process forward model configuration positions on activation frame
    pinocchio::forwardKinematics(model_, *data_, q_current_);
    pinocchio::updateFramePlacements(model_, *data_);
    x_current_ = data_->oMf[ee_frame_id_].translation();
    quat_current_ = Eigen::Quaterniond(data_->oMf[ee_frame_id_].rotation());

    x_target_ = x_current_;
    quat_target_ = quat_current_;

    TargetPose reset;
    reset.valid = false;
    rt_target_pose_ptr_.writeFromNonRT(reset);
    last_target_time_ = get_node()->now();

    RCLCPP_INFO(get_node()->get_logger(), "Controller activated. Holding current task space pose.");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)  {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityController::update(const rclcpp::Time& time, const rclcpp::Duration& period)  {
    const double dt = period.seconds();

    // Fetch tracking commands from buffer
    TargetPose* target_ptr = rt_target_pose_ptr_.readFromRT();
    if (target_ptr && target_ptr->valid) {
        x_target_ = target_ptr->position;
        quat_target_ = target_ptr->orientation;
        last_target_time_ = time;
    }
    
    // Safety Watchdog Check: Decelerate immediately if channel times out past 100ms
    if ((time - last_target_time_).seconds() > 0.1) {
        for (size_t i = 0; i < 7; ++i) command_interfaces_[i].set_value(0.0);
        dq_cmd_prev_.setZero();
        return controller_interface::return_type::OK;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        q_current_(i) = state_interfaces_[i].get_value();
    }

    // Compute active frame geometric layouts structures via Pinocchio
    pinocchio::computeJointJacobians(model_, *data_, q_current_);
    pinocchio::updateFramePlacements(model_, *data_);
    pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_);

    x_current_ = data_->oMf[ee_frame_id_].translation();
    quat_current_ = Eigen::Quaterniond(data_->oMf[ee_frame_id_].rotation());
    quat_current_.normalize();

    // Solve Cartesian translation and rotation metrics
    pos_error_ = x_target_ - x_current_;

    quat_error_ = quat_target_ * quat_current_.inverse();
    if (quat_error_.w() < 0) {
        quat_error_.coeffs() *= -1.0; 
    }

    ori_error_ = 2.0 * quat_error_.vec(); 

    twist_error_ << pos_error_, ori_error_;
    const double error_norm = twist_error_.norm();

    // Solve adaptive Levenberg damping configuration tracking limits proxy
    JJt_.noalias() = jacobian_ * jacobian_.transpose();
    const double manipulability_proxy = JJt_.trace() / 6.0; 
    {
        constexpr double lambda_min = 1e-4;
        constexpr double lambda_max = 5e-2;
        constexpr double w_threshold = 1e-3; 
        lambda_damping_ = lambda_min;
        if (manipulability_proxy < w_threshold) {
            const double r = 1.0 - (manipulability_proxy / w_threshold);
            lambda_damping_ = lambda_min + (lambda_max - lambda_min) * r * r;
        }
    }

    // Build regularized Jacobian pseudo-inverse matrix inversion
    JJt_.diagonal().array() += lambda_damping_ * lambda_damping_;
    llt_solver_.compute(JJt_);
    JJt_inv_.noalias() = llt_solver_.solve(I6_);
    J_pinv_.noalias() = jacobian_.transpose() * JJt_inv_;
    
    // Smooth initial velocity ramps avoid startup jerks profiles
    activation_ramp_ = std::min(activation_ramp_ + dt / 0.5, 1.0); 
    dq_task_.noalias() = J_pinv_ * (K_gain_ * twist_error_);

    // Sub-scheduled low frequency evaluations track manipulability indices across nullspace gradients
    grad_cycle_counter_++;
    if (grad_cycle_counter_ >= GRAD_CYCLE_PERIOD) {
        grad_cycle_counter_ = 0;

        const double w_curr = std::sqrt((jacobian_ * jacobian_.transpose()).determinant() + 1e-12);
        constexpr double delta = 1e-4;
        
        for (int i = 0; i < 7; ++i) {
            q_perturbed_ = q_current_;
            q_perturbed_(i) += delta;

            pinocchio::computeJointJacobians(model_, *data_tmp_, q_perturbed_);
            pinocchio::updateFramePlacements(model_, *data_tmp_);
            J_tmp_.setZero();
            pinocchio::getFrameJacobian(model_, *data_tmp_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J_tmp_);

            Eigen::Matrix<double, 6, 6> JJt_tmp = J_tmp_ * J_tmp_.transpose();
            JJt_tmp.diagonal().array() += 1e-12;
            const double w_tmp = std::sqrt(JJt_tmp.determinant());
            grad_w_cached_(i) = (w_tmp - w_curr) / delta;
        }

        if (grad_w_cached_.norm() > 1e-8) {
            grad_w_cached_ /= grad_w_cached_.norm();
        }
    }

    // Joint boundary repulsion potential field tracking calculation
    for (int i = 0; i < 7; ++i) {
        constexpr double eps = 1e-3;
        const double dist_upper = std::max(model_.upperPositionLimit[i] - q_current_(i), eps);
        const double dist_lower = std::max(q_current_(i) - model_.lowerPositionLimit[i], eps);
        const double range  = model_.upperPositionLimit[i] - model_.lowerPositionLimit[i];
        dq_limit_(i) = (dist_upper < 0.3 * range || dist_lower < 0.3 * range) ? (1.0 / dist_lower - 1.0 / dist_upper) : 0.0;
    }

    // Blend out nullspace variations near target positions convergence zones to filter chatter oscillations
    const double null_blend = (error_norm > 0.01) ? 1.0 : (error_norm < 0.001) ? 0.0 : (error_norm - 0.001) / 0.009;

    dq_null_.noalias() = K_null_ * (w_manip_weight_ * grad_w_cached_ + w_limit_weight_ * dq_limit_);
    N_.noalias() = I7_ - J_pinv_ * jacobian_;

    // Combine tasks targets commands vectors elements
    dq_cmd_ = activation_ramp_ * dq_task_ + null_blend * N_ * dq_null_;

    // Global limits tracking scaling operations bounds safety protection checks
    {
        double scale = 1.0;
        for (int i = 0; i < 7; ++i) {
            const double ratio = std::abs(dq_cmd_(i)) / dq_max_(i);
            if (ratio > 1.0) scale = std::min(scale, 1.0 / ratio);
        }
        dq_cmd_ *= scale;
    }

    // Structural joint workspace hardware slowing buffer zone clamping limits safeties
    {
        constexpr double margin      = 0.262; 
        constexpr double hard_margin = 0.020; 
        for (int i = 0; i < 7; ++i) {
            const double dist_upper = model_.upperPositionLimit[i] - q_current_(i);
            const double dist_lower = q_current_(i) - model_.lowerPositionLimit[i];

            if (dq_cmd_(i) > 0.0 && dist_upper < margin) {
                dq_cmd_(i) *= (dist_upper < hard_margin) ? 0.0 : (dist_upper - hard_margin) / (margin - hard_margin);
            }
            if (dq_cmd_(i) < 0.0 && dist_lower < margin) {
                dq_cmd_(i) *= (dist_lower < hard_margin) ? 0.0 : (dist_lower - hard_margin) / (margin - hard_margin);
            }
        }
    }

    // Low pass filter profile pass dampens tracking transients
    const double alpha = (error_norm > 0.05) ? 0.4 : 0.15;
    dq_cmd_ = alpha * dq_cmd_ + (1.0 - alpha) * dq_cmd_prev_;
    dq_cmd_prev_ = dq_cmd_;

    // Stream filtered configuration commands down to core interface expansion buses links
    for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(dq_cmd_(i));
    }
    
    // Diagnostics data publishing outputs
    if (rt_error_pub_ && rt_error_pub_->trylock()) {
        rt_error_pub_->msg_.header.stamp    = time;
        rt_error_pub_->msg_.header.frame_id = end_effector_frame_;
        rt_error_pub_->msg_.twist.linear.x  = pos_error_.x();
        rt_error_pub_->msg_.twist.linear.y  = pos_error_.y();
        rt_error_pub_->msg_.twist.linear.z  = pos_error_.z();
        rt_error_pub_->msg_.twist.angular.x = ori_error_.x();
        rt_error_pub_->msg_.twist.angular.y = ori_error_.y();
        rt_error_pub_->msg_.twist.angular.z = ori_error_.z();
        rt_error_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

}  // namespace lai_franka_controllers

PLUGINLIB_EXPORT_CLASS(lai_franka_controllers::CartesianVelocityController, controller_interface::ControllerInterface)