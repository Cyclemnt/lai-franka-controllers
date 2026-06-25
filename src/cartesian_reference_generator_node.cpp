#include "lai_franka_controllers/cartesian_reference_generator_node.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace lai_franka_controllers {

CartesianReferenceGeneratorNode::CartesianReferenceGeneratorNode() : Node("cartesian_reference_generator_node") {
    
    this->declare_parameter("use_sim_time", rclcpp::ParameterValue(false));
    
    joint_names = {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
    this->declare_parameter("joint_names", joint_names);
    this->declare_parameter("end_effector_frame", "fr3_link8");
    this->declare_parameter("robot_description", "");
    this->declare_parameter("control_frequency", 100.0);

    joint_names = this->get_parameter("joint_names").as_string_array();
    end_effector_frame = this->get_parameter("end_effector_frame").as_string();
    std::string urdf_string = this->get_parameter("robot_description").as_string();
    control_frequency_ = this->get_parameter("control_frequency").as_double();

    if (urdf_string.empty()) {
        RCLCPP_ERROR(this->get_logger(), "robot_description is empty. Cannot build Pinocchio model.");
        throw std::runtime_error("Empty URDF");
    }

    pinocchio::urdf::buildModelFromXML(urdf_string, model);
    data = std::make_shared<pinocchio::Data>(model);
    data_tmp = std::make_shared<pinocchio::Data>(model);
    ee_frame_id = model.getFrameId(end_effector_frame);

    size_t num_joints = joint_names.size();
    dq_max << 2.62, 2.62, 2.62, 2.62, 5.26, 5.26, 5.26;
    dq_cmd.setZero();
    dq_cmd_prev.setZero();
    q_virtual.setZero();

    K_gain.setZero();
    K_gain.diagonal() << 10.0, 10.0, 10.0, 5.0, 5.0, 5.0;
    jacobian.setZero();
    I6.setIdentity();
    JJt.setZero();
    JJt_inv.setZero();
    llt_solver.compute(Eigen::Matrix<double, 6, 6>::Identity());
    
    K_null = 0.8;
    I7.setIdentity();
    grad_w_cached.setZero();

    for (size_t i = 0; i < num_joints; ++i) {
        double upper = model.upperPositionLimit[i];
        double lower = model.lowerPositionLimit[i];
        if (upper > 1e3 || lower < -1e3) q_mid(i) = 0.0;
        else q_mid(i) = (upper + lower) / 2.0;
    }

    joint_cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_pd_velocity_controller/joint_commands", 10);
    error_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/tracking_error", 10);

    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("~/target_pose", 10, std::bind(&CartesianReferenceGeneratorNode::target_pose_callback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&CartesianReferenceGeneratorNode::joint_state_callback, this, std::placeholders::_1));

    int timer_ms = std::max(1, static_cast<int>(1000.0 / control_frequency_));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms), std::bind(&CartesianReferenceGeneratorNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for initial /joint_states to align the CLIK virtual model...");
}

void CartesianReferenceGeneratorNode::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_target_.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    current_target_.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).normalized();
    current_target_.valid = true;
    last_target_time_ = this->get_clock()->now();
}

void CartesianReferenceGeneratorNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (is_initialized_) return;

    std::vector<double> initial_positions(7, 0.0);
    int matched_joints = 0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
        for (size_t j = 0; j < joint_names.size(); ++j) {
            if (msg->name[i] == joint_names[j]) {
                initial_positions[j] = msg->position[i];
                matched_joints++;
            }
        }
    }

    if (matched_joints == 7) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        q_virtual = Eigen::VectorXd::Map(initial_positions.data(), 7);
        
        pinocchio::forwardKinematics(model, *data, q_virtual);
        pinocchio::updateFramePlacements(model, *data);
        
        current_target_.position = data->oMf[ee_frame_id].translation();
        current_target_.orientation = Eigen::Quaterniond(data->oMf[ee_frame_id].rotation());
        current_target_.valid = true;

        last_time_ = this->get_clock()->now();
        last_target_time_ = last_time_;
        is_initialized_ = true;
        
        joint_state_sub_.reset();
        RCLCPP_INFO(this->get_logger(), "Virtual Model Aligned. CLIK Loop Active.");
    }
}

void CartesianReferenceGeneratorNode::timer_callback() {
    if (!is_initialized_) return;

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    Eigen::Vector3d local_target_pos;
    Eigen::Quaterniond local_target_ori;
    bool has_valid_target = false;
    double time_since_target = 0.0;
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        local_target_pos = current_target_.position;
        local_target_ori = current_target_.orientation;
        has_valid_target = current_target_.valid;
        time_since_target = (current_time - last_target_time_).seconds();
    }

    // Watchdog Timeout: Stop moving, but not stop tracking virtual joints
    if (!has_valid_target || time_since_target > 0.5) {
        dq_cmd.setZero();
        dq_cmd_prev.setZero();
    } else {
        // Integrate virtual joints ONLY if we are moving
        q_virtual += dq_cmd * dt;

        pinocchio::computeJointJacobians(model, *data, q_virtual);
        pinocchio::updateFramePlacements(model, *data);
        pinocchio::getFrameJacobian(model, *data, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

        x_current = data->oMf[ee_frame_id].translation();
        quat_current = Eigen::Quaterniond(data->oMf[ee_frame_id].rotation());
        quat_current.normalize();

        pos_error = local_target_pos - x_current;
        quat_error = local_target_ori * quat_current.inverse();
        if (quat_error.w() < 0) quat_error.coeffs() *= -1.0;
        ori_error = 2.0 * quat_error.vec();

        twist_error << pos_error, ori_error;
        const double error_norm = twist_error.norm();

        JJt.noalias() = jacobian * jacobian.transpose();
        const double manipulability_proxy = JJt.trace() / 6.0;
        
        {
            constexpr double lambda_min = 1e-4;
            constexpr double lambda_max = 5e-2;
            constexpr double w_threshold = 1e-3;
            lambda_damping = lambda_min;
            if (manipulability_proxy < w_threshold) {
                const double r = 1.0 - (manipulability_proxy / w_threshold);
                lambda_damping = lambda_min + (lambda_max - lambda_min) * r * r;
            }
        }

        JJt.diagonal().array() += lambda_damping * lambda_damping;
        llt_solver.compute(JJt);
        JJt_inv.noalias() = llt_solver.solve(I6);
        J_pinv.noalias() = jacobian.transpose() * JJt_inv;
        
        activation_ramp = std::min(activation_ramp + dt / 0.5, 1.0);
        dq_task.noalias() = J_pinv * (K_gain * twist_error);

        grad_cycle_counter++;
        if (grad_cycle_counter >= grad_cycle_period) {
            grad_cycle_counter = 0;
            const double w_curr = std::sqrt((jacobian * jacobian.transpose()).determinant() + 1e-12);
            constexpr double delta = 1e-4;
            for (int i = 0; i < 7; ++i) {
                q_perturbed = q_virtual;
                q_perturbed(i) += delta;
                pinocchio::computeJointJacobians(model, *data_tmp, q_perturbed);
                pinocchio::updateFramePlacements(model, *data_tmp);
                J_tmp.setZero();
                pinocchio::getFrameJacobian(model, *data_tmp, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_tmp);
                Eigen::Matrix<double, 6, 6> JJt_tmp = J_tmp * J_tmp.transpose();
                JJt_tmp.diagonal().array() += 1e-12;
                const double w_tmp = std::sqrt(JJt_tmp.determinant());
                grad_w_cached(i) = (w_tmp - w_curr) / delta;
            }
            if (grad_w_cached.norm() > 1e-8) grad_w_cached /= grad_w_cached.norm();
        }

        for (int i = 0; i < 7; ++i) {
            constexpr double eps = 1e-3;
            const double dist_upper = std::max(model.upperPositionLimit[i] - q_virtual(i), eps);
            const double dist_lower = std::max(q_virtual(i) - model.lowerPositionLimit[i], eps);
            const double range = model.upperPositionLimit[i] - model.lowerPositionLimit[i];
            dq_limit(i) = (dist_upper < 0.3 * range || dist_lower < 0.3 * range) ? (1.0 / dist_lower - 1.0 / dist_upper) : 0.0;
        }

        const double null_blend = (error_norm > 0.01) ? 1.0 : (error_norm < 0.001) ? 0.0 : (error_norm - 0.001) / 0.009;
        dq_null.noalias() = K_null * (w_manip_weight * grad_w_cached + w_limit_weight * dq_limit);
        N.noalias() = I7 - J_pinv * jacobian;

        dq_cmd = activation_ramp * dq_task + null_blend * N * dq_null;

        {
            double scale = 1.0;
            for (int i = 0; i < 7; ++i) {
                const double ratio = std::abs(dq_cmd(i)) / dq_max(i);
                if (ratio > 1.0) scale = std::min(scale, 1.0 / ratio);
            }
            dq_cmd *= scale;
        }

        {
            constexpr double margin = 0.262;
            constexpr double hard_margin = 0.020;
            for (int i = 0; i < 7; ++i) {
                const double dist_upper = model.upperPositionLimit[i] - q_virtual(i);
                const double dist_lower = q_virtual(i) - model.lowerPositionLimit[i];

                if (dq_cmd(i) > 0.0 && dist_upper < margin) {
                    dq_cmd(i) *= (dist_upper < hard_margin) ? 0.0 : (dist_upper - hard_margin) / (margin - hard_margin);
                }
                if (dq_cmd(i) < 0.0 && dist_lower < margin) {
                    dq_cmd(i) *= (dist_lower < hard_margin) ? 0.0 : (dist_lower - hard_margin) / (margin - hard_margin);
                }
            }
        }

        const double alpha = (error_norm > 0.05) ? 0.4 : 0.15;
        dq_cmd = alpha * dq_cmd + (1.0 - alpha) * dq_cmd_prev;
        dq_cmd_prev = dq_cmd;
        
        // Publish Tracking Error
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        twist_msg.header.stamp = current_time;
        twist_msg.header.frame_id = end_effector_frame;
        twist_msg.twist.linear.x = pos_error.x();
        twist_msg.twist.linear.y = pos_error.y();
        twist_msg.twist.linear.z = pos_error.z();
        twist_msg.twist.angular.x = ori_error.x();
        twist_msg.twist.angular.y = ori_error.y();
        twist_msg.twist.angular.z = ori_error.z();
        error_pub_->publish(twist_msg);
    }

    // Publish Virtual State to PD Controller
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = current_time;
    joint_msg.name = joint_names;
    joint_msg.position.resize(7);
    joint_msg.velocity.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        joint_msg.position[i] = q_virtual(i);
        joint_msg.velocity[i] = dq_cmd(i);
    }
    joint_cmd_pub_->publish(joint_msg);
}

} // namespace lai_franka_controllers

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lai_franka_controllers::CartesianReferenceGeneratorNode>());
    rclcpp::shutdown();
    return 0;
}