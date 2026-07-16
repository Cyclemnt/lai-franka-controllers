/// @file pose.cpp
/// @brief Cartesian tracking and DLS singularity dampening mechanics.

#include "task/pose.hpp"
#include <cmath>

namespace task {

Pose::Pose(FrankaKinematics* robot_kinematics, char constraint_sense, const Eigen::VectorXd& DLS_weights, double gain) {
    robotKinematics_ = robot_kinematics;
    selectedTask_ = robotKinematics_->getSelectTask();
    constraintSense_ = constraint_sense;
    DLS_weights_ = DLS_weights;
    gain_ = gain;

    initial_task_value_.resize(3);
    initial_task_value_[0] = Eigen::VectorXd::Zero(7); 
    initial_task_value_[1] = Eigen::VectorXd::Zero(6); 
    initial_task_value_[2] = Eigen::VectorXd::Zero(6); 
    desired_task_value_ = initial_task_value_;

    int rows = 6;
    if (selectedTask_->at(0) && !selectedTask_->at(1)) rows = 3;
    else if (!selectedTask_->at(0) && selectedTask_->at(1)) rows = 3;

    K_gain_ = Eigen::MatrixXd::Identity(rows, rows) * gain_;
    A_matrix_ = Eigen::MatrixXd::Zero(rows, 7);
    b_vector_ = Eigen::VectorXd::Zero(rows);
    
    current_error_ = Eigen::VectorXd::Zero(rows);
    feed_forward_ = Eigen::VectorXd::Zero(rows);
    franka_jacobian_ = Eigen::MatrixXd::Zero(6, 7);
}

void Pose::update() {
    franka_jacobian_ = robotKinematics_->getJacobian();
    
    bool use_pos = selectedTask_->at(0);
    bool use_ori = selectedTask_->at(1);

    if (use_pos && !use_ori) {
        A_matrix_ = franka_jacobian_.topRows(3);
    } else if (!use_pos && use_ori) {
        A_matrix_ = franka_jacobian_.bottomRows(3);
    } else {
        A_matrix_ = franka_jacobian_;
    }

    applyDisabledDoFs(A_matrix_);

    current_error_ = robotKinematics_->getError();
    feed_forward_ = desired_task_value_[1];

    // Combine Closed-Loop Error Correction and Feed-Forward rates: b = x_dot_d + K * e
    b_vector_.noalias() = feed_forward_ + (K_gain_ * current_error_);
}

void Pose::set_initial_task_value(int derivation_order, const Eigen::VectorXd& value) { initial_task_value_[derivation_order] = value; }
void Pose::set_desired_task_value(int derivation_order, const Eigen::VectorXd& value) { desired_task_value_[derivation_order] = value; }

Eigen::MatrixXd Pose::get_DLS_quadratic_objective_matrix(const Eigen::MatrixXd& J_in, double lambda, double threshold, const Eigen::MatrixXd& DLS_weights_in) {
    Eigen::MatrixXd W = Eigen::MatrixXd(J_in.cols(), J_in.cols());
    
    // Evaluate singular structures using standard SVD mapping routines
    Eigen::JacobiSVD<Eigen::MatrixXd> SVD(J_in.transpose() * J_in, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = SVD.matrixV();
    
    // Clear numeric chatter under 1e-4 thresholds
    V = (V.array().abs() < 1e-4).select(0.0, V); 

    Eigen::VectorXd singular_values = SVD.singularValues();
    Eigen::VectorXd eigen_values = Eigen::VectorXd::Zero(singular_values.size());
    Eigen::MatrixXd delta_S = Eigen::MatrixXd::Zero(J_in.cols(), J_in.cols());

    for (int i = 0; i < eigen_values.size(); i++) {
        if (std::abs(singular_values(i)) < 1e-4) {
            singular_values(i) = 0.0;
        }

        if (singular_values(i) < lambda) {
            delta_S(i, i) = 0.0;
        } else if (singular_values(i) >= threshold) {
            delta_S(i, i) = 1.0;
        } else {
            singular_values(i) = (singular_values(i) - lambda) / (threshold - lambda);
        }
        
        // Smooth scaling step interpolates boundary profiles smoothly
        delta_S(i, i) = singular_values(i) * singular_values(i) * (3.0 - 2.0 * singular_values(i));
    }

    Eigen::MatrixXd Q = V * delta_S * V.transpose() + 1e-3 * Eigen::MatrixXd::Identity(V.rows(), V.rows());
    W = DLS_weights_in.asDiagonal();
    Q = W * Q * W;

    return Q;
}

}  // namespace task
