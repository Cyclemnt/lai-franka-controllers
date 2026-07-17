/// @file JointTracking.cpp
/// @brief Implementation of the joint-space trajectory tracking task.

#include "task/jointTracking.hpp"

namespace task {

JointTracking::JointTracking(robot_kinematics::FrankaKinematics* robot_kinematics, char constraint_sense, double gain) {
    robotKinematics_ = robot_kinematics;
    constraintSense_ = constraint_sense;

    // 7-DOF Franka configuration: A is an Identity matrix mapping directly to joint velocities
    A_matrix_ = Eigen::MatrixXd::Identity(7, 7);
    b_vector_ = Eigen::VectorXd::Zero(7);
    
    // Proportional gain matrix
    K_gain_ = Eigen::MatrixXd::Identity(7, 7) * gain;
    
    // Initialize targets to zero
    q_desired_ = Eigen::VectorXd::Zero(7);
    dq_desired_ = Eigen::VectorXd::Zero(7);
}

void JointTracking::set_desired_state(const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d) {
    q_desired_ = q_d;
    dq_desired_ = dq_d;
}

void JointTracking::update() {
    // Reset A_matrix_ to identity in case dynamic DOF selections altered it previously
    A_matrix_ = Eigen::MatrixXd::Identity(7, 7);
    
    // Mask out any disabled joints in the solver stack (inherited from base Task class)
    applyDisabledDoFs(A_matrix_);

    // Fetch actual current joint positions from the kinematics engine
    Eigen::VectorXd q_current = robotKinematics_->get_q();

    // Compute the target objective: b = dq_d + K * (q_d - q)
    b_vector_.noalias() = dq_desired_ + K_gain_ * (q_desired_ - q_current);
}

}  // namespace task