/// @file jointsConfigurationLimits.cpp
/// @brief Proportional position bounding execution logic.

#include "task/jointsConfigurationLimits.hpp"

namespace task {

JointsConfigurationLimits::JointsConfigurationLimits(FrankaKinematics* robotKinematics, const Eigen::VectorXd& q_bounds, char constraintSense, double gain) {
    robotKinematics_ = robotKinematics;
    q_bounds_ = q_bounds;
    constraintSense_ = constraintSense;
    gain_ = gain;

    // Pre-allocate identity mapping for the joint configuration dimensions
    A_matrix_ = Eigen::MatrixXd::Identity(7, 7);
    applyDisabledDoFs(A_matrix_);
    
    b_vector_ = Eigen::VectorXd::Zero(7);
}

void JointsConfigurationLimits::update() {
    const Eigen::VectorXd q = robotKinematics_->get_q();
    b_vector_.noalias() = gain_ * (q_bounds_ - q);
}

}  // namespace task