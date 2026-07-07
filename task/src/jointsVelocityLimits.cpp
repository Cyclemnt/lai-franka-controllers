/// @file jointsVelocityLimits.cpp
/// @brief Actuator ceiling velocity threshold bounding logic.

#include "task/jointsVelocityLimits.hpp"

namespace task {

JointsVelocityLimits::JointsVelocityLimits(FrankaKinematics* robotKinematics, const Eigen::VectorXd& dq_bounds, char constraintSense, double gain) {
    robotKinematics_ = robotKinematics;
    dq_bounds_ = dq_bounds;
    constraintSense_ = constraintSense;
    gain_ = gain;

    A_matrix_ = Eigen::MatrixXd::Identity(7, 7);
    applyDisabledDoFs(A_matrix_);
    
    b_vector_ = dq_bounds_;
}

void JointsVelocityLimits::update() {
    // Static bounds map natively directly to the pre-allocated vector. 
    // No runtime operations needed inside the real-time loop thread.
}

}  // namespace task