/// @file jointsVelocityLimits.hpp
/// @brief Joint space maximum velocity constraint task.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__JOINTS_VELOCITY_LIMITS_HPP_
#define TASK__JOINTS_VELOCITY_LIMITS_HPP_

#include "task/task.hpp"

namespace task {

/// @class JointsVelocityLimits
/// @brief Enforces hardware velocity saturation constraints over the optimization problem.
///
/// Ensures joint command outputs stay strictly inside the physical capabilities of the FR3 actuator rings.
class JointsVelocityLimits : public Task {
private:
    Eigen::VectorXd dq_bounds_;

public:
    /// @brief Parameterized constructor framing the constraint problem.
    JointsVelocityLimits(FrankaKinematics* robotKinematics, const Eigen::VectorXd& dq_bounds, char constraintSense, double gain);

    /// @brief Static assignment wrapper implementation.
    void update() override;
};

}  // namespace task

#endif // TASK__JOINTS_VELOCITY_LIMITS_HPP_