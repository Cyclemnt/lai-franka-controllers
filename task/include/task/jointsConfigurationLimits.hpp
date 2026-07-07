/// @file jointsConfigurationLimits.hpp
/// @brief Joint space positional boundary constraint task.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__JOINTS_CONFIGURATION_LIMITS_HPP_
#define TASK__JOINTS_CONFIGURATION_LIMITS_HPP_

#include "task/task.hpp"

namespace task {

/// @class JointsConfigurationLimits
/// @brief Formulates configuration-space boundary metrics inside the prioritized optimization stack.
///
/// Implements $A = I_{7 \times 7}$ and projects proportional braking vectors into $b$ to restrict 
/// joint positions from exceeding lower or upper physical workspace thresholds.
class JointsConfigurationLimits : public Task {
private:
    Eigen::VectorXd q_bounds_;

public:
    /// @brief Parameterized constructor pre-allocating state matrices.
    JointsConfigurationLimits(FrankaKinematics* robotKinematics, const Eigen::VectorXd& q_bounds, char constraintSense, double gain);

    /// @brief Overrides update step to project current positional tracking errors.
    void update() override;
};

}  // namespace task

#endif // TASK__JOINTS_CONFIGURATION_LIMITS_HPP_