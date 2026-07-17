/// @file JointTracking.hpp
/// @brief Joint-space tracking task for combined position and velocity references.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__JOINT_TRACKING_TASK_HPP_
#define TASK__JOINT_TRACKING_TASK_HPP_

#include "task/task.hpp"
#include "robot_kinematics/FrankaKinematics.hpp"
#include <Eigen/Dense>

namespace task {

/// @class JointTracking
/// @brief Tracks desired joint positions with feed-forward velocities.
class JointTracking : public Task {
private:
    robot_kinematics::FrankaKinematics* robotKinematics_;
    Eigen::MatrixXd K_gain_;
    
    Eigen::VectorXd q_desired_;
    Eigen::VectorXd dq_desired_;

public:
    /// @brief Initializes a 7-DOF identity mapping for joint tracking.
    JointTracking(robot_kinematics::FrankaKinematics* robot_kinematics, char constraint_sense, double gain);

    /// @brief Overrides task update block to compute proportional tracking vectors.
    void update() override;

    /// @brief Injects the target joint states for the current control cycle.
    void set_desired_state(const Eigen::VectorXd& q_d, const Eigen::VectorXd& dq_d);
};

}  // namespace task

#endif // TASK__JOINT_TRACKING_TASK_HPP_