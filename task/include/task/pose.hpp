/// @file pose.hpp
/// @brief 6D operational space pose tracking task formulation.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__POSE_HPP_
#define TASK__POSE_HPP_

#include "task/task.hpp"
#include <vector>

namespace task {

/// @class Pose
/// @brief Tracks 6D Cartesian targets, selectively extracting position or orientation subsets.
class Pose : public Task {
private:
    std::vector<bool>* selectedTask_;
    std::vector<Eigen::VectorXd> initial_task_value_;
    std::vector<Eigen::VectorXd> desired_task_value_;
    Eigen::VectorXd DLS_weights_;
    Eigen::MatrixXd K_gain_;

    // Real-time pre-allocated scratch variables
    Eigen::VectorXd current_error_;
    Eigen::VectorXd feed_forward_;
    Eigen::MatrixXd franka_jacobian_;

public:
    /// @brief Frames task dimensions based on requested operational subspaces.
    Pose(FrankaKinematics* robot_kinematics, char constraint_sense, const Eigen::VectorXd& DLS_weights, double gain);
    
    /// @brief Extracts relevant Jacobian rows and structures feedforward loops.
    void update() override;

    /// @brief Solves Damped Least Squares (DLS) regularized quadratic tracking weighting fields.
    Eigen::MatrixXd get_DLS_quadratic_objective_matrix(const Eigen::MatrixXd& J_in, double lambda, double threshold, const Eigen::MatrixXd& DLS_weights_in);

    void set_initial_task_value(int derivation_order, const Eigen::VectorXd& value);
    void set_desired_task_value(int derivation_order, const Eigen::VectorXd& value);
};

}  // namespace task

#endif // TASK__POSE_HPP_