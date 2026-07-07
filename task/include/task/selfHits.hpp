/// @file selfHits.hpp
/// @brief Self-collision avoidance safety constraint formulation.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__SELF_HITS_HPP_
#define TASK__SELF_HITS_HPP_

#include "task/task.hpp"

namespace task {

/// @class SelfHits
/// @brief Protects the manipulator from self-collision via distance bounding fields.
///
/// Projects joint-to-joint relative distance vectors onto task Jacobians to enforce minimum safety
/// distances ($d_{min}$) using linear velocity limits in the optimization problem.
class SelfHits : public Task {
private:
    Eigen::VectorXi points_to_keep_safe_;
    Eigen::VectorXi points_to_stay_away_from_;

    double d_min_;
    Eigen::VectorXd distances_;

    // Pre-allocated temporaries to prevent real-time heap allocation
    Eigen::MatrixXd J_safe_;
    Eigen::MatrixXd J_avoid_;

public:
    /// @brief Parameterized constructor establishing collision check pairs.
    SelfHits(FrankaKinematics* robot_kinematics, Eigen::VectorXi points_to_keep_safe, Eigen::VectorXi points_to_stay_away_from, double d_min, char constraint_sense, double gain);

    /// @brief Recalculates relative link separation vectors and normal matrices rows.
    void update() override;

    /// @brief Returns the vector containing runtime calculated Euclidean distances.
    Eigen::VectorXd get_distances_vector() const { return distances_; }
};

}  // namespace task

#endif // TASK__SELF_HITS_HPP_