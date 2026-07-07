/// @file virtualWall.hpp
/// @brief Planar workspace boundary protection task.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__VIRTUAL_WALL_HPP_
#define TASK__VIRTUAL_WALL_HPP_

#include "task/task.hpp"

namespace task {

/// @class VirtualWall
/// @brief Instantiates a geometric bounding plane to act as an un-breachable hard virtual boundary.
///
/// Solves point-to-plane equations for specific critical joint clusters to scale back velocity vectors 
/// before contact errors can occur.
class VirtualWall : public Task {
private:
    Eigen::VectorXi joints_to_keep_safe_;
    Eigen::VectorXd distances_;
    double d_min_;

    Eigen::Vector3d first_point_;
    Eigen::Vector3d second_point_;
    Eigen::Vector3d third_point_;
    Eigen::Vector3d normal_vec_;

    // Pre-allocated temporary Jacobian block
    Eigen::MatrixXd J_temp_;

    Eigen::Vector3d compute_unit_normal();
    double compute_distance_to_plane(const Eigen::Vector3d& point);

public:
    /// @brief Parameterized constructor calculating target plane boundaries.
    VirtualWall(FrankaKinematics* robot_kinematics, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::VectorXi& joints_safe, double d_min, char constraint_sense, double gain);
    
    /// @brief Computes actual positions inside the workspace and maps safety projections.
    void update() override;

    /// @brief Returns the vector containing computed joint-to-plane distances.
    Eigen::VectorXd get_distances_vector() const { return distances_; }
};

}  // namespace task

#endif // TASK__VIRTUAL_WALL_HPP_