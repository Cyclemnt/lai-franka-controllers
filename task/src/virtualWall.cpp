/// @file virtualWall.cpp
/// @brief Linear workspace workspace bounding plane mathematical projections.

#include "task/virtualWall.hpp"

namespace task {

VirtualWall::VirtualWall(FrankaKinematics* robot_kinematics, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::VectorXi& joints_safe, double d_min, char constraint_sense, double gain) {
    robotKinematics_ = robot_kinematics;
    joints_to_keep_safe_ = joints_safe;
    first_point_ = p1;
    second_point_ = p2;
    third_point_ = p3;
    d_min_ = d_min;
    constraintSense_ = constraint_sense;
    gain_ = gain;

    normal_vec_ = compute_unit_normal();

    int num_safe_joints = joints_to_keep_safe_.size();
    A_matrix_ = Eigen::MatrixXd::Zero(num_safe_joints, 7);
    b_vector_ = Eigen::VectorXd::Zero(num_safe_joints);
    distances_ = Eigen::VectorXd::Zero(num_safe_joints);
    J_temp_ = Eigen::MatrixXd::Zero(3, 7);
}

void VirtualWall::update() {
    for (int i = 0; i < joints_to_keep_safe_.size(); ++i) {
        int idx = joints_to_keep_safe_(i);
        
        const Eigen::Vector3d p = robotKinematics_->getJointPosition(idx);
        J_temp_ = robotKinematics_->getLinearJacobian(idx);
        
        double d = compute_distance_to_plane(p);
        distances_(i) = d;

        // Plane boundary projection: A = n^T * J
        A_matrix_.row(i).noalias() = normal_vec_.transpose() * J_temp_;
        b_vector_(i) = gain_ * (d_min_ - d);
    }
    applyDisabledDoFs(A_matrix_);
}

Eigen::Vector3d VirtualWall::compute_unit_normal() {
    Eigen::Vector3d unit_vector_n = (second_point_ - first_point_).cross(third_point_ - first_point_);
    return unit_vector_n / unit_vector_n.norm();
}

double VirtualWall::compute_distance_to_plane(const Eigen::Vector3d& point) {
    return (point - first_point_).dot(normal_vec_);
}

}  // namespace task