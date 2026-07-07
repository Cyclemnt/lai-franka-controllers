/// @file selfHits.cpp
/// @brief Interactive geometric distance threshold constraint logic.

#include "task/selfHits.hpp"

namespace task {

SelfHits::SelfHits(FrankaKinematics* robot_kinematics, Eigen::VectorXi points_to_keep_safe, Eigen::VectorXi points_to_stay_away_from, double d_min, char constraint_sense, double gain) {
    robotKinematics_ = robot_kinematics;
    points_to_keep_safe_ = points_to_keep_safe;
    points_to_stay_away_from_ = points_to_stay_away_from;
    d_min_ = d_min;
    constraintSense_ = constraint_sense;
    gain_ = gain;

    int total_constraints = points_to_keep_safe_.size() * points_to_stay_away_from_.size();
    distances_ = Eigen::VectorXd::Zero(total_constraints);
    A_matrix_ = Eigen::MatrixXd::Zero(total_constraints, 7);
    b_vector_ = Eigen::VectorXd::Zero(total_constraints);

    // Properly resize the internal pre-allocated temporary Jacobian fields
    J_safe_ = Eigen::MatrixXd::Zero(3, 7);
    J_avoid_ = Eigen::MatrixXd::Zero(3, 7);
}

void SelfHits::update() {
    int row_idx = 0;
    for (int i = 0; i < points_to_keep_safe_.size(); ++i) {
        const Eigen::Vector3d p_s = robotKinematics_->getJointPosition(points_to_keep_safe_(i));
        J_safe_ = robotKinematics_->getLinearJacobian(points_to_keep_safe_(i));

        for (int j = 0; j < points_to_stay_away_from_.size(); ++j) {
            const Eigen::Vector3d p_a = robotKinematics_->getJointPosition(points_to_stay_away_from_(j));
            J_avoid_ = robotKinematics_->getLinearJacobian(points_to_stay_away_from_(j));

            const Eigen::Vector3d diff = p_s - p_a;
            double dist = diff.norm();
            if (dist < 1e-6) dist = 1e-6; // Cap threshold to bypass singularity divide-by-zero errors

            distances_(row_idx) = dist;
            const Eigen::Vector3d normal = diff / dist;

            // Formulate HQP row boundary configuration: A = n^T * (J_safe - J_avoid)
            A_matrix_.row(row_idx).noalias() = normal.transpose() * (J_safe_ - J_avoid_);
            b_vector_(row_idx) = gain_ * (d_min_ - dist);
            
            row_idx++;
        }
    }
    applyDisabledDoFs(A_matrix_);
}

}  // namespace task