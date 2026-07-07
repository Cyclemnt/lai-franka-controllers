/// @file jointSineTask.cpp
/// @brief Sinusoidal evaluation loop implementation.

#include "task/jointSineTask.hpp"
#include <cmath>

namespace task {

JointSineTask::JointSineTask() {
    current_time_ = 0.0;
    A_matrix_ = Eigen::MatrixXd::Identity(7, 7);
    b_vector_ = Eigen::VectorXd::Zero(7);
    constraintSense_ = '=';
}

void JointSineTask::set_time(double t) {
    current_time_ = t;
}

void JointSineTask::update() {
    // Parameter tables defining complex, pseudo-random joint excitation trajectories
    double a[6] = {0.35, 0.30, 0.25, 0.20, 0.15, 0.12}; 
    double f[6] = {0.03, 0.07, 0.11, 0.17, 0.23, 0.29};
    double phase[7][6] = {
        {0.1, 0.2, 0.3, 0.4, 0.5, 0.6},
        {0.2, 0.4, 0.6, 0.8, 1.0, 1.2},
        {0.3, 0.6, 0.9, 1.2, 1.5, 1.8},
        {0.4, 0.8, 1.2, 1.6, 2.0, 2.4},
        {0.5, 1.0, 1.5, 2.0, 2.5, 3.0},
        {0.6, 1.2, 1.8, 2.4, 3.0, 3.6},
        {0.7, 1.4, 2.1, 2.8, 3.5, 4.2}
    };

    for (int i = 0; i < 7; i++) {
        b_vector_(i) = 0.0;
        for (int j = 0; j < 6; j++) {
            b_vector_(i) += a[j] * std::sin(2.0 * M_PI * f[j] * current_time_ + phase[i][j]);
        }
    }
}

}  // namespace task