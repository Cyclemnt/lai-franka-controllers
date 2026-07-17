/// @file jointSine.hpp
/// @brief Multi-frequency joint-space sinusoidal trajectory tracking task.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__JOINT_SINE_TASK_HPP_
#define TASK__JOINT_SINE_TASK_HPP_

#include "task/task.hpp"

namespace task {

/// @class jointSine
/// @brief Generates complex multi-joint sinusoidal reference patterns for diagnostic validation.
class jointSine : public Task {
private:
    double current_time_;

public:
    /// @brief Default constructor initializing pre-allocated 7-DOF identity mapping.
    jointSine();

    /// @brief Syncs the current timestamp reference.
    void set_time(double t);

    /// @brief Overrides update block to compute superimposed Fourier sine waves.
    void update() override;
};

}  // namespace task

#endif // TASK__JOINT_SINE_TASK_HPP_