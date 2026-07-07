/// @file task.cpp
/// @brief Abstract mathematical task infrastructure implementation details.

#include "task/task.hpp"

namespace task {

Task::Task() {
    smooth_activation_value_ = 1.0;
    smooth_activation_state_ = false;
    task_state_ = true;
    robotKinematics_ = nullptr;
}

char Task::getConstraintSense() const { return constraintSense_; }
double Task::getGain() const { return gain_; }
int Task::getPriorityLevel() const { return priority_level_; }
bool Task::getSlacksState() const { return slacks_state_; }
bool Task::isEnabled() const { return task_state_; }
double Task::getSmoothActivationValue() const { return smooth_activation_value_; }
bool Task::getSmoothActivationState() const { return smooth_activation_state_; }

void Task::setRobotKinematics(FrankaKinematics* robotKinematics) { robotKinematics_ = robotKinematics; }
void Task::setConstraintSense(char constraintSense) { constraintSense_ = constraintSense; }
void Task::setGain(double gain) { gain_ = gain; }
void Task::setPriorityLevel(int priority_level) { priority_level_ = priority_level; }
void Task::setSlacksState(bool slacks_state) { slacks_state_ = slacks_state; }
void Task::setTaskState(bool task_state) { task_state_ = task_state; }
void Task::setSmoothActivationValue(double smooth_activation_value) { smooth_activation_value_ = smooth_activation_value; }
void Task::setSmoothActivationState(bool smooth_activation_state) { smooth_activation_state_ = smooth_activation_state; }

void Task::applyDisabledDoFs(Eigen::MatrixXd& A) const {
    if (!robotKinematics_) return; 
    
    std::vector<bool>* status_DoFs = robotKinematics_->getSelectDOF();
    for (int i = 0; i < 7; i++) {
        if (!status_DoFs->at(i)) {
            A.col(i).setZero(); 
        }
    }
}

}  // namespace task