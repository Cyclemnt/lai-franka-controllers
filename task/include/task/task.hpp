/// @file task.hpp
/// @brief Abstract base class layout for hierarchical optimization tasks.
/// @author Clement Lamouller
/// @date 2026

#ifndef TASK__TASK_HPP_
#define TASK__TASK_HPP_

#include <robot_kinematics/FrankaKinematics.hpp>
#include <Eigen/Dense>

using namespace robot_kinematics;

namespace task {

/// @class Task
/// @brief Polymorphic template enforcing mathematical structures across the HQP optimization cascade.
///
/// Encapsulates matrix $A$ and vector $b$ descriptors mapping inequality ($A \dot{q} \le b$) or 
/// equality ($A \dot{q} = b$) kinematic objectives within prioritized stacks.
class Task {
protected:
    // ---- Dependencies & Configurations ----
    FrankaKinematics* robotKinematics_;

    char constraintSense_;
    double gain_;
    int priority_level_;
    bool slacks_state_;
    bool task_state_;

    double smooth_activation_value_;
    bool smooth_activation_state_;
    
    // ---- Cached Real-Time Allocation Variables ----
    Eigen::MatrixXd A_matrix_;
    Eigen::VectorXd b_vector_;

    /// @brief Modifies A directly in-place to disable column mappings for unselected DoFs.
    /// @param A Reference matrix being projected onto enabled workspace boundaries.
    void applyDisabledDoFs(Eigen::MatrixXd& A) const;

public:
    /// @brief Base constructor assigning initial activation thresholds.
    Task();
    
    /// @brief Virtual Default Destructor ensuring correct derived lifecycle invocation.
    virtual ~Task() = default;

    // ---- Core Read-Only Access Getters ----
    char getConstraintSense() const;
    double getGain() const;
    int getPriorityLevel() const;
    bool getSlacksState() const;
    bool isEnabled() const;
    double getSmoothActivationValue() const;
    bool getSmoothActivationState() const;

    // ---- Parameter Modification Setters ----
    void setRobotKinematics(FrankaKinematics* robotKinematics);
    void setConstraintSense(char constraintSense);
    void setGain(double gain);
    void setPriorityLevel(int priority_level);
    void setSlacksState(bool slacks_state);
    void setTaskState(bool task_state);
    void setSmoothActivationValue(double smooth_activation_value);
    void setSmoothActivationState(bool smooth_activation_state);

    /// @brief Pure virtual evaluation step executing objective matrix recalculations.
    virtual void update() = 0;
    
    // ---- Real-Time References Optimization Mappings ----
    const Eigen::MatrixXd& get_A() const { return A_matrix_; }
    const Eigen::VectorXd& get_b() const { return b_vector_; }
};

}  // namespace task

#endif // TASK__TASK_HPP_