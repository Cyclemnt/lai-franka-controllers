/// @file FrankaKinematics.hpp
/// @brief Analytical tracking wrapper wrapping custom codegen C-style Modified DH kinematical engines.
/// @author Clement Lamouller
/// @date 2026

#ifndef ROBOT_KINEMATICS__FRANKA_KINEMATICS_HPP_
#define ROBOT_KINEMATICS__FRANKA_KINEMATICS_HPP_

#include <Eigen/Dense>
#include <vector>

namespace robot_kinematics {

/// @class FrankaKinematics
/// @brief Interface wrapper handling forward kinematics, analytical Jacobians, and error tracking metrics.
///
/// Converts linear array buffers evaluated via high-performance codegen routines into Eigen layouts 
/// used extensively inside the optimization stack loop layers.
class FrankaKinematics {
private:
    // ---- Parameters ----
    double mDH_table_[28];
    double A7e_matrix_[16];
    
    // ---- State and Selection ----
    Eigen::VectorXd current_q_;
    Eigen::VectorXd previous_dq_;
    std::vector<bool> selected_dofs_;
    std::vector<bool> selected_tasks_;
    
    // ---- Desired Target Reference Pose ----
    Eigen::Vector3d pos_d_;
    Eigen::Quaterniond quat_d_;

public:
    /// @brief Constructor initializing standard 7-DOF configurations tracking presets.
    FrankaKinematics();

    /// @brief Virtual Default Destructor.
    virtual ~FrankaKinematics() = default;

    /// @brief Loads parameters vectors into fixed array tracking blocks buffers.
    /// @param mDH_in 28-element linear matrix describing analytical kinematics segments transformations.
    /// @param A7e_in 16-element transform mapping joint 7 output flange onto true Hand TCP tracking center.
    void setParameters(const std::vector<double>& mDH_in, const std::vector<double>& A7e_in);
    
    /// @brief Updates state maps and injects joint inputs positions straight into tracking parameters columns.
    /// @param q Current actual or integrated joint position state configurations vector.
    void updateJointStates(const Eigen::VectorXd& q);
    
    /// @brief Overwrites internal reference goals definitions targets variables properties.
    /// @param p Targeted position 3-axis coordinates in workspace meters.
    /// @param q Targeted orientation frame rotation quaternion.
    void setDesiredPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    
    /// @brief Caches joint command output velocities to preserve continuity profiles history tracking updates.
    /// @param previous_dq Joint velocity states array from the preceding solver loop iteration cycle.
    void setPreviousVelocities(const Eigen::VectorXd& previous_dq);

    // ---- Native Data Extraction Access Read Properties ----
    Eigen::VectorXd get_q() const;
    Eigen::VectorXd getPreviousVelocities() const;
    Eigen::MatrixXd getJacobian() const;
    Eigen::Vector3d getJointPosition(int joint_index) const;
    Eigen::MatrixXd getLinearJacobian(int joint_index) const;
    Eigen::VectorXd getError() const;
    Eigen::VectorXd getPose() const;
    Eigen::Vector3d getPosition() const;
    Eigen::Quaterniond getQuaternion() const;
    
    std::vector<bool>* getSelectDOF();
    std::vector<bool>* getSelectTask();

    const double* get_mDH_array() const { return mDH_table_; }
    const double* get_A7e_array() const { return A7e_matrix_; }
};

} // namespace robot_kinematics

#endif // ROBOT_KINEMATICS__FRANKA_KINEMATICS_HPP_