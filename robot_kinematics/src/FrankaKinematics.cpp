/// @file FrankaKinematics.cpp
/// @brief Multi-joint transform mapping and frame projection routines implementation details.

#include "robot_kinematics/FrankaKinematics.hpp"
#include "robot_kinematics/Jacobian_mDH.h"
#include "robot_kinematics/DirectKinematics_mDH.h"
#include <algorithm>

namespace robot_kinematics {

FrankaKinematics::FrankaKinematics() {
    current_q_ = Eigen::VectorXd::Zero(7);
    previous_dq_ = Eigen::VectorXd::Zero(7);
    selected_dofs_.assign(7, true);   
    selected_tasks_.assign(2, true);  
    pos_d_.setZero();
    quat_d_.setIdentity();
}

void FrankaKinematics::setParameters(const std::vector<double>& mDH_in, const std::vector<double>& A7e_in) {
    std::copy(mDH_in.begin(), mDH_in.end(), mDH_table_);
    std::copy(A7e_in.begin(), A7e_in.end(), A7e_matrix_);
}

void FrankaKinematics::updateJointStates(const Eigen::VectorXd& q) {
    current_q_ = q;
    // Inject position variables directly into internal tracking theta columns entries
    for (int i = 0; i < 7; i++) {
        mDH_table_[i + 21] = q(i); 
    }
}

void FrankaKinematics::setDesiredPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    pos_d_ = p;
    quat_d_ = q;
}

void FrankaKinematics::setPreviousVelocities(const Eigen::VectorXd& previous_dq) {
    previous_dq_ = previous_dq;
}

Eigen::VectorXd FrankaKinematics::get_q() const {
    return current_q_;
}

Eigen::VectorXd FrankaKinematics::getPreviousVelocities() const {
    return previous_dq_;
}

Eigen::MatrixXd FrankaKinematics::getJacobian() const {
    double J_raw[42];
    Jacobian_mDH(mDH_table_, A7e_matrix_, J_raw);
    return Eigen::Map<const Eigen::Matrix<double, 6, 7>>(J_raw);
}

Eigen::Vector3d FrankaKinematics::getJointPosition(int joint_index) const {
    joint_index = std::clamp(joint_index, 0, 6);

    double T0[112];
    DirectKinematics_mDH(mDH_table_, A7e_matrix_, T0);
    
    // Each 4x4 homogenous matrix frame slice spans a 16-element contiguous double span
    // Translation column elements are systematically indexed at positions [12, 13, 14]
    int offset = joint_index * 16;
    return Eigen::Vector3d(T0[offset + 12], T0[offset + 13], T0[offset + 14]);
}

Eigen::MatrixXd FrankaKinematics::getLinearJacobian(int joint_index) const {
    joint_index = std::clamp(joint_index, 0, 6);

    double T0[112];
    DirectKinematics_mDH(mDH_table_, A7e_matrix_, T0);
    
    Eigen::MatrixXd J_lin = Eigen::MatrixXd::Zero(3, 7);
    
    int offset_k = joint_index * 16;
    Eigen::Vector3d p_k(T0[offset_k + 12], T0[offset_k + 13], T0[offset_k + 14]);
    
    // Cross product calculation determines positional dependencies across the kinematic chain
    for (int i = 0; i <= joint_index; i++) {
        int offset_i = i * 16;
        
        Eigen::Vector3d z_i(T0[offset_i + 8], T0[offset_i + 9], T0[offset_i + 10]);
        Eigen::Vector3d p_i(T0[offset_i + 12], T0[offset_i + 13], T0[offset_i + 14]);
        
        J_lin.col(i) = z_i.cross(p_k - p_i);
    }
    
    return J_lin;
}

std::vector<bool>* FrankaKinematics::getSelectDOF() { 
    return &selected_dofs_; 
}

std::vector<bool>* FrankaKinematics::getSelectTask() { 
    return &selected_tasks_; 
}

Eigen::VectorXd FrankaKinematics::getError() const {
    double T0_raw[112];
    DirectKinematics_mDH(mDH_table_, A7e_matrix_, T0_raw);

    // Frame index 7 maps target properties straight onto end-effector matrices entries [96-111]
    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_ee(&T0_raw[96]);
    
    Eigen::Vector3d pos_curr = T_ee.block<3,1>(0,3);
    Eigen::Matrix3d rot_curr = T_ee.block<3,3>(0,0);
    Eigen::Quaterniond quat_curr(rot_curr);

    Eigen::Vector3d pos_err = pos_d_ - pos_curr;

    // Evaluates axis angles errors ensuring calculation tracks shortest path configurations bounds
    Eigen::Quaterniond q_err = quat_curr.inverse() * quat_d_;
    if (q_err.w() < 0.0) {
        q_err.coeffs() *= -1.0;
    }
    Eigen::Vector3d ori_err = rot_curr * q_err.vec(); 

    Eigen::VectorXd full_error(6);
    full_error << pos_err, ori_err;
    return full_error;
}

Eigen::VectorXd FrankaKinematics::getPose() const {
    double T0_raw[112];
    DirectKinematics_mDH(mDH_table_, A7e_matrix_, T0_raw);
    
    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_ee(&T0_raw[96]);
    
    Eigen::Vector3d pos_curr = T_ee.block<3,1>(0,3);
    Eigen::Matrix3d rot_curr = T_ee.block<3,3>(0,0);
    Eigen::Quaterniond quat_curr(rot_curr);
    
    if (quat_curr.w() < 0.0) {
        quat_curr.coeffs() *= -1.0;
    }
    Eigen::Vector3d ori_curr = rot_curr * quat_curr.vec();

    Eigen::VectorXd full_pose(6);
    full_pose << pos_curr, ori_curr;
    return full_pose;
}

Eigen::Vector3d FrankaKinematics::getPosition() const {
    double T0_raw[112];
    DirectKinematics_mDH(mDH_table_, A7e_matrix_, T0_raw);
    
    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_ee(&T0_raw[96]);
    return T_ee.block<3,1>(0,3);
}

Eigen::Quaterniond FrankaKinematics::getQuaternion() const {
    double T0_raw[112];
    DirectKinematics_mDH(mDH_table_, A7e_matrix_, T0_raw);
    
    Eigen::Map<const Eigen::Matrix<double, 4, 4>> T_ee(&T0_raw[96]);
    Eigen::Matrix3d rot_curr = T_ee.block<3,3>(0,0);
    Eigen::Quaterniond quat_curr(rot_curr);
    
    if (quat_curr.w() < 0.0) {
        quat_curr.coeffs() *= -1.0;
    }
    return quat_curr;
}

} // namespace robot_kinematics
