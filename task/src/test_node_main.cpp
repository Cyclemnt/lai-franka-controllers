#include <rclcpp/rclcpp.hpp>
#include "task/allTasks.hpp"
#include <qp/QPSolver.h>
#include <robot_kinematics/FrankaKinematics.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("task_test_node");

    RCLCPP_INFO(node->get_logger(), "Starting Franka Task/QP Integration Test...");

    // Parameter Loading
    node->declare_parameter<std::vector<double>>("mod_DH.a", std::vector<double>());
    node->declare_parameter<std::vector<double>>("mod_DH.alpha", std::vector<double>());
    node->declare_parameter<std::vector<double>>("mod_DH.d", std::vector<double>());
    node->declare_parameter<std::vector<double>>("mod_DH.theta", std::vector<double>());
    node->declare_parameter<std::vector<double>>("A7e", std::vector<double>());

    auto a = node->get_parameter("mod_DH.a").as_double_array();
    auto alpha = node->get_parameter("mod_DH.alpha").as_double_array();
    auto d = node->get_parameter("mod_DH.d").as_double_array();
    auto theta = node->get_parameter("mod_DH.theta").as_double_array();
    auto a7e_vec = node->get_parameter("A7e").as_double_array();

    // Flatten DH params into 28-element vector [a, alpha, d, theta]
    std::vector<double> mDH_combined;
    mDH_combined.insert(mDH_combined.end(), a.begin(), a.end());
    mDH_combined.insert(mDH_combined.end(), alpha.begin(), alpha.end());
    mDH_combined.insert(mDH_combined.end(), d.begin(), d.end());
    mDH_combined.insert(mDH_combined.end(), theta.begin(), theta.end());

    // Setup Kinematics
    FrankaKinematics kinematics;
    if (mDH_combined.size() == 28 && a7e_vec.size() == 16) {
        kinematics.setParameters(mDH_combined, a7e_vec);
        RCLCPP_INFO(node->get_logger(), "DH Parameters and A7e matrix loaded.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Parameter size mismatch! Expected 28 DH and 16 A7e.");
        return 1;
    }

    // Set initial joint state (Home)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(7);
    q << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    kinematics.updateJointStates(q);

    // Initialize Selection Vectors
    kinematics.getSelectDOF()->assign(7, true);
    kinematics.getSelectTask()->assign(6, true);

    // Set a Desired Target (Current + Offset)
    Eigen::Vector3d target_p(0.4, 0.1, 0.5); 
    Eigen::Quaterniond target_q(1.0, 0.0, 0.0, 0.0);
    kinematics.setDesiredPose(target_p, target_q);

    // Setup Task and Solver
    Eigen::VectorXd weights = Eigen::VectorXd::Ones(6); 
    task::Pose pose_task(&kinematics, '=', weights, 2.0);
    pose_task.set_desired_task_value(0, Eigen::VectorXd::Zero(6)); 
    pose_task.set_desired_task_value(1, Eigen::VectorXd::Zero(6));

    QPSolver solver; 

    // Retrieve matrices for debugging
    Eigen::MatrixXd A = pose_task.get_A(); 
    Eigen::VectorXd b = pose_task.get_b();

    std::cout << "\n--- DEBUG INFO ---" << std::endl;
    std::cout << "Jacobian A (6x7) norm: " << A.norm() << std::endl;
    std::cout << "Target Vector b norm: " << b.norm() << std::endl;
    if (A.norm() < 1e-10) {
        std::cout << "WARNING: Jacobian is near zero. Check DH calculations!" << std::endl;
    }
    std::cout << "------------------\n" << std::endl;

    try {
        solver.addVariables(7, 'C');
        solver.addConstraints(A, b, '=', false); 
        solver.setObjectiveFunction(Eigen::MatrixXd::Identity(7, 7)); 

        RCLCPP_INFO(node->get_logger(), "Invoking Gurobi Solver...");
        solver.solve(); 

        Eigen::VectorXd dq = solver.getVariablesValue();

        std::cout << "\n========================================" << std::endl;
        std::cout << "QP SOLVE SUCCESSFUL" << std::endl;
        std::cout << "Joint Velocities (dq): " << dq.transpose() << std::endl;
        std::cout << "Constraint Residual:   " << (A * dq - b).norm() << std::endl;
        std::cout << "========================================\n" << std::endl;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Solver Exception: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}