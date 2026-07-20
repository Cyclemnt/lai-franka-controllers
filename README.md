# LAI Franka HQP Control Stack

This repository contains an advanced suite of real-time Cartesian motion controllers, mathematical task formulations, and analytical kinematics engines for the Franka Emika FR3 manipulator.

Developed during an internship at the **LAI Robotics Lab**, this stack implements a strictly prioritized Hierarchical Quadratic Programming (HQP) architecture. It separates complex optimization bounds (virtual walls, self-collision, kinematics limits) from the real-time 1 kHz hardware loop, ensuring mathematically guaranteed safety boundaries during autonomous trajectory execution and teleoperation.

---

## Repository Structure

This repository contains three core packages that work together to control the robot:

| Package | Description |
| --- | --- |
| **`lai_franka_controllers`** | `ros2_control` hardware plugins, standalone HQP reference generation nodes, and gamepad teleoperation nodes. |
| **`task`** | Mathematical constraints formulation engine. Maps physical objectives (Pose tracking, Virtual Walls, Self-Collision) into standard $A$ and $b$ matrices for the solver. |
| **`robot_kinematics`** | High-performance, codegen-backed analytical kinematics engine providing real-time Jacobians, forward kinematics, and shortest-path error tracking. |

*(For detailed information on each module, please refer to the individual `README.md` files located inside each package folder).*

---

## Important: Proprietary Lab Dependencies

**Please Note:** This repository serves as a showcase of the control architectures and task formulations developed during my internship. It **cannot be compiled out-of-the-box** by the general public.

The code relies on three closed-source, proprietary optimization wrapper libraries developed internally by the LAI Robotics Lab. These packages are not included in this repository to protect the laboratory's intellectual property:

* **`qp`**: Internal Quadratic Programming solver wrapper interface.
* **`hierarchical_qp`**: Internal HQP cascade solver engine.
* **`auxiliaries_function_and_structures`**: Lab-specific mathematical utilities and structures.

To compile this workspace, you must have these packages sourced in your local ROS 2 underlay.

---

## Standard System Dependencies

If the proprietary lab packages are present, the following standard dependencies must also be installed:

* **ROS 2 Humble** (Ubuntu 22.04)
* **libfranka** & **franka_ros2**
* **Gurobi Optimizer** (Valid system license strictly required)
* **Eigen3** & **OpenMP**
* **Pinocchio** (For legacy controller support)
* **libusb-1.0-0-dev** (For WSL2 direct gamepad teleoperation support)

---

## Build Instructions

Assuming all open-source and LAI dependencies are installed and sourced, the stack must be compiled in `Release` mode to ensure the Gurobi optimization solver and matrix multiplications evaluate within the strict 1 ms hardware timing constraint.

```bash
cd ~/franka_ros2_ws/

# Clean legacy CMake cache (Optional but recommended)
rm -rf build/robot_kinematics build/task build/lai_franka_controllers
rm -rf install/robot_kinematics install/task install/lai_franka_controllers

# Build the custom stack
colcon build --packages-select robot_kinematics task lai_franka_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

```

---

## Quick Start & Launch Architecture

The architecture utilizes a decoupled **Internal Model Control (IMC)** approach. The mathematical bounds are resolved inside an isolated virtual reference generator, which then streams safe trajectories to a lean Joint PD Feedforward controller.

### 1. Hardware / Simulation Bringup

Launch the robot or Gazebo simulation using the tracking controller. Make sure `joint_pd_velocity_controller` is registered in your `controllers.yaml`.

**For Real Hardware:**
```bash
ros2 launch franka__bringup example.launch.py \
    controller:=joint_pd_velocity_controller \
    robot_type:=fr3 \
    robot_ip:=172.16.0.2" \
    load_gripper:=true

```

**For Gazebo Simulation:**
```bash
ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py \
    controller:=joint_pd_velocity_controller \
    gz_args:="-r -s empty.sdf"

```

### 2. Start the HQP Optimization Engine

In a new terminal, launch the virtual reference generator. This node reads the robot states, evaluates the `task` matrices, solves the Gurobi HQP constraints, and streams safe targets to the PD controller.

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

### 3. Send Goals (Trajectories, Teleoperation, or Diagnostics)

With the safety boundaries active, you can safely interact with the robot using automated nodes, teleoperation, or by publishing manual targets directly from the command line.

#### Method A: Standalone Applications
Ensure your `primary_task.mode` setting in `hqp_node_params.yaml` matches the space you are commanding (e.g., `"cartesian"` or `"joint"`).

```bash
# Automated Quintic Trajectories and Boundary Stress Tests (Cartesian Mode)
ros2 run lai_franka_controllers trajectory_generator_node

# Xbox/Logitech Gamepad Teleoperation (Cartesian Mode)
ros2 launch lai_franka_controllers joy_teleop.launch.py

# Multi-Joint Feedforward Sinusoidal Diagnostics (Joint Mode)
ros2 run lai_franka_controllers hqp_joint_trajectory_test_node

```

#### Method B: Manual Command-Line (CLI) Target Injection

You can manually publish static targets directly to the HQP stack or the active trajectory planner using the `--once` flag.

* **For Cartesian Mode via the Trajectory Planner (Recommended):**
  When the `trajectory_generator_node` is active, publish your target to the global `/goal_pose` topic. The planner will intercept this, calculate a smooth quintic S-curve trajectory profile, and feed it incrementally to the HQP layer:
  ```bash
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'fr3_link0'}, pose: {position: {x: 0.5, y: 0.1, z: 0.4}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"
    ```

*(Note: Setting a negative Z-value like `z: -0.1` here will automatically trigger the multi-waypoint boundary boundary stress test).*


* **For Cartesian Tracking Mode (Bypassing Planner):**
Publish to the `/hqp_reference_generator_node/target_pose` topic. The solver will safely compute optimal joint velocities to track the 6D target:
```bash
ros2 topic pub --once /hqp_reference_generator_node/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'fr3_link0'}, pose: {position: {x: 0.4, y: 0.0, z: 0.4}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"

```


* **For Joint Tracking Mode:**
Publish to the `/hqp_reference_generator_node/target_joint` topic. The updated name-mapping parser routes the targets dynamically. Specify the exact joint name array along with target positions and optional feedforward velocities:
```bash
ros2 topic pub --once /hqp_reference_generator_node/target_joint sensor_msgs/msg/JointState \
"{name: ['fr3_joint1'], position: [-1.5], velocity: [0.0]}"

```

> **Hardware Diagnostic Note:** If you need to verify the low-level physical tracking performance and bypass the HQP optimization stack entirely, you can stream commands directly to the hardware driver plugin topic (`/joint_pd_velocity_controller/joint_commands`) by running `ros2 run lai_franka_controllers joint_pd_test_node`.
