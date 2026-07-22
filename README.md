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

***(For localized information on each module, please refer to the individual `README.md` files located inside each package folder).***

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

## Configuration Profiles (`.yaml`)

The behavior of the HQP solver and Teleoperation suite is heavily parameterized. You can adjust kinematic models, safety limits, and tuning gains via their respective YAML files in `lai_franka_controllers/config/` without recompiling.

* **`hqp_node_params.yaml`**: Governs the mathematical constraints of the HQP solver. Adjusts the modified DH parameters, End-Effector TCP offset, joint velocity/position limits, self-collision safety buffers, and the 6-sided virtual wall workspace boundary dimensions. This file also contains `enabled: true/false` flags for each task priority and configures `primary_task.mode` (`"cartesian"` or `"joint"`).
* **`joy_teleop_params.yaml`**: Governs gamepad responsiveness. Sets hard caps for Cartesian translation/rotation velocities, acceleration ramps to prevent hardware jerks, anti-windup leashes, and gripper tuning constants.

---

## Complete Quick Start & Usage Guide

The architecture utilizes a decoupled **Internal Model Control (IMC)** approach. Mathematical bounds are resolved inside an isolated virtual reference generator (500 Hz), which streams safe trajectories to a lean Joint PD Feedforward controller (1000 Hz).

### Step 1: Hardware / Simulation Bringup

Make sure `joint_pd_velocity_controller` is registered in your bringup package's `controllers.yaml`.

**For Real Hardware:**

```bash
ros2 launch franka_bringup example.launch.py \
    controller:=joint_pd_velocity_controller \
    robot_type:=fr3 \
    robot_ip:=172.16.0.2 \
    load_gripper:=true

```

**For Gazebo Simulation:**

```bash
ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py \
    controller:=joint_pd_velocity_controller \
    gz_args:="-r -s empty.sdf"

```

---

### Step 2: Running Execution Scenarios

#### Scenario A: Low-Level Hardware Diagnostics

To verify the physical tracking performance of the PD controller and bypass the HQP solver entirely:

```bash
# Streams multi-joint sine waves directly to the hardware controller
ros2 run lai_franka_controllers joint_pd_test_node

```

#### Scenario B: High-Level HQP Joint Tracking

To verify the HQP solver in joint space. *(Ensure `primary_task.mode` is set to `"joint"` in `hqp_node_params.yaml`)*.

**Terminal 1:** Start the HQP Solver

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

**Terminal 2:** Launch the Diagnostic Sine Generator **OR** Publish Static Goals

* **Option 1 (Sine Generator):**
```bash
ros2 run lai_franka_controllers hqp_joint_trajectory_test_node

```


* **Option 2 (Manual CLI Target):**
```bash
ros2 topic pub --once /hqp_reference_generator_node/target_joint sensor_msgs/msg/JointState \
"{name: ['fr3_joint1'], position: [-1.5], velocity: [0.0]}"

```



> **Note:** Unlike Cartesian mode, Joint mode does *not* feature an automated point-to-point path planner. It relies strictly on the diagnostic sine generator or direct manual target publications.

#### Scenario C: Cartesian Operations (HQP + Planner / Direct Targets)

Standard Cartesian operations. *(Ensure `primary_task.mode` is set to `"cartesian"` in `hqp_node_params.yaml`)*.

**Terminal 1:** Start the HQP Solver

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

**Terminal 2:** Choose Trajectory Method

* **Option 1: Using the Smooth Quintic Trajectory Planner (Recommended)**
Start the planner node:
```bash
ros2 run lai_franka_controllers trajectory_generator_node

```


Send a smooth pose target to `/goal_pose`:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'fr3_link0'}, pose: {position: {x: 0.5, y: 0.1, z: 0.4}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"

```


*(Note: Setting `z < 0.0` automatically triggers the automated multi-waypoint boundary stress test).*
* **Option 2: Direct Target Ingestion (Bypassing the Planner)**
Publish directly to the reference generator without running `trajectory_generator_node`:
```bash
ros2 topic pub --once /hqp_reference_generator_node/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'fr3_link0'}, pose: {position: {x: 0.4, y: 0.0, z: 0.4}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}"

```



#### Scenario D: Gamepad Teleoperation (HQP + Joy)

Safely drive the robot around using an Xbox/Logitech controller. The HQP solver will automatically prevent wall crashes or self-collisions.

**Terminal 1:** Start the HQP Solver

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

**Terminal 2:** Start the Teleoperation Node

```bash
ros2 launch lai_franka_controllers joy_teleop.launch.py

```

*(**WSL2 Note:** If standard `/dev/input/js0` fails inside a virtual machine, open `joy_teleop.launch.py` and uncomment the `raw_usb_joy_node` section to enable direct USB polling).*

---

## Diagnostics & System Monitoring

The HQP Reference generator streams lock-free diagnostics for real-time analysis. Open PlotJuggler to monitor system state and bounds:

```bash
ros2 run plotjuggler plotjuggler

```

**Key Diagnostic Topics:**

* `~/hqp_reference_generator_node/tracking_error` *(Twist error vector between virtual reference and target)*
* `~/hqp_reference_generator_node/virtual_wall_distances` *(Array of distances to the 6 virtual bounding planes)*
* `~/hqp_reference_generator_node/self_hits_distances` *(Distance between end-effector and base links)*
* `~/hqp_reference_generator_node/current_pose` *(Live 6D pose of the internal virtual robot model)*
* `/joint_pd_velocity_controller/joint_commands` *(Commanded targets generated by the HQP solver)*
* `/joint_pd_velocity_controller/output_dq_cmd` *(Actual velocities dispatched to physical motor drivers)*