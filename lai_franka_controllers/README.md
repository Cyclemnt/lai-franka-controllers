# LAI Franka ROS 2 Controllers

This package implements an advanced suite of real-time controllers, Hierarchical Quadratic Programming (HQP) safety filters, and teleoperation nodes for the Franka Emika FR3 manipulator using the `ros2_control` framework.

This repository utilizes a **decoupled Internal Model Control (IMC) architecture**. Instead of calculating complex optimizations inside the hardware loop, mathematical bounds (virtual walls, self-collision, kinematics) are resolved at 500 Hz inside an isolated virtual reference generator. Safe trajectories are then streamed to a real-time Joint PD Feedforward controller running strictly at 1000 Hz.

## Package Structure

### 1. `ros2_control` Hardware Plugins (1000 Hz)

* **`joint_pd_velocity_controller.hpp`**: *(Recommended)* Joint-space tracking controller with velocity feedforward and timeout watchdogs. Designed to pair with the HQP Reference Generator.
* **`hqp_cartesian_velocity_controller.hpp`**: *(Legacy)* Cartesian controller calculating Gurobi HQP optimization directly inside the 1 kHz loop.
* **`cartesian_velocity_controller.hpp`**: *(Legacy)* Pinocchio-based Jacobian pseudo-inverse controller with Levenberg-Marquardt damping and nullspace limits repulsion.

### 2. Standalone Processing Nodes

* **`hqp_reference_generator_node.hpp`**: The mathematical core. Solves strict-priority HQP constraints (joint limits, self-collision, virtual 6D bounding boxes) using Gurobi over a virtual robot model.
* **`trajectory_generator_node.hpp`**: Quintic S-curve Cartesian path planner with an automated HQP boundary "Stress Test".
* **`hqp_joint_trajectory_test_node.hpp`**: Diagnostic trajectory generator specifically designed to feed smooth sine waves (position and velocity) into the HQP solver to test the high-level `JointTracking` task and optimization bounds.
* **`joint_pd_test_node.hpp`**: Low-level diagnostic tool that bypasses the HQP solver, publishing directly to the PD controller to test physical tracking performance and hardware lag.

### 3. Teleoperation Suite

* **`joy_teleop_node.hpp`**: Translates raw gamepad inputs into smooth Cartesian velocity integrations with anti-windup leashes and asynchronous Franka gripper action hooks.
* **`raw_usb_joy_node.hpp`**: WSL2-compatible direct `libusb` interrupt reader that bypasses missing Linux joystick drivers in Windows VMs.

---

## Configuration Profiles (`.yaml`)

The behavior of the HQP solver and Teleoperation suite is heavily parameterized. You can adjust kinematic models, safety limits, and tuning gains via their respective YAML files without recompiling.

### 1. `hqp_node_params.yaml`
Governs the mathematical constraints of the HQP solver.
* **`mod_DH` & `A7e`**: Defines the Modified Denavit-Hartenberg kinematic chain and the End-Effector TCP offset (including the 45° Z-axis flange rotation).
* **Priority 1 (`joint_limits`)**: Hard physical constraints for `q_max`, `q_min`, and `dq_max`.
* **Priority 2 (`self_collision`)**: Defines which points on the robot monitor each other (e.g., tracking the EE vs the Base) and the minimum allowable distance buffer.
* **Priority 3 (`virtual_walls`)**: Defines a 6-sided Cartesian bounding box (`floor_z`, `ceiling_z`, `front_x`, etc.). You can specify which joints are protected from crossing this boundary and set the repulsion `gain`.
* **Priority 4 (`primary_task`)**: Configures the primary operational goal. Set `mode` to either `"cartesian"` (tracks 6D poses) or `"joint"` (tracks joint trajectories).

### 2. `joy_teleop_params.yaml`
Governs gamepad responsiveness and safety leashes.
* **Max Velocities**: Hard caps for Cartesian translation (`v_max`) and rotation (`omega_max`) at 100% stick deflection.
* **Ramping Constraints**: `accel_limit_trans` and `accel_limit_rot` prevent aggressive joystick snaps from causing hardware jerks.
* **Anti-Windup Leashes**: `max_translation_lead` and `max_rotation_lead` cap how far the virtual integrated target is allowed to "pull" ahead of the physical robot if the arm gets stuck or lags, preventing violent snapping behaviors when freed.
* **Gripper Constants**: Tuning profiles for pinch force (`gripper_grasp_force`), closure speed, and maximum open width.

---

## Dependencies & Installation

### Prerequisites

* **ROS 2 Humble**
* **libfranka** & **franka_ros2** (franka_bringup / franka_gazebo_bringup)
* **Gurobi Optimizer** (Valid license required)
* **Pinocchio** & **Eigen3**
* **libusb-1.0-0-dev** (For WSL2 direct gamepad support)

### Building the Workspace

Building in `Release` mode is strictly required to ensure the optimization solvers execute within the required real-time hardware bounds.

```bash
cd ~/franka_ros2_ws/

# Clean old artifacts to avoid CMake cache collisions
rm -rf build/lai_franka_controllers install/lai_franka_controllers

# Build in Release Mode
colcon build --packages-select lai_franka_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

```

---

## Usage Guide

### Step 1: Registering Controllers in `franka_bringup`

Before launching the robot or simulation, ROS 2 needs to know these controllers exist. Add them to `controllers.yaml` located in `franka_bringup/config/` (for real hardware) or `franka_gazebo_bringup/config/` (for simulation).

```yaml
joint_pd_velocity_controller:
  ros__parameters:
    type: lai_franka_controllers/JointPdVelocityController

hqp_cartesian_controller:
  ros__parameters:
    type: lai_franka_controllers/HqpCartesianVelocityController

cartesian_controller:
  ros__parameters:
    type: lai_franka_controllers/CartesianVelocityController

```

### Step 2: Launch the Robot / Simulation

Launch the Franka environment with the modern `joint_pd_velocity_controller` active.

**For Real Hardware:**

```bash
ros2 launch franka__bringup example.launch.py \
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

### Scenario A: Low-Level Hardware Diagnostics

To verify the PD controller is tracking perfectly before adding the HQP layer, launch the physical diagnostic node:

```bash
# Bypasses HQP, sends sine waves directly to hardware
ros2 run lai_franka_controllers joint_pd_test_node

```

### Scenario B: High-Level HQP Joint Tracking

To verify the HQP solver correctly optimizes feedforward velocities and respects joint limits. *(Ensure `primary_task.mode` is set to `"joint"` in your YAML).*

**Terminal 1:** Start the HQP Virtual Model Solver

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

**Terminal 2:** Start the HQP Trajectory Generator

```bash
# Streams position and velocity target trajectories into the HQP solver
ros2 run lai_franka_controllers hqp_joint_trajectory_test_node

```

### Scenario C: Automated Cartesian Trajectories (HQP + Quintic Planner)

The standard usage for Cartesian operations. *(Ensure `primary_task.mode` is set to `"cartesian"` in your YAML).*

**Terminal 1:** Start the HQP Virtual Model Solver

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

**Terminal 2:** Start the Trajectory Generator

```bash
ros2 run lai_franka_controllers trajectory_generator_node

```

* **Send a Goal:** Publish to `/goal_pose`. The node generates a smooth quintic trajectory.
* **Stress Test:** Publish a goal with a negative Z-value (`z < 0.0`) to trigger the automated multi-waypoint boundary stress test.

### Scenario D: Gamepad Teleoperation (HQP + Joy)

Safely drive the robot around with an Xbox/Logitech controller. The HQP node will automatically stop you from crashing into virtual walls or self-colliding.

**Terminal 1:** Start the HQP Virtual Model Solver

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

**Terminal 2:** Start the Teleoperation Suite

```bash
ros2 launch lai_franka_controllers joy_teleop.launch.py

```

*(**Note:** If you are running inside WSL2 and standard `/dev/input/js0` fails, open `joy_teleop.launch.py` and uncomment the `raw_usb_joy_node` section to enable direct USB polling).*

---

## Diagnostics & Monitoring

The HQP Reference generator streams lock-free diagnostics for visualization. Open PlotJuggler to monitor system health:

```bash
ros2 run plotjuggler plotjuggler

```

**Key Diagnostic Topics:**

* `~/hqp_reference_generator_node/tracking_error` *(Twist error between virtual and physical)*
* `~/hqp_reference_generator_node/virtual_wall_distances` *(Array of distances to the 6D box)*
* `~/hqp_reference_generator_node/self_hits_distances` *(Distance between EE and Base)*
* `~/hqp_reference_generator_node/current_pose` *(Current cartesian pose of the EE)*
* `/joint_pd_velocity_controller/joint_commands` *(Commands sent to PD controller)*
* `/joint_pd_velocity_controller/output_dq_cmd` *(Actual commands sent to hardware)*