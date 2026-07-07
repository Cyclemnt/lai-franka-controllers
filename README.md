# LAI Franka ROS 2 Controllers

This package implements an advanced suite of real-time controllers, Hierarchical Quadratic Programming (HQP) safety filters, and teleoperation nodes for the Franka Emika FR3 manipulator using the `ros2_control` framework.

This repository recently transitioned to a **decoupled Internal Model Control (IMC) architecture**. Instead of calculating complex optimizations inside the hardware loop, mathematical bounds (virtual walls, self-collision, kinematics) are resolved at 500 Hz inside an isolated virtual reference generator. Safe trajectories are then streamed to a real-time Joint PD Feedforward controller running strictly at 1000 Hz.

## Package Structure

### 1. `ros2_control` Hardware Plugins (1000 Hz)

* **`joint_pd_velocity_controller.hpp`**: *(Recommended)* Joint-space tracking controller with velocity feedforward and timeout watchdogs. Designed to pair with the HQP Reference Generator.
* **`hqp_cartesian_velocity_controller.hpp`**: *(Legacy)* Cartesian controller calculating Gurobi HQP optimization directly inside the 1kHz loop.
* **`cartesian_velocity_controller.hpp`**: *(Legacy)* Pinocchio-based Jacobian pseudo-inverse controller with Levenberg-Marquardt damping and nullspace limits repulsion.

### 2. Standalone Processing Nodes

* **`hqp_reference_generator_node.hpp`**: The mathematical core. Solves strict-priority HQP constraints (joint limits, self-collision, virtual 6D bounding boxes) using Gurobi over a virtual robot model.
* **`trajectory_generator_node.hpp`**: Quintic S-curve Cartesian path planner with an automated HQP boundary "Stress Test".
* **`joint_sine_publisher_node.hpp`**: Diagnostic tool that publishes smooth, zero-started sinusoidal waves to test low-level joint tracking performance.

### 3. Teleoperation Suite

* **`joy_teleop_node.hpp`**: Translates raw gamepad inputs into smooth Cartesian velocity integrations with anti-windup leashes and asynchronous Franka gripper action hooks.
* **`raw_usb_joy_node.hpp`**: WSL2-compatible direct `libusb` interrupt reader that bypasses missing Linux joystick drivers in Windows VMs.

### 4. Configuration & Launch

* **`hqp_node_params.yaml` / `hqp_node.launch.py`**: Configures virtual walls, DH parameters, safety margins, and starts the HQP solver.
* **`joy_teleop_params.yaml` / `joy_teleop.launch.py`**: Configures max velocities, acceleration ramps, gamepad deadzones, and gripper forces.

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

Before launching the robot or simulation, ROS 2 needs to know these controllers exist. You must add them to the `controllers.yaml` file located in `franka_bringup/config/` (for real hardware) or `franka_gazebo_bringup/config/` (for simulation).

Add the following to your `controllers.yaml`:

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
    robot_ip:=172.16.0.2" \
    load_gripper:=true

```

**For Gazebo Simulation:**

```bash
ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py \
    controller:=joint_pd_velocity_controller \
    gz_args:="-r -s empty.sdf"

```

*(Note: To use the legacy controllers, replace `joint_pd_velocity_controller` with `my_hqp_cartesian_controller` or `my_cartesian_controller` in the commands above).*

---

### Scenario A: Diagnostic Hardware Test (Sine Wave)

To verify that the PD controller is tracking perfectly before using Cartesian commands, launch the diagnostic sine publisher:

```bash
# Ensure the robot is launched with joint_pd_velocity_controller
ros2 run lai_franka_controllers joint_sine_publisher_node

```

---

### Scenario B: Automated Cartesian Trajectories (HQP + Quintic Planner)

This is the standard usage. The HQP node generates safe references, and the trajectory generator feeds it goals.

**Terminal 1: Start the HQP Virtual Model Solver**

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

*(This loads boundaries from `hqp_node_params.yaml` and connects to the PD controller).*

**Terminal 2: Start the Trajectory Generator**

```bash
ros2 run lai_franka_controllers trajectory_generator_node

```

* **Send a Goal:** Publish to `/goal_pose`. The node generates a smooth quintic trajectory.
* **Stress Test:** Publish a goal with a negative Z-value (`z < 0.0`) to trigger the automated multi-waypoint HQP boundary stress test sequence.

---

### Scenario C: Gamepad Teleoperation (HQP + Joy)

Safely drive the robot around with an Xbox/Logitech controller. The HQP node will automatically stop you from crashing into virtual walls or self-colliding.

**Terminal 1: Start the HQP Virtual Model Solver**

```bash
ros2 launch lai_franka_controllers hqp_node.launch.py

```

**Terminal 2: Start the Teleoperation Suite**

```bash
ros2 launch lai_franka_controllers joy_teleop.launch.py

```

*(This loads configurations from `joy_teleop_params.yaml`. **Note:** If you are running inside WSL2 and standard `/dev/input/js0` fails, open `joy_teleop.launch.py` and uncomment the `raw_usb_joy_node` section to enable direct USB polling).*

**Gamepad Controls:**

* **Left Stick:** Cartesian X / Y Translation
* **Triggers (LT/RT):** Cartesian Z Translation
* **Right Stick:** Cartesian Pitch / Roll
* **Bumpers (LB/RB):** Cartesian Yaw
* **D-Pad Left/Right:** Live Speed Scaling (5% to 100%)
* **A Button:** Toggle Franka Gripper (Grasp / Open)

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
* `/joint_pd_velocity_controller/output_dq_cmd` *(Actual commands sent to hardware)*