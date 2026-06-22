# My Franka ROS 2 Controllers

This package implements an advanced suite of real-time Cartesian motion controllers and trajectory planning nodes for the Franka Emika FR3 manipulator using the ros2_control framework. 

The flagship implementation is a strict-priority Hierarchical Quadratic Programming (HQP) controller. It ensures mathematically guaranteed safety boundaries (joint limits, self-collision, virtual walls) while actively optimizing Cartesian tracking directly within the 1 kHz hardware communication loop.

## Features

- **Real-Time HQP Architecture**: Solves complex, priority-based optimizations (via Gurobi) at 1000 Hz to safely bound commands before they reach the physical hardware.
- **Strict Safety Priorities**: 
  1. Joint Configuration & Velocity Limits (Hard constraints)
  2. Self-Collision Avoidance (Hard constraints)
  3. Virtual Workspace Walls (Hard constraints)
  4. Cartesian Pose Tracking (Soft constraints)
- **Quintic Trajectory Generation**: Standalone node for generating smooth, synchronized S-curve motion profiles. Includes an automated "Stress Test" sequence to validate HQP boundaries.
- **WSL2 Native Teleoperation**: Custom libusb endpoint reader to bypass WSL2's missing joystick driver support, allowing direct teleoperation of the end-effector via Xbox/Logitech gamepads.

## Package Structure

- **src/hqp_cartesian_velocity_controller.cpp**: Core 1kHz HQP controller plugin.
- **src/cartesian_velocity_controller.cpp**: Legacy CLIK velocity controller using Pinocchio.
- **src/trajectory_generator_node.cpp**: Quintic S-curve Cartesian path planner.
- **src/joy_teleop_node.cpp**: Maps gamepad inputs to Cartesian velocity integrations.
- **src/raw_usb_joy_node.cpp**: WSL2-compatible direct libusb gamepad driver.

## Dependencies

- **ROS 2 Humble**
- **libfranka** & **franka_ros2**
- **Gurobi Optimizer** (Valid license required)
- **libusb-1.0-0-dev** (For WSL gamepad node)
- **Pinocchio** (For legacy CLIK controller/kinematics)

## Installation & Build Instructions

Building in Release mode is strictly required to ensure the Gurobi optimization solver can complete its cascade within the 1 ms hardware timing constraint.

```bash
cd ~/franka_ros2_ws/

# Optional: Build the whole workspace with symlinks for fast iteration
colcon build --symlink-install

# Mandatory: Build the controllers package in Release mode
colcon build --packages-select my_franka_controllers --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

```

## Usage Guide

### 1. Launch the Simulation (Gazebo)

Start the headless Gazebo simulation with the HQP controller activated:

```bash
ros2 launch franka_gazebo_bringup gazebo_franka_arm_example_controller.launch.py \
    controller:=my_hqp_cartesian_controller \
    gz_args:="-r -s empty.sdf"

```

*(To use the legacy unconstrained controller, replace my_hqp_cartesian_controller with my_cartesian_controller)*

### 2. Send Target Poses (CLI)

You can send direct Cartesian targets to the controller. The HQP will automatically cap velocities and respect virtual walls to reach these goals safely:

```bash
ros2 topic pub -1 /my_hqp_cartesian_controller/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.6, y: 0.0, z: 0.4}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"

ros2 topic pub -1 /my_hqp_cartesian_controller/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.306889, y: 0.0, z: 0.590272}, orientation: {x: 0.923879, y: -0.382684, z: 0.0, w: 0.0}}}"

```

### 3. Smooth Trajectory Generation

Instead of step-inputs via CLI, use the generator to feed smooth S-curves to the controller:

```bash
ros2 run my_franka_controllers trajectory_generator_node

```

* **Manual Goals**: Publish standard goals to `/goal_pose`. The node will generate a quintic trajectory from the current EE position to the goal.
* **Stress Test Sequence**: Sending a goal with a negative Z-value (`z < 0.0`) triggers an automated multi-waypoint stress test designed to dynamically test the HQP limits.

### 4. Gamepad Teleoperation (WSL2)

If running inside WSL2 where standard `/dev/input/js0` is unavailable, use the raw USB node alongside the teleop integration node:

```bash
# Run the raw USB reader (requires Logitech F310/F710 in X-Input mode)
ros2 run my_franka_controllers raw_usb_joy_node

# Run the teleop integration node in a separate terminal
ros2 run my_franka_controllers joy_teleop_node

```

## Diagnostics & Monitoring

The controller runs lock-free real-time publishers to stream state constraints. You can record these for analysis:

```bash
ros2 bag record /my_hqp_cartesian_controller/target_pose \
                /my_hqp_cartesian_controller/tracking_error \
                /my_hqp_cartesian_controller/dq_cmd \
                /tf \
                /joint_states

```

To visualize the tracking errors, wall distances, and self-collision avoidance in real-time, launch PlotJuggler:

```bash
ros2 run plotjuggler plotjuggler

```

**Key diagnostic topics to monitor:**

* `/my_hqp_cartesian_controller/tracking_error`
* `/my_hqp_cartesian_controller/virtual_wall_distances`
* `/my_hqp_cartesian_controller/self_hits_distances`
