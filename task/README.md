# LAI Task Formulation Engine

This package provides the core mathematical building blocks (Objectives and Constraints) for the strict-priority Hierarchical Quadratic Programming (HQP) controller.

It defines an abstract, polymorphic `Task` architecture that maps physical robot goals (like tracking a pose or avoiding a wall) into standardized linear algebra matrices ($A$ and $b$) which are then fed directly into the Gurobi optimization solver.

## Core Architecture

Every task inherits from the virtual `Task` base class and is responsible for formulating its objective into either an equality or inequality constraint for the solver:

* **Equality Tasks** ($A \dot{q} = b$): Used for strict tracking (e.g., reaching a specific Cartesian pose).
* **Inequality Tasks** ($A \dot{q} \le b$): Used for safety boundaries (e.g., joint limits, collision avoidance).

The package optimizes real-time performance by aggressively pre-allocating memory during instantiation and passing operations via `Eigen::noalias()` to prevent heap allocations during the 1 kHz control loop.

## Available Tasks

### Safety Constraints (Hard Priorities)

* **`JointsConfigurationLimits`**: Enforces strict joint-space boundaries to prevent the robot from exceeding its physical range of motion.
* **`JointsVelocityLimits`**: Caps actuator velocity commands to prevent hardware saturation and over-current faults.
* **`SelfHits`**: Computes relative link-to-link distances and formulates repulsion vectors to strictly prevent the robot from colliding with itself.
* **`VirtualWall`**: Defines an un-breachable planar 3D boundary in the workspace, scaling back velocity vectors before the end-effector can cross the plane.

### Operational Objectives (Soft Priorities)

* **`Pose`**: Solves 6D Cartesian tracking errors (position and/or orientation) and projects them into joint velocities. Features a built-in Damped Least Squares (DLS) singularity robustness filter.
* **`JointSineTask`**: A diagnostic trajectory generator that superimposes multi-frequency Fourier sine waves to safely excite all joints for system identification.

## Dependencies

* **ROS 2 Humble**
* **Gurobi Optimizer** (Valid system license required)
* **Eigen3** & **OpenMP**
* **robot_kinematics** (For analytical Jacobian and frame evaluations)
* **qp** (Custom solver wrapper package)

## Usage in other ROS 2 Packages

This package exports a shared library `task` containing all the pre-compiled constraint behaviors.

### 1. `package.xml`

Add the dependency to your downstream package (e.g., your controllers package):

```xml
<depend>task</depend>

```

### 2. `CMakeLists.txt`

Because this package leverages modern CMake target exports, linking to it automatically resolves the Gurobi, Eigen, and internal include paths for you:

```cmake
find_package(task REQUIRED)

add_library(my_hqp_controller src/my_controller.cpp)

# Automatically links the shared library and inherits all includes!
ament_target_dependencies(my_hqp_controller
  rclcpp
  task
)

```

### 3. C++ Implementation Example

To keep your controller headers clean, include the unified aggregator file:

```cpp
#include "task/allTasks.hpp"

using namespace task;

// 1. Instantiate a task
std::shared_ptr<VirtualWall> wall_task = std::make_shared<VirtualWall>(
    &kinematics_engine, p1, p2, p3, safe_joints_vector, margin, GRB_GREATER_EQUAL, gain
);

// 2. Evaluate the math in the real-time loop
wall_task->update();

// 3. Extract the evaluated matrices for the QP Solver
Eigen::MatrixXd A = wall_task->get_A();
Eigen::VectorXd b = wall_task->get_b();

```

## Build Instructions

To ensure the matrix multiplications resolve within the strict 1 ms hardware loop, `-O3` optimizations are enforced. Build in `Release` mode:

```bash
cd ~/franka_ros2_ws/

# Clean old artifacts
rm -rf build/task install/task

# Build the task library
colcon build --packages-select task --cmake-args -DCMAKE_BUILD_TYPE=Release

```