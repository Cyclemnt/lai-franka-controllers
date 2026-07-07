# Robot Kinematics

A standalone ROS 2 library providing analytical kinematics for the Franka Emika FR3 manipulator.

This package serves as the mathematical foundation for real-time Cartesian controllers and Hierarchical Quadratic Programming (HQP) optimization stacks. It wraps raw, C-style code-generated Modified Denavit-Hartenberg (mDH) arrays into a clean, modern `Eigen3` C++ interface.

## Features

* **Real-Time Optimized:** Designed to run strictly inside 1 kHz control loops. Avoids dynamic memory allocation by mapping pre-allocated contiguous arrays directly into `Eigen::Map` blocks.
* **Direct Kinematics:** Fast evaluation of End-Effector (EE) position, orientation (quaternions), and full 6D poses.
* **Analytical Jacobians:** Computes the full 6x7 base Jacobian and sub-chain linear Jacobians for arbitrary joint indices (critical for self-collision avoidance).
* **Tracking Metrics:** Native `getError()` computation utilizing shortest-path quaternion math to prevent orientation wind-up.

## Package Structure

This package exports a single core C++ library (`robot_kinematics`).

* **`FrankaKinematics`**: The primary Object-Oriented wrapper class.
* **`DirectKinematics_mDH`**: Internal C-style codegen engine for forward kinematics.
* **`Jacobian_mDH`**: Internal C-style codegen engine for Jacobian matrices.
* **`Homogeneous_mDH`**: Internal transform mapping logic.

## Dependencies

* **ROS 2 Humble**
* **Eigen3**
* **ament_cmake**

## Usage in other ROS 2 Packages

This package is designed as a pure library. It does not contain any executables or ROS 2 nodes. To use the `FrankaKinematics` class in your own controller or task package, update your `CMakeLists.txt` and `package.xml`.

### 1. `package.xml`

Add the dependency to your downstream package:

```xml
<depend>robot_kinematics</depend>

```

### 2. `CMakeLists.txt`

Find the package and link it to your target. The modern export definitions in this package mean you only need to link it via the `ament` macro-header paths and Eigen dependencies are inherited automatically.

```cmake
find_package(robot_kinematics REQUIRED)

add_library(my_controller_lib src/my_controller.cpp)

# Automatically links the shared library and includes the headers
ament_target_dependencies(my_controller_lib
  rclcpp
  robot_kinematics
)

```

### 3. C++ Implementation Example

```cpp
#include "robot_kinematics/FrankaKinematics.hpp"
#include <Eigen/Dense>
#include <vector>

// 1. Instantiate the wrapper
FrankaKinematics kinematics_engine;

// 2. Load physical robot parameters
std::vector<double> mDH_table = { /* 28 element array */ };
std::vector<double> A7e_matrix = { /* 16 element transform */ };
kinematics_engine.setParameters(mDH_table, A7e_matrix);

// 3. Update state inside the real-time loop
Eigen::VectorXd current_q = Eigen::VectorXd::Zero(7);
kinematics_engine.updateJointStates(current_q);

// 4. Extract real-time kinematics directly as Eigen objects
Eigen::MatrixXd J = kinematics_engine.getJacobian();
Eigen::Vector3d ee_pos = kinematics_engine.getPosition();

```

## Build Instructions

Because this library performs heavy matrix multiplications, it is highly recommended to compile it in `Release` mode to enable `-O3` compiler optimizations.

```bash
cd ~/franka_ros2_ws/

# Clean old artifacts
rm -rf build/robot_kinematics install/robot_kinematics

# Build the kinematics library
colcon build --packages-select robot_kinematics --cmake-args -DCMAKE_BUILD_TYPE=Release

```