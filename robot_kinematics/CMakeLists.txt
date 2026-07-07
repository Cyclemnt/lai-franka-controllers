cmake_minimum_required(VERSION 3.8)
project(robot_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ---- Find Core Workspace Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ==========================================
# TARGET 1: CORE ROBOT KINEMATICS LIBRARY
# ==========================================
# Kept as robot_kinematics_lib for workspace backward compatibility
add_library(robot_kinematics_lib SHARED
  src/Homogeneous_mDH.cpp
  src/DirectKinematics_mDH.cpp
  src/Jacobian_mDH.cpp
  src/FrankaKinematics.cpp
)

# Enforce C++17 compilation features cleanly scoped to this target
target_compile_features(robot_kinematics_lib PUBLIC cxx_std_17)

target_include_directories(robot_kinematics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(robot_kinematics_lib
  Eigen3::Eigen
)

ament_target_dependencies(robot_kinematics_lib
  rclcpp
)

# ==========================================
# INSTALLATION AND EXPORT CONFIGURATIONS
# ==========================================
install(DIRECTORY include/ DESTINATION include)

install(TARGETS robot_kinematics_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export macros ensure workspace packages view the library safely
ament_export_include_directories(include)
ament_export_libraries(robot_kinematics_lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

ament_package()