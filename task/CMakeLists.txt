cmake_minimum_required(VERSION 3.8)
project(task)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ---- Find Core ROS 2 and System Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- Find Custom Workspace Underlay Dependencies ----
find_package(auxiliaries_function_and_structures REQUIRED)
find_package(robot_kinematics REQUIRED)
find_package(qp REQUIRED)

# ---- GUROBI ----
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(GUROBI REQUIRED)
if(GUROBI_FOUND)
  message(STATUS "GUROBI OPTIMIZATION ENGINE FOUND")
else()
  message(FATAL_ERROR "GUROBI OPTIMIZATION ENGINE NOT FOUND!")
endif()

# ---- Portable Link Overlay Setup ----
if(DEFINED ENV{COLCON_PREFIX_PATH})
  link_directories($ENV{COLCON_PREFIX_PATH}/qp/lib)
endif()

# ==========================================
# TARGET 1: THE CORE TASK ENGINE LIBRARY
# ==========================================
add_library(${PROJECT_NAME} SHARED
  src/task.cpp
  src/jointsConfigurationLimits.cpp
  src/jointsVelocityLimits.cpp
  src/selfHits.cpp
  src/virtualWall.cpp
  src/pose.cpp
  src/jointSineTask.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
)

# Removed robot_kinematics_lib from here to let ament handle it
target_link_libraries(${PROJECT_NAME}
  qpsolver_lib
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  auxiliaries_function_and_structures
  robot_kinematics
  qp
)

# ==========================================
# TARGET 2: INTERACTION TEST NODE EXECUTABLE
# ==========================================
add_executable(test_task_node src/test_node_main.cpp)

target_include_directories(test_task_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${GUROBI_INCLUDE_DIRS}
)

target_link_libraries(test_task_node
  ${PROJECT_NAME}
  qpsolver_lib
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
)

ament_target_dependencies(test_task_node
  rclcpp
  qp
)

# ==========================================
# INSTALLATION AND EXPORT CONFIGURATIONS
# ==========================================
install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS test_task_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Export definitions to share include trees and objects downstream safely
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 auxiliaries_function_and_structures robot_kinematics qp)

ament_package()