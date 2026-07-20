cmake_minimum_required(VERSION 3.8)
project(lai_franka_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# ---- Find Core ROS 2 and Client Dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# ---- Find Message Dependencies ----
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(franka_msgs REQUIRED)

# ---- Native Message Generation Dependency ----
find_package(rosidl_default_generators REQUIRED)

# ---- Find Math & External Solver Components ----
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(task REQUIRED)
find_package(qp REQUIRED)
find_package(hierarchical_qp REQUIRED)
find_package(robot_kinematics REQUIRED)
find_package(auxiliaries_function_and_structures REQUIRED)
find_package(OpenMP REQUIRED)

# ---- Ingest System Level USB Interfaces ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 REQUIRED libusb-1.0)

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

set(PACKAGE_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==========================================
# GENERATE NATIVE MESSAGES
# ==========================================
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE INTERNAL "Forcing legacy Python path for rosidl")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HqpDistances.msg"
  DEPENDENCIES std_msgs
)

# ==========================================
# TARGET 1: CONTROLLER HARDWARE PLUGIN LIBRARY
# ==========================================
add_library(${PROJECT_NAME}_lib SHARED
  src/joint_pd_velocity_controller.cpp
  src/hqp_cartesian_velocity_controller.cpp
  src/cartesian_velocity_controller.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC ${PACKAGE_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_lib
  Eigen3::Eigen
  pinocchio::pinocchio
  ${qp_LIBRARIES}
  ${task_LIBRARIES}
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
)

# Bind internal custom message definitions typesupport frames
rosidl_get_typesupport_target(msg_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib ${msg_typesupport_target})

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  controller_interface
  hardware_interface
  geometry_msgs
  sensor_msgs
  std_msgs
  realtime_tools
  pluginlib
  task
  qp
  hierarchical_qp
  robot_kinematics
  auxiliaries_function_and_structures
)

# ==========================================
# STANDALONE EXECUTABLES
# ==========================================

# Trajectory Generator
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
target_include_directories(trajectory_generator_node PUBLIC ${PACKAGE_INCLUDE_DIRS})
target_link_libraries(trajectory_generator_node ${PROJECT_NAME}_lib)
ament_target_dependencies(trajectory_generator_node rclcpp geometry_msgs tf2_ros tf2_eigen Eigen3)

# Joystick Teleoperation Processing Engine
add_executable(joy_teleop_node src/joy_teleop_node.cpp)
target_include_directories(joy_teleop_node PUBLIC ${PACKAGE_INCLUDE_DIRS})
target_link_libraries(joy_teleop_node ${PROJECT_NAME}_lib)
ament_target_dependencies(joy_teleop_node rclcpp rclcpp_action geometry_msgs sensor_msgs franka_msgs tf2_ros tf2_eigen Eigen3)

# Direct Libusb Interrupt Poller
add_executable(raw_usb_joy_node src/raw_usb_joy_node.cpp)
target_include_directories(raw_usb_joy_node PRIVATE ${PACKAGE_INCLUDE_DIRS} ${LIBUSB1_INCLUDE_DIRS})
target_link_libraries(raw_usb_joy_node ${LIBUSB1_LIBRARIES})
ament_target_dependencies(raw_usb_joy_node rclcpp sensor_msgs)

# Diagnostics Waveform Publisher
add_executable(joint_sine_publisher_node src/joint_sine_publisher_node.cpp)
target_include_directories(joint_sine_publisher_node PUBLIC ${PACKAGE_INCLUDE_DIRS})
target_link_libraries(joint_sine_publisher_node ${PROJECT_NAME}_lib)
ament_target_dependencies(joint_sine_publisher_node rclcpp sensor_msgs)

# Core HQP Reference Optimization Node
add_executable(hqp_reference_generator_node src/hqp_reference_generator_node.cpp)
target_include_directories(hqp_reference_generator_node PUBLIC ${PACKAGE_INCLUDE_DIRS})
target_link_libraries(hqp_reference_generator_node ${PROJECT_NAME}_lib ${msg_typesupport_target})
ament_target_dependencies(hqp_reference_generator_node rclcpp geometry_msgs sensor_msgs std_msgs task qp hierarchical_qp robot_kinematics auxiliaries_function_and_structures)

# Core HQP Position Test Node (100 Hz Test)
add_executable(hqp_position_test_node src/hqp_position_test_node.cpp)
target_include_directories(hqp_position_test_node PUBLIC ${PACKAGE_INCLUDE_DIRS})
target_link_libraries(hqp_position_test_node ${PROJECT_NAME}_lib ${msg_typesupport_target})
ament_target_dependencies(hqp_position_test_node rclcpp geometry_msgs sensor_msgs std_msgs task qp hierarchical_qp robot_kinematics auxiliaries_function_and_structures)

# HQP Joint Trajectory Test Node
add_executable(hqp_joint_trajectory_test_node src/hqp_joint_trajectory_test_node.cpp)
target_include_directories(hqp_joint_trajectory_test_node PRIVATE include)
ament_target_dependencies(hqp_joint_trajectory_test_node rclcpp sensor_msgs)

# ==========================================
# PLUGIN REGISTRATION AND INSTALLATION BLOCKS
# ==========================================
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  raw_usb_joy_node 
  joy_teleop_node 
  trajectory_generator_node 
  joint_sine_publisher_node
  hqp_reference_generator_node
  hqp_position_test_node
  hqp_joint_trajectory_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()