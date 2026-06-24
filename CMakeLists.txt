cmake_minimum_required(VERSION 3.8)
project(my_franka_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(task REQUIRED)
find_package(qp REQUIRED)
find_package(hierarchical_qp REQUIRED)
find_package(robot_kinematics REQUIRED)
find_package(auxiliaries_function_and_structures REQUIRED)
find_package(OpenMP REQUIRED)
find_package(my_franka_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB1 REQUIRED libusb-1.0)

# GUROBI
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(GUROBI REQUIRED)
if(GUROBI_FOUND)
  message(STATUS "GUROBI FOUND")
else(GUROBI_FOUND)
  message(FATAL_ERROR "GUROBI NOT FOUND!")
endif(GUROBI_FOUND)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/../qp)
link_directories(/home/cyclement/franka_ros2_ws/install/qp/lib)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/cartesian_velocity_controller.cpp
  src/hqp_cartesian_velocity_controller.cpp
  src/joint_pd_velocity_controller.cpp
)

# Include directories for library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Declare Executables (Standalone Nodes)
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
add_executable(joy_teleop_node src/joy_teleop_node.cpp)
add_executable(raw_usb_joy_node src/raw_usb_joy_node.cpp)
add_executable(joint_sine_publisher_node src/joint_sine_publisher_node.cpp)
add_executable(hqp_reference_generator_node src/hqp_reference_generator_node.cpp)

# Include directories for Executables
target_include_directories(trajectory_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(joy_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(raw_usb_joy_node PRIVATE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${LIBUSB1_INCLUDE_DIRS}
)

target_include_directories(joint_sine_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(hqp_reference_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS 2 dependencies (Standard ROS 2 Macro)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  geometry_msgs
  sensor_msgs
  realtime_tools
  pluginlib
  task
  qp
  hierarchical_qp
  robot_kinematics
  auxiliaries_function_and_structures
  my_franka_msgs
)

ament_target_dependencies(trajectory_generator_node
  rclcpp
  geometry_msgs
  tf2_ros 
  tf2_eigen 
  Eigen3
)

ament_target_dependencies(joy_teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_eigen
  Eigen3
)

ament_target_dependencies(raw_usb_joy_node rclcpp sensor_msgs)

ament_target_dependencies(joint_sine_publisher_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(hqp_reference_generator_node
  rclcpp
  geometry_msgs
  sensor_msgs
  task
  qp
  hierarchical_qp
  robot_kinematics
  auxiliaries_function_and_structures
  my_franka_msgs
)

# Link External Math Libraries & Internal Project References
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  pinocchio::pinocchio
  ${qp_LIBRARIES}
  ${task_LIBRARIES}
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
)

target_link_libraries(trajectory_generator_node
  ${PROJECT_NAME}
)

target_link_libraries(joy_teleop_node
  ${PROJECT_NAME}
)

target_link_libraries(raw_usb_joy_node ${LIBUSB1_LIBRARIES})

target_link_libraries(joint_sine_publisher_node
  ${PROJECT_NAME}
)

target_link_libraries(hqp_reference_generator_node
  Eigen3::Eigen
  ${qp_LIBRARIES}
  ${task_LIBRARIES}
  ${GUROBI_CXX_LIBRARY}
  ${GUROBI_LIBRARY}
)

# Plugin Registration and Installation
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(TARGETS 
  raw_usb_joy_node 
  joy_teleop_node 
  trajectory_generator_node 
  joint_sine_publisher_node
  hqp_reference_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()