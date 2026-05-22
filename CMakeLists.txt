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
# #GUROBI
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
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
add_executable(joy_teleop_node src/joy_teleop_node.cpp)
add_executable(raw_usb_joy_node src/raw_usb_joy_node.cpp)

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

# Link ROS 2 dependencies (Standard ROS 2 Macro)
# This handles the headers and linking for all ROS-based packages correctly.
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

# Link External Math Libraries
# We do NOT use "PUBLIC" or "PRIVATE" here to match the 
# "plain" signature used by ament_target_dependencies above.
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

# Plugin Registration and Installation
pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(TARGETS raw_usb_joy_node joy_teleop_node trajectory_generator_node DESTINATION lib/${PROJECT_NAME})
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()