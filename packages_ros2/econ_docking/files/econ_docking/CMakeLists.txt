cmake_minimum_required(VERSION 3.5)
project(econ_docking)

# Ensure we use C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required ROS 2 and system dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ros2_base_interfaces REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# === C++ Executables ===

# 1. ArUco marker detector
add_executable(aruco_marker_detector src/aruco_marker_detector.cpp)
ament_target_dependencies(aruco_marker_detector
  rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

# 2. ArUco marker pose estimation with TF
add_executable(aruco_marker_pose_estimation_tf src/aruco_marker_pose_estimation_tf.cpp)
ament_target_dependencies(aruco_marker_pose_estimation_tf
  rclcpp sensor_msgs std_msgs geometry_msgs cv_bridge tf2 tf2_ros OpenCV)

# 3. Navigate to charging dock with no nav2
add_executable(navigate_to_charging_dock_no_nav2 src/navigate_to_charging_dock_no_nav2.cpp)
ament_target_dependencies(navigate_to_charging_dock_no_nav2
  rclcpp rclcpp_action geometry_msgs sensor_msgs std_msgs action_msgs ros2_base_interfaces)

# 5. battery_monitor_and_docking
add_executable(battery_monitor_and_docking src/battery_monitor_and_docking.cpp)
ament_target_dependencies(battery_monitor_and_docking
  rclcpp geometry_msgs sensor_msgs std_msgs)

# === Installation ===
install(TARGETS
  aruco_marker_detector
  aruco_marker_pose_estimation_tf
  navigate_to_charging_dock_no_nav2
  battery_monitor_and_docking
  DESTINATION lib/${PROJECT_NAME}
)

# === Config and Launch ===
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
