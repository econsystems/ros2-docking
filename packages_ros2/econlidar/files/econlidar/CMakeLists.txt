cmake_minimum_required(VERSION 3.8)
project(econlidar)

# Enable C++14 or higher (ROS 2 typically requires at least C++14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Executable: battery_monitor_and_navigate_docking
add_executable(battery_monitor_and_navigate_docking src/battery_monitor_and_navigate_docking.cpp)
target_link_libraries(battery_monitor_and_navigate_docking yaml-cpp)
ament_target_dependencies(battery_monitor_and_navigate_docking
  rclcpp
  nav2_msgs
  rclcpp_action
  yaml-cpp
  std_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  battery_monitor_and_navigate_docking
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory if exists
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
