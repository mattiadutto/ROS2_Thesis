cmake_minimum_required(VERSION 3.5)
project(robot_path)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(pose_to_path src/pose_to_path.cpp)

ament_target_dependencies(pose_to_path rclcpp geometry_msgs nav_msgs)

install(TARGETS
  pose_to_path
  DESTINATION lib/${PROJECT_NAME})

ament_package()
