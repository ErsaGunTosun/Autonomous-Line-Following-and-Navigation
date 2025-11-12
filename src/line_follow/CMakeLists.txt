cmake_minimum_required(VERSION 3.8)
project(line_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(turtlebot3_description REQUIRED)
find_package(xacro REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)


ament_export_include_directories(include)

ament_export_dependencies(
  rclcpp
  nav2_msgs
  geometry_msgs
  rclcpp_action
  std_srvs
  builtin_interfaces
)

include_directories(
  include
)

add_executable(line_follow_node src/line_follow_node.cpp)
ament_target_dependencies(line_follow_node 
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  OpenCV
  slam_toolbox
  tf2
  tf2_geometry_msgs
)

# Goal Publisher Node
# add_executable(goal_publisher_node src/goal_publisher_node.cpp)
# ament_target_dependencies(goal_publisher_node
#   rclcpp
#   geometry_msgs
#   nav2_msgs
#   rclcpp_action
#   tf2
#   tf2_geometry_msgs
# )

add_executable(waypoint_selector_node src/waypoint_selector_node.cpp)
ament_target_dependencies(waypoint_selector_node
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
  tf2
  tf2_geometry_msgs
)

add_executable(green_point_navigator_node src/green_point_navigator_node.cpp)
ament_target_dependencies(green_point_navigator_node
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  line_follow_node
  # goal_publisher_node
  waypoint_selector_node
  green_point_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models maps config urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
