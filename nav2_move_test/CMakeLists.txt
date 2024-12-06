cmake_minimum_required(VERSION 3.5)
project(nav2_move_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(motion_action_client src/motion_controller.cpp)
ament_target_dependencies(motion_action_client 
  rclcpp 
  geometry_msgs 
  nav2_msgs 
  rclcpp_action 
  nav_msgs 
  tf2_ros 
  tf2_geometry_msgs
)

install(TARGETS motion_action_client
  DESTINATION lib/${PROJECT_NAME})

add_executable(cmd_vel_controller src/cmd_vel_controller.cpp)
ament_target_dependencies(cmd_vel_controller 
  rclcpp 
  geometry_msgs 
  nav2_msgs 
  rclcpp_action 
  nav_msgs 
  tf2_ros 
  tf2_geometry_msgs
)

install(TARGETS cmd_vel_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
