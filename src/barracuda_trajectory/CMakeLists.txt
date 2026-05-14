cmake_minimum_required(VERSION 3.8)
project(barracuda_trajectory)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
# For MoveIt TrajOpt integration
find_package(nvblox_msgs REQUIRED)

add_executable(trajectory_generator src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator rclcpp nav_msgs geometry_msgs trajectory_msgs std_msgs nav2_msgs nav2_costmap_2d nvblox_msgs)

install(TARGETS trajectory_generator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
