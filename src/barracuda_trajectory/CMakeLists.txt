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
find_package(nvblox_msgs REQUIRED)

add_library(trajectory_generator_node src/trajectory_generator_node.cpp)
target_include_directories(trajectory_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  trajectory_generator_node
  rclcpp
  nav_msgs
  geometry_msgs
  trajectory_msgs
  std_msgs
  nav2_msgs
  nav2_costmap_2d
  nvblox_msgs)

add_executable(trajectory_generator src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator trajectory_generator_node)
ament_target_dependencies(trajectory_generator rclcpp)

install(TARGETS trajectory_generator trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
