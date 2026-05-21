#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nvblox_msgs/msg/distance_map_slice.hpp>
#include <rclcpp/rclcpp.hpp>

class TrajectoryGenerator : public rclcpp::Node {
public:
  TrajectoryGenerator();

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void costmap_callback(const nvblox_msgs::msg::DistanceMapSlice::SharedPtr msg);

  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_raw_astar_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_endpoint_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_densified_pub_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_current_pose_{false};
  bool has_goal_{false};
};
