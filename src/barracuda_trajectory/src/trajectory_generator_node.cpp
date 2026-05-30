#include "barracuda_trajectory/trajectory_generator_node.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <utility>
#include <vector>

// Simple 2D A* node
struct AStarNode {
  int x, y;
  float g, h;
  AStarNode * parent;
  float f() const { return g + h; }
  AStarNode(int x_, int y_, float g_, float h_, AStarNode * parent_ = nullptr)
  : x(x_), y(y_), g(g_), h(h_), parent(parent_) {}
};

struct CompareAStarNode {
  bool operator()(const AStarNode * a, const AStarNode * b) const { return a->f() > b->f(); }
};

static nav_msgs::msg::Path points_to_path(
  const std::vector<std::pair<float, float>> & points,
  const std_msgs::msg::Header & header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  for (const auto & pt : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = pt.first;
    pose.pose.position.y = pt.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

// B-spline helper (degree 3, uniform)
std::vector<std::pair<float, float>> bspline(
  const std::vector<std::pair<float, float>> & points,
  int num_points)
{
  if (points.size() < 4) {
    return points;
  }
  std::vector<std::pair<float, float>> result;
  int n = static_cast<int>(points.size()) - 1;
  int k = 3;
  int m = n + k + 1;
  std::vector<float> knots(m + 1);
  for (int i = 0; i <= m; ++i) {
    knots[i] = static_cast<float>(i);
  }
  float step = (knots[m - k] - knots[k]) / static_cast<float>(num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    float u = knots[k] + static_cast<float>(i) * step;
    std::vector<std::pair<float, float>> temp = points;
    for (int d = 1; d <= k; ++d) {
      for (int j = n; j >= d; --j) {
        float alpha = (u - knots[j]) / (knots[j + k + 1 - d] - knots[j]);
        temp[j].first = (1 - alpha) * temp[j - 1].first + alpha * temp[j].first;
        temp[j].second = (1 - alpha) * temp[j - 1].second + alpha * temp[j].second;
      }
    }
    result.push_back(temp[n]);
  }
  return result;
}

TrajectoryGenerator::TrajectoryGenerator()
: Node("trajectory_generator")
{
  costmap_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
    "/barracuda/nvblox_node/static_map_slice", 10,
    std::bind(&TrajectoryGenerator::costmap_callback, this, std::placeholders::_1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/barracuda/zed_node/pose", 10,
    std::bind(&TrajectoryGenerator::pose_callback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_pose", 10,
    std::bind(&TrajectoryGenerator::goal_callback, this, std::placeholders::_1));
  traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_trajectory", 10);
  debug_raw_astar_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/debug/trajectory_raw_astar", 10);
  debug_endpoint_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/debug/trajectory_endpoint_corrected", 10);
  debug_densified_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/debug/trajectory_densified", 10);
}

void TrajectoryGenerator::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  has_current_pose_ = true;
}

void TrajectoryGenerator::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_ = *msg;
  has_goal_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "Updated target pose: x=%.2f y=%.2f z=%.2f frame=%s",
    goal_pose_.pose.position.x,
    goal_pose_.pose.position.y,
    goal_pose_.pose.position.z,
    goal_pose_.header.frame_id.c_str());
}

void TrajectoryGenerator::costmap_callback(const nvblox_msgs::msg::DistanceMapSlice::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received DistanceMapSlice, generating trajectory...");
  int width = msg->width;
  int height = msg->height;
  float resolution = msg->resolution;
  float unknown = msg->unknown_value;

  if (width < 3 || height < 3) {
    RCLCPP_WARN(this->get_logger(), "DistanceMapSlice too small (%d x %d), skipping", width, height);
    return;
  }

  const size_t expected_size = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (msg->data.size() < expected_size) {
    RCLCPP_WARN(
      this->get_logger(),
      "DistanceMapSlice data size mismatch (got %zu, expected at least %zu), skipping",
      msg->data.size(), expected_size);
    return;
  }

  if (!has_goal_) {
    RCLCPP_WARN(this->get_logger(), "No /target_pose received yet, skipping trajectory generation");
    return;
  }
  if (!has_current_pose_) {
    RCLCPP_WARN(this->get_logger(), "No current pose received yet, skipping trajectory generation");
    return;
  }
  if (!current_pose_.header.frame_id.empty() &&
    current_pose_.header.frame_id != msg->header.frame_id)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Current pose frame (%s) does not match map frame (%s), skipping trajectory generation",
      current_pose_.header.frame_id.c_str(), msg->header.frame_id.c_str());
    return;
  }
  if (!goal_pose_.header.frame_id.empty() && goal_pose_.header.frame_id != msg->header.frame_id) {
    RCLCPP_WARN(
      this->get_logger(),
      "Goal pose frame (%s) does not match map frame (%s), skipping trajectory generation",
      goal_pose_.header.frame_id.c_str(), msg->header.frame_id.c_str());
    return;
  }

  const auto world_to_cell = [msg, resolution](float wx, float wy) {
      return std::pair<int, int>{
        static_cast<int>(std::floor((wx - msg->origin.x) / resolution)),
        static_cast<int>(std::floor((wy - msg->origin.y) / resolution))
      };
    };

  const auto [start_x, start_y] = world_to_cell(
    current_pose_.pose.position.x, current_pose_.pose.position.y);
  const auto [goal_x, goal_y] = world_to_cell(
    goal_pose_.pose.position.x, goal_pose_.pose.position.y);

  if (start_x < 1 || start_y < 1 || start_x >= width - 1 || start_y >= height - 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "Current pose outside map bounds (start_x=%d start_y=%d width=%d height=%d), skipping",
      start_x, start_y, width, height);
    return;
  }
  if (goal_x < 1 || goal_y < 1 || goal_x >= width - 1 || goal_y >= height - 1) {
    RCLCPP_WARN(
      this->get_logger(),
      "Target pose outside map bounds (goal_x=%d goal_y=%d width=%d height=%d), skipping",
      goal_x, goal_y, width, height);
    return;
  }

  std::vector<std::vector<bool>> closed(width, std::vector<bool>(height, false));
  std::priority_queue<AStarNode *, std::vector<AStarNode *>, CompareAStarNode> open;
  AStarNode * start = new AStarNode(
    start_x, start_y, 0, std::hypot(goal_x - start_x, goal_y - start_y));
  open.push(start);
  AStarNode * goal = nullptr;
  std::vector<std::pair<int, int>> directions = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}, {1, -1}, {-1, -1}};
  while (!open.empty()) {
    AStarNode * curr = open.top();
    open.pop();
    if (curr->x == goal_x && curr->y == goal_y) {
      goal = curr;
      break;
    }
    if (closed[curr->x][curr->y]) {
      continue;
    }
    closed[curr->x][curr->y] = true;
    for (auto & d : directions) {
      int nx = curr->x + d.first;
      int ny = curr->y + d.second;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }
      const size_t idx =
        static_cast<size_t>(ny) * static_cast<size_t>(width) + static_cast<size_t>(nx);
      if (idx >= msg->data.size()) {
        continue;
      }
      if (msg->data[idx] == unknown || msg->data[idx] < 0.3F) {
        continue;
      }
      if (closed[nx][ny]) {
        continue;
      }
      float g = curr->g + std::hypot(static_cast<float>(d.first), static_cast<float>(d.second));
      float h = std::hypot(goal_x - nx, goal_y - ny);
      open.push(new AStarNode(nx, ny, g, h, curr));
    }
  }

  std::vector<std::pair<float, float>> path_points;
  // Keep exact pose endpoints so visualization matches incoming pose/goal.
  float start_wx = static_cast<float>(current_pose_.pose.position.x);
  float start_wy = static_cast<float>(current_pose_.pose.position.y);
  float goal_wx = static_cast<float>(goal_pose_.pose.position.x);
  float goal_wy = static_cast<float>(goal_pose_.pose.position.y);
  if (goal) {
    for (AStarNode * n = goal; n; n = n->parent) {
      float wx = msg->origin.x + static_cast<float>(n->x) * resolution;
      float wy = msg->origin.y + static_cast<float>(n->y) * resolution;
      path_points.push_back({wx, wy});
    }
    std::reverse(path_points.begin(), path_points.end());
    debug_raw_astar_pub_->publish(points_to_path(path_points, msg->header));
    if (!path_points.empty()) {
      path_points.front() = {start_wx, start_wy};
      path_points.back() = {goal_wx, goal_wy};
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "A* failed to find a path, publishing straight-line fallback");
    path_points.push_back({start_wx, start_wy});
    path_points.push_back({goal_wx, goal_wy});
  }
  debug_endpoint_path_pub_->publish(points_to_path(path_points, msg->header));

  // Use a densified polyline instead of uniform B-spline.
  // The previous spline can create non-interpolating jumps right after the first point,
  // which looks like teleportation in visualization.
  std::vector<std::pair<float, float>> smooth_points;
  if (path_points.empty()) {
    smooth_points = {{start_wx, start_wy}, {goal_wx, goal_wy}};
  } else {
    smooth_points.push_back(path_points.front());
    const float step = std::max(0.05F, resolution * 0.5F);
    for (size_t i = 1; i < path_points.size(); ++i) {
      const float x0 = path_points[i - 1].first;
      const float y0 = path_points[i - 1].second;
      const float x1 = path_points[i].first;
      const float y1 = path_points[i].second;
      const float dx = x1 - x0;
      const float dy = y1 - y0;
      const float dist = std::hypot(dx, dy);
      if (dist <= step) {
        smooth_points.push_back(path_points[i]);
        continue;
      }

      const int segments = std::max(1, static_cast<int>(std::ceil(dist / step)));
      for (int s = 1; s <= segments; ++s) {
        const float t = static_cast<float>(s) / static_cast<float>(segments);
        smooth_points.push_back({x0 + t * dx, y0 + t * dy});
      }
    }
  }

  if (!smooth_points.empty()) {
    smooth_points.front() = {start_wx, start_wy};
    smooth_points.back() = {goal_wx, goal_wy};
  }
  debug_densified_pub_->publish(points_to_path(smooth_points, msg->header));
  nav_msgs::msg::Path path = points_to_path(smooth_points, msg->header);
  traj_pub_->publish(path);
  RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu poses", path.poses.size());
}
