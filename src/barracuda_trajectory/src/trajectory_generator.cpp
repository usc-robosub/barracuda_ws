// trajectory_generator.cpp
// Basic ROS 2 node skeleton for trajectory generation from a costmap

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nvblox_msgs/msg/distance_map_slice.hpp>
#include <vector>
#include <queue>
#include <cmath>

// Simple 2D A* node
struct AStarNode {
  int x, y;
  float g, h;
  AStarNode* parent;
  float f() const { return g + h; }
  AStarNode(int x_, int y_, float g_, float h_, AStarNode* parent_ = nullptr)
    : x(x_), y(y_), g(g_), h(h_), parent(parent_) {}
};

struct CompareAStarNode {
  bool operator()(const AStarNode* a, const AStarNode* b) const { return a->f() > b->f(); }
};

// B-spline helper (degree 3, uniform)
std::vector<std::pair<float, float>> bspline(const std::vector<std::pair<float, float>>& points, int num_points) {
  if (points.size() < 4) return points;
  std::vector<std::pair<float, float>> result;
  int n = points.size() - 1;
  int k = 3;
  int m = n + k + 1;
  std::vector<float> knots(m + 1);
  for (int i = 0; i <= m; ++i) knots[i] = i;
  float step = (knots[m - k] - knots[k]) / (num_points - 1);
  for (int i = 0; i < num_points; ++i) {
    float u = knots[k] + i * step;
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

class TrajectoryGenerator : public rclcpp::Node {
public:
  TrajectoryGenerator() : Node("trajectory_generator") {
    // Subscribe to nvblox DistanceMapSlice
    costmap_sub_ = this->create_subscription<nvblox_msgs::msg::DistanceMapSlice>(
      "/barracuda/nvblox_node/static_map_slice", 10,
      std::bind(&TrajectoryGenerator::costmap_callback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_pose", 10,
      std::bind(&TrajectoryGenerator::goal_callback, this, std::placeholders::_1));
    // Publisher for trajectory
    traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_trajectory", 10);
  }

private:
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

  void costmap_callback(const nvblox_msgs::msg::DistanceMapSlice::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received DistanceMapSlice, generating trajectory...");
    // Extract map info
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

    int start_x = 1, start_y = 1; // Example start (could use robot pose)

    if (!has_goal_) {
      RCLCPP_WARN(this->get_logger(), "No /target_pose received yet, skipping trajectory generation");
      return;
    }

    int goal_x = static_cast<int>((goal_pose_.pose.position.x - msg->origin.x) / resolution);
    int goal_y = static_cast<int>((goal_pose_.pose.position.y - msg->origin.y) / resolution);
    if (goal_x < 1 || goal_y < 1 || goal_x >= width - 1 || goal_y >= height - 1) {
      RCLCPP_WARN(
        this->get_logger(),
        "Target pose outside map bounds (goal_x=%d goal_y=%d width=%d height=%d), skipping",
        goal_x, goal_y, width, height);
      return;
    }
    // A* search
    std::vector<std::vector<bool>> closed(width, std::vector<bool>(height, false));
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, CompareAStarNode> open;
    AStarNode* start = new AStarNode(start_x, start_y, 0, std::hypot(goal_x - start_x, goal_y - start_y));
    open.push(start);
    AStarNode* goal = nullptr;
    std::vector<std::pair<int, int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}};
    while (!open.empty()) {
      AStarNode* curr = open.top(); open.pop();
      if (curr->x == goal_x && curr->y == goal_y) { goal = curr; break; }
      if (closed[curr->x][curr->y]) continue;
      closed[curr->x][curr->y] = true;
      for (auto& d : directions) {
        int nx = curr->x + d.first, ny = curr->y + d.second;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        const size_t idx = static_cast<size_t>(ny) * static_cast<size_t>(width) + static_cast<size_t>(nx);
        if (idx >= msg->data.size()) continue;
        if (msg->data[idx] == unknown || msg->data[idx] < 0.3) continue; // Obstacle/unknown threshold (tune as needed)
        if (closed[nx][ny]) continue;
        float g = curr->g + std::hypot(d.first, d.second);
        float h = std::hypot(goal_x - nx, goal_y - ny);
        open.push(new AStarNode(nx, ny, g, h, curr));
      }
    }
    std::vector<std::pair<float, float>> path_points;
    float start_wx = msg->origin.x + start_x * resolution;
    float start_wy = msg->origin.y + start_y * resolution;
    float goal_wx = msg->origin.x + goal_x * resolution;
    float goal_wy = msg->origin.y + goal_y * resolution;
    if (goal) {
      for (AStarNode* n = goal; n; n = n->parent) {
        float wx = msg->origin.x + n->x * resolution;
        float wy = msg->origin.y + n->y * resolution;
        path_points.push_back({wx, wy});
      }
      std::reverse(path_points.begin(), path_points.end());
    } else {
      RCLCPP_WARN(this->get_logger(), "A* failed to find a path, publishing straight-line fallback");
      path_points.push_back({start_wx, start_wy});
      path_points.push_back({goal_wx, goal_wy});
    }
    // B-spline smoothing
    auto smooth_points = bspline(path_points, std::max(20, (int)path_points.size()));
    // Publish as nav_msgs/Path
    nav_msgs::msg::Path path;
    path.header = msg->header;
    for (auto& pt : smooth_points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose.position.x = pt.first;
      pose.pose.position.y = pt.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    traj_pub_->publish(path);
    RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu poses", path.poses.size());
  }

  rclcpp::Subscription<nvblox_msgs::msg::DistanceMapSlice>::SharedPtr costmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_goal_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}
