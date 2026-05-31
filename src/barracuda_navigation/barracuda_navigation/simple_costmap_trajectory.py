#!/usr/bin/env python3
"""Simple costmap-based trajectory publisher.

Inputs:
  - /barracuda/nvblox_node/static_map_slice (nvblox_msgs/DistanceMapSlice)
  - /target_pose (geometry_msgs/PoseStamped) published manually
    - TF map->base (default map->base_link), with optional pose fallback

Output:
  - /planned_trajectory (nav_msgs/Path)

The planner is intentionally minimal: it publishes a straight-line path in the
costmap global frame and truncates the path before blocked cells.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nvblox_msgs.msg import DistanceMapSlice
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


class SimpleCostmapTrajectory(Node):
    def __init__(self) -> None:
        super().__init__("simple_costmap_trajectory")

        self.declare_parameter("costmap_topic", "/barracuda/nvblox_node/static_map_slice")
        self.declare_parameter("goal_topic", "/target_pose")
        self.declare_parameter("trajectory_topic", "/planned_trajectory")
        self.declare_parameter("pose_fallback_topic", "/barracuda/zed_node/pose")
        self.declare_parameter("visualization_costmap_topic", "/global_costmap/costmap")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("obstacle_distance_threshold", 0.30)
        self.declare_parameter("min_step_m", 0.10)
        self.declare_parameter("visualization_max_distance", 2.0)

        self._goal: PoseStamped | None = None
        self._pose_fallback: PoseStamped | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        costmap_topic = self.get_parameter("costmap_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        traj_topic = self.get_parameter("trajectory_topic").value
        pose_fallback_topic = self.get_parameter("pose_fallback_topic").value
        viz_costmap_topic = self.get_parameter("visualization_costmap_topic").value

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._traj_pub = self.create_publisher(Path, traj_topic, 10)
        self._viz_costmap_pub = self.create_publisher(OccupancyGrid, viz_costmap_topic, map_qos)
        self.create_subscription(DistanceMapSlice, costmap_topic, self._on_costmap, 10)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.create_subscription(PoseStamped, pose_fallback_topic, self._on_pose_fallback, 10)

        self.get_logger().info(
            f"SimpleCostmapTrajectory ready. costmap={costmap_topic} goal={goal_topic} "
            f"trajectory={traj_topic} viz_costmap={viz_costmap_topic}"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        self.get_logger().info(
            f"Received goal: x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f} "
            f"frame={msg.header.frame_id or '<empty>'}"
        )

    def _on_pose_fallback(self, msg: PoseStamped) -> None:
        self._pose_fallback = msg

    def _lookup_start_xy(self, map_frame: str) -> tuple[float, float] | None:
        base_frame = self.get_parameter("base_frame").value
        try:
            tf_msg = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
            return (
                float(tf_msg.transform.translation.x),
                float(tf_msg.transform.translation.y),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"TF lookup failed ({map_frame} <- {base_frame}), trying pose fallback: {exc}",
                throttle_duration_sec=5.0,
            )

        if self._pose_fallback is None:
            return None

        pose_frame = self._pose_fallback.header.frame_id
        if pose_frame and pose_frame != map_frame:
            self.get_logger().warning(
                f"Pose fallback frame ({pose_frame}) != map frame ({map_frame})",
                throttle_duration_sec=5.0,
            )
            return None

        return (
            float(self._pose_fallback.pose.position.x),
            float(self._pose_fallback.pose.position.y),
        )

    @staticmethod
    def _cell_index(
        x: float,
        y: float,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> tuple[int, int]:
        return (
            int(math.floor((x - origin_x) / resolution)),
            int(math.floor((y - origin_y) / resolution)),
        )

    @staticmethod
    def _in_bounds(cx: int, cy: int, width: int, height: int) -> bool:
        return 0 <= cx < width and 0 <= cy < height

    def _publish_visualization_costmap(
        self,
        msg: DistanceMapSlice,
        width: int,
        height: int,
        unknown: float,
        obstacle_thresh: float,
    ) -> None:
        max_distance = max(
            obstacle_thresh,
            float(self.get_parameter("visualization_max_distance").value),
        )

        grid = OccupancyGrid()
        grid.header = msg.header
        grid.info.map_load_time = msg.header.stamp
        grid.info.resolution = float(msg.resolution)
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(msg.origin.x)
        grid.info.origin.position.y = float(msg.origin.y)
        grid.info.origin.position.z = float(msg.origin.z)
        grid.info.origin.orientation.w = 1.0

        data: list[int] = []
        for raw_distance in msg.data[: width * height]:
            distance = float(raw_distance)
            if distance == unknown:
                data.append(-1)
                continue

            if distance <= obstacle_thresh:
                data.append(100)
                continue

            scaled = 100.0 * (1.0 - min(distance / max_distance, 1.0))
            data.append(max(0, min(100, int(round(scaled)))))

        grid.data = data
        self._viz_costmap_pub.publish(grid)

    def _on_costmap(self, msg: DistanceMapSlice) -> None:
        width = int(msg.width)
        height = int(msg.height)
        if width < 1 or height < 1:
            self.get_logger().warning("Invalid costmap dimensions, skipping")
            return

        expected = width * height
        if len(msg.data) < expected:
            self.get_logger().warning(
                f"Costmap size mismatch (got {len(msg.data)}, expected {expected}), skipping"
            )
            return

        map_frame = msg.header.frame_id or self.get_parameter("map_frame").value
        start_xy = self._lookup_start_xy(map_frame)
        if start_xy is None:
            self.get_logger().warning("No valid start pose from TF or fallback pose", throttle_duration_sec=2.0)
            return

        goal_frame = self._goal.header.frame_id
        if goal_frame and goal_frame != map_frame:
            self.get_logger().warning(
                f"Goal frame ({goal_frame}) != map frame ({map_frame}), skipping",
                throttle_duration_sec=2.0,
            )
            return

        sx, sy = start_xy
        gx = float(self._goal.pose.position.x)
        gy = float(self._goal.pose.position.y)

        dist = math.hypot(gx - sx, gy - sy)
        min_step = float(self.get_parameter("min_step_m").value)
        step = max(min_step, float(msg.resolution))
        samples = max(2, int(math.ceil(dist / step)) + 1)

        origin_x = float(msg.origin.x)
        origin_y = float(msg.origin.y)
        resolution = float(msg.resolution)
        unknown = float(msg.unknown_value)
        obstacle_thresh = float(self.get_parameter("obstacle_distance_threshold").value)
        self._publish_visualization_costmap(msg, width, height, unknown, obstacle_thresh)

        if self._goal is None:
            self.get_logger().warning("No /target_pose yet, skipping", throttle_duration_sec=2.0)
            return

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = map_frame

        for i in range(samples):
            t = i / float(samples - 1)
            px = sx + t * (gx - sx)
            py = sy + t * (gy - sy)
            cx, cy = self._cell_index(px, py, origin_x, origin_y, resolution)

            if not self._in_bounds(cx, cy, width, height):
                break

            idx = cy * width + cx
            d = float(msg.data[idx])
            if d == unknown or d < obstacle_thresh:
                break

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        if len(path.poses) < 2:
            self.get_logger().warning("Path blocked or too short, not publishing", throttle_duration_sec=2.0)
            return

        self._traj_pub.publish(path)
        self.get_logger().info(f"Published simple trajectory with {len(path.poses)} poses")


def main() -> None:
    rclpy.init()
    node = SimpleCostmapTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
