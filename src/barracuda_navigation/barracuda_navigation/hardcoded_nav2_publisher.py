#!/usr/bin/env python3
"""Hardcoded test publisher for Nav2 MPPI.

Publishes:
  - TF: map -> odom -> base_link   (required by Nav2 planner + controller)
  - /barracuda/nvblox_node/static_map_slice  (DistanceMapSlice consumed by NvbloxCostmapLayer)
  - /odometry/filtered             (nav_msgs/Odometry, used by bt_navigator)
  - /target_pose                   (goal bridged to NavigateToPose by goal_bridge)

Map layout (each cell = 0.1 m, 100x100 = 10 m x 10 m):
    - Free space everywhere
    - One rectangular obstacle block in the middle of the map
    - Robot starts at (1.0, 5.0)
    - Goal is on the other side of the obstacle at (8.0, 5.0)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from nvblox_msgs.msg import DistanceMapSlice
from tf2_ros import TransformBroadcaster


RESOLUTION = 0.1      # m/cell
WIDTH = 100
HEIGHT = 100
ORIGIN_X = 0.0
ORIGIN_Y = 0.0

ROBOT_X = 1.0
ROBOT_Y = 5.0

GOAL_X = 8.0
GOAL_Y = 5.0

# Central obstacle block: x in [4.5, 5.5], y in [4.0, 6.0]
BLOCK_MIN_COL = 45
BLOCK_MAX_COL = 55
BLOCK_MIN_ROW = 40
BLOCK_MAX_ROW = 60


def _build_slice() -> list[float]:
    """Build distance map slice: positive distance = free, 0 = obstacle."""
    data = [1.0] * (WIDTH * HEIGHT)

    # Fill one rectangular obstacle block in the center.
    for row in range(HEIGHT):
        for col in range(WIDTH):
            in_block = (
                BLOCK_MIN_ROW <= row <= BLOCK_MAX_ROW and
                BLOCK_MIN_COL <= col <= BLOCK_MAX_COL
            )
            if in_block:
                data[row * WIDTH + col] = 0.0

    # propagate a rough distance field (simple falloff, good enough for costmap)
    for row in range(HEIGHT):
        for col in range(WIDTH):
            if data[row * WIDTH + col] == 0.0:
                continue

            # Distance to the rectangular block boundary.
            dx_cells = max(BLOCK_MIN_COL - col, 0, col - BLOCK_MAX_COL)
            dy_cells = max(BLOCK_MIN_ROW - row, 0, row - BLOCK_MAX_ROW)
            dist = math.hypot(dx_cells * RESOLUTION, dy_cells * RESOLUTION)
            data[row * WIDTH + col] = max(0.01, min(dist, 2.0))

    return data


_SLICE_DATA = _build_slice()


class HardcodedNav2Publisher(Node):
    def __init__(self):
        super().__init__("hardcoded_nav2_publisher")
        self._tf_broadcaster = TransformBroadcaster(self)

        self._slice_pub = self.create_publisher(
            DistanceMapSlice, "/barracuda/nvblox_node/static_map_slice", 10
        )
        self._odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self._target_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self._pose_pub = self.create_publisher(PoseStamped, "/barracuda/zed_node/pose", 10)

        self._start_time_sec = self.get_clock().now().nanoseconds / 1e9
        self._last_goal_pub_sec = 0.0
        self._timer = self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(
            f"HardcodedNav2Publisher: robot=({ROBOT_X},{ROBOT_Y}) goal=({GOAL_X},{GOAL_Y})"
        )

    def _tick(self):
        now = self.get_clock().now().to_msg()

        # --- TF: map -> odom (identity) ---
        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = now
        tf_map_odom.header.frame_id = "map"
        tf_map_odom.child_frame_id = "odom"
        tf_map_odom.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_map_odom)

        # --- TF: odom -> base_link (robot at fixed start pose) ---
        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = now
        tf_odom_base.header.frame_id = "odom"
        tf_odom_base.child_frame_id = "base_link"
        tf_odom_base.transform.translation.x = ROBOT_X
        tf_odom_base.transform.translation.y = ROBOT_Y
        tf_odom_base.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf_odom_base)

        # --- Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = ROBOT_X
        odom.pose.pose.position.y = ROBOT_Y
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

        # --- Robot pose (for old trajectory_generator compat) ---
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = ROBOT_X
        pose.pose.position.y = ROBOT_Y
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        # --- Distance map slice ---
        sl = DistanceMapSlice()
        sl.header.stamp = now
        sl.header.frame_id = "map"
        sl.resolution = RESOLUTION
        sl.width = WIDTH
        sl.height = HEIGHT
        sl.origin.x = ORIGIN_X
        sl.origin.y = ORIGIN_Y
        sl.origin.z = 0.0
        sl.unknown_value = -1.0
        sl.data = _SLICE_DATA
        self._slice_pub.publish(sl)

        # --- Goal (publish after stack warmup, then republish periodically) ---
        now_sec = self.get_clock().now().nanoseconds / 1e9
        elapsed = now_sec - self._start_time_sec
        should_publish_goal = elapsed > 2.0 and (now_sec - self._last_goal_pub_sec) > 1.0
        if should_publish_goal:
            goal = PoseStamped()
            goal.header.stamp = now
            goal.header.frame_id = "map"
            goal.pose.position.x = GOAL_X
            goal.pose.position.y = GOAL_Y
            goal.pose.orientation.w = 1.0
            self._target_pub.publish(goal)
            self._last_goal_pub_sec = now_sec
            self.get_logger().info(f"Goal published: ({GOAL_X}, {GOAL_Y})")


def main():
    rclpy.init()
    node = HardcodedNav2Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
