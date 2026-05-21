#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nvblox_msgs.msg import DistanceMapSlice


class TrajectoryVizTestPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_viz_test_publisher")
        self.pose_pub = self.create_publisher(PoseStamped, "/barracuda/zed_node/pose", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/target_pose", 10)
        self.slice_pub = self.create_publisher(
            DistanceMapSlice, "/barracuda/nvblox_node/static_map_slice", 10
        )
        self.occ_pub = self.create_publisher(
            OccupancyGrid, "/barracuda/nvblox_node/static_occupancy_grid", 10
        )

        self.timer = self.create_timer(0.2, self.publish_all)  # 5 Hz
        self.get_logger().info("Publishing hardcoded map/slice/pose/target at 5Hz")

    def publish_all(self):
        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = 2.0
        pose.pose.position.y = 2.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        target = PoseStamped()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.pose.position.x = 8.0
        target.pose.position.y = 2.0
        target.pose.position.z = 0.0
        target.pose.orientation.w = 1.0

        width, height = 20, 20
        gap_y = 10

        dist = [1.0] * (width * height)
        occ = [0] * (width * height)
        for y in range(height):
            if y == gap_y:
                continue
            idx = y * width + 5
            dist[idx] = 0.0
            occ[idx] = 100

        msg_slice = DistanceMapSlice()
        msg_slice.header.stamp = now
        msg_slice.header.frame_id = "map"
        msg_slice.origin.x = 0.0
        msg_slice.origin.y = 0.0
        msg_slice.origin.z = 0.0
        msg_slice.resolution = 1.0
        msg_slice.width = width
        msg_slice.height = height
        msg_slice.unknown_value = -1.0
        msg_slice.data = dist

        msg_occ = OccupancyGrid()
        msg_occ.header.stamp = now
        msg_occ.header.frame_id = "map"
        msg_occ.info.map_load_time = now
        msg_occ.info.resolution = 1.0
        msg_occ.info.width = width
        msg_occ.info.height = height
        msg_occ.info.origin.position.x = 0.0
        msg_occ.info.origin.position.y = 0.0
        msg_occ.info.origin.position.z = 0.0
        msg_occ.info.origin.orientation.w = 1.0
        msg_occ.data = occ

        self.pose_pub.publish(pose)
        self.target_pub.publish(target)
        self.slice_pub.publish(msg_slice)
        self.occ_pub.publish(msg_occ)


def main():
    rclpy.init()
    node = TrajectoryVizTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
