#!/usr/bin/env python3
"""Relay PoseStamped into Odometry for replay-driven navigation."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class PoseToOdometry(Node):
    def __init__(self):
        super().__init__("pose_to_odometry")
        self.declare_parameter("pose_topic", "/barracuda/zed_node/pose")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self._odom_topic = self.get_parameter("odom_topic").value
        self._odom_frame_id = self.get_parameter("odom_frame_id").value
        self._child_frame_id = self.get_parameter("child_frame_id").value

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._pose_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            10,
        )
        self.get_logger().info(
            f"Relaying PoseStamped to Odometry on {self._odom_topic}"
        )

    def _on_pose(self, msg: PoseStamped):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self._odom_frame_id
        odom.child_frame_id = self._child_frame_id
        odom.pose.pose = msg.pose
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = PoseToOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
