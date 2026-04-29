#!/usr/bin/env python3
"""Relay ZED PoseStamped to TransformStamped for nvblox topic transforms."""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node


class PoseToTransformNode(Node):
    def __init__(self) -> None:
        super().__init__("pose_to_transform")
        self.declare_parameter("pose_topic", "zed_node/pose")
        self.declare_parameter("transform_topic", "transform")
        self.declare_parameter("child_frame_id", "barracuda_camera_center")

        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        transform_topic = (
            self.get_parameter("transform_topic").get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )

        self.publisher = self.create_publisher(TransformStamped, transform_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, pose_topic, self.pose_callback, 10
        )

    def pose_callback(self, pose_msg: PoseStamped) -> None:
        transform_msg = TransformStamped()
        transform_msg.header = pose_msg.header
        transform_msg.child_frame_id = self.child_frame_id
        transform_msg.transform.translation.x = pose_msg.pose.position.x
        transform_msg.transform.translation.y = pose_msg.pose.position.y
        transform_msg.transform.translation.z = pose_msg.pose.position.z
        transform_msg.transform.rotation = pose_msg.pose.orientation
        self.publisher.publish(transform_msg)


def main() -> None:
    rclpy.init()
    node = PoseToTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()