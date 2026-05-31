#!/usr/bin/env python3
"""Bridge replayed ZED pose/odometry topics into Nav2-friendly TF and odometry."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> list[list[float]]:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _matrix_to_quat(rot: list[list[float]]) -> tuple[float, float, float, float]:
    trace = rot[0][0] + rot[1][1] + rot[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2][1] - rot[1][2]) / s
        y = (rot[0][2] - rot[2][0]) / s
        z = (rot[1][0] - rot[0][1]) / s
    elif rot[0][0] > rot[1][1] and rot[0][0] > rot[2][2]:
        s = math.sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]) * 2.0
        w = (rot[2][1] - rot[1][2]) / s
        x = 0.25 * s
        y = (rot[0][1] + rot[1][0]) / s
        z = (rot[0][2] + rot[2][0]) / s
    elif rot[1][1] > rot[2][2]:
        s = math.sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]) * 2.0
        w = (rot[0][2] - rot[2][0]) / s
        x = (rot[0][1] + rot[1][0]) / s
        y = 0.25 * s
        z = (rot[1][2] + rot[2][1]) / s
    else:
        s = math.sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]) * 2.0
        w = (rot[1][0] - rot[0][1]) / s
        x = (rot[0][2] + rot[2][0]) / s
        y = (rot[1][2] + rot[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _transpose(rot: list[list[float]]) -> list[list[float]]:
    return [[rot[j][i] for j in range(3)] for i in range(3)]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [
            a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def _matvec(rot: list[list[float]], vec: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(
        rot[row][0] * vec[0] + rot[row][1] * vec[1] + rot[row][2] * vec[2]
        for row in range(3)
    )


class ReplayTfBridge(Node):
    def __init__(self):
        super().__init__("replay_tf_bridge")
        self.declare_parameter("pose_topic", "/barracuda/zed_node/pose")
        self.declare_parameter("odom_topic", "/barracuda/zed_node/odom")
        self.declare_parameter("filtered_odom_topic", "/odometry/filtered")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "barracuda/base_link")

        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        self._latest_pose: PoseStamped | None = None
        self._latest_odom: Odometry | None = None

        self._tf_broadcaster = TransformBroadcaster(self)
        self._filtered_odom_pub = self.create_publisher(
            Odometry, self.get_parameter("filtered_odom_topic").value, 10
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            10,
        )
        self.get_logger().info(
            "Replay TF bridge ready: pose + odom -> /odometry/filtered and TF"
        )

    def _on_pose(self, msg: PoseStamped):
        self._latest_pose = msg
        self._publish_if_ready()

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg
        filtered = Odometry()
        filtered.header = msg.header
        filtered.header.frame_id = self._odom_frame
        filtered.child_frame_id = self._base_frame
        filtered.pose = msg.pose
        filtered.twist = msg.twist
        self._filtered_odom_pub.publish(filtered)
        if self._latest_pose is None:
            self._publish_fallback_from_odom(msg)
        self._publish_if_ready()

    def _publish_fallback_from_odom(self, odom: Odometry):
        stamp = odom.header.stamp

        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = stamp
        tf_map_odom.header.frame_id = self._map_frame
        tf_map_odom.child_frame_id = self._odom_frame
        tf_map_odom.transform.translation.x = 0.0
        tf_map_odom.transform.translation.y = 0.0
        tf_map_odom.transform.translation.z = 0.0
        tf_map_odom.transform.rotation.x = 0.0
        tf_map_odom.transform.rotation.y = 0.0
        tf_map_odom.transform.rotation.z = 0.0
        tf_map_odom.transform.rotation.w = 1.0

        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = stamp
        tf_odom_base.header.frame_id = self._odom_frame
        tf_odom_base.child_frame_id = self._base_frame
        tf_odom_base.transform.translation.x = odom.pose.pose.position.x
        tf_odom_base.transform.translation.y = odom.pose.pose.position.y
        tf_odom_base.transform.translation.z = odom.pose.pose.position.z
        tf_odom_base.transform.rotation = odom.pose.pose.orientation

        self._tf_broadcaster.sendTransform([tf_map_odom, tf_odom_base])

    def _publish_if_ready(self):
        if self._latest_pose is None or self._latest_odom is None:
            return

        pose = self._latest_pose
        odom = self._latest_odom

        map_rot = _quat_to_matrix(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        odom_rot = _quat_to_matrix(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )
        odom_rot_inv = _transpose(odom_rot)

        map_pos = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        odom_pos = (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
        )

        map_to_odom_rot = _matmul(map_rot, odom_rot_inv)
        map_to_odom_offset = _matvec(map_to_odom_rot, odom_pos)
        map_to_odom_pos = tuple(map_pos[i] - map_to_odom_offset[i] for i in range(3))
        map_to_odom_quat = _matrix_to_quat(map_to_odom_rot)

        stamp = odom.header.stamp

        tf_map_odom = TransformStamped()
        tf_map_odom.header.stamp = stamp
        tf_map_odom.header.frame_id = self._map_frame
        tf_map_odom.child_frame_id = self._odom_frame
        tf_map_odom.transform.translation.x = map_to_odom_pos[0]
        tf_map_odom.transform.translation.y = map_to_odom_pos[1]
        tf_map_odom.transform.translation.z = map_to_odom_pos[2]
        tf_map_odom.transform.rotation.x = map_to_odom_quat[0]
        tf_map_odom.transform.rotation.y = map_to_odom_quat[1]
        tf_map_odom.transform.rotation.z = map_to_odom_quat[2]
        tf_map_odom.transform.rotation.w = map_to_odom_quat[3]

        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = stamp
        tf_odom_base.header.frame_id = self._odom_frame
        tf_odom_base.child_frame_id = self._base_frame
        tf_odom_base.transform.translation.x = odom_pos[0]
        tf_odom_base.transform.translation.y = odom_pos[1]
        tf_odom_base.transform.translation.z = odom_pos[2]
        tf_odom_base.transform.rotation = odom.pose.pose.orientation

        self._tf_broadcaster.sendTransform([tf_map_odom, tf_odom_base])


def main():
    rclpy.init()
    node = ReplayTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
