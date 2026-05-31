#!/usr/bin/env python3
"""Localization node: ZED pose -> map -> odom -> barracuda/base_link TF.

Subscribes to the ZED pose topic, which gives the camera center pose in the
map frame (T_map_zedm_camera_center).  Looks up the static transform from
pose_frame to base_frame (published by robot_state_publisher from the URDF) to
get T_pose_base, then composes:

    T_map_base = T_map_pose * T_pose_base

Publishes:
    map  -> odom                 (identity: odom origin == map origin)
    odom -> barracuda/base_link  (fixed-rate publish from latest ZED pose)

ZED should be launched with publish_tf=false and publish_map_tf=false so it
does not conflict with this node for TF ownership.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import (
    Buffer,
    ExtrapolationException,
    LookupException,
    TransformBroadcaster,
    TransformListener,
)


def _qmul(q1x, q1y, q1z, q1w, q2x, q2y, q2z, q2w):
    """Hamilton product of two quaternions, returns (x, y, z, w)."""
    return (
        q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y,
        q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x,
        q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w,
        q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z,
    )


def _rotate(qx, qy, qz, qw, vx, vy, vz):
    """Rotate vector (vx, vy, vz) by quaternion (qx, qy, qz, qw)."""
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rx = (1 - 2 * (yy + zz)) * vx + 2 * (xy - wz) * vy + 2 * (xz + wy) * vz
    ry = 2 * (xy + wz) * vx + (1 - 2 * (xx + zz)) * vy + 2 * (yz - wx) * vz
    rz = 2 * (xz - wy) * vx + 2 * (yz + wx) * vy + (1 - 2 * (xx + yy)) * vz
    return rx, ry, rz


class ZedLocalization(Node):
    def __init__(self):
        super().__init__("zed_localization")

        self.declare_parameter("pose_topic", "/barracuda/zed_node/pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "barracuda/base_link")
        self.declare_parameter("pose_frame", "barracuda/zedm_camera_center")
        self.declare_parameter("tf_lookup_timeout_sec", 1.0)
        self.declare_parameter("odom_base_publish_hz", 30.0)

        self._map_frame = self.get_parameter("map_frame").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._pose_frame = self.get_parameter("pose_frame").value
        self._tf_timeout = Duration(
            seconds=self.get_parameter("tf_lookup_timeout_sec").value
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Cached static pose_frame -> base_frame transform (set once the URDF
        # TF tree is available, then reused on every pose callback).
        self._T_pose_base = None
        self._latest_odom_base_tf = None

        self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            qos_profile_sensor_data,
        )

        # Keep map->odom alive at 10 Hz so the TF tree is never stale before
        # the first ZED pose arrives.
        self.create_timer(0.1, self._publish_identity_map_odom)

        odom_base_hz = float(self.get_parameter("odom_base_publish_hz").value)
        if odom_base_hz <= 0.0:
            self.get_logger().warn("odom_base_publish_hz <= 0; using 30.0 Hz")
            odom_base_hz = 30.0
        self.create_timer(1.0 / odom_base_hz, self._publish_latest_odom_base)

        self.get_logger().info(
            f"ZED localization ready: "
            f"{self.get_parameter('pose_topic').value} -> "
            f"{self._map_frame} -> {self._odom_frame} -> {self._base_frame}"
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _lookup_pose_to_base(self) -> bool:
        """Try to cache the static TF from pose_frame to base_frame."""
        if self._T_pose_base is not None:
            return True
        try:
            self._T_pose_base = self._tf_buffer.lookup_transform(
                self._pose_frame,
                self._base_frame,
                Time(),
                self._tf_timeout,
            )
            self.get_logger().info(
                f"Cached static TF {self._pose_frame} -> {self._base_frame}"
            )
            return True
        except (LookupException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"Waiting for static TF {self._pose_frame} -> "
                f"{self._base_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            return False

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_pose(self, msg: PoseStamped):
        if not self._lookup_pose_to_base():
            return

        # T_map_pose: camera center in map frame from ZED.
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        # T_pose_base: static offset from camera center to robot base link.
        t = self._T_pose_base.transform
        bx = t.translation.x
        by = t.translation.y
        bz = t.translation.z
        bqx = t.rotation.x
        bqy = t.rotation.y
        bqz = t.rotation.z
        bqw = t.rotation.w

        # Compose: T_map_base = T_map_pose * T_pose_base
        # Rotation: q_map_base = q_map_pose * q_pose_base
        rx, ry, rz, rw = _qmul(qx, qy, qz, qw, bqx, bqy, bqz, bqw)
        # Translation: p_map_base = p_map_pose + R(q_map_pose) * p_pose_base
        dx, dy, dz = _rotate(qx, qy, qz, qw, bx, by, bz)
        tx, ty, tz = px + dx, py + dy, pz + dz

        stamp = msg.header.stamp

        # odom -> barracuda/base_link: robot pose driven by ZED.
        # map -> odom stays as identity and is kept alive by the timer.
        tf_odom_base = TransformStamped()
        tf_odom_base.header.stamp = stamp
        tf_odom_base.header.frame_id = self._odom_frame
        tf_odom_base.child_frame_id = self._base_frame
        tf_odom_base.transform.translation.x = tx
        tf_odom_base.transform.translation.y = ty
        tf_odom_base.transform.translation.z = tz
        tf_odom_base.transform.rotation.x = rx
        tf_odom_base.transform.rotation.y = ry
        tf_odom_base.transform.rotation.z = rz
        tf_odom_base.transform.rotation.w = rw

        # Cache latest transform; fixed-rate timer handles publication.
        self._latest_odom_base_tf = tf_odom_base

    def _publish_latest_odom_base(self):
        """Publish latest odom->base_link transform at a fixed rate."""
        stamp = self.get_clock().now().to_msg()
        if self._latest_odom_base_tf is None:
            # Keep a connected TF tree before first pose arrives.
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(tf)
            return
        self._latest_odom_base_tf.header.stamp = stamp
        self._tf_broadcaster.sendTransform(self._latest_odom_base_tf)

    def _publish_identity_map_odom(self):
        """Publish identity map->odom at a fixed rate (pre-ZED-pose keepalive)."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._map_frame
        tf.child_frame_id = self._odom_frame
        tf.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = ZedLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
