#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    import gtsam
    import numpy as np
    from gtsam.symbol_shorthand import X
except ImportError:  # pragma: no cover - runtime fallback when gtsam is absent
    gtsam = None
    np = None
    X = None


class ZedPoseGraphNode(Node):
    """
    Live Pose3 GTSAM graph built directly from the ZED pose topic.

    This node turns accepted ZED pose samples into Pose3 states, adds a
    BetweenFactorPose3 against the previous pose, optimizes the graph online,
    and publishes the latest optimized pose plus the full optimized path.
    """

    def __init__(self) -> None:
        super().__init__("zed_pose_graph_node")

        self.declare_parameter("topics.zed_pose", "/barracuda/zed_node/pose")
        self.declare_parameter("topics.gtsam_pose", "gtsam_pose")
        self.declare_parameter("topics.gtsam_path", "gtsam_path")
        self.declare_parameter("topics.health_output", "gtsam_health")
        self.declare_parameter("topics.debug_output", "gtsam_debug")
        self.declare_parameter("sample_stride", 5)
        self.declare_parameter("trans_sigma", 0.05)
        self.declare_parameter("rot_sigma", 0.05)
        self.declare_parameter("init_jitter_xyz", 0.03)
        self.declare_parameter("init_jitter_rot", 0.02)
        self.declare_parameter("max_poses", 250)

        self.pose_topic = str(self.get_parameter("topics.zed_pose").value)
        self.sample_stride = max(1, int(self.get_parameter("sample_stride").value))
        self.max_poses = max(2, int(self.get_parameter("max_poses").value))
        self.trans_sigma = float(self.get_parameter("trans_sigma").value)
        self.rot_sigma = float(self.get_parameter("rot_sigma").value)
        self.init_jitter_xyz = float(self.get_parameter("init_jitter_xyz").value)
        self.init_jitter_rot = float(self.get_parameter("init_jitter_rot").value)

        self.pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("topics.gtsam_pose").value), 10
        )
        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("topics.gtsam_path").value), 10
        )
        self.health_pub = self.create_publisher(
            Bool, str(self.get_parameter("topics.health_output").value), 10
        )
        self.debug_pub = self.create_publisher(
            String, str(self.get_parameter("topics.debug_output").value), 10
        )

        self.create_subscription(PoseStamped, self.pose_topic, self._on_pose, 20)

        self.gtsam_available = gtsam is not None
        self.message_count = 0
        self.pose_count = 0
        self.last_msg: Optional[PoseStamped] = None
        self.last_frame_id = "odom"
        self.rng = np.random.default_rng(7) if np is not None else None

        self.graph = gtsam.NonlinearFactorGraph() if gtsam is not None else None
        self.initial = gtsam.Values() if gtsam is not None else None
        self.result = gtsam.Values() if gtsam is not None else None
        if gtsam is not None and np is not None:
            sigmas = np.array(
                [
                    self.rot_sigma,
                    self.rot_sigma,
                    self.rot_sigma,
                    self.trans_sigma,
                    self.trans_sigma,
                    self.trans_sigma,
                ],
                dtype=float,
            )
            self.prior_noise = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
            self.between_noise = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        else:
            self.prior_noise = None
            self.between_noise = None

        self.reference_poses: list[gtsam.Pose3] = []
        self.optimized_poses: list[gtsam.Pose3] = []

        self.create_timer(0.5, self._publish_status)

        if not self.gtsam_available:
            self.get_logger().error(
                "Python gtsam is not available; zed_pose_graph_node will not optimize."
            )
        else:
            self.get_logger().info(
                f"Listening for live ZED poses on {self.pose_topic} and publishing optimized Pose3 results."
            )

    def _on_pose(self, msg: PoseStamped) -> None:
        self.last_msg = msg
        if msg.header.frame_id:
            self.last_frame_id = msg.header.frame_id
        self.message_count += 1

        if not self.gtsam_available or self.message_count % self.sample_stride != 0:
            return

        pose = self._pose3_from_msg(msg)
        if self.pose_count >= self.max_poses:
            self.get_logger().warn(
                f"Reached max_poses={self.max_poses}; ignoring additional ZED pose samples."
            )
            return

        if self.pose_count == 0:
            self._add_first_pose(pose)
        else:
            self._add_pose(pose)

        self._optimize_and_publish()

    def _pose3_from_msg(self, msg: PoseStamped):
        assert gtsam is not None
        pose = msg.pose
        rot = gtsam.Rot3.Quaternion(
            float(pose.orientation.w),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
        )
        point = gtsam.Point3(
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        )
        return gtsam.Pose3(rot, point)

    def _add_first_pose(self, pose) -> None:
        assert gtsam is not None and X is not None
        assert self.graph is not None and self.initial is not None
        self.graph.add(gtsam.PriorFactorPose3(X(0), pose, self.prior_noise))
        self.initial.insert(X(0), pose)
        self.reference_poses.append(pose)
        self.pose_count = 1

    def _add_pose(self, pose) -> None:
        assert gtsam is not None and X is not None and np is not None
        assert self.graph is not None and self.initial is not None and self.rng is not None

        prev_idx = self.pose_count - 1
        idx = self.pose_count
        prev_pose = self.reference_poses[-1]
        rel = prev_pose.between(pose)
        self.graph.add(gtsam.BetweenFactorPose3(X(prev_idx), X(idx), rel, self.between_noise))

        jitter_t = np.array(
            [
                float(self.rng.normal(0.0, self.init_jitter_xyz)),
                float(self.rng.normal(0.0, self.init_jitter_xyz)),
                float(self.rng.normal(0.0, self.init_jitter_xyz)),
            ],
            dtype=float,
        )
        jitter_r = np.array(
            [
                float(self.rng.normal(0.0, self.init_jitter_rot)),
                float(self.rng.normal(0.0, self.init_jitter_rot)),
                float(self.rng.normal(0.0, self.init_jitter_rot)),
            ],
            dtype=float,
        )
        perturb = gtsam.Pose3(gtsam.Rot3.RzRyRx(jitter_r), gtsam.Point3(*jitter_t))
        self.initial.insert(X(idx), pose.compose(perturb))
        self.reference_poses.append(pose)
        self.pose_count += 1

    def _optimize_and_publish(self) -> None:
        assert gtsam is not None and X is not None
        assert self.graph is not None and self.initial is not None

        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial)
        self.result = optimizer.optimize()
        self.initial = self.result
        self.optimized_poses = [self.result.atPose3(X(i)) for i in range(self.pose_count)]

        if not self.optimized_poses or self.last_msg is None:
            return

        latest = self.optimized_poses[-1]
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.last_msg.header.stamp
        pose_msg.header.frame_id = self.last_frame_id
        pose_msg.pose.position.x = float(latest.x())
        pose_msg.pose.position.y = float(latest.y())
        pose_msg.pose.position.z = float(latest.z())
        quat = latest.rotation().toQuaternion()
        pose_msg.pose.orientation.x = float(quat.x())
        pose_msg.pose.orientation.y = float(quat.y())
        pose_msg.pose.orientation.z = float(quat.z())
        pose_msg.pose.orientation.w = float(quat.w())
        self.pose_pub.publish(pose_msg)

        path = Path()
        path.header = pose_msg.header
        for pose3 in self.optimized_poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = pose_msg.header
            pose_stamped.pose.position.x = float(pose3.x())
            pose_stamped.pose.position.y = float(pose3.y())
            pose_stamped.pose.position.z = float(pose3.z())
            quat = pose3.rotation().toQuaternion()
            pose_stamped.pose.orientation.x = float(quat.x())
            pose_stamped.pose.orientation.y = float(quat.y())
            pose_stamped.pose.orientation.z = float(quat.z())
            pose_stamped.pose.orientation.w = float(quat.w())
            path.poses.append(pose_stamped)
        self.path_pub.publish(path)

    def _publish_status(self) -> None:
        healthy = self.gtsam_available and self.pose_count > 0
        self.health_pub.publish(Bool(data=healthy))
        debug = {
            "gtsam_available": self.gtsam_available,
            "sample_stride": self.sample_stride,
            "messages_seen": self.message_count,
            "poses_in_graph": self.pose_count,
            "pose_topic": self.pose_topic,
            "graph_mode": "Pose3",
        }
        self.debug_pub.publish(String(data=str(debug)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedPoseGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
