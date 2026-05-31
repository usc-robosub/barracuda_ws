#!/usr/bin/env python3

from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu, PointCloud2, Range
from std_msgs.msg import Bool, String

from .gtsam_estimator import GtsamEstimator
from .measurement_types import DepthSample, DvlSample, PoseSample


class BarracudaEstimatorNode(Node):
    """
    Direct-subscription estimator entry point for the Barracuda stack.

    This node subscribes to the active `/barracuda/...` sensor interfaces,
    forwards measurements into `GtsamEstimator`, and publishes estimator
    health, debug status, and the latest pose output.
    """

    def __init__(self) -> None:
        super().__init__("estimator_node")

        self.declare_parameter("topics.imu", "/barracuda/zed_node/imu/data")
        self.declare_parameter("topics.depth_range", "/barracuda/depth")
        self.declare_parameter("topics.depth_pressure", "")
        self.declare_parameter("topics.dvl_odometry", "/barracuda/dvl/odometry")
        self.declare_parameter("topics.zed_pose", "/barracuda/zed_node/pose")
        self.declare_parameter(
            "topics.point_cloud", "/barracuda/zed_node/point_cloud/cloud_registered"
        )
        self.declare_parameter("topics.pose_output", "/barracuda/estimation/pose")
        self.declare_parameter("topics.health_output", "/barracuda/estimation/health")
        self.declare_parameter("topics.debug_output", "/barracuda/estimation/debug")
        self.declare_parameter("optimize_period_sec", 0.5)
        self.declare_parameter("depth_mode", "range")

        self.imu_topic = self.get_parameter("topics.imu").value
        self.depth_range_topic = self.get_parameter("topics.depth_range").value
        self.depth_pressure_topic = self.get_parameter("topics.depth_pressure").value
        self.dvl_topic = self.get_parameter("topics.dvl_odometry").value
        self.zed_pose_topic = self.get_parameter("topics.zed_pose").value
        self.point_cloud_topic = self.get_parameter("topics.point_cloud").value
        self.depth_mode = self.get_parameter("depth_mode").value

        self.latest_imu: Optional[Imu] = None
        self.latest_dvl: Optional[Odometry] = None
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_depth: Optional[DepthSample] = None
        self.latest_point_cloud: Optional[PointCloud2] = None
        self.estimator = GtsamEstimator()

        self.pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("topics.pose_output").value, 10
        )
        self.health_pub = self.create_publisher(
            Bool, self.get_parameter("topics.health_output").value, 10
        )
        self.debug_pub = self.create_publisher(
            String, self.get_parameter("topics.debug_output").value, 10
        )

        self.create_subscription(Imu, self.imu_topic, self._on_imu, 50)
        self.create_subscription(Odometry, self.dvl_topic, self._on_dvl, 10)
        self.create_subscription(PoseStamped, self.zed_pose_topic, self._on_pose, 10)
        self.create_subscription(
            PointCloud2, self.point_cloud_topic, self._on_point_cloud, 10
        )

        if self.depth_mode == "range":
            self.create_subscription(
                Range, self.depth_range_topic, self._on_depth_range, 10
            )
        elif self.depth_mode == "fluid_pressure":
            self.create_subscription(
                FluidPressure, self.depth_pressure_topic, self._on_depth_pressure, 10
            )
        else:
            raise ValueError("depth_mode must be 'range' or 'fluid_pressure'")

        optimize_period_sec = float(self.get_parameter("optimize_period_sec").value)
        self.timer = self.create_timer(optimize_period_sec, self._estimation_step)

        self.get_logger().info(
            "Estimator listening directly to sensor topics: "
            f"imu={self.imu_topic}, depth={self.depth_range_topic or self.depth_pressure_topic}, "
            f"dvl={self.dvl_topic}, zed_pose={self.zed_pose_topic}, point_cloud={self.point_cloud_topic}"
        )

    def _on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg

    def _on_dvl(self, msg: Odometry) -> None:
        self.latest_dvl = msg
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        sample = DvlSample(
            stamp_sec=stamp_sec,
            position_xyz=(
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            ),
            velocity_xyz=(
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.linear.z),
            ),
            orientation_xyzw=(
                float(msg.pose.pose.orientation.x),
                float(msg.pose.pose.orientation.y),
                float(msg.pose.pose.orientation.z),
                float(msg.pose.pose.orientation.w),
            ),
        )
        self.estimator.add_dvl(sample)

    def _on_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        sample = PoseSample(
            stamp_sec=stamp_sec,
            position_xyz=(
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ),
            orientation_xyzw=(
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ),
        )
        self.estimator.add_pose(sample)

    def _on_point_cloud(self, msg: PointCloud2) -> None:
        self.latest_point_cloud = msg

    def _on_depth_range(self, msg: Range) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.latest_depth = DepthSample(
            stamp_sec=stamp_sec, z_value=float(msg.range), source="range"
        )
        self.estimator.add_depth(self.latest_depth)

    def _on_depth_pressure(self, msg: FluidPressure) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.latest_depth = DepthSample(
            stamp_sec=stamp_sec,
            z_value=float(msg.fluid_pressure),
            source="fluid_pressure",
        )
        self.estimator.add_depth(self.latest_depth)

    def _estimation_step(self) -> None:
        status = self.estimator.status()
        healthy = status.healthy
        self.health_pub.publish(Bool(data=healthy))

        debug = {
            "healthy": healthy,
            "has_imu": self.latest_imu is not None,
            "has_depth": status.has_depth,
            "has_dvl": status.has_dvl,
            "has_pose": status.has_pose,
            "has_point_cloud": self.latest_point_cloud is not None,
        }
        self.debug_pub.publish(String(data=str(debug)))

        stamp_sec = float(self.get_clock().now().nanoseconds) * 1e-9
        estimate = self.estimator.step(stamp_sec)
        if estimate is None:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = estimate.frame_id
        pose.pose.position.x = estimate.position_xyz[0]
        pose.pose.position.y = estimate.position_xyz[1]
        pose.pose.position.z = estimate.position_xyz[2]
        pose.pose.orientation.x = estimate.orientation_xyzw[0]
        pose.pose.orientation.y = estimate.orientation_xyzw[1]
        pose.pose.orientation.z = estimate.orientation_xyzw[2]
        pose.pose.orientation.w = estimate.orientation_xyzw[3]

        self.pose_pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BarracudaEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
