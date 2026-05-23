#!/usr/bin/env python3

from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Image, FluidPressure, Imu, Range
from std_msgs.msg import Bool, String


@dataclass
class DepthSample:
    stamp_sec: float
    z_value: float
    source: str


class BarracudaEstimatorNode(Node):
    """
    Minimal direct-subscription estimator skeleton.

    This node subscribes directly to the existing /barracuda sensor topics and keeps
    the latest measurements in one place. It is intentionally lightweight: the IMU
    preintegration / GTSAM graph update steps are placeholders for future work.
    """

    def __init__(self) -> None:
        super().__init__("estimator_node")

        self.declare_parameter("topics.imu", "/barracuda/imu/data")
        self.declare_parameter("topics.depth_range", "/barracuda/depth")
        self.declare_parameter("topics.depth_pressure", "")
        self.declare_parameter("topics.dvl_odometry", "/barracuda/dvl/odometry")
        self.declare_parameter("topics.camera_image", "/barracuda/left_camera_image")
        self.declare_parameter("topics.pose_output", "/barracuda/estimation/pose")
        self.declare_parameter("topics.health_output", "/barracuda/estimation/health")
        self.declare_parameter("topics.debug_output", "/barracuda/estimation/debug")
        self.declare_parameter("optimize_period_sec", 0.5)
        self.declare_parameter("stale_timeout_sec", 1.0)
        self.declare_parameter("depth_mode", "range")

        self.imu_topic = self.get_parameter("topics.imu").value
        self.depth_range_topic = self.get_parameter("topics.depth_range").value
        self.depth_pressure_topic = self.get_parameter("topics.depth_pressure").value
        self.dvl_topic = self.get_parameter("topics.dvl_odometry").value
        self.camera_topic = self.get_parameter("topics.camera_image").value
        self.depth_mode = self.get_parameter("depth_mode").value
        self.stale_timeout = Duration(seconds=float(self.get_parameter("stale_timeout_sec").value))

        self.latest_imu: Optional[Imu] = None
        self.latest_dvl: Optional[Odometry] = None
        self.latest_depth: Optional[DepthSample] = None
        self.latest_camera: Optional[Image] = None

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
        self.create_subscription(Image, self.camera_topic, self._on_camera, 10)

        if self.depth_mode == "range":
            self.create_subscription(Range, self.depth_range_topic, self._on_depth_range, 10)
        elif self.depth_mode == "fluid_pressure":
            self.create_subscription(
                FluidPressure, self.depth_pressure_topic, self._on_depth_pressure, 10
            )
        else:
            raise ValueError("depth_mode must be 'range' or 'fluid_pressure'")

        optimize_period_sec = float(self.get_parameter("optimize_period_sec").value)
        self.timer = self.create_timer(optimize_period_sec, self._estimation_step)

        self.get_logger().info(
            "Estimator skeleton listening directly to existing topics: "
            f"imu={self.imu_topic}, depth={self.depth_range_topic or self.depth_pressure_topic}, "
            f"dvl={self.dvl_topic}, camera={self.camera_topic}"
        )

    def _on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg
        # Future GTSAM step: append this measurement into the IMU preintegration buffer.

    def _on_dvl(self, msg: Odometry) -> None:
        self.latest_dvl = msg
        # Future GTSAM step: use body/world velocity from DVL as a factor at key updates.

    def _on_camera(self, msg: Image) -> None:
        self.latest_camera = msg
        # Future SLAM step: feed vision landmarks or loop-closure observations.

    def _on_depth_range(self, msg: Range) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.latest_depth = DepthSample(stamp_sec=stamp_sec, z_value=float(msg.range), source="range")

    def _on_depth_pressure(self, msg: FluidPressure) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.latest_depth = DepthSample(
            stamp_sec=stamp_sec,
            z_value=float(msg.fluid_pressure),
            source="fluid_pressure",
        )

    def _estimation_step(self) -> None:
        healthy = self._inputs_ready()
        self.health_pub.publish(Bool(data=healthy))

        debug = {
            "healthy": healthy,
            "has_imu": self.latest_imu is not None,
            "has_depth": self.latest_depth is not None,
            "has_dvl": self.latest_dvl is not None,
            "has_camera": self.latest_camera is not None,
        }
        self.debug_pub.publish(String(data=str(debug)))

        if not healthy:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"

        # Placeholder pose output so the node has a concrete ROS interface today.
        # Future work: replace this with the current optimized pose from the GTSAM graph.
        if self.latest_dvl is not None:
            pose.pose.position.x = self.latest_dvl.pose.pose.position.x
            pose.pose.position.y = self.latest_dvl.pose.pose.position.y
            pose.pose.position.z = self.latest_dvl.pose.pose.position.z
            pose.pose.orientation = self.latest_dvl.pose.pose.orientation

        self.pose_pub.publish(pose)

    def _inputs_ready(self) -> bool:
        return (
            self.latest_imu is not None
            and self.latest_depth is not None
            and self.latest_dvl is not None
        )


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
