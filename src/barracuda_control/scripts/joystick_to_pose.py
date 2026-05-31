#!/usr/bin/python3
import rclpy
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy

AXES_DEADZONE = 0.025

def yaw_to_quaternion(yaw: float):
    """
    Convert yaw angle to quaternion.
    Roll and pitch are assumed to be 0.
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw

class JoystickToPose(Node):
    def __init__(self):
        super().__init__("joystick_to_pose")
        self.declare_parameter("max_linear_velocity", 1.0)   # m/s
        self.declare_parameter("max_angular_velocity", 1.0)  # rad/s
        self.declare_parameter("publish_rate", 20.0)         # Hz
        self.declare_parameter("frame_id", "map")

        self.latest_joy = None
        self.last_time = self.get_clock().now()

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.get_parameter("frame_id").value

        # Initial target pose.
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 0.0
        self.initialized = False

        self.target_yaw = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.target_yaw)
        self.target_pose.pose.orientation.x = qx
        self.target_pose.pose.orientation.y = qy
        self.target_pose.pose.orientation.z = qz
        self.target_pose.pose.orientation.w = qw

        self.joy_subscription = self.create_subscription(
            Joy,
            "joy",
            self.joy_subscriber_callback,
            10,
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped,
            "target_pose",
            10,
        )

        self.current_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "dvl/pose",
            self.current_pose_callback,
            10,
        )

        publish_rate = self.get_parameter("publish_rate").value
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.timer_callback,
        )

    def current_pose_callback(self, msg):
        if not self.initialized:
            self.target_pose.pose.position = msg.pose.pose.position
            self.target_pose.pose.orientation = msg.pose.pose.orientation
            self.initialized = True

    def joy_subscriber_callback(self, msg):
        if not msg.axes or not msg.buttons:
            self.get_logger().warn("Received empty Joy message")
            return
        self.latest_joy = msg
    
    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.target_pose.header.stamp = now.to_msg()
        self.target_pose.header.frame_id = self.get_parameter("frame_id").value

        if self.latest_joy is None:
            self.pose_publisher.publish(self.target_pose)
            return
        
        msg = self.latest_joy
        max_linear_velocity = self.get_parameter("max_linear_velocity").value
        max_angular_velocity = self.get_parameter("max_angular_velocity").value
        deadzone_corrected_axes = [
            a if abs(a) > AXES_DEADZONE else 0.0
            for a in msg.axes
        ]

        linear_cmd = (
            deadzone_corrected_axes[1],
            deadzone_corrected_axes[0],
            1.0 if msg.buttons[12] == 1 else (-1.0 if msg.buttons[13] == 1 else 0.0),
        )

        self.target_pose.pose.position.x += linear_cmd[0] * max_linear_velocity * dt
        self.target_pose.pose.position.y += linear_cmd[1] * max_linear_velocity * dt
        self.target_pose.pose.position.z += linear_cmd[2] * max_linear_velocity * dt

        yaw_cmd = (
            1.0 if msg.buttons[4] == 1 else
            (-1.0 if msg.buttons[5] == 1 else 0.0)
        )

        self.target_yaw += yaw_cmd * max_angular_velocity * dt

        qx, qy, qz, qw = yaw_to_quaternion(self.target_yaw)
        self.target_pose.pose.orientation.x = qx
        self.target_pose.pose.orientation.y = qy
        self.target_pose.pose.orientation.z = qz
        self.target_pose.pose.orientation.w = qw

        self.pose_publisher.publish(self.target_pose)

def main(args=None):
    rclpy.init(args=args)

    joystick_to_pose = JoystickToPose()

    rclpy.spin(joystick_to_pose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_to_pose.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()