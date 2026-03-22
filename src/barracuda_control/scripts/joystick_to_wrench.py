#!/usr/bin/python3
import rclpy
from geometry_msgs.msg import Vector3, Wrench
from rclpy.node import Node
from sensor_msgs.msg import Joy

AXES_DEADZONE = 0.025

class JoystickToWrench(Node):
    def __init__(self):
        super().__init__("joystick_to_wrench")
        
        self.declare_parameter("max_force", 4.0)
        self.declare_parameter("max_torque", 4.0)

        self.joy_subscription = self.create_subscription(
            Joy, "joy", self.joy_subscriber_callback, 10
        )
        self.joy_subscription  # prevent unused variable warning

        self.wrench_publisher = self.create_publisher(Wrench, "wrench", 10)

    def joy_subscriber_callback(self, msg):
        if not msg.axes or not msg.buttons:
            self.get_logger().warning("malformatted joy message")
            return

        max_force = self.get_parameter("max_force").value
        max_torque = self.get_parameter("max_torque").value

        wrench = Wrench()

        deadzone_corrected_axes = [a * (abs(a) > AXES_DEADZONE) for a in msg.axes]
        force = (
            deadzone_corrected_axes[1],
            deadzone_corrected_axes[0],
            1.0 if msg.buttons[12] == 1 else (-1.0 if msg.buttons[13] == 1 else 0.0),
        )
        wrench.force = Vector3()
        for axis, value in zip(("x", "y", "z"), force):
            setattr(wrench.force, axis, value * max_force)

        torque = [
            -deadzone_corrected_axes[2],
            deadzone_corrected_axes[3],
            1.0 if msg.buttons[4] == 1 else (-1.0 if msg.buttons[5] == 1 else 0.0),
        ]
        wrench.torque = Vector3()
        for axis, value in zip(("x", "y", "z"), torque):
            setattr(wrench.torque, axis, value * max_torque)

        self.wrench_publisher.publish(wrench)


def main(args=None):
    rclpy.init(args=args)

    joystick_to_wrench = JoystickToWrench()

    rclpy.spin(joystick_to_wrench)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick_to_wrench.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()