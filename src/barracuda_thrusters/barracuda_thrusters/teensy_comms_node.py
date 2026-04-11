import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from .t200_thruster_constants import T200
from .teensys import Teensys


class TeensyComms(Node):
    def __init__(self):
        super().__init__("teensy_comms")

        cmd_thrust_subscription = self.create_subscription(
            JointState, "cmd_thrust", self.joint_state_subscriber_callback, 10
        )

        # this is just here to avoid unused var warning
        cmd_thrust_subscription

        self.teensys = Teensys()
        

    def joint_state_subscriber_callback(self, msg: JointState):
        # msg.effort is list of desired force/thrust ouputs for thrusters, should have same length as number
        # of total thrusters specified by T200.NTHRUSTERS
        if len(msg.effort) != T200.NTHRUSTERS:
            self.get_logger.error("unexpected number of thrusters in effort field of JointState messages published to cmd_thrust topic")
        self.teensys.set_pwm_outputs(msg.effort)


def main():
    rclpy.init()

    teensy_comms = TeensyComms()

    rclpy.spin(teensy_comms)

    teensy_comms.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
