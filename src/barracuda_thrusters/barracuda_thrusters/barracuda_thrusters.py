import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float64

from . import teensy


class BarracudaThrusters(Node):
    def __init__(self):
        super().__init__("barracuda_thrusters")

        self.n_thrusters = 8

        cmd_thrust_subscription = self.create_subscription(
            JointState, "cmd_thrust", self.joint_state_subscriber_callback, 10
        )

        # killswitch gpio setup #
        #########################
        try:
            import Jetson.GPIO as GPIO
            self.killswitch_pin = 7
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.killswitch_pin, GPIO.IN)
            GPIO.remove_event_detect(self.killswitch_pin)

            # def write_to_killswitch_regs(killed):
            def write_to_killswitch_regs(channel):
                self.get_logger().info(
                    f"killswitch signal is now {'lo' if GPIO.input(self.killswitch_pin) == GPIO.LOW  else 'hi'}"
                )
                for addr in teensy.i2c_addresses:
                    teensy.write_i2c_char(addr, teensy.KILLSWITCH_REG, '0'.encode() if GPIO.input(self.killswitch_pin) == GPIO.LOW else '1'.encode())

            # killed reg on teensys is set to '1' by default - if the latch is closed on node startup,
            # this line sets the killed reg on teensys to '0' to enable the thrusters
            if GPIO.input(self.killswitch_pin) == GPIO.LOW:
                write_to_killswitch_regs("0".encode())

            # "pressed": killswitch pin went lo (latch was closed) --> set killed = '0'
            # "released": killswitch pin went hi (latch was opened)--> set killed = '1'
            GPIO.add_event_detect(
                self.killswitch_pin,
                GPIO.BOTH,
                callback=write_to_killswitch_regs
            )

        except Exception as e:
            self.get_logger().warn(f"problem with gpio setup: {e}")

    def joint_state_subscriber_callback(self, msg):
        thruster_efforts = msg.effort

        for thruster_idx in range(self.n_thrusters):
            # writes to teensy 0 for thrusters 0-3, teensy 1 for thrusters 4-7
            try:
                teensy.write_i2c_float(
                    teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)],
                    teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)],
                    thruster_efforts[thruster_idx],
                )
            except Exception as e:
                self.get_logger().warning(
                    f"Write failed at addr {teensy.i2c_addresses[thruster_idx // (self.n_thrusters // 2)]:#04x}, reg {teensy.thruster_registers[thruster_idx % (self.n_thrusters // 2)]}: {e}"
                )


def main():
    rclpy.init()

    barracuda_thrusters = BarracudaThrusters()

    rclpy.spin(barracuda_thrusters)

    GPIO.cleanup()

    barracuda_thrusters.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
