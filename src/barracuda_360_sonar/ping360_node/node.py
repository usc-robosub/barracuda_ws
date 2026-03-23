import os
from math import cos, pi, sin

import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header

# Import from bluerobotics-ping library
from brping import Ping360


class Ping360Node(Node):
    """ROS2 Node for Ping360 Sonar"""

    def __init__(self):
        super().__init__('ping360_node')

        # Declare parameters
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('connection_type', 'serial')
        self.declare_parameter('udp_address', '0.0.0.0')
        self.declare_parameter('udp_port', 12345)
        self.declare_parameter('debug', False)
        self.declare_parameter('range_max', 2)
        self.declare_parameter('angle_step', 1)
        self.declare_parameter('gain', 0)
        self.declare_parameter('frequency', 740)
        self.declare_parameter('speed_of_sound', 1500)
        self.declare_parameter('number_of_samples', 200)
        self.declare_parameter('frame', 'sonar_frame')
        self.declare_parameter('publish_image', True)
        self.declare_parameter('publish_scan', True)
        self.declare_parameter('publish_echo', False)
        self.declare_parameter('fallback_emulated', True)
        self.declare_parameter('angle_sector', 360)
        self.declare_parameter('scan_threshold', 200)
        self.declare_parameter('image_size', 300)
        self.declare_parameter('sonar_timeout', 8000)

        # Get parameters
        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        self.connection_type = self.get_parameter('connection_type').value
        self.udp_address = self.get_parameter('udp_address').value
        self.udp_port = self.get_parameter('udp_port').value
        self.debug = self.get_parameter('debug').value
        self.range_max = self.get_parameter('range_max').value
        self.step = self.get_parameter('angle_step').value
        self.gain = self.get_parameter('gain').value
        self.frequency = self.get_parameter('frequency').value
        self.speed_of_sound = self.get_parameter('speed_of_sound').value
        self.number_of_samples = self.get_parameter('number_of_samples').value
        self.frame_id = self.get_parameter('frame').value
        self.publish_image = self.get_parameter('publish_image').value
        self.publish_scan = self.get_parameter('publish_scan').value
        self.publish_echo = self.get_parameter('publish_echo').value
        self.fallback_emulated = self.get_parameter('fallback_emulated').value
        self.angle_sector = self.get_parameter('angle_sector').value
        self.scan_threshold = self.get_parameter('scan_threshold').value
        self.image_size = self.get_parameter('image_size').value
        self.sonar_timeout = self.get_parameter('sonar_timeout').value

        # Calculate sonar parameters
        self.sample_period = self.calculate_sample_period(
            self.range_max, self.number_of_samples, self.speed_of_sound
        )
        self.transmit_duration = self.adjust_transmit_duration(
            self.range_max, self.sample_period, self.speed_of_sound
        )

        # Field of view calculations
        self.max_angle = self.angle_sector // 2
        self.min_angle = -self.max_angle if self.angle_sector == 360 else 0
        self.fov = self.max_angle - self.min_angle
        self.oscillate = True if self.angle_sector < 360 else False
        self.current_angle = self.min_angle
        self.angle_sign = 1

        # Validation
        if self.fov <= 0:
            self.get_logger().error(
                f"Invalid angle configuration: min_angle={self.min_angle}, max_angle={self.max_angle}"
            )
            raise ValueError("Invalid angle configuration")

        if self.step >= self.fov:
            self.get_logger().error(
                f"Step ({self.step}) is larger than FOV ({self.fov})"
            )
            raise ValueError("Invalid step configuration")

        # Basic device visibility checks before init
        if self.connection_type == 'serial':
            if not os.path.exists(device):
                self.get_logger().error(f"Device path not found: {device}")
            else:
                readable = os.access(device, os.R_OK)
                writable = os.access(device, os.W_OK)
                self.get_logger().info(
                    "Device path OK: %s (readable=%s, writable=%s, euid=%s, egid=%s)"
                    % (device, readable, writable, os.geteuid(), os.getegid())
                )

        # Initialize sensor
        self.sensor = None
        try:
            self.sensor = Ping360()
            if self.connection_type == 'udp':
                connected = self.sensor.connect_udp(self.udp_address, int(self.udp_port))
            else:
                connected = self.sensor.connect_serial(device, int(baudrate))
            self.get_logger().info(f"Sensor connected: {connected} (type={type(connected).__name__})")
            # Some brping versions return None on success; only treat explicit False as failure.
            if connected is False:
                self.get_logger().error("Sensor connection failed")
                self.sensor = None
                return

            init_result = self.sensor.initialize()
            self.get_logger().info(f"Sensor initialized: {init_result}")
            if not init_result:
                self.get_logger().error("Sensor initialize returned false")
                self.sensor = None
            else:
                self._log_device_info()
        except Exception as e:
            self.get_logger().error(
                "Failed to initialize sensor (%s): %r" % (type(e).__name__, e)
            )
            if not self.fallback_emulated:
                raise

        # Publishers
        self.image_pub = self.create_publisher(Image, 'scan_image', 10)
        self.echo_pub = self.create_publisher(Image, 'echo', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Update sonar configuration
        self.update_sonar_config()

        # Create image
        self.image = np.zeros((self.image_size, self.image_size, 1), np.uint8)
        self.bridge = CvBridge()

        # LaserScan state
        self.ranges = [0.0]
        self.intensities = [0.0]

        # Center point
        self.center = (float(self.image_size / 2), float(self.image_size / 2))

        # Create timer for main loop (10 Hz)
        self.create_timer(0.1, self.main_loop)

        self.get_logger().info("Ping360 Node initialized successfully")

    def _log_device_info(self):
        """Best-effort device info probe for debug."""
        for method_name in (
            "read_device_information",
            "readDeviceInformation",
            "request_device_information",
            "get_device_information",
        ):
            if hasattr(self.sensor, method_name):
                try:
                    info = getattr(self.sensor, method_name)()
                    self.get_logger().info(f"Device info ({method_name}): {info}")
                except Exception as e:
                    self.get_logger().warn(f"Device info failed ({method_name}): {e}")
                return
        self.get_logger().warn("No device info method found on Ping360 instance")

    def main_loop(self):
        """Main sensor reading loop"""
        try:
            # Get sonar response
            data = self.get_sonar_data(self.current_angle)

            # Publish raw echo data
            if self.publish_echo and data:
                self.publish_echo_msg(self.current_angle, data)

            # Publish laser scan
            if self.publish_scan and data:
                self.publish_scan_msg(data)

            # Publish image
            if self.publish_image and data:
                self.publish_image_msg(data)

            # Update angle for next iteration
            self.current_angle += self.angle_sign * self.step
            if self.current_angle >= self.max_angle:
                if self.oscillate:
                    self.angle_sign = -1
                else:
                    self.current_angle = self.min_angle
            elif self.current_angle <= self.min_angle and self.oscillate:
                self.angle_sign = 1

        except Exception as e:
            self.get_logger().error(f"Error in main loop: {e}")

    def get_sonar_data(self, angle):
        """Get sonar data for a specific angle"""
        try:
            if self.sensor is None:
                raise RuntimeError("Sensor not initialized")
            # Ensure angle is in valid range (0-400)
            angle = max(0, min(400, int(angle)))
            self.sensor.transmitAngle(angle)
            data = bytearray(getattr(self.sensor, '_data', []))
            return list(data)
        except Exception as e:
            self.get_logger().warn(f"Failed to get sonar data: {e}")
            return [0] * self.number_of_samples

    def publish_echo_msg(self, angle, data):
        """Publish raw echo message"""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # For now, just publish as image (future: create SonarEcho message)
        try:
            echo_array = np.array(data, dtype=np.uint8).reshape(1, -1, 1)
            msg = self.bridge.cv2_to_imgmsg(echo_array, "mono8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.echo_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish echo: {e}")

    def publish_scan_msg(self, data):
        """Publish LaserScan message"""
        # Find first high intensity value
        for i, intensity in enumerate(data):
            if intensity >= self.scan_threshold:
                distance = self.calculate_range(
                    i + 1, self.sample_period, self.speed_of_sound
                )
                if 0.75 <= distance <= self.range_max:
                    self.ranges[0] = distance
                    self.intensities[0] = float(intensity) / 255.0
                    if self.debug:
                        self.get_logger().info(
                            f"Object at {self.current_angle}°: {distance}m - {intensity*100/255:.1f}%"
                        )
                    break

        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 2 * pi * self.min_angle / 400.0
        msg.angle_max = 2 * pi * self.max_angle / 400.0
        msg.angle_increment = 2 * pi * self.step / 400.0
        msg.time_increment = 0.0
        msg.range_min = 0.75
        msg.range_max = float(self.range_max)
        msg.ranges = self.ranges
        msg.intensities = self.intensities
        self.scan_pub.publish(msg)

    def publish_image_msg(self, data):
        """Publish sonar image message"""
        try:
            if len(data) == 0:
                self.get_logger().warn("Empty sonar data, skipping image publish")
                return

            linear_factor = float(len(data)) / float(self.center[0])
            for i in range(int(self.center[0])):
                if i < len(data):
                    point_color = data[int(i * linear_factor - 1)] if (i * linear_factor - 1) >= 0 else 0
                else:
                    point_color = 0

                for k in np.linspace(0, self.step, 8 * self.step):
                    theta = 2 * pi * (self.current_angle + k) / 400.0
                    x = float(i) * cos(theta)
                    y = float(i) * sin(theta)
                    try:
                        self.image[
                            int(self.center[0] + x), int(self.center[1] + y), 0
                        ] = point_color
                    except IndexError:
                        pass

            msg = self.bridge.cv2_to_imgmsg(self.image, "mono8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.image_pub.publish(msg)

        except CvBridgeError as e:
            self.get_logger().warn(f"CV Bridge error: {e}")

    def update_sonar_config(self):
        """Update sonar configuration"""
        try:
            self.sensor.set_gain_setting(int(self.gain))
            self.sensor.set_transmit_frequency(int(self.frequency))
            self.sensor.set_transmit_duration(int(self.transmit_duration))
            self.sensor.set_sample_period(int(self.sample_period))
            self.sensor.set_number_of_samples(int(self.number_of_samples))
        except Exception as e:
            self.get_logger().error(f"Failed to update sonar config: {e}")

    @staticmethod
    def calculate_range(number_of_samples, sample_period, speed_of_sound, sample_period_tick_duration=25e-9):
        """Calculate range based on sample information"""
        return (
            number_of_samples * speed_of_sound * sample_period_tick_duration * sample_period / 2
        )

    @staticmethod
    def calculate_sample_period(distance, number_of_samples, speed_of_sound, sample_period_tick_duration=25e-9):
        """Calculate sample period based on range"""
        return 2 * distance / (number_of_samples * speed_of_sound * sample_period_tick_duration)

    @staticmethod
    def get_sample_period(sample_period, sample_period_tick_duration=25e-9):
        """Get sample period in nanoseconds"""
        return sample_period * sample_period_tick_duration

    @staticmethod
    def transmit_duration_max(sample_period, firmware_max_transmit_duration=500):
        """Get maximum transmit duration"""
        return min(
            firmware_max_transmit_duration,
            Ping360Node.get_sample_period(sample_period) * 64e6
        )

    @staticmethod
    def adjust_transmit_duration(distance, sample_period, speed_of_sound, firmware_min_transmit_duration=5):
        """Adjust transmit duration for specific range"""
        duration = 8000 * distance / speed_of_sound
        transmit_duration = max(
            2.5 * Ping360Node.get_sample_period(sample_period) / 1000, duration
        )
        return max(
            firmware_min_transmit_duration,
            min(Ping360Node.transmit_duration_max(sample_period), transmit_duration)
        )


def main(args=None):
    rclpy.init(args=args)
    node = Ping360Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
