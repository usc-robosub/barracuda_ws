#!/usr/bin/env python3

"""
Water Linked DVL A50 ROS Driver

This ROS node interfaces with the Water Linked DVL A50 using the TCP JSON protocol.
It publishes odometry and pose information and provides a service to control acoustics.
"""

import json
import socket
from math import cos, sin

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Range
from std_srvs.srv import SetBool
import tf2_ros


class WaterLinkedDVLDriver(Node):
    """ROS 2 driver for Water Linked DVL A50"""

    def __init__(self):
        super().__init__('waterlinked_dvl_driver')

        # Parameters
        self.declare_parameter('dvl_host', '192.168.8.148')
        self.declare_parameter('dvl_port', 16171)
        self.declare_parameter('client_address', '0.0.0.0')
        self.declare_parameter('frame_id', 'dvl_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('connection_timeout', 5.0)
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('topics.odometry', 'dvl/odometry')
        self.declare_parameter('topics.pose', 'dvl/pose')
        self.declare_parameter('topics.altitude', 'dvl/altitude')
        self.declare_parameter('services.acoustic_control', 'dvl/set_acoustic_enabled')

        self.dvl_host = self.get_parameter('dvl_host').value
        self.dvl_port = int(self.get_parameter('dvl_port').value)
        self.client_address = self.get_parameter('client_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_tf = self._get_bool_param('publish_tf', True)
        self.connection_timeout = float(self.get_parameter('connection_timeout').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)
        self.topic_odom = self.get_parameter('topics.odometry').value
        self.topic_pose = self.get_parameter('topics.pose').value
        self.topic_alt = self.get_parameter('topics.altitude').value
        self.srv_acoustic = self.get_parameter('services.acoustic_control').value

        self.get_logger().info(f'Parameters loaded: dvl_host={self.dvl_host}, dvl_port={self.dvl_port}, client_address={self.client_address}')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, self.topic_odom, 10)
        self.pose_pub = self.create_publisher(PoseWithCovariance, self.topic_pose, 10)
        self.altitude_pub = self.create_publisher(Range, self.topic_alt, 10)

        # Services
        self.acoustic_service = self.create_service(SetBool, self.srv_acoustic, self.set_acoustic_enabled_callback)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        # Socket and connection management
        self.socket = None
        self.connected = False
        self.running = True
        self._recv_buffer = ""

        # Data storage
        self.last_velocity_msg = None
        self.last_position_msg = None

        self.get_logger().info('Water Linked DVL Driver initialized')
        self.get_logger().info(f'Connecting to DVL at {self.dvl_host}:{self.dvl_port}')
        self.get_logger().info(f'Client address: {self.client_address}')
        
        # Start connection and timer
        self.connect_to_dvl()
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz polling

    def _get_bool_param(self, name, default):
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value) if value is not None else default
        
    def connect_to_dvl(self):
        """Establish TCP connection to DVL"""
        try:
            # Close any existing socket
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # Bind to local address if specified
            if self.client_address and self.client_address != '0.0.0.0':
                self.socket.bind((self.client_address, 0))
            
            # Connect with timeout (blocking)
            self.socket.settimeout(self.connection_timeout)
            self.socket.connect((self.dvl_host, self.dvl_port))
            
            # Set to non-blocking after successful connect
            self.socket.setblocking(False)
            self.connected = True
            self._recv_buffer = ""
            self.get_logger().info(f'Connected to DVL at {self.dvl_host}:{self.dvl_port}')
            return True
                
        except socket.error as e:
            self.get_logger().warning(f'Failed to connect to DVL: {e}')
            self.connected = False
            if self.socket:
                try:
                    self.socket.close()
                except Exception:
                    pass
                self.socket = None
            return False
    
    def disconnect_from_dvl(self):
        """Close TCP connection to DVL"""
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None
        self.connected = False
        self._recv_buffer = ""
    
    def get_data(self):
        """Read available data and return next complete JSON line, or None"""
        if not self.connected or not self.socket:
            return None
            
        try:
            chunk = self.socket.recv(4096).decode('utf-8', errors='ignore')
            if not chunk:
                # Remote closed the socket
                self.get_logger().warning('Socket closed by DVL; reconnecting...')
                self.disconnect_from_dvl()
                return None
            self._recv_buffer += chunk
        except BlockingIOError:
            # No data available right now
            return None
        except socket.error as e:
            self.get_logger().warning(f'Socket error: {e}; reconnecting...')
            self.disconnect_from_dvl()
            return None
        
        # Check if we have a complete line
        if '\n' in self._recv_buffer:
            line, _sep, rest = self._recv_buffer.partition('\n')
            self._recv_buffer = rest
            return line.strip()
        
        return None
    
    def timer_callback(self):
        """Timer callback to poll for DVL data"""
        # Check connection
        if not self.connected:
            # Try to reconnect
            if self.connect_to_dvl():
                return
            else:
                # Don't spam logs, reconnect attempt built into connect_to_dvl
                return
        
        # Read and process data
        raw_line = self.get_data()
        if raw_line is None:
            return
        
        if not raw_line:
            return
        
        try:
            json_data = json.loads(raw_line)
            
            # Route data based on type
            msg_type = json_data.get('type')
            if msg_type == 'velocity':
                self.parse_velocity_report(json_data)
            elif msg_type == 'position_local':
                self.parse_position_report(json_data)
            elif msg_type == 'response':
                self.get_logger().debug(f"Received response: {json_data}")
                
        except json.JSONDecodeError as e:
            self.get_logger().warning(f'JSON parse error: {e}; line: {raw_line}')
    
    def send_command(self, command_dict):
        """Send JSON command to DVL and return response"""
        try:
            if not self.connected or not self.socket:
                return None
            
            command_str = json.dumps(command_dict) + '\n'
            self.socket.send(command_str.encode('utf-8'))
            
            # Read response (blocking for commands)
            self.socket.setblocking(True)
            response_data = ""
            while True:
                chunk = self.socket.recv(1024).decode('utf-8')
                if not chunk:
                    break
                response_data += chunk
                if '\n' in response_data:
                    break
            self.socket.setblocking(False)
            
            if response_data.strip():
                return json.loads(response_data.strip())
            return None
                
        except (socket.error, json.JSONDecodeError) as e:
            self.get_logger().warning(f'Error sending command: {e}')
            self.connected = False
            return None
    
    def set_acoustic_enabled_callback(self, req):
        """Service callback to enable/disable acoustics"""
        response = SetBool.Response()

        command = {
            "command": "set_config",
            "parameters": {
                "acoustic_enabled": req.data
            }
        }
        
        result = self.send_command(command)
        
        if result and result.get('success', False):
            response.success = True
            response.message = f"Acoustics {'enabled' if req.data else 'disabled'} successfully"
            self.get_logger().info(response.message)
        else:
            response.success = False
            error_msg = result.get('error_message', 'Unknown error') if result else 'Communication error'
            response.message = f"Failed to set acoustics: {error_msg}"
            self.get_logger().warning(response.message)
        
        return response
    
    def parse_velocity_report(self, data):
        """Parse velocity-and-transducer report and publish odometry"""
        try:
            if data.get('type') != 'velocity':
                return
            
            current_time = self.get_clock().now().to_msg()
            
            # Create odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = self.odom_frame_id
            odom_msg.child_frame_id = self.frame_id
            
            # Velocity data (body frame)
            odom_msg.twist.twist.linear.x = data.get('vx', 0.0)
            odom_msg.twist.twist.linear.y = data.get('vy', 0.0)
            odom_msg.twist.twist.linear.z = data.get('vz', 0.0)
            
            # Covariance matrix for velocity
            covariance = data.get('covariance', [[0]*3 for _ in range(3)])
            if len(covariance) == 3 and len(covariance[0]) == 3:
                # Fill 6x6 covariance matrix (only linear velocities)
                twist_cov = [0.0] * 36
                for i in range(3):
                    for j in range(3):
                        twist_cov[i*6 + j] = covariance[i][j]
                odom_msg.twist.covariance = twist_cov
            
            # Publish odometry
            self.odom_pub.publish(odom_msg)
            
            # Publish altitude as Range message
            if data.get('velocity_valid', False) and 'altitude' in data:
                range_msg = Range()
                range_msg.header.stamp = current_time
                range_msg.header.frame_id = self.frame_id
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.1  # Approximate beam width
                range_msg.min_range = 0.05
                range_msg.max_range = 50.0
                range_msg.range = data['altitude']
                self.altitude_pub.publish(range_msg)
            
            self.last_velocity_msg = odom_msg
            
            self.get_logger().debug(
                f"Published velocity: vx={data.get('vx', 0):.3f}, "
                f"vy={data.get('vy', 0):.3f}, vz={data.get('vz', 0):.3f}"
            )
            
        except Exception as e:
            self.get_logger().warning(f'Error parsing velocity report: {e}')
    
    def parse_position_report(self, data):
        """Parse dead-reckoning report and publish pose"""
        try:
            if data.get('type') != 'position_local':
                return
            
            current_time = self.get_clock().now().to_msg()
            
            # Create pose message
            pose_msg = PoseWithCovariance()
            
            # Position
            pose_msg.pose.position.x = data.get('x', 0.0)
            pose_msg.pose.position.y = data.get('y', 0.0)
            pose_msg.pose.position.z = data.get('z', 0.0)
            
            # Orientation from roll, pitch, yaw (in degrees)
            roll = data.get('roll', 0.0) * 0.017453292519943295
            pitch = data.get('pitch', 0.0) * 0.017453292519943295
            yaw = data.get('yaw', 0.0) * 0.017453292519943295

            # Convert to quaternion
            quat = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            # Covariance (simplified - using std as diagonal elements)
            std = data.get('std', 0.01)
            pose_cov = [0.0] * 36
            # Position covariance
            pose_cov[0] = std * std   # x
            pose_cov[7] = std * std   # y
            pose_cov[14] = std * std  # z
            # Orientation covariance (rough estimate)
            pose_cov[21] = 0.01  # roll
            pose_cov[28] = 0.01  # pitch
            pose_cov[35] = 0.01  # yaw
            pose_msg.covariance = pose_cov
            
            # Publish pose
            self.pose_pub.publish(pose_msg)
            
            # Publish TF if enabled
            if self.publish_tf:
                transform = TransformStamped()
                transform.header.stamp = current_time
                transform.header.frame_id = self.odom_frame_id
                transform.child_frame_id = self.frame_id
                
                transform.transform.translation.x = pose_msg.pose.position.x
                transform.transform.translation.y = pose_msg.pose.position.y
                transform.transform.translation.z = pose_msg.pose.position.z
                
                transform.transform.rotation = pose_msg.pose.orientation

                self.tf_broadcaster.sendTransform(transform)
            
            self.last_position_msg = pose_msg
            
            self.get_logger().debug(
                f"Published pose: x={data.get('x', 0):.3f}, "
                f"y={data.get('y', 0):.3f}, z={data.get('z', 0):.3f}"
            )
            
        except Exception as e:
            self.get_logger().warning(f'Error parsing position report: {e}')
    
    def run(self):
        """Main execution function"""
        self.get_logger().info('DVL driver started. Publishing on topics:')
        self.get_logger().info(f'  - {self.topic_odom} (nav_msgs/Odometry)')
        self.get_logger().info(f'  - {self.topic_pose} (geometry_msgs/PoseWithCovariance)')
        self.get_logger().info(f'  - {self.topic_alt} (sensor_msgs/Range)')
        self.get_logger().info('Services available:')
        self.get_logger().info(f'  - {self.srv_acoustic} (std_srvs/SetBool)')
        
        # Keep the node running
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down DVL driver...')
        finally:
            self.running = False
            self.disconnect_from_dvl()
            self.destroy_node()


def quaternion_from_euler(roll, pitch, yaw):
    """Convert roll, pitch, yaw (rad) to quaternion (x, y, z, w)."""
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def main():
    """Main entry point"""
    rclpy.init()
    try:
        driver = WaterLinkedDVLDriver()
        driver.run()
    except Exception as e:
        rclpy.logging.get_logger('waterlinked_dvl_driver').error(
            f'Failed to start DVL driver: {e}'
        )
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
