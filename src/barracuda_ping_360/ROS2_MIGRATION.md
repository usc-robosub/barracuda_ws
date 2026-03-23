# ROS2 Migration Guide

This document outlines the migration from ROS1 to ROS2 for the Barracuda Ping360 Sonar project.

## Branch Information
- **Branch Name**: `ros2`
- **Based On**: Original project with ROS1 support
- **compatible with**: ROS Humble (Debian-based with Ubuntu 22.04)

## Key Changes Made

### 1. Package Configuration Files

#### package.xml (ROS2 Format 3)
- Changed format from `format="2"` to `format="3"`
- Updated buildtool from `catkin` to `ament_python`
- Replaced `rospy` with `rclpy`
- Removed `dynamic_reconfigure` dependencies
- Added `rosidl_default_generators` and `rosidl_default_runtime`
- Version bumped to 2.0.0

#### CMakeLists.txt
- Updated to use `ament_cmake` and `ament_cmake_python`
- Replaced catkin-based message generation with rosidl-based
- Uses `ament_python_install_package()` for Python packages
- Proper installation paths for ROS2

#### setup.py
- Converted from catkin-based setup to setuptools
- Added entry points for console scripts
- Updated Python version to Python 3
- Included resource package marker

### 2. Node Implementation

#### node.py (rclpy replacement)
- Replaced `rospy` import with `rclpy`
- Converted node logic from functional to class-based (inherits from `rclpy.Node`)
- Parameter management:
  - Replaced `rospy.get_param()` with `self.declare_parameter()` and `self.get_parameter()`
  - All parameters are now declared in `__init__`
- Message Publishing:
  - Replaced `rospy.Publisher()` with `self.create_publisher()`
  - Updated header generation to use `self.get_clock().now().to_msg()`
- Timer-based loop:
  - Replaced `rospy.Rate()` with `self.create_timer()`
  - Changed from while loop to callback-based execution
- Logging:
  - Replaced `rospy.loginfo/logerr/logwarn` with `self.get_logger()`
- Main entry point:
  - Added standalone `main()` function using `rclpy.init()` and `rclpy.spin()`

### 3. Launch Files

#### ping360.launch.py
- Converted from ROS1 XML launch format to ROS2 Python launch format
- Uses `launch` and `launch_ros` packages
- Parameter declaration via `DeclareLaunchArgument`
- Node launching via `launch_ros.actions.Node`
- Supports namespace and dynamic parameter passing

#### barracuda_ping_360/launch/ping360.launch.py
- Meta-package launch file
- Includes ping360_sonar launch file
- Provides namespace and device configuration

### 4. Docker Support

#### Dockerfile
- Base image changed from `ros:melodic-ros-base-bionic` to `ros:humble-ros-base`
- Build system changed from catkin to colcon
- Updated system dependencies to Python3-based packages
- Automatic build during container creation

#### docker-compose.yml
- Removed ROS1-specific `ROS_MASTER_URI` variable
- Added `ROS_DISTRO=humble`
- Maintains RMW_IMPLEMENTATION and ROS_DOMAIN_ID for DDS configuration

#### entrypoint.sh
- Updated ROS distribution from melodic to humble
- Changed build system from `catkin build` to `colcon build`
- Updated sourcing from `devel/setup.bash` to `install/setup.bash`
- Updated launch command to use `ros2 launch`

### 5. Custom Messages

#### SonarEcho.msg
- Remains mostly unchanged but now uses rosidl IDL format
- Uses standard ROS2 header format
- All field names follow snake_case convention

## Building the Project

### Prerequisites
- ROS2 Humble installed (or other recent LTS)
- colcon-common-extensions installed
- Python 3.8 or later

### Build Instructions

#### Local Build
```bash
# Navigate to workspace
cd catkin_ws

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Build packages
colcon build --symlink-install

# Source the built workspace
source install/setup.bash
```

#### Docker Build
```bash
# Build the Docker image
docker build -t barracuda-ping360:ros2 .

# Run the container
docker run --device=/dev/ttyUSB0 -it barracuda-ping360:ros2
```

#### Docker Compose
```bash
docker-compose up --build
```

## Running the Node

### Launch from Source
```bash
# Terminal 1: Source the workspace
source install/setup.bash
ros2 launch barracuda_ping_360 ping360.launch.py

# With custom parameters
ros2 launch barracuda_ping_360 ping360.launch.py device:=/dev/ttyUSB0 namespace:=barracuda
```

### Command Line Node
```bash
# Run the node directly
ros2 run ping360_sonar ping360.py

# Run with parameters
ros2 run ping360_sonar ping360.py --ros-args -p device:=/dev/ttyUSB0 -p range_max:=10
```

## ROS2 Parameter System

Unlike ROS1's dynamic reconfigure, ROS2 uses a simpler parameter system. Parameters can be:
- Set via launch files
- Modified at runtime with `ros2 param set`
- Declared with default values in the node

### Available Parameters
- `device`: Serial device port (default: `/dev/ttyUSB0`)
- `baudrate`: Serial baudrate (default: `115200`)
- `range_max`: Maximum sonar range in meters (default: `2`)
- `angle_step`: Angular resolution in degrees (default: `1`)
- `gain`: Sonar gain 0-2 (default: `0`)
- `frequency`: Sonar frequency in kHz (default: `740`)
- `speed_of_sound`: Speed of sound in m/s (default: `1500`)
- `number_of_samples`: Number of samples per scan (default: `200`)
- `frame`: TF frame ID (default: `sonar_frame`)
- `publish_image`: Enable image publishing (default: `True`)
- `publish_scan`: Enable LaserScan publishing (default: `True`)
- `publish_echo`: Enable raw echo publishing (default: `False`)
- `angle_sector`: Angular sector in degrees (default: `360`)
- `scan_threshold`: Intensity threshold for LaserScan (default: `200`)
- `image_size`: Sonar image size in pixels (default: `300`)

## ROS2 Topics

### Subscribed Topics
None

### Published Topics
- `/barracuda/scan_image` (sensor_msgs/Image): Sonar image visualization
- `/barracuda/echo` (ping360_sonar/SonarEcho): Raw sonar echo data
- `/barracuda/scan` (sensor_msgs/LaserScan): Detected objects as LaserScan

## Migration Checklist

- [x] Convert package.xml to ROS2 format
- [x] Update CMakeLists.txt for ROS2
- [x] Convert setup.py for ROS2
- [x] Rewrite node.py using rclpy
- [x] Create Python launch files
- [x] Update Dockerfile for ROS2
- [x] Update docker-compose.yml
- [x] Update entrypoint.sh
- [x] Create comprehensive documentation
- [ ] Add unit tests
- [ ] Integration testing with actual hardware
- [ ] Documentation on message interfaces

## Known Issues & Notes

### Python Module Structure
- Module relative imports have been updated for rclpy
- Make sure to run from properly sourced environment

### Dynamic Reconfiguration
- ROS2 doesn't have dynamic_reconfigure
- Simple parameter system used instead
- For dynamic updates, use `ros2 param set` command or create parameter change callbacks

### Parameter Remapping
- ROS2 uses different remapping syntax in launch files
- Topics are remapped using `remappings` parameter in Node declaration

## Testing

### Unit Tests
```bash
# Run pytest on the package
cd catkin_ws
colcon test --packages-select ping360_sonar
```

### Manual Testing
```bash
# Check node is running
ros2 node list

# Inspect parameters
ros2 param list /ping360_node

# Set a parameter at runtime
ros2 param set /ping360_node debug true

# Listen to topics
ros2 topic echo /barracuda/scan
```

## References

- [ROS2 Documentation](https://docs.ros.org/)
- [ROS2 Python Client Library](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [ROS2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)
- [ROS1 to ROS2 Migration Guide](https://docs.ros.org/en/humble/Guides/ROS-1-Migration-Guide.html)

## Support

For issues or questions about the ROS2 migration, please refer to:
- Official ROS2 documentation
- GitHub issues in this repository
