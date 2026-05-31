# Barracuda Ping360 (ROS + Docker)

Minimal ROS wrapper and Dockerized setup for the Blue Robotics Ping360 sonar.
It bundles the upstream `ping360_sonar` ROS package and provides a ready-to-run
launch file plus a container that exposes device access and ROS networking.

## Overview
- Uses ROS Melodic (Ubuntu 18.04 base) via Docker.
- Includes `catkin_ws` with:
  - `barracuda_ping_360`: launch + packaging.
  - `ping360_sonar`: upstream driver (as a Git submodule).
- Publishes sonar topics under the `barracuda` namespace.

## Requirements
- Docker and Docker Compose installed.
- Ping360 connected to the host (default `/dev/ttyUSB0`).
- Linux user has permission for the serial device (often `dialout` group).

## Quick Start (Docker)
1) Plug the Ping360 and identify its device path (e.g. `/dev/ttyUSB0`).
2) Update `docker-compose.yml` if your device path differs.
3) Build and run:
   - `docker compose up --build` (Docker Compose v2)

The container will:
- Build the workspace with `catkin build`.
- Launch: `roslaunch barracuda_ping_360 launch_ping360.launch --wait`.

By default, the launch uses:
- Device: `/dev/ttyUSB0`
- Baudrate: `115200`
- Namespace: `barracuda`

To emulate without hardware, edit `catkin_ws/src/barracuda_ping_360/launch/launch_ping360.launch`
and set `<env name="emulated_sonar" value="true" />`.

## Topics
With the default launch, topics are remapped and appear under:
- `sonar/images` (sensor_msgs/Image)
- `sonar/data` (custom SonarEcho msg from upstream)
- `sonar/scan` (sensor_msgs/LaserScan)


## Configuration
Modify parameters in `launch/launch_ping360.launch`:
- `device`, `baudrate`, `sonarRange`, `transmitFrequency`, `speedOfSound`
- `numberOfSamples`, `step`, `gain`, `threshold`, `imgSize`
- `enableImageTopic`, `enableScanTopic`, `enableDataTopic`
- `minAngle`, `maxAngle` (in grads), `oscillate`

Refer to upstream docs for parameter semantics:
`catkin_ws/src/ping360_sonar/README.md` and Blue Robotics Ping360 documentation.


## Troubleshooting
- Permission denied on `/dev/ttyUSB0`:
  - Add your user to `dialout`: `sudo usermod -aG dialout $USER` (relog).
- No topics visible from host:
  - Ensure `network_mode: host` is used and master/ROS IP env vars match your setup.
- Different serial path:
  - Update `docker-compose.yml` device mapping and the launch `device` param.
- Submodule missing after fresh clone:
  - Run `git submodule update --init --recursive`.

## License and Credits
- This repo: see `LICENSE`.
- Driver package: `ping360_sonar` (MIT) by Centrale Nantes Robotics.
- Uses Blue Robotics `bluerobotics-ping` Python library.

