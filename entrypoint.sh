#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

echo "============================"
echo " Barracuda Workspace Ready! "
echo "============================"

ros2 launch barracuda_onboard barracuda_onboard.launch.py

exec "$@"
