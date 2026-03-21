#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

echo "============================"
echo " Barracuda Workspace Ready! "
echo "============================"

ros2 daemon stop
ros2 daemon start
if [ -n "${IMG_ID}" ]
then echo "IMG_ID environment variable set: ${IMG_ID}"
fi

if [ -n "${PKG_SEL}" ]
then echo "PKG_SEL environment variable set: ${PKG_SEL}"
fi

if [ -n "${NO_LAUNCH}" ]
then echo "NO_LAUNCH environment variable set" && exec /bin/bash
else echo "launching: barracuda_onboard.launch.py" && ros2 launch barracuda_onboard barracuda_onboard.launch.py
fi

exec "$@"
