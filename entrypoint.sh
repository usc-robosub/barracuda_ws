
#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Build on first run, or when selected packages are missing from install.
SHOULD_BUILD=0
if [ ! -f /root/barracuda_ws/install/setup.bash ]; then
  SHOULD_BUILD=1
elif [ -n "${PKG_SEL}" ]; then
  for pkg in ${PKG_SEL}; do
    if [ -f "/root/barracuda_ws/src/${pkg}/package.xml" ]; then
      PKG_NAME=$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "/root/barracuda_ws/src/${pkg}/package.xml" | head -n 1)
      if [ -n "${PKG_NAME}" ] && [ ! -d "/root/barracuda_ws/install/${PKG_NAME}" ]; then
        echo "Missing installed package: ${PKG_NAME} (from ${pkg}), rebuilding workspace..."
        SHOULD_BUILD=1
        break
      fi
    fi
  done
fi

if [ "${SHOULD_BUILD}" = "1" ]; then
  echo "First run: building workspace..."
  cd /root/barracuda_ws

  PKG_PATHS=""
  PKG_SKIP="zed_debug isaac_ros_nvblox nvblox_examples_bringup nvblox_image_padding nvblox_ros nvblox_msgs nvblox_ros_common nvblox_ros_python_utils nvblox_rviz_plugin nvblox_nav2 realsense_splitter semantic_label_conversion"
  INCLUDE_ZED_WRAPPER=0

  if [ -n "${PKG_SEL}" ]; then
    for pkg in ${PKG_SEL}; do
      if [ "${pkg}" != "barracuda_onboard" ] && [ -f "/root/barracuda_ws/src/${pkg}/package.xml" ]; then
        PKG_NAME=$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "/root/barracuda_ws/src/${pkg}/package.xml" | head -n 1)
        PKG_PATHS="${PKG_PATHS} ${PKG_NAME:-$pkg}"
        if [ "${pkg}" = "barracuda_camera" ]; then
          INCLUDE_ZED_WRAPPER=1
        fi
      fi
    done

    PKG_PATHS="barracuda_onboard${PKG_PATHS:+${PKG_PATHS}}"
    if [ "${INCLUDE_ZED_WRAPPER}" = "1" ] && [ -f "/root/barracuda_ws/src/zed-ros2-wrapper/zed_wrapper/package.xml" ]; then
      PKG_PATHS="${PKG_PATHS} zed_wrapper"
    fi
  fi

  cd /root/barracuda_ws/src
  rosdep install --from-paths . -y --ignore-src --skip-keys="ament_python bluerobotics-ping isaac_ros_dnn_image_encoder isaac_ros_gxf isaac_ros_peoplesemseg_models_install isaac_ros_test isaac_ros_triton isaac_ros_unet isaac_ros_visual_slam nova_carter_navigation"

  cd /root/barracuda_ws
  colcon build --symlink-install --packages-skip ${PKG_SKIP} ${PKG_PATHS:+--packages-up-to ${PKG_PATHS}}
  cd -
fi

pip install bluerobotics-ping

source /root/barracuda_ws/install/setup.bash

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
else
  echo "launching: barracuda_onboard.launch.py"
  ros2 launch barracuda_onboard barracuda_onboard.launch.py
fi

exec "$@"
