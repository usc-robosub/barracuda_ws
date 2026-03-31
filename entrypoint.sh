#!/bin/bash
set -e

# Workspace root used by build and runtime setup.
WS_ROOT="/home/ros/barracuda_ws"

# Fail early if mounted paths are not writable by the non-root runtime user.
for path in \
    "$WS_ROOT/build" \
    "$WS_ROOT/install" \
    "$WS_ROOT/log" \
    "$WS_ROOT/third-party/foxglove-sdk/ros/src/foxglove_msgs/msg"
do
    if [ -e "$path" ] && [ ! -w "$path" ]; then
        echo "Path is not writable by the container user: $path"
        echo "On the host, run:"
        echo "  sudo chown -R \$(id -u):\$(id -g) build install log third-party/foxglove-sdk/ros/src/foxglove_msgs/msg"
        exit 1
    fi
done

# Source base ROS and Foxglove environments.
source /opt/ros/humble/setup.bash
source /opt/foxglove/ros/install/local_setup.bash

# Detect stale artifacts from previous /root workspace path.
if [ -d "$WS_ROOT/build" ] && find "$WS_ROOT/build" -name CMakeCache.txt -exec grep -l "/root/barracuda_ws" {} + | grep -q .; then
    echo "Stale build artifacts detected."
    echo "This workspace was previously built under /root/barracuda_ws and must be cleaned once after moving to /home/ros/barracuda_ws."
    echo "On the host, run:"
    echo "  rm -rf build install log"
    echo "Then restart the container."
    exit 1
fi

# Build workspace at startup when source volume is mounted.
if [ -d "$WS_ROOT/src" ] && [ "$(ls -A "$WS_ROOT/src")" ]; then
    echo "Building barracuda workspace..."
    cd "$WS_ROOT/src"
    export PKG_PATHS=${PKG_SEL:+barracuda_onboard $PKG_SEL}
    echo "Skipping rosdep install because the container now builds as the non-root ros user."
    echo "Rebuild the image if new system dependencies were added."
    cd "$WS_ROOT"
    colcon build --symlink-install ${PKG_PATHS:+--packages-select $PKG_PATHS}
    echo "Build complete."
fi

# Source workspace overlays after build.
source "$WS_ROOT/install/setup.bash"

# Startup banner.
echo "============================"
echo " Barracuda Workspace Ready! "
echo "============================"

# Reset ROS 2 daemon to avoid stale graph state.
ros2 daemon stop
ros2 daemon start
if [ -n "${IMG_ID}" ]
then echo "IMG_ID environment variable set: ${IMG_ID}"
fi

if [ -n "${PKG_SEL}" ]
then echo "PKG_SEL environment variable set: ${PKG_SEL}"
fi

# Launch behavior: shell-only mode when NO_LAUNCH is set, otherwise onboard launch.
if [ -n "${NO_LAUNCH}" ]
then echo "NO_LAUNCH environment variable set" && exec /bin/bash
else echo "launching: barracuda_onboard.launch.py" && ros2 launch barracuda_onboard barracuda_onboard.launch.py
fi

# Fallback exec for additional command args.
exec "$@"
