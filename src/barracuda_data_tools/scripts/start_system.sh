mkdir -p ~/robot_ws/scripts
cat > ~/robot_ws/scripts/start_system.sh <<'EOF'
#!/bin/bash

# load ROS2 and workspace
source /opt/ros/humble/setup.bash
if [ -f ~/robot_ws/install/setup.bash ]; then
  source ~/robot_ws/install/setup.bash
fi

echo "Starting ZED wrapper..."
# Run in foreground so Ctrl+C cleanly shuts down all launched ROS2 nodes
exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm enable_positional_tracking:=true
EOF

# 可执行权限
chmod +x ~/robot_ws/scripts/start_system.sh