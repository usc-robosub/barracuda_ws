# AGENTS.md — barracuda_estimation

Python `ament_python` package. ROS2 Humble. Runs inside Docker (see workspace-root AGENTS.md).

## Package layout

```
barracuda_estimation/
  estimator_node.py       # ROS2 node entry point (subscribes sensors, calls GtsamEstimator)
  gtsam_estimator.py      # GTSAM backend (no ROS deps)
  imu_buffer.py           # IMU preintegration buffer
  measurement_types.py    # ImuSample, DepthSample, DvlSample dataclasses
  state_estimate.py       # StateEstimate dataclass
config/
  ekf_odom.yaml           # robot_localization local EKF (odom → base_link)
  ekf_map.yaml            # robot_localization global EKF (map → odom)
launch/
  barracuda_estimation.launch.py  # starts ekf_odom, ekf_map, estimator_node
```

## Entry points (setup.py console_scripts)

```
estimator_node  → barracuda_estimation.estimator_node:main
```

## Running

```bash
# Full package (EKFs + estimator_node):
ros2 launch barracuda_estimation barracuda_estimation.launch.py

# Offline backend replay (no ROS needed):
python3 experiments/gtsam_minimal/replay_estimator_backend.py
```

## Topic wiring

### estimator_node subscribes

| Parameter | Default topic |
|-----------|--------------|
| `topics.imu` | `/barracuda/zed_node/imu/data` |
| `topics.depth_range` | `/barracuda/depth` |
| `topics.depth_pressure` | `` (empty — use `depth_mode:fluid_pressure` to activate) |
| `topics.dvl_odometry` | `/barracuda/dvl/odometry` |
| `topics.camera_image` | `/barracuda/zed_node/rgb/color/rect/image` |

### estimator_node publishes

| Topic | Type |
|-------|------|
| `/barracuda/estimation/pose` | `geometry_msgs/PoseStamped` |
| `/barracuda/estimation/health` | `std_msgs/Bool` |
| `/barracuda/estimation/debug` | `std_msgs/String` |

### EKF nodes (robot_localization)

- `ekf_odom` currently fuses `/barracuda/zed_node/odom` (velocity) + `/barracuda/zed_node/imu/data` (angular rates)
- Outputs remapped to `/barracuda/odometry/filtered/local` and `/barracuda/odometry/filtered/global`
- Frames: `map`, `barracuda/odom`, `barracuda/base_link`

## Depth mode

`estimator_node` `depth_mode` parameter:
- `"range"` (default) — subscribes `Range` on `topics.depth_range`
- `"fluid_pressure"` — subscribes `FluidPressure` on `topics.depth_pressure`

**Live depth source is unsettled.** The current `ekf_odom.yaml` comment says to replace ZED odom with `/barracuda/dvl/odometry` when DVL driver is back online.

## GTSAM

`gtsam` is hard-imported in `gtsam_estimator.py` — missing package crashes at startup. It is **not** in `package.xml` (not a rosdep-installable ROS dep) and must be available in the Python environment separately.

## GtsamEstimator internals

- `step()` runs `_run_graph_update()` (LevenbergMarquardt batch optimization) every 0.5 s (configurable via `optimize_period_sec`)
- IMU buffer is **cleared after every `step()` call** — do not hold references to old samples
- `_dvl_to_initial_pose3()` builds the initial pose guess from DVL position/orientation only; depth enters as an independent `GPSFactor` via `_depth_factor()`
- Depth z is negated (`z = -depth.z_value`) when passed to the GPSFactor
- `is_ready()` requires non-empty IMU buffer AND latest_depth AND latest_dvl — `health=false` until all three arrive

## Known incomplete areas

- Full IMU+DVL+depth GTSAM backend wired but **not tuned** — replay validation only
- Live depth source unclear; depth path in `estimator_node` exists but not exercised live
- Initial pose guess for subsequent keyframes uses DVL dead-reckoning; should use `pim.predict()` for correctness

## Offline experiment harness

`experiments/gtsam_minimal/` (workspace root, not inside this package):
- `replay_estimator_backend.py` — drives `GtsamEstimator` directly without ROS
- Validates IMU preintegration + depth + DVL factor wiring; not a tuned estimator
