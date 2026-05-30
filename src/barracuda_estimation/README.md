# barracuda_estimation

ROS2 estimation package for Barracuda that subscribes directly to the active
`/barracuda/...` sensor topics and hosts the EKF and GTSAM-based estimation
paths used by the current stack.

## What is in this package

This package currently does four things:

* subscribes directly to the expected sensor topics
  * `/barracuda/zed_node/imu/data`
  * `/barracuda/depth`
  * `/barracuda/dvl/odometry`
  * `/barracuda/zed_node/rgb/color/rect/image`
* launches local/global `robot_localization` EKF nodes
* runs an estimator node that publishes:
  * `/barracuda/estimation/health`
  * `/barracuda/estimation/debug`
  * `/barracuda/estimation/pose`
* runs a live ZED pose graph node that publishes:
  * `/barracuda/gtsam_pose`
  * `/barracuda/gtsam_path`
  * `/barracuda/gtsam_health`
  * `/barracuda/gtsam_debug`

The direct-subscription design replaced the older idea of building a separate
sensor republisher layer. The estimator side now consumes the existing
`/barracuda/...` interfaces directly.

At the moment, the live camera/IMU defaults are aligned with the ZED topics
that are actually published in the current Barracuda stack. The depth input is
still modeled as a `Range` or `FluidPressure` measurement; using the ZED depth
image directly will require a separate adapter step rather than a simple topic
rename.

## Current implementation status

The package currently includes:

* a ROS2-facing node in `barracuda_estimation/estimator_node.py`
* a backend estimator in `barracuda_estimation/gtsam_estimator.py`
* an IMU sample buffer in `barracuda_estimation/imu_buffer.py`
* typed measurement containers in `barracuda_estimation/measurement_types.py`
* a shared state container in `barracuda_estimation/state_estimate.py`
* a live Pose3 graph node in `barracuda_estimation/zed_pose_graph_node.py`

The backend has two execution paths:

* without Python `gtsam`
  * falls back to a lightweight DVL-seeded estimate
* with Python `gtsam`
  * initializes pose / velocity / bias states
  * preintegrates IMU measurements between updates
  * inserts depth constraints
  * inserts DVL velocity constraints
  * runs batch optimization with Levenberg-Marquardt
  * returns the current state estimate

This is still an early online-style integration, not a final tuned estimator.

The live ZED pose graph is a separate camera-only path for the current hardware
reality: it subscribes to `/barracuda/zed_node/pose`, builds a simple online
Pose3 graph in GTSAM, and publishes the optimized pose/path for external
visualization. This is intended as a practical intermediate step while the full
IMU + depth + DVL graph is still blocked by sensor availability.

## EKF structure

The package includes ROS2 `robot_localization` configs adapted from the archived
ROS1 Barracuda localization stack:

* `config/ekf_odom.yaml`
  * local filter for `odom -> base_link`
* `config/ekf_map.yaml`
  * global filter for `map -> odom`

The launch file starts both EKF nodes along with `estimator_node`.

## Validation done so far

This package has already been exercised in a few ways:

* Jetson / Docker launch validation
  * the package built and launched on the Jetson in Docker
  * the estimator node, both EKF nodes, and the ZED pose graph node came up
  * health stayed false in the no-sensor test, which matched the fact that the
    expected sensor topics had zero publishers in that session
* backend-only replay validation
  * `experiments/gtsam_minimal/replay_estimator_backend.py` replays the
    synthetic underwater dataset directly through `GtsamEstimator`
  * this confirms the backend can ingest IMU, depth, and DVL samples and
    produce step-by-step estimates without needing the ROS graph

The backend replay currently proves that the estimator path is wired up, but it
is not tuned well enough yet to outperform the simple dead-reckoned baseline.

## Relationship to experiments/gtsam_minimal

The offline and reference GTSAM work still lives under:

* [experiments/gtsam_minimal](/Users/harvardsummer/barracuda_ws/experiments/gtsam_minimal)

That directory is where we validate:

* Pose2 batch graph behavior
* Pose2 `iSAM2` incremental behavior
* synthetic underwater IMU + depth + DVL factor-graph tests
* backend replay into `GtsamEstimator`

The intention is:

* `experiments/gtsam_minimal`
  * prove out graph logic and datasets
* `barracuda_estimation`
  * become the online ROS2 estimator package

## Running

Launch the ROS2 package:

```bash
ros2 launch barracuda_estimation barracuda_estimation.launch.py
```

Run only the live ZED pose graph node:

```bash
ros2 run barracuda_estimation zed_pose_graph_node
```

Run the backend-only replay harness:

```bash
MPLCONFIGDIR=/private/tmp/mplcache python3 experiments/gtsam_minimal/replay_estimator_backend.py
```

## Near-term next steps

* improve keyframe / update policy in `GtsamEstimator`
* improve depth and DVL factor modeling
* tune bias and process noise handling
* validate against replayed or live ROS sensor data
* add camera / SLAM factors after the IMU + depth + DVL path is stable
