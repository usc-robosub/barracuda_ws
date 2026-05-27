# barracuda_estimation

ROS2 estimation package for Barracuda that subscribes directly to the existing
`/barracuda/...` sensor topics and provides the current landing place for EKF
and future GTSAM-based state estimation work.

## What is in this package

This package currently does three things:

* subscribes directly to the expected sensor topics
  * `/barracuda/imu/data`
  * `/barracuda/depth`
  * `/barracuda/dvl/odometry`
  * `/barracuda/left_camera_image`
* launches local/global `robot_localization` EKF nodes
* runs an estimator node that publishes:
  * `/barracuda/estimation/health`
  * `/barracuda/estimation/debug`
  * `/barracuda/estimation/pose`

The direct-subscription design replaced the older idea of building a separate
sensor republisher layer. The estimator side now consumes the existing
`/barracuda/...` interfaces directly.

## Current implementation status

The package is no longer just a topic placeholder.

It now includes:

* a ROS2-facing node in `barracuda_estimation/estimator_node.py`
* a backend estimator in `barracuda_estimation/gtsam_estimator.py`
* an IMU sample buffer in `barracuda_estimation/imu_buffer.py`
* typed measurement containers in `barracuda_estimation/measurement_types.py`
* a shared state container in `barracuda_estimation/state_estimate.py`

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
  * the estimator node and both EKF nodes came up
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
