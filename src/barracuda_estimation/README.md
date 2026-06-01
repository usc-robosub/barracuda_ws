# barracuda_estimation

ROS2 estimation package for Barracuda that subscribes directly to the active
`/barracuda/...` sensor topics and runs the current in-package estimator path.

## What is in this package

This package currently does three things:

* subscribes directly to the expected sensor topics
  * `/barracuda/zed_node/imu/data`
  * `/barracuda/depth`
  * `/barracuda/dvl/odometry`
  * `/barracuda/zed_node/point_cloud/cloud_registered`
* runs an estimator node that publishes:
  * `/barracuda/estimation/health`
  * `/barracuda/estimation/debug`
  * `/barracuda/estimation/pose`

The direct-subscription design replaced the older idea of building a separate
sensor republisher layer. The estimator side now consumes the existing
`/barracuda/...` interfaces directly.

## Current live topic flow

The current solver path is an IMU + DVL + depth GTSAM backend.

Right now the active live data flow is:

* `/barracuda/zed_node/imu/data`
  * buffered and preintegrated between estimator updates
  * inserted into the graph through `CombinedImuFactor`
* `/barracuda/dvl/odometry`
  * used as the DVL velocity measurement
  * inserted through a DVL `CustomFactor` on `X(k)` and `V(k)`
* `/barracuda/depth`
  * used as the altimeter/depth measurement
  * inserted through an altimeter `CustomFactor` on `X(k)`
* `/barracuda/zed_node/point_cloud/cloud_registered`
  * used as the input to the in-package ICP frontend
  * converted into relative pose measurements for camera `BetweenFactorPose3` updates
* `barracuda_estimation/estimator_node`
  * collects the active measurements
  * forwards them into `GtsamEstimator`
* `barracuda_estimation/icp_frontend.py`
  * parses point clouds and estimates relative poses between consecutive clouds
* `barracuda_estimation/gtsam_estimator.py`
  * owns the graph update step, factor creation, and optimizer state
* `/barracuda/estimation/pose`
  * latest optimized pose as `geometry_msgs/PoseStamped`
* `/barracuda/estimation/health`
  * readiness flag for whether the estimator has enough required inputs for the current solve path
* `/barracuda/estimation/debug`
  * string debug/status topic describing which live inputs are currently present

In short, the current live path is:

`IMU + DVL + depth + point-cloud ICP -> GTSAM graph -> /barracuda/estimation/pose`

The main live topics involved right now are:

* listened to by `estimator_node`
  * `/barracuda/zed_node/imu/data`
  * `/barracuda/depth`
  * `/barracuda/dvl/odometry`
  * `/barracuda/zed_node/point_cloud/cloud_registered`
* published by `estimator_node`
  * `/barracuda/estimation/pose`
  * `/barracuda/estimation/health`
  * `/barracuda/estimation/debug`

## Current implementation status

The package currently includes:

* a ROS2-facing node in `barracuda_estimation/estimator_node.py`
* a front-end estimator coordinator in `barracuda_estimation/gtsam_estimator.py`
  * this keeps estimator flow separate from the ROS2 node code
  * it buffers IMU samples and collects DVL/depth measurements before advancing the graph
  * it also owns the current factor creation helpers and optimizer state
* an IMU buffer in `barracuda_estimation/imu_buffer.py`
* a point-cloud frontend in `barracuda_estimation/icp_frontend.py`
  * this converts consecutive point clouds into relative pose measurements for the graph
* typed measurement containers in `barracuda_estimation/measurement_types.py`
* a shared state container in `barracuda_estimation/state_estimate.py`

The current estimator backend uses Python `gtsam` to:

* preintegrate IMU with `CombinedImuFactor`
* insert DVL velocity constraints through a DVL `CustomFactor`
* insert depth / altimeter constraints through an altimeter `CustomFactor`
* insert camera relative-pose constraints through `BetweenFactorPose3`
* run batch optimization with Levenberg-Marquardt
* return the current optimized pose estimate

This is still an early integration, not a final tuned estimator.

## Validation done so far

This package has already been exercised in a few ways:

* Jetson / Docker launch validation
  * the package built and launched on the Jetson in Docker
  * the launch configuration includes the estimator node directly
* backend-only replay validation
  * `experiments/gtsam_minimal/replay_estimator_backend.py` replays the
    synthetic underwater dataset directly through `GtsamEstimator`

The current backend replay path is still useful for wiring checks, but the live
solver is now centered on the in-package IMU + DVL + depth graph path.

## Relationship to experiments/gtsam_minimal

The offline and reference GTSAM work still lives under:

* [experiments/gtsam_minimal](/Users/harvardsummer/barracuda_ws/experiments/gtsam_minimal)

That directory is where we validate:

* Pose2 batch graph behavior
* Pose2 `iSAM2` incremental behavior
* synthetic underwater factor-graph tests
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
* tune and harden the ICP frontend for real underwater data
* improve depth and DVL factor modeling
* validate against replayed or live ROS sensor data
* harden and tune the in-package estimator as the main source of truth
