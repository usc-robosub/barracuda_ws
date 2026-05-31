# barracuda_estimation

ROS2 estimation package for Barracuda that subscribes directly to the active
`/barracuda/...` sensor topics and runs the current in-package estimator path.

## What is in this package

This package currently does three things:

* subscribes directly to the expected sensor topics
  * `/barracuda/zed_node/imu/data`
  * `/barracuda/depth`
  * `/barracuda/dvl/odometry`
  * `/barracuda/zed_node/pose`
  * `/barracuda/zed_node/point_cloud/cloud_registered`
* launches local/global `robot_localization` EKF nodes
* runs an estimator node that publishes:
  * `/barracuda/estimation/health`
  * `/barracuda/estimation/debug`
  * `/barracuda/estimation/pose`

The direct-subscription design replaced the older idea of building a separate
sensor republisher layer. The estimator side now consumes the existing
`/barracuda/...` interfaces directly.

## Current live topic flow

The current solver path is a pose-only SE3 factor graph.

Right now the active live data flow is:

* `/barracuda/zed_node/pose`
  * used as the pose guess for each graph state
* `/barracuda/dvl/odometry`
  * used as the relative motion constraint between consecutive graph states
* `/barracuda/depth`
  * used as the vertical depth constraint
* `/barracuda/zed_node/point_cloud/cloud_registered`
  * tracked as the planned direct input for future ICP factors
  * not inserted into the graph yet
* `barracuda_estimation/estimator_node`
  * collects the active measurements
  * forwards them into `GtsamEstimator`
* `barracuda_estimation/gtsam_estimator.py`
  * builds the current graph update step
* `barracuda_estimation/factor_graph.py`
  * owns the GTSAM factor graph and optimizer
* `/barracuda/estimation/pose`
  * latest optimized pose as `geometry_msgs/PoseStamped`
* `/barracuda/estimation/health`
  * readiness flag for whether the estimator has enough required inputs for the current solve path
* `/barracuda/estimation/debug`
  * string debug/status topic describing which live inputs are currently present

In short, the current live path is:

`ZED pose guess + DVL motion + depth constraint -> factor graph -> /barracuda/estimation/pose`

The main live topics involved right now are:

* listened to by `estimator_node`
  * `/barracuda/zed_node/imu/data`
  * `/barracuda/depth`
  * `/barracuda/dvl/odometry`
  * `/barracuda/zed_node/pose`
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
  * it collects depth, DVL, and ZED pose measurements before advancing the graph
* a graph back-end in `barracuda_estimation/factor_graph.py`
  * this owns the GTSAM factor graph, inserted values, and optimization step
* typed measurement containers in `barracuda_estimation/measurement_types.py`
* a shared state container in `barracuda_estimation/state_estimate.py`

The current estimator backend uses Python `gtsam` to:

* initialize `Pose3` graph states from the incoming ZED pose guess
* insert depth constraints on vertical position
* insert DVL `BetweenFactorPose3` motion constraints when DVL is available
* run batch optimization with Levenberg-Marquardt
* return the current optimized pose estimate

This is still an early online-style integration, not a final tuned estimator.

## EKF structure

The package still includes ROS2 `robot_localization` configs adapted from the archived
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
  * the launch configuration includes the estimator node and both EKF nodes
* backend-only replay validation
  * `experiments/gtsam_minimal/replay_estimator_backend.py` replays the
    synthetic underwater dataset directly through `GtsamEstimator`

The current backend replay path is still useful for wiring checks, but the live
solver is now a pose-only graph instead of the earlier inertial graph design.

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

* turn the point-cloud input into ICP between-factors
* decide whether IMU should stay outside the graph or come back with a full inertial state
* improve keyframe / update policy in `GtsamEstimator`
* improve depth and DVL factor modeling
* validate against replayed or live ROS sensor data
* replace or remove the EKF path if the in-package estimator becomes the main source of truth
