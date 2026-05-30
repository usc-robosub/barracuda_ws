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

The current depth-sensor situation is still uncertain. The package has a depth
input path, but the exact live depth source and the right way to integrate it
in the current stack are not settled yet.

## Current live topic flow

The current live GTSAM path is a camera-driven pose-graph path, not yet the
full IMU + depth + DVL backend.

Right now the active live data flow is:

* `/barracuda/zed_node/pose`
  * input pose stream from the ZED tracking stack
* `barracuda_estimation/zed_pose_graph_node`
  * subscribes to `/barracuda/zed_node/pose`
  * converts accepted samples into a live GTSAM pose graph
* `/barracuda/gtsam_pose`
  * latest optimized pose output from the live graph as a `geometry_msgs/PoseStamped`
  * this is the main topic the remote visualization computer listens to
  * it is intended to represent the current optimized robot pose from the live ZED pose graph
* `/barracuda/gtsam_path`
  * full optimized trajectory from the live graph as a `nav_msgs/Path`
  * useful for plotting the accumulated optimized path instead of only the latest pose
* `/barracuda/gtsam_health`
  * boolean health/status topic
  * `true` means the live graph node is up, GTSAM is available, and at least one pose has entered the graph
  * `false` means the graph is not yet producing a valid optimization result
* `/barracuda/gtsam_debug`
  * string debug/status topic
  * reports fields such as whether GTSAM is available, how many pose messages have been seen, how many poses have been inserted into the graph, the pose input topic name, and the current graph mode
* ROS 2 network transport
  * makes the optimized topic available to other machines on the ROS graph
* external visualization computer
  * subscribes to `/barracuda/gtsam_pose`
  * displays the optimized trajectory or pose output remotely

In short, the current live demonstration path is:

`ZED pose -> GTSAM -> /barracuda/gtsam_pose -> ROS 2 -> visualization on another computer`

The main live topics involved right now are:

* listened to by `estimator_node`
  * `/barracuda/zed_node/imu/data`
  * `/barracuda/depth`
  * `/barracuda/dvl/odometry`
  * `/barracuda/zed_node/rgb/color/rect/image`
* listened to by `zed_pose_graph_node`
  * `/barracuda/zed_node/pose`
* published by `zed_pose_graph_node`
  * `/barracuda/gtsam_pose`
  * `/barracuda/gtsam_path`
  * `/barracuda/gtsam_health`
  * `/barracuda/gtsam_debug`

## Current implementation status

The package currently includes:

* a ROS2-facing node in `barracuda_estimation/estimator_node.py`
* a backend estimator in `barracuda_estimation/gtsam_estimator.py`
* an IMU sample buffer in `barracuda_estimation/imu_buffer.py`
* typed measurement containers in `barracuda_estimation/measurement_types.py`
* a shared state container in `barracuda_estimation/state_estimate.py`
* a live Pose3 graph node in `barracuda_estimation/zed_pose_graph_node.py`

The intended estimator backend path uses Python `gtsam` to:

* initialize pose / velocity / bias states
* preintegrate IMU measurements between updates
* insert depth constraints
* insert DVL velocity constraints
* run batch optimization with Levenberg-Marquardt
* return the current state estimate

This is still an early online-style integration, not a final tuned estimator.

The live ZED pose graph is a separate camera-only path for the current hardware
reality: it subscribes to `/barracuda/zed_node/pose`, builds a simple online
Pose3 graph in GTSAM, and publishes the optimized pose/path for external
visualization on another computer that listens to `/barracuda/gtsam_pose`.
This is intended as a practical intermediate step while the full IMU + depth +
DVL graph is still blocked by sensor availability.

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
  * the launch configuration includes the estimator node, both EKF nodes, and the ZED pose graph node
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

* move from the current ZED-pose-only live graph to a live camera + IMU GTSAM path
* listen to the IMU stream as part of the live pose-graph / factor-graph update path
* run GTSAM with both camera-derived pose information and IMU measurements together
* if the depth path remains unclear, the next likely live integration target is DVL + camera + IMU
* improve keyframe / update policy in `GtsamEstimator`
* improve depth and DVL factor modeling
* tune bias and process noise handling
* validate against replayed or live ROS sensor data
* add camera / SLAM factors after the IMU + depth + DVL path is stable
