# barracuda_estimation

Minimal ROS2 estimator skeleton that subscribes directly to the existing
`/barracuda/...` sensor topics.

## Purpose

This package demonstrates the direct-subscription design:

* `/barracuda/imu/data`
* `/barracuda/depth`
* `/barracuda/dvl/odometry`
* `/barracuda/left_camera_image`

The current node only stores the latest measurements and publishes placeholder
health/debug/pose outputs. It is meant as the bridge from the existing sensor
topics to a future EKF or GTSAM estimator implementation.

## Included starting point from the archived localization stack

This package also contains ROS2 `robot_localization` configs modeled after the
archived ROS1 Barracuda localization package:

* `config/ekf_odom.yaml` for `odom -> base_link`
* `config/ekf_map.yaml` for `map -> odom`

The launch file starts both EKF nodes plus the direct-subscription estimator
skeleton. The configs were adapted to the current `/barracuda/...` topic
conventions and should be treated as a starting point, not a validated final tune.

## Future work

* IMU preintegration buffer
* Depth factor insertion
* DVL velocity factor insertion
* GTSAM optimization and pose publication
