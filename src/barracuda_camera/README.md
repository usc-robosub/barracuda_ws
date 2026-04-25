# barracuda_camera

Launch wrapper package for bringing up the ZED camera through `zed_wrapper`.

## Launch file behavior
`launch/barracuda_camera.launch.py` attempts to locate the `zed_wrapper` package and include `zed_camera.launch.py` with Barracuda defaults:
- `camera_model:=zedm`
- `camera_name:=barracuda`
- `enable_positional_tracking:=true`

If `zed_wrapper` is unavailable in the current environment, the launch file logs a warning and returns an empty `LaunchDescription`.

## Dependency note
This package declares `zed_wrapper` as an execution dependency in `package.xml` because camera launch depends on that package at runtime.

## Typical usage
Run directly:
- `ros2 launch barracuda_camera barracuda_camera.launch.py`

Or as part of top-level startup via:
- `ros2 launch barracuda_onboard barracuda_onboard.launch.py`
