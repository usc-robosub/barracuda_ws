# barracuda_thermal_warning

## Task Name

Add `barracuda_thermal_warning` for Jetson Overheating Warning and Detection

## Summary

Create a ROS 2 package named `barracuda_thermal_warning` that monitors Jetson thermal state and warns the onboard system when temperatures exceed safe operating thresholds.

This package should act as an onboard health-monitoring node rather than a hardware interrupt. It should periodically read Jetson temperature data, detect `warning` and `critical` thermal conditions, and publish/log status so operators and other nodes can respond appropriately.

## Why This Belongs in a New Package

The current workspace structure separates functionality by subsystem:

- `barracuda_onboard` is the umbrella launch package
- `barracuda_thrusters` handles thruster and killswitch logic
- `barracuda_dvl` handles the DVL driver
- `barracuda_control` handles control-related behavior

Jetson overheating is a system health concern, so it fits best as its own onboard package instead of being added to thruster or sensor code.

## Recommended Package Layout

```text
src/barracuda_thermal_warning/
├── package.xml
├── setup.py
├── resource/
├── barracuda_thermal_warning/
│   ├── __init__.py
│   └── jetson_thermal_warning.py
├── launch/
│   └── barracuda_thermal_warning.launch.py
└── config/
    └── thermal_params.yaml
```

## Expected Behavior

The node should:

- run on a timer at a fixed interval
- read Jetson thermal data from a supported source such as `/sys/class/thermal` or `tegrastats`
- compare readings against configurable `warning` and `critical` thresholds
- publish system thermal status on a ROS 2 topic
- log warnings when thresholds are crossed
- optionally expose a simple overheat flag for other nodes to subscribe to

## Suggested ROS Interfaces

### Parameters

- `poll_rate_hz`
- `warning_temp_c`
- `critical_temp_c`
- `sensor_source`
- `thermal_zone_path` or equivalent source-specific path

### Topics

- `/jetson/thermal_status`
- `/jetson/overheat_warning`

If a custom message is not desired yet, the first version can use standard message types such as:

- `std_msgs/String` for status level
- `std_msgs/Bool` for overheat flag
- `std_msgs/Float32` for current max temperature

## Initial Detection Logic

Example state handling:

- `NORMAL`: all monitored temperatures are below warning threshold
- `WARNING`: one or more temperatures are greater than or equal to warning threshold
- `CRITICAL`: one or more temperatures are greater than or equal to critical threshold

Recommended first behavior:

- publish current status every cycle
- emit a warning log on transition into `WARNING`
- emit an error or high-priority warning on transition into `CRITICAL`

## Integration With Existing Workspace

This package should include a default launch file named:

`barracuda_thermal_warning.launch.py`

That matches the workspace convention used by `barracuda_onboard`, which auto-includes package launch files from `src/`.

## Acceptance Criteria

- a new ROS 2 package named `barracuda_thermal_warning` exists in `src/`
- the node runs on the Jetson without crashing when thermal data is available
- warning and critical thresholds are configurable through ROS parameters
- the node publishes thermal state and current temperature
- the node logs clear messages when overheat thresholds are crossed
- the package can be launched through its default launch file
- the package is compatible with the onboard master launch flow

## Nice-to-Have Follow-Up Work

- add hysteresis to prevent warning-state log spam
- add a cooldown or recovery state
- integrate with a broader health/status dashboard
- trigger protective behavior in other nodes when critical overheat is detected
- add test coverage for threshold transitions

## Short Issue Description

Implement a new onboard ROS 2 package, `barracuda_thermal_warning`, to monitor Jetson temperature and detect overheating conditions. The node should periodically read thermal data, classify the system into normal, warning, or critical temperature states, and publish/log those states so the rest of the onboard stack can respond safely.
