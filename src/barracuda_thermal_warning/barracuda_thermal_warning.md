# barracuda_thermal_warning

## Summary

`barracuda_thermal_warning` is a ROS 2 package that reads Jetson thermal zone data and sends ROS messages when the temperature crosses a warning or critical threshold.

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

## Current Feature

The node should:

- run on a timer at a fixed interval
- read Jetson thermal data from `/sys/class/thermal`
- compare readings against configurable `warning` and `critical` thresholds
- publish ROS topics with the current thermal state
- log when the Jetson is in `WARNING` or `CRITICAL`

## Current Parameters

- `poll_rate_hz`
- `warning_temp_c`
- `critical_temp_c`
- `zone_name_filter`
- `status_topic`
- `overheat_topic`
- `max_temp_topic`

## Current Topics

- `/jetson/thermal_status`
- `/jetson/overheat_warning`
- `std_msgs/String` for status level
- `std_msgs/Bool` for overheat flag
- `std_msgs/Float32` for current max temperature

## Current Detection Logic

- `NORMAL`: max temperature is below `warning_temp_c`
- `WARNING`: max temperature is greater than or equal to `warning_temp_c`
- `CRITICAL`: max temperature is greater than or equal to `critical_temp_c`

## Integration With Existing Workspace

This package should include a default launch file named:

`barracuda_thermal_warning.launch.py`

That matches the workspace convention used by `barracuda_onboard`, which auto-includes package launch files from `src/`.

## Acceptance Criteria

- a new ROS 2 package named `barracuda_thermal_warning` exists in `src/`
- the node reads Jetson thermal zones from `/sys/class/thermal`
- warning and critical thresholds are configurable through ROS parameters
- the node publishes thermal state, overheat flag, and current max temperature
- the node logs warning or critical messages when thresholds are crossed
- the package can be launched through its default launch file
- the package is compatible with the onboard master launch flow
