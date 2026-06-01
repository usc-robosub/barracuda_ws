# barracuda_thermal_warning

ROS 2 package for a first-step Jetson thermal warning prototype.

## Behavior

The `jetson_thermal_warning` node periodically reads thermal zone temperatures from
`/sys/class/thermal`, checks whether the current temperature exceeds a warning or
critical threshold, and publishes ROS messages so the Jetson can report overheating.

## Topics

- `jetson/thermal_status` (`std_msgs/String`)
- `jetson/overheat_warning` (`std_msgs/Bool`)
- `jetson/max_temperature_c` (`std_msgs/Float32`)

## Parameters

- `poll_rate_hz`
- `warning_temp_c`
- `critical_temp_c`
- `zone_name_filter`

## Notes

The default thresholds are starter values for the first warning prototype and should
be tuned on the target Jetson after validating which thermal zones are most useful.
