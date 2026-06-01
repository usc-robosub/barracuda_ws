# AGENTS.md — barracuda_thermal_warning

Python `ament_python` package. ROS2 Humble. Intended to run in the Barracuda Docker workspace and on the NVIDIA Jetson target.

## Runtime / dev workflow

- Build and run inside the Barracuda ROS2 workspace after sourcing the workspace environment.
- Always try to run `ruff` on Python changes when it is available.
- Package-local `ruff` config lives in `pyproject.toml`.
- Prefer readable code over clever code:
  - keep functions small and direct
  - use descriptive variable names
  - make thermal polling / classification / publishing flow easy to follow
  - add short comments only when they clarify non-obvious behavior
- Preserve the package’s simple structure: one node file, one launch file, one config file.

## Package layout

```text
barracuda_thermal_warning/
  jetson_thermal_warning.py   # ROS2 node entry point; reads thermal sysfs and publishes warning topics
config/
  thermal_params.yaml         # threshold and topic parameter defaults
launch/
  barracuda_thermal_warning.launch.py  # starts jetson_thermal_warning with thermal_params.yaml
resource/
  barracuda_thermal_warning   # ament resource marker
README.md
package.xml
setup.py
setup.cfg
```

## Entry points (setup.py console_scripts)

```text
jetson_thermal_warning  → barracuda_thermal_warning.jetson_thermal_warning:main
```

## Running

```bash
# Launch the thermal warning node with the package config:
ros2 launch barracuda_thermal_warning barracuda_thermal_warning.launch.py

# Or run the node directly after sourcing the workspace:
ros2 run barracuda_thermal_warning jetson_thermal_warning

# Lint Python changes when available:
ruff check src/barracuda_thermal_warning

# Optionally format after reviewing changes:
ruff format src/barracuda_thermal_warning
```

## Topic wiring

### jetson_thermal_warning subscribes

None.

### jetson_thermal_warning publishes

| Topic | Type | Meaning |
|-------|------|---------|
| `jetson/thermal_status` | `std_msgs/String` | `NORMAL`, `WARNING`, or `CRITICAL` |
| `jetson/overheat_warning` | `std_msgs/Bool` | `true` when state is `WARNING` or `CRITICAL` |
| `jetson/max_temperature_c` | `std_msgs/Float32` | Maximum observed thermal-zone temperature in Celsius |

## Parameters

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `poll_rate_hz` | `1.0` | Timer frequency for thermal polling |
| `warning_temp_c` | `70.0` | Warning threshold in Celsius |
| `critical_temp_c` | `80.0` | Critical threshold in Celsius |
| `zone_name_filter` | `""` | Optional case-insensitive substring filter for thermal zone names |
| `status_topic` | `jetson/thermal_status` | Published status topic name |
| `overheat_topic` | `jetson/overheat_warning` | Published overheat flag topic name |
| `max_temp_topic` | `jetson/max_temperature_c` | Published max-temperature topic name |

## Thermal data source

- Reads from Linux thermal sysfs under `/sys/class/thermal`
- Iterates `thermal_zone*` directories
- Reads:
  - `type` as the thermal-zone name
  - `temp` as the raw temperature value
- Converts raw temperature to Celsius with `float(raw_temp) / 1000.0`

## Node behavior

- `timer_callback()` polls thermal data on a timer, computes the max zone temperature, classifies state, publishes topics, and logs the result
- `classify_state()` returns:
  - `CRITICAL` when `max_temp_c >= critical_temp_c`
  - `WARNING` when `max_temp_c >= warning_temp_c`
  - `NORMAL` otherwise
- `publish_state()` always publishes the current state, overheat boolean, and max temperature
- `log_current_state()` logs:
  - `error` for `CRITICAL`
  - `warning` for `WARNING`
  - `debug` for `NORMAL`

## Known limitations

- This package only monitors and reports temperature; it does not perform thermal control or throttling
- It does not use Jetson-specific tooling such as `jtop`, `jetson_stats`, or `tegrastats`
- No hysteresis is implemented, so repeated warning/critical logs may occur while temperatures remain above threshold
- If `/sys/class/thermal` is unavailable or unreadable in the runtime environment, the node will warn once and publish nothing
