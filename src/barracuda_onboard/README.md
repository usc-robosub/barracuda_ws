# barracuda_onboard

Top-level launch/orchestration package for onboard runtime.

## Purpose
This package provides `barracuda_onboard.launch.py`, which is the default launch target started by the workspace container entrypoint.

At startup, it:
- always launches Foxglove Bridge,
- scans `/root/barracuda_ws/src` for packages that contain a `launch/` directory,
- includes each package launch file named `pkg_name.launch.py`.

This keeps the top-level launch behavior scalable as packages are added.

## Selection behavior (`PKG_SEL`)
If `PKG_SEL` is set in the container environment, only the listed package names are launched (when they exist in `src/`).

Examples:
- `PKG_SEL="barracuda_dvl barracuda_thrusters"` launches only those package launch files (plus Foxglove Bridge).
- Invalid package names are ignored; if no valid names remain, a warning is logged.

## Relationship to directory layout
Packages that should be auto-launched by this file should stay under `src/` and provide `launch/pkg_name.launch.py`.

Dependencies that are vendored for build/runtime support but are not intended to be launched directly by this auto-scan live under `src/dependency-pkgs/`.
