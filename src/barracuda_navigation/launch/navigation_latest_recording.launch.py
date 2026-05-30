#!/usr/bin/env python3
"""Launch NVBlox + Nav2 against the newest local rosbag recording."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _default_recordings_dir() -> str:
    return os.path.join(os.path.expanduser("~"), "barracuda_ws", "recordings")


def _find_latest_recording_dir(recordings_dir: str) -> Path:
    root = Path(recordings_dir).expanduser()
    if not root.is_dir():
        raise FileNotFoundError(f"Recordings directory does not exist: {root}")

    candidates = []
    for child in root.iterdir():
        if not child.is_dir():
            continue
        if any(child.glob("*.db3")):
            latest_mtime = max(path.stat().st_mtime for path in child.rglob("*"))
            candidates.append((latest_mtime, child))

    if not candidates:
        raise FileNotFoundError(f"No rosbag recordings found under: {root}")

    return max(candidates, key=lambda item: item[0])[1]


def _launch_latest_recording(_: LaunchContext, recordings_dir, playback_rate):
    latest_bag = _find_latest_recording_dir(recordings_dir.perform(_))
    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                str(latest_bag),
                "--clock",
                playback_rate.perform(_),
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    nvblox_launch = PythonLaunchDescriptionSource([
        FindPackageShare("barracuda_nvblox"),
        "/launch/barracuda_nvblox.launch.py",
    ])
    navigation_launch = PythonLaunchDescriptionSource([
        FindPackageShare("barracuda_navigation"),
        "/launch/barracuda_navigation.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "recordings_dir",
            default_value=_default_recordings_dir(),
            description="Directory containing rosbag recording subdirectories.",
        ),
        DeclareLaunchArgument(
            "playback_rate",
            default_value="1.0",
            description="Playback clock rate passed to ros2 bag play --clock.",
        ),
        IncludeLaunchDescription(
            nvblox_launch,
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
        IncludeLaunchDescription(
            navigation_launch,
            launch_arguments={
                "use_sim_time": "true",
                "use_hardcoded_test_data": "false",
            }.items(),
        ),
        OpaqueFunction(
            function=_launch_latest_recording,
            args=[
                LaunchConfiguration("recordings_dir"),
                LaunchConfiguration("playback_rate"),
            ],
        ),
    ])
