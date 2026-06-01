from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ImuSample:
    stamp_sec: float
    linear_accel: tuple[float, float, float]
    angular_vel: tuple[float, float, float]


@dataclass
class DepthSample:
    stamp_sec: float
    z_value: float
    source: str


@dataclass
class DvlSample:
    stamp_sec: float
    position_xyz: tuple[float, float, float]
    velocity_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


@dataclass
class CameraRelativePoseSample:
    stamp_sec: float
    translation_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
