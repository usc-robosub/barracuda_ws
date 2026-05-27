from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class StateEstimate:
    stamp_sec: float = 0.0
    frame_id: str = "odom"
    position_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    velocity_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bias_vector: tuple[float, float, float, float, float, float] = field(
        default_factory=lambda: (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
