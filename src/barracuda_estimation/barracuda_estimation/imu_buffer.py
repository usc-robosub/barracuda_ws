from __future__ import annotations

from dataclasses import dataclass, field

from .measurement_types import ImuSample


@dataclass
class ImuPreintegrationBuffer:
    """
    Lightweight buffer for IMU samples collected between estimator updates.

    The GTSAM estimator consumes this buffered sequence when it builds the
    next preintegrated IMU factor.
    """

    samples: list[ImuSample] = field(default_factory=list)

    def append(self, sample: ImuSample) -> None:
        self.samples.append(sample)

    def clear(self) -> None:
        self.samples.clear()

    def __len__(self) -> int:
        return len(self.samples)

    def latest_timestamp(self) -> float | None:
        if not self.samples:
            return None
        return self.samples[-1].stamp_sec
