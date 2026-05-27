from __future__ import annotations

from dataclasses import dataclass, field

from .measurement_types import ImuSample


@dataclass
class ImuPreintegrationBuffer:
    """
    Lightweight placeholder for future GTSAM IMU preintegration.

    Today this only stores the IMU samples collected between estimator updates.
    Later it can own a `gtsam.PreintegratedImuMeasurements` object and expose
    a method that returns the preintegrated factor payload for the next keyframe.
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
