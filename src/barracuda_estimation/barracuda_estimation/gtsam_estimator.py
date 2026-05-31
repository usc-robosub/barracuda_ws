from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import gtsam

from .factor_graph import GtsamFactorGraphBackend, GtsamGraphConfig
from .measurement_types import DepthSample, DvlSample, PoseSample
from .state_estimate import StateEstimate


@dataclass
class GtsamEstimatorStatus:
    healthy: bool
    has_depth: bool
    has_dvl: bool
    has_pose: bool


class GtsamEstimator:
    """
    Front-end coordinator for Barracuda's GTSAM-based estimator.

    This class keeps ROS2 node code out of the estimator flow and passes the
    current sensor snapshot into the pose-only factor-graph back-end.
    """

    def __init__(self) -> None:
        self.latest_depth: Optional[DepthSample] = None
        self.latest_dvl: Optional[DvlSample] = None
        self.latest_pose: Optional[PoseSample] = None
        self.previous_dvl: Optional[DvlSample] = None
        self.latest_state: Optional[StateEstimate] = None

        # First-pass sensor noise values from linked datasheets/docs:
        # - Ping Sonar depth modeled as ~0.5% of measured range:
        #   https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/
        self.depth_sigma_scale = 0.005
        self.depth_sigma_min = 0.01

        self.graph_backend = GtsamFactorGraphBackend(
            GtsamGraphConfig(
                prior_pose_sigmas=(0.05, 0.05, 0.05, 0.03, 0.03, 0.03),
                dvl_between_sigmas=(0.08, 0.08, 0.08, 0.05, 0.05, 0.05),
                depth_sigma_scale=self.depth_sigma_scale,
                depth_sigma_min=self.depth_sigma_min,
            )
        )

    def add_depth(self, sample: DepthSample) -> None:
        self.latest_depth = sample

    def add_dvl(self, sample: DvlSample) -> None:
        self.latest_dvl = sample

    def add_pose(self, sample: PoseSample) -> None:
        self.latest_pose = sample

    def status(self) -> GtsamEstimatorStatus:
        return GtsamEstimatorStatus(
            healthy=self.is_ready(),
            has_depth=self.latest_depth is not None,
            has_dvl=self.latest_dvl is not None,
            has_pose=self.latest_pose is not None,
        )

    def is_ready(self) -> bool:
        return self.latest_depth is not None and self.latest_pose is not None

    def step(self, stamp_sec: float) -> Optional[StateEstimate]:
        """
        Advance the current in-package estimator path.

        The current backend path is a pose-only SE3 graph:
        the incoming ZED pose is used as the pose guess for each state,
        depth is used to constrain vertical position, and DVL is used as the
        motion constraint between consecutive poses when it is available.
        """
        if not self.is_ready() or self.latest_pose is None or self.latest_depth is None:
            return None

        estimate = self._run_graph_update(stamp_sec)
        self.latest_state = estimate
        return estimate

    def _run_graph_update(self, stamp_sec: float) -> StateEstimate:
        assert self.latest_depth is not None
        assert self.latest_pose is not None

        current_pose = self._pose3_initial_guess()
        current_velocity = (0.0, 0.0, 0.0)
        if self.latest_dvl is not None:
            current_velocity = tuple(float(v) for v in self.latest_dvl.velocity_xyz)
        depth_z = -float(self.latest_depth.z_value)

        if self.graph_backend.key_index < 0:
            estimate = self.graph_backend.initialize(
                stamp_sec=stamp_sec,
                pose_guess=current_pose,
                velocity_guess=current_velocity,
                depth_z=depth_z,
            )
            self.previous_dvl = self.latest_dvl
            return estimate

        dvl_relative_pose = self._dvl_relative_pose()
        estimate = self.graph_backend.advance(
            stamp_sec=stamp_sec,
            pose_guess=current_pose,
            velocity_guess=current_velocity,
            depth_z=depth_z,
            dvl_relative_pose=dvl_relative_pose,
        )
        self.previous_dvl = self.latest_dvl
        return estimate

    def _pose3_initial_guess(self):
        assert self.latest_pose is not None

        x, y, z = self.latest_pose.position_xyz
        qx, qy, qz, qw = self.latest_pose.orientation_xyzw
        rot = gtsam.Rot3.Quaternion(qw, qx, qy, qz)
        return gtsam.Pose3(rot, gtsam.Point3(float(x), float(y), float(z)))

    def _dvl_relative_pose(self):
        if self.latest_dvl is None or self.previous_dvl is None:
            return None

        prev_pose = self._pose3_from_dvl_sample(self.previous_dvl)
        current_pose = self._pose3_from_dvl_sample(self.latest_dvl)
        return prev_pose.between(current_pose)

    def _pose3_from_dvl_sample(self, sample: DvlSample):
        x, y, z = sample.position_xyz
        qx, qy, qz, qw = sample.orientation_xyzw
        rot = gtsam.Rot3.Quaternion(qw, qx, qy, qz)
        return gtsam.Pose3(rot, gtsam.Point3(float(x), float(y), float(z)))
