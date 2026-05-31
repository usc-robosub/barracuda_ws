from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import gtsam
from gtsam.symbol_shorthand import X

from .state_estimate import StateEstimate


@dataclass
class GtsamGraphConfig:
    prior_pose_sigmas: tuple[float, float, float, float, float, float]
    dvl_between_sigmas: tuple[float, float, float, float, float, float]
    depth_sigma_scale: float
    depth_sigma_min: float


class GtsamFactorGraphBackend:
    """
    Pose-only SE3 factor-graph back-end for Barracuda's estimator.

    This class owns the GTSAM graph, the inserted values, and the optimizer
    state. The front-end estimator feeds it pose guesses, depth constraints,
    and DVL relative-pose constraints between states.
    """

    def __init__(self, config: GtsamGraphConfig) -> None:
        self.config = config
        self.key_index = -1
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

        self.prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(config.prior_pose_sigmas, dtype=float)
        )
        self.dvl_between_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(config.dvl_between_sigmas, dtype=float)
        )

    def initialize(
        self,
        stamp_sec: float,
        pose_guess,
        velocity_guess: tuple[float, float, float],
        depth_z: float,
    ) -> StateEstimate:
        self.key_index = 0
        self.graph.add(gtsam.PriorFactorPose3(X(0), pose_guess, self.prior_pose_noise))
        self.graph.add(self._depth_factor(0, pose_guess, depth_z))

        self.initial.insert(X(0), pose_guess)
        return self._optimize_to_state(stamp_sec, velocity_guess)

    def advance(
        self,
        stamp_sec: float,
        pose_guess,
        velocity_guess: tuple[float, float, float],
        depth_z: float,
        dvl_relative_pose=None,
    ) -> StateEstimate:
        prev_idx = self.key_index
        next_idx = prev_idx + 1

        if dvl_relative_pose is not None:
            self.graph.add(
                gtsam.BetweenFactorPose3(
                    X(prev_idx),
                    X(next_idx),
                    dvl_relative_pose,
                    self.dvl_between_noise,
                )
            )
        self.graph.add(self._depth_factor(next_idx, pose_guess, depth_z))

        self.initial.insert(X(next_idx), pose_guess)
        self.key_index = next_idx
        return self._optimize_to_state(stamp_sec, velocity_guess)

    def _depth_factor(self, key_index: int, pose_guess, depth_z: float):
        depth_sigma = max(
            float(self.config.depth_sigma_min),
            float(self.config.depth_sigma_scale) * abs(float(depth_z)),
        )
        depth_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([1000.0, 1000.0, depth_sigma], dtype=float)
        )
        return gtsam.GPSFactor(
            X(key_index),
            gtsam.Point3(float(pose_guess.x()), float(pose_guess.y()), float(depth_z)),
            depth_noise,
        )

    def _optimize_to_state(
        self, stamp_sec: float, velocity_guess: tuple[float, float, float]
    ) -> StateEstimate:
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial)
        result = optimizer.optimize()
        self.initial = result

        pose = result.atPose3(X(self.key_index))
        quat = pose.rotation().toQuaternion()

        return StateEstimate(
            stamp_sec=stamp_sec,
            frame_id="odom",
            position_xyz=(float(pose.x()), float(pose.y()), float(pose.z())),
            orientation_xyzw=(
                float(quat.x()),
                float(quat.y()),
                float(quat.z()),
                float(quat.w()),
            ),
            velocity_xyz=velocity_guess,
        )
