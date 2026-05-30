from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

import gtsam
from gtsam.symbol_shorthand import B, V, X

from .imu_buffer import ImuPreintegrationBuffer
from .measurement_types import DepthSample, DvlSample, ImuSample
from .state_estimate import StateEstimate


@dataclass
class GtsamEstimatorStatus:
    healthy: bool
    has_imu: bool
    has_depth: bool
    has_dvl: bool
    imu_buffer_size: int


class GtsamEstimator:
    """
    GTSAM-backed estimation backend for Barracuda AUV.

    Keeps ROS2 concerns out of the core estimation flow.
    Fuses IMU preintegration, depth constraints, and DVL velocity
    constraints via batch Levenberg-Marquardt optimization.
    """

    def __init__(self) -> None:
        self.imu_buffer = ImuPreintegrationBuffer()
        self.latest_depth: Optional[DepthSample] = None
        self.latest_dvl: Optional[DvlSample] = None
        self.latest_state: Optional[StateEstimate] = None
        self.key_index = -1

        # First-pass sensor noise values from linked datasheets/docs:
        # - ZED Mini IMU continuous-time noise densities and random walk terms:
        #   https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS
        #   https://support.stereolabs.com/hc/article_attachments/27901442262551
        # - Ping Sonar depth modeled as ~0.5% of measured range:
        #   https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/
        self.gravity = 9.81
        self.accel_noise_density = 1.4e-03
        self.accel_bias_random_walk = 8.0e-05
        self.gyro_noise_density = 8.6e-05
        self.gyro_bias_random_walk = 2.2e-06
        self.depth_sigma_scale = 0.005
        self.depth_sigma_min = 0.01
        self.zero_bias = gtsam.imuBias.ConstantBias()
        self.params = gtsam.PreintegrationParams.MakeSharedU(self.gravity)
        self.params.setAccelerometerCovariance(np.eye(3) * self.accel_noise_density**2)
        self.params.setGyroscopeCovariance(np.eye(3) * self.gyro_noise_density**2)
        self.params.setIntegrationCovariance(np.eye(3) * 1e-6)
        self._set_optional_preintegration_noise("setBiasAccCovariance", self.accel_bias_random_walk)
        self._set_optional_preintegration_noise("setBiasOmegaCovariance", self.gyro_bias_random_walk)
        self._set_optional_preintegration_noise("setBiasAccOmegaInit", max(self.accel_bias_random_walk, 1e-6))

        self.prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.05, 0.05, 0.05, 0.03, 0.03, 0.03], dtype=float)
        )
        self.prior_vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        self.prior_bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)
        self.bias_between_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)
        self.dvl_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.04, 0.04, 0.05], dtype=float))
        self.depth_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1000.0, 1000.0, 0.03], dtype=float))

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

    def _set_optional_preintegration_noise(self, method_name: str, sigma: float) -> None:
        method = getattr(self.params, method_name, None)
        if method is None:
            return
        method(np.eye(3) * float(sigma) ** 2)

    def add_imu(self, sample: ImuSample) -> None:
        self.imu_buffer.append(sample)

    def add_depth(self, sample: DepthSample) -> None:
        self.latest_depth = sample

    def add_dvl(self, sample: DvlSample) -> None:
        self.latest_dvl = sample

    def status(self) -> GtsamEstimatorStatus:
        return GtsamEstimatorStatus(
            healthy=self.is_ready(),
            has_imu=len(self.imu_buffer) > 0,
            has_depth=self.latest_depth is not None,
            has_dvl=self.latest_dvl is not None,
            imu_buffer_size=len(self.imu_buffer),
        )

    def is_ready(self) -> bool:
        return len(self.imu_buffer) > 0 and self.latest_depth is not None and self.latest_dvl is not None

    def step(self, stamp_sec: float) -> Optional[StateEstimate]:
        """Advance the estimator. Returns None if not yet ready (missing IMU/depth/DVL)."""
        if not self.is_ready():
            return None

        estimate = self._run_graph_update(stamp_sec)
        self.latest_state = estimate
        self.imu_buffer.clear()
        return estimate

    def _run_graph_update(self, stamp_sec: float) -> StateEstimate:
        assert self.latest_dvl is not None
        assert self.latest_depth is not None

        current_pose = self._dvl_to_initial_pose3()
        current_velocity = np.asarray(self.latest_dvl.velocity_xyz, dtype=float)

        if self.key_index < 0:
            self.key_index = 0
            self.graph.add(gtsam.PriorFactorPose3(X(0), current_pose, self.prior_pose_noise))
            self.graph.add(gtsam.PriorFactorVector(V(0), current_velocity, self.prior_vel_noise))
            self.graph.add(gtsam.PriorFactorConstantBias(B(0), self.zero_bias, self.prior_bias_noise))
            self.graph.add(self._depth_factor(0, current_pose))
            self.graph.add(gtsam.PriorFactorVector(V(0), current_velocity, self.dvl_noise))

            self.initial.insert(X(0), current_pose)
            self.initial.insert(V(0), current_velocity)
            self.initial.insert(B(0), self.zero_bias)
        else:
            prev_idx = self.key_index
            next_idx = self.key_index + 1

            pim = gtsam.PreintegratedImuMeasurements(self.params, self.zero_bias)
            samples = self.imu_buffer.samples
            for i, sample in enumerate(samples):
                if i + 1 < len(samples):
                    dt = max(samples[i + 1].stamp_sec - sample.stamp_sec, 1e-3)
                else:
                    dt = 0.05
                pim.integrateMeasurement(
                    np.asarray(sample.linear_accel, dtype=float),
                    np.asarray(sample.angular_vel, dtype=float),
                    dt,
                )

            self.graph.add(gtsam.ImuFactor(X(prev_idx), V(prev_idx), X(next_idx), V(next_idx), B(prev_idx), pim))
            self.graph.add(
                gtsam.BetweenFactorConstantBias(B(prev_idx), B(next_idx), self.zero_bias, self.bias_between_noise)
            )
            self.graph.add(self._depth_factor(next_idx, current_pose))
            self.graph.add(gtsam.PriorFactorVector(V(next_idx), current_velocity, self.dvl_noise))

            self.initial.insert(X(next_idx), current_pose)
            self.initial.insert(V(next_idx), current_velocity)
            self.initial.insert(B(next_idx), self.zero_bias)
            self.key_index = next_idx

        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.initial)
        result = optimizer.optimize()
        self.initial = result

        pose = result.atPose3(X(self.key_index))
        velocity = result.atVector(V(self.key_index))
        bias = result.atConstantBias(B(self.key_index))
        bias_acc = bias.accelerometer()
        bias_gyro = bias.gyroscope()

        quat = pose.rotation().toQuaternion()
        return StateEstimate(
            stamp_sec=stamp_sec,
            frame_id="odom",
            position_xyz=(float(pose.x()), float(pose.y()), float(pose.z())),
            orientation_xyzw=(float(quat.x()), float(quat.y()), float(quat.z()), float(quat.w())),
            velocity_xyz=(float(velocity[0]), float(velocity[1]), float(velocity[2])),
            bias_vector=(
                float(bias_acc[0]),
                float(bias_acc[1]),
                float(bias_acc[2]),
                float(bias_gyro[0]),
                float(bias_gyro[1]),
                float(bias_gyro[2]),
            ),
        )

    def _dvl_to_initial_pose3(self):
        """Build initial pose guess from DVL position and orientation only.
        Depth enters the graph as an independent factor via _depth_factor."""
        assert self.latest_dvl is not None

        x, y, z = self.latest_dvl.position_xyz
        qx, qy, qz, qw = self.latest_dvl.orientation_xyzw
        rot = gtsam.Rot3.Quaternion(qw, qx, qy, qz)
        return gtsam.Pose3(rot, gtsam.Point3(float(x), float(y), float(z)))

    def _depth_factor(self, key_index: int, pose) -> Any:
        """Add depth as a z-only constraint. x/y columns use large sigma (1000)
        so only the z component is constrained by the depth sensor."""
        assert self.latest_depth is not None
        depth_sigma = self._depth_sigma_for_measurement()
        depth_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([1000.0, 1000.0, depth_sigma], dtype=float)
        )
        depth_z = -float(self.latest_depth.z_value)
        return gtsam.GPSFactor(
            X(key_index),
            gtsam.Point3(float(pose.x()), float(pose.y()), depth_z),
            depth_noise,
        )

    def _depth_sigma_for_measurement(self) -> float:
        assert self.latest_depth is not None
        return max(self.depth_sigma_min, self.depth_sigma_scale * abs(float(self.latest_depth.z_value)))
