from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np
import gtsam
from gtsam.symbol_shorthand import B, V, X

from .imu_buffer import ImuPreintegrationBuffer
from .measurement_types import (
    CameraRelativePoseSample,
    DepthSample,
    DvlSample,
    ImuSample,
)
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

    Keeps ROS2 concerns out of the core estimation flow and fuses IMU
    preintegration, camera relative-pose factors, depth constraints, and DVL
    velocity constraints through batch Levenberg-Marquardt optimization.
    """

    def __init__(self) -> None:
        self.imu_buffer = ImuPreintegrationBuffer()
        self.depth_buffer: list[DepthSample] = []
        self.latest_camera_relative_pose: Optional[CameraRelativePoseSample] = None
        self.latest_depth: Optional[DepthSample] = None
        self.latest_dvl: Optional[DvlSample] = None
        self.latest_state: Optional[StateEstimate] = None
        self.key_index = -1

        # First-pass sensor noise values from linked datasheets/docs.
        # ZED Mini IMU:
        # https://support.stereolabs.com/hc/en-us/articles/360012749113-How-can-I-use-Kalibr-with-the-ZED-Mini-camera-in-ROS
        # https://support.stereolabs.com/hc/article_attachments/27901442262551
        # Ping Sonar depth modeled as ~0.5% of measured range:
        #   https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/
        self.gravity = 9.81
        self.accel_noise_density = 1.4e-03
        self.accel_bias_random_walk = 8.0e-05
        self.gyro_noise_density = 8.6e-05
        self.gyro_bias_random_walk = 2.2e-06
        self.depth_sigma_scale = 0.005
        self.depth_sigma_min = 0.01
        self.altimeter_floor_z_world = 0.0
        self.altimeter_low_dynamics_speed_mps = 0.1
        self.altimeter_max_angle_from_vertical_rad = np.deg2rad(30.0)
        self.altimeter_boresight_body = self._normalize_vector(
            np.array([0.0, 0.0, 1.0], dtype=float)
        )
        self.zero_bias = gtsam.imuBias.ConstantBias()
        self.params = gtsam.PreintegrationCombinedParams.MakeSharedU(self.gravity)
        self.params.setAccelerometerCovariance(np.eye(3) * self.accel_noise_density**2)
        self.params.setGyroscopeCovariance(np.eye(3) * self.gyro_noise_density**2)
        self.params.setIntegrationCovariance(np.eye(3) * 1e-6)
        self._set_optional_preintegration_noise(
            "setBiasAccCovariance", self.accel_bias_random_walk
        )
        self._set_optional_preintegration_noise(
            "setBiasOmegaCovariance", self.gyro_bias_random_walk
        )
        self._set_optional_preintegration_noise(
            "setBiasAccOmegaInit", max(self.accel_bias_random_walk, 1e-6)
        )

        self.prior_pose_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.05, 0.05, 0.05, 0.03, 0.03, 0.03], dtype=float)
        )
        self.prior_vel_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        self.prior_bias_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-3)
        self.dvl_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.04, 0.04, 0.05], dtype=float)
        )
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

    def _set_optional_preintegration_noise(
        self, method_name: str, sigma: float
    ) -> None:
        method = getattr(self.params, method_name, None)
        if method is None:
            return
        method(np.eye(3) * float(sigma) ** 2)

    def add_imu(self, sample: ImuSample) -> None:
        self.imu_buffer.append(sample)

    def add_depth(self, sample: DepthSample) -> None:
        self.latest_depth = sample
        self.depth_buffer.append(sample)

    def add_dvl(self, sample: DvlSample) -> None:
        self.latest_dvl = sample

    def add_camera_relative_pose(self, sample: CameraRelativePoseSample) -> None:
        self.latest_camera_relative_pose = sample

    def status(self) -> GtsamEstimatorStatus:
        return GtsamEstimatorStatus(
            healthy=self.is_ready(),
            has_imu=len(self.imu_buffer) > 0,
            has_depth=self.latest_depth is not None,
            has_dvl=self.latest_dvl is not None,
            imu_buffer_size=len(self.imu_buffer),
        )

    def is_ready(self) -> bool:
        return (
            len(self.imu_buffer) > 0
            and self.latest_depth is not None
            and self.latest_dvl is not None
        )

    def step(self, stamp_sec: float) -> Optional[StateEstimate]:
        """Advance the estimator once enough measurements are available."""
        if not self.is_ready():
            return None

        estimate = self._run_graph_update(stamp_sec)
        self.latest_state = estimate
        self.imu_buffer.clear()
        self.depth_buffer.clear()
        self.latest_camera_relative_pose = None
        return estimate

    def _run_graph_update(self, stamp_sec: float) -> StateEstimate:
        assert self.latest_dvl is not None
        assert self.latest_depth is not None

        self.latest_depth = self._collapse_depth_measurements(stamp_sec)
        current_pose = self._dvl_to_initial_pose3()
        current_velocity = np.asarray(self.latest_dvl.velocity_xyz, dtype=float)

        if self.key_index < 0:
            self.key_index = 0
            self.graph.add(
                gtsam.PriorFactorPose3(X(0), current_pose, self.prior_pose_noise)
            )
            self.graph.add(self._velocity_prior_factor(0, current_velocity))
            self.graph.add(
                gtsam.PriorFactorConstantBias(
                    B(0), self.zero_bias, self.prior_bias_noise
                )
            )
            altimeter_factor = self._altimeter_factor(0, current_pose)
            if altimeter_factor is not None:
                self.graph.add(altimeter_factor)
            self.graph.add(self._dvl_factor(0, current_velocity))

            self.initial.insert(X(0), current_pose)
            self.initial.insert(V(0), current_velocity)
            self.initial.insert(B(0), self.zero_bias)
        else:
            prev_idx = self.key_index
            next_idx = self.key_index + 1

            pim = self._build_pim_from_buffer()
            self.graph.add(self._imu_factor(prev_idx, next_idx, pim))
            if self.latest_camera_relative_pose is not None:
                self.graph.add(
                    self._camera_factor(
                        prev_idx,
                        next_idx,
                        self._camera_relative_pose3(self.latest_camera_relative_pose),
                    )
                )
            altimeter_factor = self._altimeter_factor(next_idx, current_pose)
            if altimeter_factor is not None:
                self.graph.add(altimeter_factor)
            self.graph.add(self._dvl_factor(next_idx, current_velocity))

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
            orientation_xyzw=(
                float(quat.x()),
                float(quat.y()),
                float(quat.z()),
                float(quat.w()),
            ),
            velocity_xyz=(
                float(velocity[0]),
                float(velocity[1]),
                float(velocity[2]),
            ),
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
        """
        Build the initial pose guess from DVL position and orientation only.

        Altimeter/depth enters the graph separately through `_altimeter_factor()`.
        """
        assert self.latest_dvl is not None

        x, y, z = self.latest_dvl.position_xyz
        qx, qy, qz, qw = self.latest_dvl.orientation_xyzw
        rot = gtsam.Rot3.Quaternion(qw, qx, qy, qz)
        return gtsam.Pose3(rot, gtsam.Point3(float(x), float(y), float(z)))

    def _build_pim_from_buffer(self):
        """
        Build the pending IMU preintegration payload from buffered samples.

        This keeps the IMU measurement accumulation logic in one place before
        the actual IMU factor is created.
        """
        pim = gtsam.PreintegratedCombinedMeasurements(self.params, self.zero_bias)
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
        return pim

    def _imu_factor(self, prev_idx: int, next_idx: int, pim):
        """
        Return the IMU factor used between consecutive graph states.

        This uses `CombinedImuFactor`, which carries the preintegrated IMU
        constraint together with bias evolution between keyframes.
        """
        return gtsam.CombinedImuFactor(
            X(prev_idx),
            V(prev_idx),
            X(next_idx),
            V(next_idx),
            B(prev_idx),
            B(next_idx),
            pim,
        )

    def _velocity_prior_factor(self, key_index: int, current_velocity: np.ndarray):
        """Return the initial velocity prior factor for a new graph."""
        return gtsam.PriorFactorVector(
            V(key_index), current_velocity, self.prior_vel_noise
        )

    def _dvl_factor(self, key_index: int, current_velocity: np.ndarray):
        """
        Return the current DVL measurement factor.

        The factor connects pose and world-frame velocity, rotates that
        velocity prediction into the body frame, and compares it to the DVL
        body-frame velocity measurement.
        """
        keys = gtsam.KeyVector()
        keys.append(X(key_index))
        keys.append(V(key_index))

        measured_velocity_body = np.asarray(current_velocity, dtype=float)

        def error_func(this, values, jacobians):
            pose = values.atPose3(X(key_index))
            velocity_world = np.asarray(values.atVector(V(key_index)), dtype=float)
            rotation_world_body = pose.rotation().matrix().T
            predicted_velocity_body = rotation_world_body @ velocity_world
            residual = predicted_velocity_body - measured_velocity_body

            if jacobians is not None:
                jacobians[0] = self._finite_difference_pose_jacobian(
                    lambda test_pose: (
                        (test_pose.rotation().matrix().T @ velocity_world)
                        - measured_velocity_body
                    ),
                    pose,
                    residual_dim=3,
                )
                jacobians[1] = rotation_world_body

            return residual

        return gtsam.CustomFactor(self.dvl_noise, keys, error_func)

    def _camera_factor(self, prev_idx: int, next_idx: int, relative_pose):
        """
        Return the camera pose factor between consecutive states.

        This helper converts the relative pose measurement produced by the ICP
        frontend into a graph factor between consecutive keyframes.
        """
        return gtsam.BetweenFactorPose3(
            X(prev_idx),
            X(next_idx),
            relative_pose,
            self.prior_pose_noise,
        )

    def _camera_relative_pose3(self, sample: CameraRelativePoseSample):
        qx, qy, qz, qw = sample.orientation_xyzw
        tx, ty, tz = sample.translation_xyz
        rotation = gtsam.Rot3.Quaternion(qw, qx, qy, qz)
        return gtsam.Pose3(rotation, gtsam.Point3(float(tx), float(ty), float(tz)))

    def _altimeter_factor(self, key_index: int, pose) -> Any:
        """
        Return the current altimeter factor.

        The active sensor path is a range sensor. This factor models the
        expected beam range to a known floor plane using the vehicle pose and
        the altimeter boresight direction in the body frame.
        """
        assert self.latest_depth is not None
        keys = gtsam.KeyVector()
        keys.append(X(key_index))
        altimeter_noise = gtsam.noiseModel.Isotropic.Sigma(
            1, self._depth_sigma_for_measurement(self.latest_depth)
        )

        if not self._altimeter_beam_is_valid(pose):
            return None

        def error_func(this, values, jacobians):
            test_pose = values.atPose3(X(key_index))
            residual = np.array(
                [self._altimeter_residual_from_pose(test_pose)], dtype=float
            )
            if jacobians is not None:
                jacobians[0] = self._finite_difference_pose_jacobian(
                    lambda candidate_pose: np.array(
                        [self._altimeter_residual_from_pose(candidate_pose)],
                        dtype=float,
                    ),
                    test_pose,
                    residual_dim=1,
                )
            return residual

        return gtsam.CustomFactor(altimeter_noise, keys, error_func)

    def _depth_sigma_for_measurement(
        self, sample: Optional[DepthSample] = None
    ) -> float:
        if sample is None:
            sample = self.latest_depth
        assert sample is not None
        return max(
            self.depth_sigma_min,
            self.depth_sigma_scale * abs(float(sample.z_value)),
        )

    def _altimeter_residual_from_pose(self, pose) -> float:
        assert self.latest_depth is not None

        boresight_world_z = self._altimeter_boresight_world_z(pose)
        if boresight_world_z is None:
            return 0.0

        expected_range = (self.altimeter_floor_z_world - float(pose.z())) / boresight_world_z
        return expected_range - float(self.latest_depth.z_value)

    def _finite_difference_pose_jacobian(
        self, error_fn, pose, residual_dim: int, eps: float = 1e-6
    ) -> np.ndarray:
        """
        Compute a numerical Jacobian of a pose-dependent error function.

        Uses a central-difference approximation in the 6D Pose3 tangent space
        and returns the matrix expected by GTSAM custom factors.
        """
        jacobian = np.zeros((residual_dim, 6), dtype=float)
        for col in range(6):
            delta = np.zeros(6, dtype=float)
            delta[col] = eps
            error_plus = np.asarray(error_fn(pose.retract(delta)), dtype=float).reshape(
                residual_dim
            )
            error_minus = np.asarray(
                error_fn(pose.retract(-delta)), dtype=float
            ).reshape(residual_dim)
            jacobian[:, col] = (error_plus - error_minus) / (2.0 * eps)
        return jacobian

    def _collapse_depth_measurements(self, stamp_sec: float) -> DepthSample:
        """
        Collapse multiple depth readings into one measurement for the current keyframe.

        If the vehicle is moving slowly, average the accumulated readings.
        Otherwise use a simple linear interpolation/extrapolation from the most
        recent two samples to the estimator time.
        """
        if not self.depth_buffer:
            assert self.latest_depth is not None
            return self.latest_depth

        if len(self.depth_buffer) == 1:
            return self.depth_buffer[-1]

        if self._is_low_dynamics():
            values = np.array(
                [sample.z_value for sample in self.depth_buffer], dtype=float
            )
            stamps = np.array(
                [sample.stamp_sec for sample in self.depth_buffer], dtype=float
            )
            return DepthSample(
                stamp_sec=float(stamps.mean()),
                z_value=float(values.mean()),
                source=self.depth_buffer[-1].source,
            )

        sample_a = self.depth_buffer[-2]
        sample_b = self.depth_buffer[-1]
        dt = sample_b.stamp_sec - sample_a.stamp_sec
        if abs(dt) < 1e-6:
            return sample_b

        alpha = (stamp_sec - sample_a.stamp_sec) / dt
        interpolated_value = float(sample_a.z_value) + alpha * (
            float(sample_b.z_value) - float(sample_a.z_value)
        )
        return DepthSample(
            stamp_sec=stamp_sec,
            z_value=interpolated_value,
            source=sample_b.source,
        )

    def _is_low_dynamics(self) -> bool:
        if self.latest_dvl is None:
            return True
        speed = float(
            np.linalg.norm(np.asarray(self.latest_dvl.velocity_xyz, dtype=float))
        )
        return speed <= self.altimeter_low_dynamics_speed_mps

    def _normalize_vector(self, vector: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= 0.0:
            raise ValueError("Altimeter boresight vector must be non-zero.")
        return vector / norm

    def _altimeter_beam_is_valid(self, pose) -> bool:
        boresight_world_z = self._altimeter_boresight_world_z(pose)
        if boresight_world_z is None:
            return False

        min_vertical_component = float(
            np.cos(self.altimeter_max_angle_from_vertical_rad)
        )
        return abs(boresight_world_z) >= min_vertical_component

    def _altimeter_boresight_world_z(self, pose) -> Optional[float]:
        boresight_world = pose.rotation().matrix() @ self.altimeter_boresight_body
        boresight_world_z = float(boresight_world[2])
        if abs(boresight_world_z) < 1e-6:
            return None
        return boresight_world_z
