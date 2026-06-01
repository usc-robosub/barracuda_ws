from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .measurement_types import CameraRelativePoseSample


@dataclass
class IcpFrontendConfig:
    max_points: int = 256
    max_iterations: int = 8
    correspondence_distance: float = 0.25
    convergence_tol: float = 1e-4


class IcpFrontend:
    """
    Minimal point-cloud ICP frontend.

    It keeps the previous point cloud, estimates a relative transform from the
    current cloud into the previous cloud frame, and emits that transform as a
    `CameraRelativePoseSample` for the GTSAM `BetweenFactorPose3`.
    """

    _POINTFIELD_FLOAT32 = 7

    def __init__(self, config: Optional[IcpFrontendConfig] = None) -> None:
        self.config = config or IcpFrontendConfig()
        self.previous_points: Optional[np.ndarray] = None

    def update(self, msg) -> Optional[CameraRelativePoseSample]:
        points = self._extract_xyz_points(msg)
        if points is None or len(points) < 3:
            return None

        if self.previous_points is None:
            self.previous_points = points
            return None

        transform = self._estimate_relative_transform(
            target_points=self.previous_points,
            source_points=points,
        )
        self.previous_points = points
        if transform is None:
            return None

        rotation, translation = transform
        qx, qy, qz, qw = self._rotation_matrix_to_quaternion(rotation)
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return CameraRelativePoseSample(
            stamp_sec=stamp_sec,
            translation_xyz=(
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            ),
            orientation_xyzw=(float(qx), float(qy), float(qz), float(qw)),
        )

    def _extract_xyz_points(self, msg) -> Optional[np.ndarray]:
        field_map = {field.name: field for field in msg.fields}
        if not {"x", "y", "z"}.issubset(field_map):
            return None

        x_field = field_map["x"]
        y_field = field_map["y"]
        z_field = field_map["z"]
        if (
            x_field.datatype != self._POINTFIELD_FLOAT32
            or y_field.datatype != self._POINTFIELD_FLOAT32
            or z_field.datatype != self._POINTFIELD_FLOAT32
        ):
            return None

        total_points = int(msg.width) * int(msg.height)
        if total_points <= 0:
            return None

        stride = max(1, math.ceil(total_points / self.config.max_points))
        raw = bytes(msg.data)
        points: list[tuple[float, float, float]] = []
        for idx in range(0, total_points, stride):
            base = idx * int(msg.point_step)
            try:
                x = struct.unpack_from("<f", raw, base + int(x_field.offset))[0]
                y = struct.unpack_from("<f", raw, base + int(y_field.offset))[0]
                z = struct.unpack_from("<f", raw, base + int(z_field.offset))[0]
            except struct.error:
                break
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            points.append((x, y, z))

        if len(points) < 3:
            return None
        return np.asarray(points, dtype=float)

    def _estimate_relative_transform(
        self, target_points: np.ndarray, source_points: np.ndarray
    ) -> Optional[tuple[np.ndarray, np.ndarray]]:
        """Estimate the rigid transform from source points into target points."""
        rotation = np.eye(3, dtype=float)
        translation = np.zeros(3, dtype=float)
        previous_error = None

        for _ in range(self.config.max_iterations):
            transformed_source = (rotation @ source_points.T).T + translation
            matched_target, mean_error = self._nearest_neighbors(
                target_points, transformed_source
            )
            if matched_target is None:
                return None

            delta_rotation, delta_translation = self._best_fit_transform(
                transformed_source, matched_target
            )
            rotation = delta_rotation @ rotation
            translation = delta_rotation @ translation + delta_translation

            if (
                previous_error is not None
                and abs(previous_error - mean_error) < self.config.convergence_tol
            ):
                break
            previous_error = mean_error

        return rotation, translation

    def _nearest_neighbors(
        self, target_points: np.ndarray, transformed_source: np.ndarray
    ) -> tuple[Optional[np.ndarray], float]:
        """Match each transformed source point to the closest target point."""
        matched: list[np.ndarray] = []
        distances: list[float] = []
        max_sq = self.config.correspondence_distance**2

        for source_point in transformed_source:
            deltas = target_points - source_point
            squared_distances = np.einsum("ij,ij->i", deltas, deltas)
            best_idx = int(np.argmin(squared_distances))
            best_sq = float(squared_distances[best_idx])
            if best_sq > max_sq:
                continue
            matched.append(target_points[best_idx])
            distances.append(best_sq)

        if len(matched) < 3:
            return None, float("inf")

        return np.asarray(matched, dtype=float), float(np.sqrt(np.mean(distances)))

    def _best_fit_transform(
        self, source_points: np.ndarray, target_points: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Solve the best-fit rigid alignment with a standard SVD step."""
        source_centroid = source_points.mean(axis=0)
        target_centroid = target_points.mean(axis=0)

        source_centered = source_points - source_centroid
        target_centered = target_points - target_centroid

        covariance = source_centered.T @ target_centered
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T

        translation = target_centroid - rotation @ source_centroid
        return rotation, translation

    def _rotation_matrix_to_quaternion(
        self, rotation: np.ndarray
    ) -> tuple[float, float, float, float]:
        trace = float(np.trace(rotation))
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * s
            qx = (rotation[2, 1] - rotation[1, 2]) / s
            qy = (rotation[0, 2] - rotation[2, 0]) / s
            qz = (rotation[1, 0] - rotation[0, 1]) / s
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation[0, 1] + rotation[1, 0]) / s
            qz = (rotation[0, 2] + rotation[2, 0]) / s
        elif rotation[1, 1] > rotation[2, 2]:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / s
            qx = (rotation[0, 1] + rotation[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / s
            qx = (rotation[0, 2] + rotation[2, 0]) / s
            qy = (rotation[1, 2] + rotation[2, 1]) / s
            qz = 0.25 * s
        return qx, qy, qz, qw
