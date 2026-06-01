"""Tests for the IMU + depth + DVL GtsamEstimator path."""

from __future__ import annotations

import importlib
import os
import sys
import unittest
from unittest.mock import MagicMock, patch

THIS_DIR = os.path.dirname(__file__)
PACKAGE_ROOT = os.path.abspath(os.path.join(THIS_DIR, ".."))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)


def _make_gtsam_stub():
    g = MagicMock(name="gtsam")

    def fake_x(idx):
        return f"X{idx}"

    symbol_shorthand = MagicMock(name="symbol_shorthand")
    symbol_shorthand.X.side_effect = fake_x

    rot3 = MagicMock(name="Rot3Instance")
    g.Rot3.Quaternion.return_value = rot3

    class FakePose3:
        def __init__(self, rot=None, point=None):
            self._rot = rot
            self._point = point

        def x(self):
            return self._point._x

        def y(self):
            return self._point._y

        def z(self):
            return self._point._z

        def between(self, other):
            return FakePose3(other._rot, other._point)

        def rotation(self):
            q = MagicMock()
            q.x.return_value = 0.0
            q.y.return_value = 0.0
            q.z.return_value = 0.0
            q.w.return_value = 1.0
            rot = MagicMock()
            rot.matrix.return_value = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
            rot.toQuaternion.return_value = q
            return rot

    class FakePoint3:
        def __init__(self, x, y, z):
            self._x = x
            self._y = y
            self._z = z

    g.Pose3.side_effect = FakePose3
    g.Point3.side_effect = FakePoint3
    g.noiseModel.Diagonal.Sigmas.return_value = MagicMock(name="DiagNoise")
    g.NonlinearFactorGraph.return_value = MagicMock(name="Graph")
    g.Values.return_value = MagicMock(name="Values")
    g.PriorFactorPose3.return_value = MagicMock(name="PriorFactorPose3")
    g.PriorFactorVector.return_value = MagicMock(name="PriorFactorVector")
    g.PriorFactorConstantBias.return_value = MagicMock(name="PriorFactorConstantBias")
    g.CombinedImuFactor.return_value = MagicMock(name="CombinedImuFactor")
    g.BetweenFactorPose3.return_value = MagicMock(name="BetweenFactorPose3")
    g.imuBias.ConstantBias.return_value = MagicMock(
        name="ConstantBias",
        accelerometer=MagicMock(return_value=[0.0, 0.0, 0.0]),
        gyroscope=MagicMock(return_value=[0.0, 0.0, 0.0]),
    )
    g.PreintegratedCombinedMeasurements.return_value = MagicMock(
        name="PreintegratedCombinedMeasurements"
    )
    g.PreintegrationCombinedParams.MakeSharedU.return_value = MagicMock(
        name="PreintegrationCombinedParams"
    )
    g.noiseModel.Isotropic.Sigma.return_value = MagicMock(name="IsoNoise")

    class FakeCustomFactor:
        instances = []

        def __init__(self, noise, keys, error_func):
            self.noise = noise
            self.keys = list(keys)
            self.error_func = error_func
            FakeCustomFactor.instances.append(self)

    g.CustomFactor.side_effect = FakeCustomFactor

    result = MagicMock(name="Result")
    result.atPose3.return_value = FakePose3(rot3, FakePoint3(1.0, 2.0, -3.0))
    result.atVector.return_value = [0.4, 0.5, 0.6]
    result.atConstantBias.return_value = g.imuBias.ConstantBias.return_value
    optimizer = MagicMock(name="Optimizer")
    optimizer.optimize.return_value = result
    g.LevenbergMarquardtOptimizer.return_value = optimizer

    return g, symbol_shorthand, FakeCustomFactor


def _imu(ax=0.0, ay=0.0, az=9.81, gx=0.0, gy=0.0, gz=0.0, stamp=0.0):
    from barracuda_estimation.measurement_types import ImuSample

    return ImuSample(
        stamp_sec=stamp,
        linear_accel=(ax, ay, az),
        angular_vel=(gx, gy, gz),
    )


def _dvl(
    x=1.0,
    y=2.0,
    z=5.0,
    vx=0.1,
    vy=0.0,
    vz=0.0,
    qx=0.0,
    qy=0.0,
    qz=0.0,
    qw=1.0,
):
    from barracuda_estimation.measurement_types import DvlSample

    return DvlSample(
        stamp_sec=0.0,
        position_xyz=(x, y, z),
        velocity_xyz=(vx, vy, vz),
        orientation_xyzw=(qx, qy, qz, qw),
    )


def _depth(z_value=3.5, source="range"):
    from barracuda_estimation.measurement_types import DepthSample

    return DepthSample(stamp_sec=0.0, z_value=z_value, source=source)


def _camera_relative_pose(tx=0.2, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    from barracuda_estimation.measurement_types import CameraRelativePoseSample

    return CameraRelativePoseSample(
        stamp_sec=0.0,
        translation_xyz=(tx, ty, tz),
        orientation_xyzw=(qx, qy, qz, qw),
    )


class TestGtsamEstimator(unittest.TestCase):
    def setUp(self):
        self.gtsam_stub, self.symbols_stub, self.fake_custom_factor = _make_gtsam_stub()
        self.fake_custom_factor.instances.clear()

    def _make_estimator(self):
        with patch.dict(
            sys.modules,
            {
                "gtsam": self.gtsam_stub,
                "gtsam.symbol_shorthand": self.symbols_stub,
            },
        ):
            import barracuda_estimation.gtsam_estimator as estimator_mod

            importlib.reload(estimator_mod)
            return estimator_mod.GtsamEstimator()

    def test_is_ready_requires_imu_depth_and_dvl(self):
        est = self._make_estimator()
        self.assertFalse(est.is_ready())
        est.add_imu(_imu())
        self.assertFalse(est.is_ready())
        est.add_depth(_depth())
        self.assertFalse(est.is_ready())
        est.add_dvl(_dvl())
        self.assertTrue(est.is_ready())

    def test_initial_pose_guess_comes_from_dvl_sample(self):
        est = self._make_estimator()
        est.add_dvl(_dvl(x=4.0, y=5.0, z=6.0, qx=0.1, qy=0.2, qz=0.3, qw=0.9))

        est._dvl_to_initial_pose3()

        point_args = self.gtsam_stub.Point3.call_args[0]
        quat_args = self.gtsam_stub.Rot3.Quaternion.call_args[0]
        self.assertEqual(point_args, (4.0, 5.0, 6.0))
        self.assertEqual(quat_args, (0.9, 0.1, 0.2, 0.3))

    def test_step_clears_imu_buffer_after_estimate(self):
        est = self._make_estimator()
        est.add_imu(_imu(stamp=0.0))
        est.add_imu(_imu(stamp=0.1))
        est.add_depth(_depth())
        est.add_dvl(_dvl())

        estimate = est.step(1.0)
        est.add_imu(_imu(stamp=0.2))
        est.add_imu(_imu(stamp=0.3))
        est.add_depth(_depth())
        est.add_dvl(_dvl())
        estimate = est.step(2.0)

        self.assertIsNotNone(estimate)
        self.assertEqual(len(est.imu_buffer), 0)
        self.assertEqual(estimate.velocity_xyz, (0.4, 0.5, 0.6))
        self.gtsam_stub.CombinedImuFactor.assert_called()

    def test_camera_relative_pose_adds_between_factor(self):
        est = self._make_estimator()
        est.add_imu(_imu(stamp=0.0))
        est.add_depth(_depth())
        est.add_dvl(_dvl())
        est.step(1.0)

        est.add_imu(_imu(stamp=0.2))
        est.add_depth(_depth())
        est.add_dvl(_dvl())
        est.add_camera_relative_pose(_camera_relative_pose(tx=0.3, ty=0.1, tz=-0.2))
        est.step(2.0)

        self.gtsam_stub.BetweenFactorPose3.assert_called()

    def test_altimeter_factor_uses_depth_measurement(self):
        est = self._make_estimator()
        est.add_imu(_imu())
        est.add_depth(_depth(z_value=3.5))
        est.add_dvl(_dvl(x=1.0, y=2.0, z=-3.5))

        estimate = est.step(1.0)

        self.assertIsNotNone(estimate)
        self.assertTrue(self.fake_custom_factor.instances)
        factor = self.fake_custom_factor.instances[0]
        pose = est._dvl_to_initial_pose3()

        class FakeValues:
            def atPose3(self, key):
                return pose

        residual = factor.error_func(None, FakeValues(), None)
        self.assertEqual(float(residual[0]), 0.0)


if __name__ == "__main__":
    unittest.main()
