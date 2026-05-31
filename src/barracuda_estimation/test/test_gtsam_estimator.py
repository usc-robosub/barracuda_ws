"""
Tests for the current pose-only GtsamEstimator path.

These tests run without the real GTSAM package by patching in a lightweight
stub before importing the estimator modules.
"""

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
    g.BetweenFactorPose3.return_value = MagicMock(name="BetweenFactorPose3")

    class FakeGPSFactor:
        instances = []

        def __init__(self, key, point, noise):
            self.key = key
            self.point = point
            self.noise = noise
            FakeGPSFactor.instances.append(self)

    g.GPSFactor.side_effect = FakeGPSFactor

    result = MagicMock(name="Result")
    result.atPose3.return_value = FakePose3(rot3, FakePoint3(1.0, 2.0, -3.0))
    optimizer = MagicMock(name="Optimizer")
    optimizer.optimize.return_value = result
    g.LevenbergMarquardtOptimizer.return_value = optimizer

    return g, symbol_shorthand, FakeGPSFactor


def _pose(x=1.0, y=2.0, z=3.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    from barracuda_estimation.measurement_types import PoseSample

    return PoseSample(
        stamp_sec=0.0,
        position_xyz=(x, y, z),
        orientation_xyzw=(qx, qy, qz, qw),
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


def _depth(z_value=3.5):
    from barracuda_estimation.measurement_types import DepthSample

    return DepthSample(stamp_sec=0.0, z_value=z_value, source="range")


class TestGtsamEstimator(unittest.TestCase):
    def setUp(self):
        self.gtsam_stub, self.symbols_stub, self.fake_gps_factor = _make_gtsam_stub()
        self.fake_gps_factor.instances.clear()

    def _make_estimator(self):
        with patch.dict(
            sys.modules,
            {
                "gtsam": self.gtsam_stub,
                "gtsam.symbol_shorthand": self.symbols_stub,
            },
        ):
            import barracuda_estimation.factor_graph as factor_graph_mod
            import barracuda_estimation.gtsam_estimator as estimator_mod

            importlib.reload(factor_graph_mod)
            importlib.reload(estimator_mod)
            return estimator_mod.GtsamEstimator()

    def test_is_ready_requires_pose_and_depth_but_not_dvl(self):
        est = self._make_estimator()
        self.assertFalse(est.is_ready())
        est.add_pose(_pose())
        self.assertFalse(est.is_ready())
        est.add_depth(_depth())
        self.assertTrue(est.is_ready())

    def test_pose_guess_comes_from_pose_sample(self):
        est = self._make_estimator()
        est.add_pose(_pose(x=4.0, y=5.0, z=6.0, qx=0.1, qy=0.2, qz=0.3, qw=0.9))

        est._pose3_initial_guess()

        point_args = self.gtsam_stub.Point3.call_args[0]
        quat_args = self.gtsam_stub.Rot3.Quaternion.call_args[0]
        self.assertEqual(point_args, (4.0, 5.0, 6.0))
        self.assertEqual(quat_args, (0.9, 0.1, 0.2, 0.3))

    def test_step_runs_without_dvl(self):
        est = self._make_estimator()
        est.add_pose(_pose())
        est.add_depth(_depth())

        estimate = est.step(1.0)

        self.assertIsNotNone(estimate)
        self.assertEqual(estimate.velocity_xyz, (0.0, 0.0, 0.0))

    def test_depth_factor_uses_depth_measurement(self):
        est = self._make_estimator()
        est.add_pose(_pose(x=1.0, y=2.0, z=8.0))
        est.add_depth(_depth(z_value=3.5))

        estimate = est.step(1.0)

        self.assertIsNotNone(estimate)
        self.assertTrue(self.fake_gps_factor.instances)
        factor = self.fake_gps_factor.instances[-1]
        self.assertEqual(factor.point._x, 1.0)
        self.assertEqual(factor.point._y, 2.0)
        self.assertEqual(factor.point._z, -3.5)


if __name__ == "__main__":
    unittest.main()
