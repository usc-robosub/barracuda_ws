"""
Tests for GtsamEstimator — focused on the depth-as-factor refactor (issue #20).

Runs without ROS2. Uses a lightweight gtsam stub so the real package is not required.
"""

from __future__ import annotations

import sys
import unittest
from unittest.mock import MagicMock, patch

# ---------------------------------------------------------------------------
# Provide a lightweight gtsam stub so tests run without the real package.
# Each test class that needs it patches gtsam at the module level.
# ---------------------------------------------------------------------------

def _make_gtsam_stub():
    """Return a minimal mock of the gtsam API used by GtsamEstimator."""
    g = MagicMock(name="gtsam")

    # Rot3.Quaternion returns a Rot3-like object
    rot3 = MagicMock(name="Rot3Instance")
    g.Rot3.Quaternion.return_value = rot3

    # Pose3 stores its point so we can inspect it
    class FakePose3:
        def __init__(self, rot, point):
            self._rot = rot
            self._point = point

        def x(self): return self._point._x
        def y(self): return self._point._y
        def z(self): return self._point._z

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

    # Noise models — return plain mocks
    g.noiseModel.Diagonal.Sigmas.return_value = MagicMock(name="DiagNoise")
    g.noiseModel.Isotropic.Sigma.return_value = MagicMock(name="IsoNoise")

    # imuBias
    g.imuBias.ConstantBias.return_value = MagicMock(name="ZeroBias")

    # PreintegrationParams
    params = MagicMock(name="PreintegrationParams")
    g.PreintegrationParams.MakeSharedU.return_value = params

    # GPSFactor records its arguments so tests can inspect them
    class FakeGPSFactor:
        instances = []
        def __init__(self, key, point, noise):
            self.key = key
            self.point = point
            self.noise = noise
            FakeGPSFactor.instances.append(self)

    g.GPSFactor.side_effect = FakeGPSFactor

    # Graph / Values / Optimizer
    graph = MagicMock(name="Graph")
    g.NonlinearFactorGraph.return_value = graph

    initial = MagicMock(name="Values")
    g.Values.return_value = initial

    # Optimizer result
    result = MagicMock(name="Result")
    pose_result = FakePose3(rot3, FakePoint3(1.0, 2.0, -3.0))
    result.atPose3.return_value = pose_result
    import numpy as np
    result.atVector.return_value = np.zeros(3)
    bias_mock = MagicMock()
    bias_mock.accelerometer.return_value = np.zeros(3)
    bias_mock.gyroscope.return_value = np.zeros(3)
    result.atConstantBias.return_value = bias_mock

    optimizer = MagicMock(name="Optimizer")
    optimizer.optimize.return_value = result
    g.LevenbergMarquardtOptimizer.return_value = optimizer

    # PriorFactor*, BetweenFactor*, ImuFactor
    for name in (
        "PriorFactorPose3", "PriorFactorVector", "PriorFactorConstantBias",
        "BetweenFactorConstantBias", "ImuFactor", "PreintegratedImuMeasurements",
    ):
        getattr(g, name).return_value = MagicMock(name=name + "Instance")

    return g, FakeGPSFactor, FakePose3, FakePoint3


# ---------------------------------------------------------------------------
# Helpers to build measurement stubs
# ---------------------------------------------------------------------------

def _dvl(x=1.0, y=2.0, z=5.0, vx=0.1, vy=0.0, vz=0.0,
         qx=0.0, qy=0.0, qz=0.0, qw=1.0):
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


def _imu(t=0.0):
    from barracuda_estimation.measurement_types import ImuSample
    return ImuSample(stamp_sec=t, linear_accel=(0.0, 0.0, 9.81), angular_vel=(0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# Tests — with stubbed GTSAM
# ---------------------------------------------------------------------------

class TestGtsamEstimatorBasic(unittest.TestCase):
    """Basic GtsamEstimator behaviour using the gtsam stub."""

    def setUp(self):
        self.gtsam_stub, *_ = _make_gtsam_stub()

    def _make_estimator(self):
        import importlib
        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            import barracuda_estimation.gtsam_estimator as mod
            importlib.reload(mod)
            mod.gtsam = self.gtsam_stub
            est = mod.GtsamEstimator()
        return est

    def test_is_ready_requires_all_sensors(self):
        est = self._make_estimator()
        self.assertFalse(est.is_ready())
        est.add_imu(_imu())
        self.assertFalse(est.is_ready())
        est.add_depth(_depth())
        self.assertFalse(est.is_ready())
        est.add_dvl(_dvl())
        self.assertTrue(est.is_ready())

class TestDvlToInitialPose3(unittest.TestCase):
    """_dvl_to_initial_pose3 must use DVL xyz and quaternion only."""

    def setUp(self):
        self.gtsam_stub, self.FakeGPSFactor, self.FakePose3, self.FakePoint3 = _make_gtsam_stub()
        self.FakeGPSFactor.instances.clear()

    def _make_estimator(self):
        import importlib
        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            import barracuda_estimation.gtsam_estimator as mod
            importlib.reload(mod)
            # Patch module-level gtsam reference that was captured at import time
            mod.gtsam = self.gtsam_stub
            est = mod.GtsamEstimator()
            est.gtsam_available = True
        return est, mod

    def test_initial_pose_uses_dvl_xyz(self):
        est, mod = self._make_estimator()
        dvl = _dvl(x=1.0, y=2.0, z=5.0)
        depth = _depth(z_value=3.5)  # −depth = −3.5, different from DVL z=5
        est.latest_dvl = dvl
        est.latest_depth = depth

        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            pose = est._dvl_to_initial_pose3()

        # Pose3 must be constructed with DVL x, y, z — NOT depth
        call_args = self.gtsam_stub.Point3.call_args
        px, py, pz = call_args[0]
        self.assertAlmostEqual(px, 1.0, msg="x must come from DVL")
        self.assertAlmostEqual(py, 2.0, msg="y must come from DVL")
        self.assertAlmostEqual(pz, 5.0, msg="z must come from DVL, not depth")
        self.assertNotAlmostEqual(pz, -3.5, msg="depth z must NOT be in pose guess")

    def test_rot3_built_from_dvl_quaternion(self):
        est, mod = self._make_estimator()
        dvl = _dvl(qx=0.1, qy=0.2, qz=0.3, qw=0.9)
        est.latest_dvl = dvl
        est.latest_depth = _depth()

        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            est._dvl_to_initial_pose3()

        self.gtsam_stub.Rot3.Quaternion.assert_called_once()
        args = self.gtsam_stub.Rot3.Quaternion.call_args[0]
        # GTSAM Rot3.Quaternion(w, x, y, z) ordering
        self.assertAlmostEqual(args[0], 0.9)  # w
        self.assertAlmostEqual(args[1], 0.1)  # x
        self.assertAlmostEqual(args[2], 0.2)  # y
        self.assertAlmostEqual(args[3], 0.3)  # z


class TestDepthFactor(unittest.TestCase):
    """_depth_factor must use raw depth z, not the pose's z."""

    def setUp(self):
        self.gtsam_stub, self.FakeGPSFactor, self.FakePose3, self.FakePoint3 = _make_gtsam_stub()
        self.FakeGPSFactor.instances.clear()

    def _make_estimator(self):
        import importlib
        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            import barracuda_estimation.gtsam_estimator as mod
            importlib.reload(mod)
            mod.gtsam = self.gtsam_stub
            est = mod.GtsamEstimator()
            est.gtsam_available = True
        return est, mod

    def test_depth_factor_z_comes_from_depth_sensor(self):
        est, mod = self._make_estimator()
        dvl_z = 5.0
        depth_z_value = 3.5
        est.latest_depth = _depth(z_value=depth_z_value)

        # Build a fake pose where z = dvl_z (the initial guess, NOT depth)
        pose = self.FakePose3(MagicMock(), self.FakePoint3(1.0, 2.0, dvl_z))

        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            est._depth_factor(0, pose)

        # GPSFactor must have been called; inspect the Point3 passed to it
        self.assertTrue(self.FakeGPSFactor.instances, "GPSFactor must be created")
        factor = self.FakeGPSFactor.instances[-1]
        factor_z = factor.point._z

        expected_z = -depth_z_value  # convention: z negated
        self.assertAlmostEqual(factor_z, expected_z,
                               msg=f"GPSFactor z must be -{depth_z_value}, got {factor_z}")
        self.assertNotAlmostEqual(factor_z, dvl_z,
                                  msg="GPSFactor z must NOT equal DVL pose z")

    def test_depth_factor_xy_matches_pose(self):
        """x/y in the GPSFactor must come from the pose (so only z is constrained)."""
        est, mod = self._make_estimator()
        est.latest_depth = _depth(z_value=3.5)
        pose = self.FakePose3(MagicMock(), self.FakePoint3(7.0, -4.0, 5.0))

        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            est._depth_factor(0, pose)

        factor = self.FakeGPSFactor.instances[-1]
        self.assertAlmostEqual(factor.point._x, 7.0)
        self.assertAlmostEqual(factor.point._y, -4.0)

    def test_depth_factor_noise_large_xy_small_z(self):
        """x/y sigmas must be large (>=100) and z sigma must be small (<1)."""
        import numpy as np
        est, mod = self._make_estimator()
        est.latest_depth = _depth(z_value=2.0)
        pose = self.FakePose3(MagicMock(), self.FakePoint3(0.0, 0.0, 0.0))

        captured = {}
        orig = self.gtsam_stub.noiseModel.Diagonal.Sigmas.side_effect

        def capture_sigmas(arr):
            captured['sigmas'] = arr
            return MagicMock()

        self.gtsam_stub.noiseModel.Diagonal.Sigmas.side_effect = capture_sigmas

        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            est._depth_factor(0, pose)

        self.assertIn('sigmas', captured)
        sigmas = captured['sigmas']
        self.assertGreaterEqual(sigmas[0], 100.0, "x sigma must be large")
        self.assertGreaterEqual(sigmas[1], 100.0, "y sigma must be large")
        self.assertLess(sigmas[2], 1.0, "z sigma must be small")


class TestDepthSigmaScaling(unittest.TestCase):
    """_depth_sigma_for_measurement must scale with depth magnitude."""

    def setUp(self):
        self.gtsam_stub, *_ = _make_gtsam_stub()

    def _make_estimator(self):
        import importlib
        with patch.dict(sys.modules, {"gtsam": self.gtsam_stub,
                                       "gtsam.symbol_shorthand": self.gtsam_stub}):
            import barracuda_estimation.gtsam_estimator as mod
            importlib.reload(mod)
            mod.gtsam = self.gtsam_stub
            est = mod.GtsamEstimator()
            est.gtsam_available = True
        return est

    def test_sigma_scales_with_depth(self):
        est = self._make_estimator()
        est.latest_depth = _depth(z_value=10.0)
        sigma_10 = est._depth_sigma_for_measurement()
        est.latest_depth = _depth(z_value=20.0)
        sigma_20 = est._depth_sigma_for_measurement()
        self.assertAlmostEqual(sigma_20, 2 * sigma_10, places=5)

    def test_sigma_respects_minimum(self):
        est = self._make_estimator()
        est.latest_depth = _depth(z_value=0.0001)  # tiny depth → hits floor
        sigma = est._depth_sigma_for_measurement()
        self.assertGreaterEqual(sigma, est.depth_sigma_min)


if __name__ == "__main__":
    unittest.main()
