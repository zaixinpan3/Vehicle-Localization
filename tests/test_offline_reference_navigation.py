"""Check physical receiver conventions and held-out batch factor isolation."""
import sys
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from offlineReferenceNavigation import novatel_rotation
from refineOfflineReferenceNavigation import integrate_gyro, solve_poses


class NavigationReferenceTest(unittest.TestCase):
    def test_gyro_integration_uses_seconds_and_subtracts_bias(self):
        times = np.linspace(10., 11., 101)
        samples = np.tile([.01, -.02, np.pi/2 + .03], (101, 1))
        delta = integrate_gyro(np.array([10., 11.]), times, samples, np.array([.01, -.02, .03]))
        np.testing.assert_allclose(delta.apply([[1., 0., 0.]]), [[0., 1., 0.]], atol=1e-12)
        with self.assertRaisesRegex(ValueError, "extrapolate"):
            integrate_gyro(np.array([9., 11.]), times, samples, np.zeros(3))

    def test_novatel_forward_axis_and_nose_up_convention(self):
        r = novatel_rotation([0, 0, 0], [0, 0, 30], [0, 90, 0])
        np.testing.assert_allclose(r.apply([[0, 1, 0]] * 3),
                                   [[0, 1, 0], [1, 0, 0], [0, np.sqrt(3)/2, .5]], atol=1e-12)

    def test_lever_arm_and_withheld_observations(self):
        times = np.arange(8) * .1
        rotation = Rotation.from_euler("z", (np.arange(8) * 10.)[:, None], degrees=True)
        position = np.column_stack([np.sin(times), np.cos(times), times * .1])
        lever = np.array([1.2, -.3, .15])
        trajectory = np.column_stack([times, position + [.2, -.1, .05], rotation.as_quat()])
        target = dict(position=position + rotation.apply(lever), rotation=rotation,
                      sigma=np.full((8, 3), .1), attitude_sigma=np.full((8, 3), .005),
                      lever_lidar_to_ins=lever)
        anchors = np.array([True, True, False, False, False, False, True, True])
        estimated, status = solve_poses(trajectory, target, anchors, max_nfev=100)
        self.assertTrue(status["success"], status)
        np.testing.assert_allclose(estimated[:, 1:4], position, atol=2e-4)
        # A hidden receiver error must not enter either the objective or pose.
        target["position"][~anchors] += 1000
        repeated, _ = solve_poses(trajectory, target, anchors, max_nfev=100)
        np.testing.assert_allclose(repeated, estimated, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
