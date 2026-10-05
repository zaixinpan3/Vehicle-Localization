"""Validate frame convention and temporal coverage of HBA scan deskew."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from prepareHbaReference import deskew


class HbaDeskewTest(unittest.TestCase):
    def test_translating_sensor_converts_points_to_scan_start_frame(self):
        trajectory = np.array([[10, 0, 0, 0, 0, 0, 0, 1], [11, 2, 0, 0, 0, 0, 0, 1]])
        # A fixed target at world x=5 appears closer as the sensor moves.
        points = np.array([[5, 0, 0], [4, 0, 0]])
        corrected = deskew(points, np.array([0, .5]), 10, trajectory)
        np.testing.assert_allclose(corrected, [[5, 0, 0], [5, 0, 0]])

    def test_no_motion_extrapolation_beyond_last_estimate(self):
        trajectory = np.array([[10, 0, 0, 0, 0, 0, 0, 1], [11, 0, 0, 0, 0, 0, 0, 1]])
        with self.assertRaisesRegex(ValueError, "extrapolation"):
            deskew(np.ones((1, 3)), np.array([.2]), 10.9, trajectory)


if __name__ == "__main__":
    unittest.main()
