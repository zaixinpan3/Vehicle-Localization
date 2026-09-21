"""Boundary checks for INSPVA-only pose preparation; requires numpy/pyproj."""
from contextlib import redirect_stdout
from pathlib import Path
import csv
import importlib.util
import io
import sys
import tempfile
import unittest

import numpy as np
from pyproj import Proj

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

SPEC = importlib.util.spec_from_file_location(
    "prepare_poses", Path(__file__).resolve().parents[1] / "scripts/prepareInspvaMappingPoses.py")
POSES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(POSES)


class InspvaPosePreparationTest(unittest.TestCase):
    def test_slerp_crosses_heading_wrap_by_short_arc(self):
        zero = np.zeros(1)
        first = POSES.quaternion_from_rpy(zero, zero, np.deg2rad([179.]))
        second = POSES.quaternion_from_rpy(zero, zero, np.deg2rad([-179.]))
        actual = POSES.interpolate_quaternions(first, second, np.array([.5]))
        np.testing.assert_allclose(np.abs(actual), [[0, 0, 0, 1]], atol=1e-12)

    def test_slerp_handles_equivalent_opposite_signs(self):
        first = np.array([[.5, .5, .5, .5]])
        actual = POSES.interpolate_quaternions(first, -first, np.array([.3]))
        np.testing.assert_allclose(actual, first, atol=1e-12)

    def test_grid_north_agrees_with_projected_north_displacement(self):
        projection = Proj("EPSG:32615")
        lon, lat = -93.2, 44.95
        east, north = projection(lon, lat)
        east2, north2 = projection(lon, lat + 1e-5)
        gamma = projection.get_factors(lon, lat).meridian_convergence
        observed = np.arctan2(north2 - north, east2 - east)
        self.assertAlmostEqual(observed, np.deg2rad(90 + gamma), places=7)

    def test_preparation_retains_free_navigation_and_interpolates_height(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            lidar, pva, output = root / "lidar.csv", root / "pva.csv", root / "poses.csv"
            lidar.write_text("frame_index,stamp_sec\n1,100.25\n2,100.75\n")
            with pva.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(['index','stamp_sec','gps_week','gps_seconds','longitude_deg','latitude_deg',
                                 'height_m','roll_deg','pitch_deg','azimuth_deg','ins_status'])
                for k, t in enumerate(np.linspace(0, 1, 21)):
                    writer.writerow([k+1,100+t,2317,200+t,-93,45,100+4*t,0,0,359+2*t,6])
            with redirect_stdout(io.StringIO()):
                POSES.prepare(lidar, pva, output)
            with output.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 2)
            self.assertEqual([r["pose_source"] for r in rows], ["INSPVA", "INSPVA"])
            self.assertEqual([r["ins_status_upper"] for r in rows], ["6", "6"])
            np.testing.assert_allclose([float(r["pose_z_m"]) for r in rows], [101, 103], atol=1e-12)
            np.testing.assert_allclose([float(r["receiver_time_sec"]) for r in rows], [0, .5], atol=1e-12)


if __name__ == "__main__":
    unittest.main()
