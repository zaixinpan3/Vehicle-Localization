"""Check reference export honesty and independent diagnostic alignment."""
import csv
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.io import loadmat

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from runOfflineReference import export_reference, validate_time_contract
from validateOfflineReference import rigid_alignment, robust_line


class ReferenceExportTest(unittest.TestCase):
    def test_ros_converted_uint32_time_cannot_be_scaled_twice(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)
            sensor = dict(global_shutter_lidar=False, autoconf_perpoint_times=False, perpoint_relative_time=True, perpoint_time_scale=1e-9)
            (path / "config_sensors.json").write_text(json.dumps(dict(sensors=sensor)))
            with self.assertRaisesRegex(ValueError, "already converts UINT32"):
                validate_time_contract(path)
            sensor["perpoint_time_scale"] = 1.0
            (path / "config_sensors.json").write_text(json.dumps(dict(sensors=sensor)))
            validate_time_contract(path)
            sensor["global_shutter_lidar"] = True
            (path / "config_sensors.json").write_text(json.dumps(dict(sensors=sensor)))
            with self.assertRaisesRegex(ValueError, "spinning Ouster"):
                validate_time_contract(path)

    def test_missing_pose_is_blank_not_interpolated(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            prepared, output = root / "prepared", root / "output"
            prepared.mkdir()
            (output / "dump").mkdir(parents=True)
            frames = [dict(frame_index=i + 1, native_start_ns=(i + 1) * 10**9,
                           original_header_ns=(i + 1) * 10**9,
                           original_bag_ns=(i + 1) * 10**9) for i in range(3)]
            with (prepared / "frames.csv").open("w") as stream:
                w = csv.DictWriter(stream, list(frames[0]))
                w.writeheader()
                w.writerows(frames)
            (prepared / "preparation.json").write_text(json.dumps(dict(frames_seen=3, rejected_frames=[])))
            np.savetxt(output / "dump/traj_lidar.txt", [[1, 0, 0, 0, 0, 0, 0, 1], [3, 1, 0, 0, 0, 0, 0, 1]])
            (output / "glim.log").write_text("an exception was caught: inconsistent arguments")
            result = export_reference(prepared, output)
            self.assertEqual(result["frames_estimated"], 2)
            self.assertEqual(result["missing_frame_indices"], [2])
            self.assertFalse(result["external_accuracy_verified"])
            self.assertTrue(result["logged_solver_issues"])
            with (output / "reference_poses.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[1]["x_m"], "")
            self.assertEqual(rows[1]["estimated"], "0")
            self.assertEqual(rows[0]["quality"], "solver_issue_do_not_score")
            matlab = loadmat(output / "reference_poses.mat")
            self.assertTrue(np.isnan(matlab["positionMeters"][1]).all())
            np.testing.assert_array_equal(matlab["estimatedMask"].ravel(), [True, False, True])

    def test_alignment_recovers_rigid_transform_without_scale(self):
        rng = np.random.default_rng(9)
        a = rng.normal(size=(50, 3))
        expected = Rotation.from_euler("xyz", [.2, -.1, .5]).as_matrix()
        target = a @ expected.T + [3, -2, 4]
        r, t, _ = rigid_alignment(a, target)
        np.testing.assert_allclose(a @ r.T + t, target, atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r), 1.)

    def test_rejected_timing_retains_original_matlab_frame_slot(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            prepared, output = root / "prepared", root / "output"
            prepared.mkdir()
            (output / "dump").mkdir(parents=True)
            frames = [dict(frame_index=i, native_start_ns=i*10**9,
                           original_header_ns=i*10**9, original_bag_ns=i*10**9) for i in [1, 3]]
            with (prepared / "frames.csv").open("w") as stream:
                writer = csv.DictWriter(stream, list(frames[0]))
                writer.writeheader()
                writer.writerows(frames)
            (prepared / "preparation.json").write_text(json.dumps(dict(frames_seen=3,
                rejected_frames=[dict(frame_index=2, reason="unverified native timestamp")])))
            np.savetxt(output / "dump/traj_lidar.txt", [[1, 1, 0, 0, 0, 0, 0, 1], [3, 3, 0, 0, 0, 0, 0, 1]])
            (output / "glim.log").write_text("")
            result = export_reference(prepared, output)
            self.assertEqual(result["frames_prepared"], 2)
            self.assertEqual(result["frames_output"], 3)
            matlab = loadmat(output / "reference_poses.mat")
            np.testing.assert_array_equal(matlab["frameIndex"].ravel(), [1, 2, 3])
            np.testing.assert_allclose(matlab["positionMeters"][[0, 2], 0], [1, 3])
            self.assertTrue(np.isnan(matlab["positionMeters"][1]).all())
            self.assertTrue(np.isnan(matlab["nativeTimeSec"].ravel()[1]))

    def test_clock_fit_rejects_receipt_spikes(self):
        x = np.arange(100, dtype=float) * .02 + 400000
        y = (x - x[0]) * 1.0001 + 9000
        y[::10] += .05
        coef, _, _, _ = robust_line(x, y)
        self.assertAlmostEqual(coef[0], 1.0001, places=6)


if __name__ == "__main__":
    unittest.main()
