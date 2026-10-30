"""Guard initialization-only alignment, lever conversion, and failure scoring."""

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np
import scoreTrajectory
from scipy.spatial.transform import Rotation


class ScoringTest(unittest.TestCase):
    def fixture(self, folder, shift=0, missing=False):
        frames = np.array([22, 23]) if not missing else np.array([22])
        times = np.arange(24) / 10
        H = Rotation.from_euler("z", 0.7)
        raw_to_ref = Rotation.from_euler("xyz", [0.01, -0.03, 0.02])
        lever = np.array([2.3, 0.15, 0.1])
        offset = np.array([10.0, -5.0, 3.0])

        def at(query):
            return np.column_stack(
                [query, np.zeros(len(query)), np.zeros(len(query))]
            ), Rotation.from_euler("z", np.zeros((len(query), 1)))

        world, _truth_r = at(times[frames - 1])
        local = H.inv().apply(world + lever - offset)
        local[:, 0] += shift
        q = (H.inv() * raw_to_ref).as_quat()
        values = np.column_stack(
            [
                frames,
                np.full(len(frames), 0.01),
                np.full(len(frames), 8),
                np.full(len(frames), 100),
                times[frames - 1],
                local,
                np.tile(q, (len(frames), 1)),
            ]
        )
        path = folder / "poses.csv"
        np.savetxt(
            path,
            values,
            delimiter=",",
            header="frame,processing_s,matched,map_points,native_end,lx,ly,lz,lqx,lqy,lqz,lqw",
            comments="",
        )
        Path(str(path) + ".anchor.json").write_text(
            json.dumps({"frame": 4, "position": [0, 0, 0], "quaternion": [0, 0, 0, 1]})
        )
        ref = {
            "anchorFrame": 4,
            "anchorRotation": H.as_matrix(),
            "anchorPosition": offset,
            "rawToRef": raw_to_ref.as_matrix(),
            "lever": lever,
            "frameTimes": times,
            "at": at,
            "commonEndFrame": 23,
            "frames": {"nativeEnd": times},
        }
        return path, ref

    def test_exact_initial_transform_and_lever(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, ref = self.fixture(Path(tmp))
            with patch.object(scoreTrajectory, "reference", return_value=ref):
                metrics, _ = scoreTrajectory.score(1, Path(tmp), path)
            self.assertTrue(metrics["complete"])
            self.assertLess(metrics["positionMaximumM"], 1e-12)
            self.assertLess(metrics["headingRmseDeg"], 1e-12)

    def test_drift_is_not_fitted_away(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, ref = self.fixture(Path(tmp), shift=3)
            with patch.object(scoreTrajectory, "reference", return_value=ref):
                metrics, _ = scoreTrajectory.score(1, Path(tmp), path)
            self.assertAlmostEqual(metrics["positionRmseM"], 3)

    def test_missing_frame_retains_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            path, ref = self.fixture(Path(tmp), missing=True)
            # genfromtxt returns a scalar for one row; production expects many.
            trajectory = np.atleast_1d(scoreTrajectory.table(path))
            with (
                patch.object(scoreTrajectory, "reference", return_value=ref),
                patch.object(scoreTrajectory, "table", return_value=trajectory),
            ):
                metrics, _ = scoreTrajectory.score(1, Path(tmp), path)
            self.assertFalse(metrics["complete"])
            self.assertTrue(np.isinf(metrics["positionRmseM"]))


if __name__ == "__main__":
    unittest.main()
