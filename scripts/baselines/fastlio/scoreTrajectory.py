"""Score a FAST-LIO trajectory with one declared initialization transform.

No trajectory-wide alignment is fitted. References enter only this scorer,
initialization, and the separate offline map builder.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from referenceSupport import reference, table
from scipy.spatial.transform import Rotation


def score(drive, temporary, path):
    ref = reference(drive, temporary)
    trajectory = np.atleast_1d(table(path))
    anchor = json.loads(Path(str(path) + ".anchor.json").read_text())
    assert anchor["frame"] == ref["anchorFrame"]
    R0 = Rotation.from_quat(anchor["quaternion"]).as_matrix()
    H = ref["anchorRotation"] @ R0.T
    offset = ref["anchorPosition"] - H @ np.array(anchor["position"])
    pose = Rotation.from_quat(
        np.column_stack([trajectory[k] for k in ["lqx", "lqy", "lqz", "lqw"]])
    )
    p = np.column_stack([trajectory[k] for k in ["lx", "ly", "lz"]]) @ H.T + offset
    R = Rotation.from_matrix(H @ pose.as_matrix() @ ref["rawToRef"].T)
    p -= R.apply(np.broadcast_to(ref["lever"], p.shape))
    frame = trajectory["frame"].astype(int)
    t = ref["frameTimes"][frame - 1]
    truth, orientation = ref["at"](t)
    yaw = R.as_euler("xyz")[:, 2]
    truthYaw = orientation.as_euler("xyz")[:, 2]
    angle = np.arctan2(np.sin(yaw - truthYaw), np.cos(yaw - truthYaw))
    distance = np.linalg.norm(p[:, :2] - truth[:, :2], axis=1)
    startFrame = 22 if drive == 1 else ref["anchorFrame"]
    mask = (frame <= ref["commonEndFrame"]) & (frame >= startFrame)
    expected = np.arange(startFrame, ref["commonEndFrame"] + 1)
    complete = np.array_equal(frame[mask], expected)
    errors = distance[mask]
    heading = angle[mask]
    metrics = {
        "drive": drive,
        "frames": len(expected),
        "completedFrames": int(mask.sum()),
        "complete": complete,
        "positionRmseM": float(np.sqrt(np.mean(errors**2)))
        if complete
        else float("inf"),
        "positionP95M": float(np.percentile(errors, 95)) if complete else float("inf"),
        "positionMaximumM": float(errors.max()) if complete else float("inf"),
        "headingRmseDeg": float(np.rad2deg(np.sqrt(np.mean(heading**2))))
        if complete
        else float("inf"),
        "processingMeanMs": float(np.mean(trajectory["processing_s"][mask]) * 1000),
        "processingP95Ms": float(
            np.percentile(trajectory["processing_s"][mask], 95) * 1000
        ),
        "updateMeanMs": float(np.mean(trajectory["update_s"][mask]) * 1000)
        if "update_s" in trajectory.dtype.names
        else None,
        "mapPointsMaximum": int(trajectory["map_points"][mask].max()),
        "matchedMean": float(trajectory["matched"][mask].mean()),
        "maximumScanEndOffsetMs": float(
            np.max(
                np.abs(trajectory["native_end"] - ref["frames"]["nativeEnd"][frame - 1])
            )
            * 1000
        ),
        "anchorFrame": anchor["frame"],
        "evaluation": "Exact common frame indices: sequence1 frames22:1169 (stored after_2s window), sequence2 frames401:778; exclude last frame as in motion-covered project replay",
        "initialization": "One reference pose at declared anchor, used as an initial coordinate transform; no whole-trajectory fit",
    }
    return metrics, {
        "time": t[mask],
        "error": distance[mask],
        "heading": np.rad2deg(heading),
        "xy": p[mask, :2],
        "reference": truth[mask, :2],
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--drive", type=int, required=True)
    p.add_argument("--temporary", type=Path, required=True)
    p.add_argument("--trajectory", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    metrics, _ = score(a.drive, a.temporary, a.trajectory)
    a.output.write_text(json.dumps(metrics, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
