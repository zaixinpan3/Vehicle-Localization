#!/usr/bin/env python3
"""Publish a deliberately selected, auditable local pseudo-reference set.

The selection JSON contains a sequences array with sequence, source, reason
and optional unavailable_reason fields. Each source is an absolute directory
containing audited exports. No accuracy is inferred from smoothness or solver
success. Original candidates and failures remain untouched.
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.io import loadmat

from runOfflineReference import sha256, validate_time_contract


def verify_timing_provenance(source, seen=None):
    """Follow actual initialization artifacts back to a verified GLIM run."""
    seen = set() if seen is None else seen
    if source in seen:
        raise ValueError("Circular reference provenance")
    seen.add(source)
    config = source / "config/config_sensors.json"
    if config.exists():
        validate_time_contract(config.parent)
        run = json.loads((source / "run.json").read_text())
        if sha256(config) != run["config_hashes"][config.name]:
            raise ValueError("Runtime sensor configuration has changed since estimation")
        return
    quality = json.loads((source / "quality.json").read_text())
    if quality.get("initial_reference_path"):
        initial = Path(quality["initial_reference_path"])
        if sha256(initial / "reference_lidar.tum") != quality["initial_trajectory_sha256"]:
            raise ValueError("Navigation smoother initialization was modified")
    else:
        preparation = json.loads((source / "preparation.json").read_text())
        input_file = Path(preparation["reference_input"])
        if preparation.get("reference_input_sha256") and sha256(input_file) != preparation["reference_input_sha256"]:
            raise ValueError("HBA initialization source was modified")
        initial = input_file.parent
    verify_timing_provenance(initial.resolve(), seen)


def publish(selection, output):
    plan = json.loads(selection.read_text())
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for choice in plan["sequences"]:
        relative = Path(choice["sequence"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Sequence paths must stay inside the output root")
        record = dict(choice)
        if not choice.get("source"):
            record["status"] = "unavailable"
            records.append(record)
            continue
        source = Path(choice["source"]).resolve()
        verify_timing_provenance(source)
        quality = json.loads((source / "quality.json").read_text())
        if quality.get("logged_solver_issues") or not quality.get("solver", {}).get("success", True):
            raise ValueError(f"Rejected solver candidate: {source}")
        if quality["reference_kind"] != "pseudo_ground_truth":
            raise ValueError("This publisher does not certify ground truth")
        trajectory = np.loadtxt(source / "reference_lidar.tum", ndmin=2)
        if trajectory.shape[1] != 8 or not np.isfinite(trajectory).all() or not np.all(np.diff(trajectory[:, 0]) > 0):
            raise ValueError("Malformed trajectory")
        if np.max(np.abs(np.linalg.norm(trajectory[:, 4:], axis=1) - 1)) > 1e-3:
            raise ValueError("Invalid quaternion")
        mat = loadmat(source / "reference_poses.mat")
        if int(mat["estimatedMask"].sum()) != quality["frames_estimated"]:
            raise ValueError("Frame-indexed MAT coverage disagrees with quality metadata")
        if len(trajectory) != quality["frames_estimated"]:
            raise ValueError("TUM coverage disagrees with original-frame estimates")
        if not np.array_equal(mat["frameIndex"].ravel(), np.arange(1, quality["source_frames_seen"] + 1)):
            raise ValueError("Original frame slots must be retained, including timing rejections")
        mask = mat["estimatedMask"].ravel().astype(bool)
        if not np.isfinite(mat["positionMeters"][mask]).all() or not np.isnan(mat["positionMeters"][~mask]).all():
            raise ValueError("Estimated and unavailable position slots must be explicit")
        if bool(mat["navigationPoseUsedForEstimation"].ravel()[0]) != quality["navigation_pose_used_for_estimation"]:
            raise ValueError("MAT navigation-use provenance disagrees with quality metadata")
        destination = output / relative
        destination.mkdir(parents=True, exist_ok=False)
        hashes = {}
        for name in ["reference_lidar.tum", "reference_poses.csv", "reference_poses.mat", "quality.json",
                     "reference_receiver_origin.tum", "reference_forward_left_up_at_receiver_origin.tum",
                     "navigation_usage.csv", "receiver_comparison.json", "receiver_comparison.csv"]:
            if (source / name).exists():
                shutil.copy2(source / name, destination / name)
                hashes[name] = sha256(destination / name)
                if hashes[name] != sha256(source / name):
                    raise ValueError("Published copy differs from its source")
        record.update(status="published_provisional", hashes=hashes, frames_estimated=quality["frames_estimated"],
                      frames_prepared=quality["frames_prepared"], pose_frame="front_ouster",
                      navigation_used=quality["navigation_pose_used_for_estimation"])
        records.append(record)
    manifest = dict(reference_kind="pseudo_ground_truth", absolute_accuracy_certified=False,
                    selection_sha256=sha256(selection), selection_protocol=plan.get("protocol"), sequences=records,
                    qualification="Independent of the project localization algorithm. Shared LiDAR and fitted receiver observations prevent treating these estimates as independent surveyed truth.")
    (output / "reference_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(dict(published=sum(r["status"] == "published_provisional" for r in records),
                          unavailable=sum(r["status"] == "unavailable" for r in records),
                          estimated_frames=sum(r.get("frames_estimated", 0) for r in records)), indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    publish(args.selection, args.output)
