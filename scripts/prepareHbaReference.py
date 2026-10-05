#!/usr/bin/env python3
"""Prepare independent GLIM-initialized scans for offline HBA refinement.

Deskew uses interpolation of the estimated LiDAR trajectory, never receiver
poses. This approximates within-scan motion; it is not a calibrated IMU spline.
Only scans fully covered by the trajectory are included, without extrapolation.
Dependencies: rosbags, numpy, scipy. Outputs are local data, not source code.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation, Slerp

from prepareOfflineReferenceBag import point_view


def deskew(points, relative_seconds, start, trajectory):
    times = trajectory[:, 0]
    query = start + relative_seconds
    if query.min() < times[0] or query.max() > times[-1]:
        raise ValueError("Scan motion is not covered; extrapolation is forbidden")
    rotations = Slerp(times, Rotation.from_quat(trajectory[:, 4:]))
    translations = np.column_stack([np.interp(query, times, trajectory[:, i]) for i in range(1, 4)])
    origin = np.array([np.interp(start, times, trajectory[:, i]) for i in range(1, 4)])
    return rotations(start).inv().apply(rotations(query).apply(points) + translations - origin)


def prepare(prepared, reference, output, voxel):
    if voxel <= 0:
        raise ValueError("Voxel resolution must be positive")
    output.mkdir(parents=True, exist_ok=False)
    (output / "pcd").mkdir()
    (output / "process1/pcd").mkdir(parents=True)
    (output / "process1/process1/pcd").mkdir(parents=True)
    trajectory = np.loadtxt(reference, ndmin=2)
    store = get_typestore(Stores.ROS2_JAZZY)
    selected = []
    with Reader(prepared / "rosbag2") as reader:
        connections = [c for c in reader.connections if c.topic == "/reference/points"]
        for connection, _, raw in reader.messages(connections=connections):
            cloud = store.deserialize_cdr(raw, connection.msgtype)
            stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
            index = np.searchsorted(trajectory[:, 0], stamp)
            if index == len(trajectory) or abs(trajectory[index, 0] - stamp) > 1e-5:
                index -= 1
            if index < 0 or abs(trajectory[index, 0] - stamp) > 1e-5:
                continue
            xyz = np.column_stack([point_view(cloud, f).ravel() for f in "xyz"])
            relative = point_view(cloud, "t").ravel() * 1e-9
            ranges = np.linalg.norm(xyz, axis=1)
            valid = np.isfinite(xyz).all(axis=1) & (ranges > 1) & (ranges < 100)
            try:
                xyz = deskew(xyz[valid], relative[valid], stamp, trajectory)
            except ValueError:
                continue
            cells = np.floor(xyz / voxel).astype(np.int64)
            _, keep = np.unique(cells, axis=0, return_index=True)
            points = np.column_stack([xyz[keep], np.ones(len(keep))]).astype("<f4")
            header = f"VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n"
            with (output / "pcd" / f"{len(selected):06d}.pcd").open("wb") as stream:
                stream.write(header.encode())
                stream.write(points.tobytes())
            selected.append(trajectory[index])
            if len(selected) % 100 == 0:
                print(f"Prepared {len(selected)} scans", flush=True)
    poses = np.asarray(selected)
    if len(poses) < 50:
        raise ValueError("Too few covered scans for the configured hierarchy")
    np.savetxt(output / "pose.json", poses[:, [1, 2, 3, 7, 4, 5, 6]], fmt="%.12f")
    np.savetxt(output / "initial_lidar.tum", poses, fmt="%.12f")
    (output / "preparation.json").write_text(json.dumps({"scans": len(poses), "voxel_m": voxel,
        "reference_input": str(reference.resolve()), "navigation_pose_used": False,
        "deskew": "LiDAR-pose linear translation / SLERP rotation; no extrapolation",
        "limitations": ["Within-scan motion approximated by scan-rate interpolation", "Dynamic objects are not semantically masked"]}, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prepared", type=Path)
    parser.add_argument("reference", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--voxel", type=float, default=0.2)
    args = parser.parse_args()
    prepare(args.prepared, args.reference, args.output, args.voxel)
