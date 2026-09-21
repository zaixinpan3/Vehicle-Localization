#!/usr/bin/env python3
"""Create INSPVA-only UTM poses at LiDAR times without a Mapping Toolbox.

Run from the repository root with:
uv run --offline --with numpy --with pyproj python scripts/prepareInspvaMappingPoses.py
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json

import numpy as np
from pyproj import Proj, Transformer
from receiverClock import ensure_clock, convert_time, native_seconds


def quaternion_from_rpy(roll, pitch, yaw):
    """Return x-forward/y-left/z-up active-rotation quaternions [w,x,y,z]."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.column_stack((cr * cp * cy + sr * sp * sy,
                            sr * cp * cy - cr * sp * sy,
                            cr * sp * cy + sr * cp * sy,
                            cr * cp * sy - sr * sp * cy))


def interpolate_quaternions(first, second, fraction):
    """Shortest-arc SLERP, including equivalent opposite quaternion signs."""
    second = np.array(second, copy=True)
    dot = np.sum(first * second, axis=1)
    second[dot < 0] *= -1
    dot = np.clip(np.abs(dot), 0, 1)
    angle = np.arccos(dot)
    result = (1 - fraction[:, None]) * first + fraction[:, None] * second
    curved = dot < 0.9995
    theta = angle[curved]
    u = fraction[curved]
    result[curved] = (np.sin((1 - u) * theta)[:, None] * first[curved]
                     + np.sin(u * theta)[:, None] * second[curved]) / np.sin(theta)[:, None]
    return result / np.linalg.norm(result, axis=1)[:, None]


def prepare(lidar_path, inspva_path, output_path):
    lidar = np.genfromtxt(lidar_path, delimiter=",", names=True)
    pva = np.genfromtxt(inspva_path, delimiter=",", names=True)
    clock = ensure_clock(inspva_path)
    receiver = native_seconds(pva["gps_week"], pva["gps_seconds"])
    target = convert_time(clock, lidar["stamp_sec"])
    assert np.all(np.diff(target) > 0) and target[0] >= receiver[0] and target[-1] <= receiver[-1]
    upper = np.clip(np.searchsorted(receiver, target, side="right"), 1, len(receiver) - 1)
    lower = upper - 1
    fraction = (target - receiver[lower]) / (receiver[upper] - receiver[lower])
    assert np.all((fraction >= 0) & (fraction <= 1))
    lon, lat = pva["longitude_deg"], pva["latitude_deg"]
    assert np.all((lon >= -96) & (lon < -90) & (lat >= 0) & (lat < 84)), "Expected UTM zone 15N."
    projection = Proj("EPSG:32615")
    east, north = Transformer.from_crs(4326, 32615, always_xy=True).transform(lon, lat)
    xyz = np.column_stack((east, north, pva["height_m"]))
    gamma = np.asarray(projection.get_factors(lon, lat).meridian_convergence)
    # Same body-axis convention as the NovAtel ROS driver, with true-to-grid
    # north convergence added because XY is projected UTM, not tangent ENU.
    yaw = np.deg2rad(90 - pva["azimuth_deg"] + gamma)
    quat = quaternion_from_rpy(np.deg2rad(pva["roll_deg"]), -np.deg2rad(pva["pitch_deg"]), yaw)
    position = (1 - fraction[:, None]) * xyz[lower] + fraction[:, None] * xyz[upper]
    orientation = interpolate_quaternions(quat[lower], quat[upper], fraction)
    assert np.all(np.isfinite(position)) and np.all(np.isfinite(orientation))
    fields = ["frame_index", "lidar_stamp_sec", "receiver_time_sec", "pose_source", "pose_crs",
              "pose_height_datum", "pose_reference_point", "pose_x_m", "pose_y_m", "pose_z_m",
              "pose_qw", "pose_qx", "pose_qy", "pose_qz", "inspva_lower_index",
              "inspva_upper_index", "interpolation_fraction", "ins_status_lower", "ins_status_upper", "clock_model_id"]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(fields)
        for k in range(len(lidar)):
            writer.writerow([int(lidar["frame_index"][k]), lidar["stamp_sec"][k], target[k] - target[0],
                             "INSPVA", "EPSG:32615", "ellipsoidal", "recorded_INS_output_point",
                             *position[k], *orientation[k], int(pva["index"][lower[k]]),
                             int(pva["index"][upper[k]]), fraction[k],
                             int(pva["ins_status"][lower[k]]), int(pva["ins_status"][upper[k]]), clock["modelId"]])
    metadata = {
        "source": "INSPVA only; no ODOM or BESTPOS position/orientation input",
        "frames": len(lidar), "native_samples": len(pva), "epsg": 32615,
        "height_datum": "INSPVA ellipsoidal height; no mixed MSL height",
        "clock": clock,
        "position_interpolation": "Linear projected XYZ at bridged receiver time",
        "orientation": "SLERP of Rz(90-azimuth+grid_convergence) Ry(-pitch) Rx(roll)",
        "grid_convergence_deg_range": [float(gamma.min()), float(gamma.max())],
        "reference_point": "Recorded INS output point; existing LiDAR calibration retained, no new lever arm inferred",
        "frames_with_non_good_bracket": int(np.count_nonzero((pva["ins_status"][lower] != 3)
                                                            | (pva["ins_status"][upper] != 3))),
        "quality_handling": "All frames retained; bracketing INS statuses recorded, no ground-truth claim",
        "inputs": [{"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                   for path in (lidar_path, inspva_path)],
        "output_sha256": hashlib.sha256(output_path.read_bytes()).hexdigest(),
    }
    output_path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    root = Path(__file__).resolve().parents[1]
    folder = root / "data/raw/Missisipi/gnss"
    stem = "raw_data_2024-06-07-12-09-31_0"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lidar", type=Path, default=folder / (stem + "_front_lidar_points.csv"))
    parser.add_argument("--inspva", type=Path, default=folder / (stem + "_inspva.csv"))
    parser.add_argument("--output", type=Path, default=folder / (stem + "_front_lidar_synchronized_pose_1_1170.csv"))
    args = parser.parse_args()
    prepare(args.lidar, args.inspva, args.output)
