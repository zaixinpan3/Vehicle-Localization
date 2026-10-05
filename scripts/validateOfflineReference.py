#!/usr/bin/env python3
"""Compare a LiDAR-only fitted reference to withheld NovAtel navigation.

Uses receiver data only after SLAM. A single rigid alignment is fitted on the
first quarter and evaluated on the rest. These are uncalibrated origin/timing
diagnostics, not certified ground-truth errors. No fitted trajectory is changed.
"""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from rosbags.rosbag1 import Reader

from extractGnssFromBag import build_typestore, inspva_row, inspvax_row
from extractOusterImuFromBag import decode_packet


def robust_line(x, y):
    x0, y0 = x[0], y[0]
    a = np.column_stack([x - x0, np.ones(len(x))])
    target = y - y0
    keep = np.ones(len(x), dtype=bool)
    for _ in range(5):
        coef = np.linalg.lstsq(a[keep], target[keep], rcond=None)[0]
        residual = target - a @ coef
        median = np.median(residual)
        mad = np.median(np.abs(residual - median))
        keep = np.abs(residual - median) < max(0.002, 4 * 1.4826 * mad)
    return coef, x0, y0, residual


def enu(latitude, longitude, height):
    lat, lon = np.deg2rad(latitude), np.deg2rad(longitude)
    a, e2 = 6378137., 6.69437999014e-3
    n = a / np.sqrt(1 - e2 * np.sin(lat)**2)
    xyz = np.column_stack([(n + height) * np.cos(lat) * np.cos(lon),
                           (n + height) * np.cos(lat) * np.sin(lon),
                           (n * (1 - e2) + height) * np.sin(lat)])
    r = np.array([[-np.sin(lon[0]), np.cos(lon[0]), 0],
                  [-np.sin(lat[0]) * np.cos(lon[0]), -np.sin(lat[0]) * np.sin(lon[0]), np.cos(lat[0])],
                  [np.cos(lat[0]) * np.cos(lon[0]), np.cos(lat[0]) * np.sin(lon[0]), np.sin(lat[0])]])
    return (xyz - xyz[0]) @ r.T


def rigid_alignment(source, target):
    sc, tc = source.mean(axis=0), target.mean(axis=0)
    u, singular, vt = np.linalg.svd((source - sc).T @ (target - tc))
    fix = np.eye(3)
    fix[-1, -1] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ fix @ u.T
    return rotation, tc - rotation @ sc, singular


def validate(bag, output):
    topics = {"/novatel/oem7/inspva", "/novatel/oem7/inspvax", "/vehicle/lidar/front_ouster/imu_packets"}
    rows, extended, imu = [], [], []
    with Reader(bag) as reader:
        if not any(c.topic == "/novatel/oem7/inspva" for c in reader.connections):
            return {"status": "no_external_navigation"}
        store = build_typestore(reader, topics)
        connections = [c for c in reader.connections if c.topic in topics]
        for c, arrival, data in reader.messages(connections=connections):
            message = store.deserialize_ros1(data, c.msgtype)
            if c.topic.endswith("inspva"):
                rows.append(inspva_row(message, arrival, len(rows) + 1))
            elif c.topic.endswith("inspvax"):
                extended.append(inspvax_row(message, arrival, len(extended) + 1))
            else:
                _, acc_ns, gyro_ns, *_ = decode_packet(bytes(message.buf))
                imu.append((arrival * 1e-9, (acc_ns + gyro_ns) * .5e-9))
    imu = np.asarray(imu)
    # Clock identification uses acquisition/arrival timing, never positions.
    nav_arrival = np.asarray([r["bag_time_sec"] for r in rows])
    gps = np.asarray([r["gps_week"] * 604800 + r["gps_seconds"] for r in rows])
    interpolated_device = np.interp(nav_arrival, imu[:, 0], imu[:, 1])
    coef, x0, y0, residual = robust_line(gps, interpolated_device)
    nav_times = (gps - x0) * coef[0] + coef[1] + y0
    xyz = enu(np.asarray([r["latitude_deg"] for r in rows]),
              np.asarray([r["longitude_deg"] for r in rows]),
              np.asarray([r["height_m"] for r in rows]))
    trajectory = np.loadtxt(output / "reference_lidar.tum", ndmin=2)
    valid = (trajectory[:, 0] >= nav_times[0]) & (trajectory[:, 0] <= nav_times[-1])
    trajectory = trajectory[valid]
    target = np.column_stack([np.interp(trajectory[:, 0], nav_times, xyz[:, k]) for k in range(3)])
    training = trajectory[:, 0] <= trajectory[0, 0] + .25 * np.ptp(trajectory[:, 0])
    rotation, translation, singular = rigid_alignment(trajectory[training, 1:4], target[training])
    aligned = trajectory[:, 1:4] @ rotation.T + translation
    errors = np.linalg.norm(aligned - target, axis=1)
    yaw = np.asarray([r["azimuth_deg"] for r in rows])
    nav_yaw = np.unwrap(np.deg2rad(90. - yaw))
    yaw_expected = np.interp(trajectory[:, 0], nav_times, nav_yaw)
    sensor_rotation = rotation @ Rotation.from_quat(trajectory[:, 4:]).as_matrix()
    observed_yaw = np.arctan2(sensor_rotation[:, 1, 0], sensor_rotation[:, 0, 0])
    delta = observed_yaw - yaw_expected
    mounting_offset = np.angle(np.mean(np.exp(1j * delta[training])))
    yaw_error = np.angle(np.exp(1j * (delta - mounting_offset)))
    diagnostics = []
    for i, pose in enumerate(trajectory):
        diagnostics.append(dict(native_time_sec=float(pose[0]), split="alignment" if training[i] else "withheld",
                                position_discrepancy_m=float(errors[i]), yaw_change_discrepancy_deg=float(np.rad2deg(yaw_error[i])),
                                aligned_east_m=float(aligned[i, 0]), aligned_north_m=float(aligned[i, 1]),
                                aligned_up_m=float(aligned[i, 2]), receiver_east_m=float(target[i, 0]),
                                receiver_north_m=float(target[i, 1]), receiver_up_m=float(target[i, 2])))
    with (output / "receiver_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(diagnostics[0]))
        writer.writeheader()
        writer.writerows(diagnostics)
    result = dict(status="diagnostic_completed", navigation_used_by_slam=False,
                  alignment="Single scale-fixed SE3 fit on first 25% of time; yaw mounting offset fitted only there",
                  training_samples=int(training.sum()), withheld_samples=int((~training).sum()),
                  withheld_position_discrepancy_rmse_m=float(np.sqrt(np.mean(errors[~training]**2))),
                  withheld_position_discrepancy_p95_m=float(np.percentile(errors[~training], 95)),
                  withheld_yaw_change_discrepancy_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(yaw_error[~training]**2)))),
                  clock_fit_native_seconds_per_gps_second=float(coef[0]),
                  clock_fit_rms_sec=float(np.sqrt(np.mean(residual**2))),
                  alignment_singular_values=singular.tolist(),
                  alignment_rotation_xyz_deg=Rotation.from_matrix(rotation).as_euler("xyz", degrees=True).tolist(),
                  alignment_training_rmse_m=float(np.sqrt(np.mean(errors[training]**2))),
                  clock_model=dict(coef=coef.tolist(), gps_origin_seconds=float(x0), native_origin_seconds=float(y0)),
                  ins_status_counts=dict(Counter(str(r["ins_status"]) for r in rows)),
                  position_type_counts=dict(Counter(str(r["position_type"]) for r in extended)),
                  median_reported_position_stdev_m={key: float(np.median([r[key] for r in extended]))
                                                     for key in ["latitude_stdev_m", "longitude_stdev_m", "height_stdev_m"]},
                  certified_ground_truth_error=False,
                  limitations=["LiDAR/INS lever arm not surveyed or compensated",
                               "Clock fit uses receipt-time association; relative physical latency unknown",
                               "Receiver navigation has its own error and is not surveyed truth",
                               "Constant orientation offset fitted on training segment; not absolute vehicle-yaw calibration"])
    all_rotation, all_translation, _ = rigid_alignment(trajectory[:, 1:4], target)
    all_errors = np.linalg.norm(trajectory[:, 1:4] @ all_rotation.T + all_translation - target, axis=1)
    result["all_sequence_alignment_diagnostic_rmse_m"] = float(np.sqrt(np.mean(all_errors**2)))
    result["all_sequence_alignment_is_not_withheld_validation"] = True
    (output / "receiver_comparison.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(validate(args.bag, args.output), indent=2))
