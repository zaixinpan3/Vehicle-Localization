"""Independent NovAtel/Ouster navigation ingestion for offline references.

Attitude follows NovAtel APN-037: body X right, Y forward, Z up;
body-to-ENU is Rz(-azimuth) Rx(pitch) Ry(roll). The receiver's configured
output body and position reference point remain unsurveyed.
"""
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from scipy.spatial.transform import Rotation

from extractGnssFromBag import build_typestore, inspva_row, inspvax_row
from extractOusterImuFromBag import decode_packet
from validateOfflineReference import enu, robust_line


def novatel_rotation(roll, pitch, azimuth):
    """Return body-to-ENU rotations from degree-valued INSPVA angles."""
    return Rotation.from_euler("ZXY", np.column_stack([-np.asarray(azimuth), pitch, roll]), degrees=True)


def read_navigation(bag):
    topics = {"/novatel/oem7/inspva", "/novatel/oem7/inspvax", "/vehicle/lidar/front_ouster/imu_packets"}
    rows, extended, imu = [], [], []
    with Reader(Path(bag)) as reader:
        if not any(c.topic.endswith("/inspva") for c in reader.connections):
            raise ValueError("This recording has no external navigation")
        store = build_typestore(reader, topics)
        for c, arrival, data in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            m = store.deserialize_ros1(data, c.msgtype)
            if c.topic.endswith("/inspva"):
                rows.append(inspva_row(m, arrival, len(rows) + 1))
            elif c.topic.endswith("/inspvax"):
                extended.append(inspvax_row(m, arrival, len(extended) + 1))
            else:
                _, acc_ns, gyro_ns, ax, ay, az, gx, gy, gz = decode_packet(bytes(m.buf))
                imu.append((arrival * 1e-9, (acc_ns + gyro_ns) * .5e-9,
                            *np.deg2rad([gx, gy, gz]), * (np.asarray([ax, ay, az]) * 9.80665)))
    if not extended or len(imu) < 100:
        raise ValueError("Navigation standard deviations and native IMU timing are required")
    gps = np.asarray([r["gps_week"] * 604800 + r["gps_seconds"] for r in rows])
    order = np.argsort(gps, kind="stable")
    order = order[np.r_[True, np.diff(gps[order]) > 0]]
    rows, gps = [rows[i] for i in order], gps[order]
    imu = np.asarray(imu)
    imu = imu[np.argsort(imu[:, 0], kind="stable")]
    arrival = np.asarray([r["bag_time_sec"] for r in rows])
    coef, x0, y0, residual = robust_line(gps, np.interp(arrival, imu[:, 0], imu[:, 1]))
    times = (gps - x0) * coef[0] + coef[1] + y0
    extended.sort(key=lambda r: (r["gps_week"], r["gps_seconds"]))
    ext_gps = np.asarray([r["gps_week"] * 604800 + r["gps_seconds"] for r in extended])
    # Conservative floors avoid treating reported formal precision as accuracy.
    sigma = np.column_stack([np.interp(gps, ext_gps, [r[k] for r in extended]) for k in
                             ["longitude_stdev_m", "latitude_stdev_m", "height_stdev_m"]])
    attitude_sigma = np.deg2rad(np.column_stack([np.interp(gps, ext_gps, [r[k] for r in extended]) for k in
                                               ["pitch_stdev_deg", "roll_stdev_deg", "azimuth_stdev_deg"]]))
    rotation = novatel_rotation([r["roll_deg"] for r in rows], [r["pitch_deg"] for r in rows],
                               [r["azimuth_deg"] for r in rows])
    return dict(times=times, gps=gps, xyz=enu(np.array([r["latitude_deg"] for r in rows]),
                np.array([r["longitude_deg"] for r in rows]), np.array([r["height_m"] for r in rows])),
                quaternion=rotation.as_quat(), sigma=np.maximum(sigma, .1),
                attitude_sigma=np.maximum(attitude_sigma, np.deg2rad(.1)),
                velocity=np.array([[r[k] for k in ["east_velocity_mps", "north_velocity_mps", "up_velocity_mps"]] for r in rows]),
                good=np.array([r["ins_status"] == 3 for r in rows]),
                origin=np.array([rows[0][k] for k in ["latitude_deg", "longitude_deg", "height_m"]]),
                gyro_times=imu[:, 1], gyro=imu[:, 2:5], acceleration=imu[:, 5:8],
                clock=dict(coef=coef.tolist(), gps_origin_seconds=float(x0), native_origin_seconds=float(y0),
                           receipt_association_rms_seconds=float(np.sqrt(np.mean(residual**2)))))
