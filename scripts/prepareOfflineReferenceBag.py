#!/usr/bin/env python3
"""Prepare ROS2 input for independent GLIM reference estimation.

Use ``uv run --with rosbags --with numpy python scripts/prepareOfflineReferenceBag.py``.
Only front Ouster measurements enter the estimator. Native packet timestamps
recover scan starts exactly; recorded ROS/header timestamps are audit fields.
No GNSS, navigation pose, semantic mask, or project estimator is an input.
"""
import argparse
import csv
import hashlib
import heapq
import json
import struct
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from extractOusterImuFromBag import decode_packet

PREFIX = "/vehicle/lidar/front_ouster/"


def lidar_columns(data):
    """Decode legacy OS1-64 column headers, rejecting other packet layouts."""
    if len(data) not in (12608, 12609):
        raise ValueError(f"Unsupported Ouster packet size: {len(data)}")
    if len(data) == 12609 and data[-1] != 0:
        raise ValueError("Nonzero packet padding")
    return [struct.unpack_from("<QHH", data, offset)
            for offset in range(0, 12608, 788)]


def point_view(message, name):
    """Preserve row padding and endianness when reading PointCloud2 fields."""
    field = next(f for f in message.fields if f.name == name)
    codes = {2: "u1", 4: "u2", 6: "u4", 7: "f4", 8: "f8"}
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": [name], "formats": [endian + codes[field.datatype]],
                      "offsets": [field.offset], "itemsize": message.point_step})
    return np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=message.data,
                      strides=(message.row_step, message.point_step))[name]


def match_scan(relative_ns, frames):
    """Find the packet frame whose column times match the cloud exactly.

    The true start satisfies timestamp[column] - t[column] = constant.
    Even adjacent scans have different column timing jitter. Require an exact
    integer match, sufficient observed columns, and an unambiguous frame.
    """
    candidates = []
    for frame_id, columns in frames.items():
        starts = [stamp - int(relative_ns[col]) for col, stamp in columns.items()
                  if col < len(relative_ns) and (relative_ns[col] != 0 or col == 0)]
        if len(starts) >= 16 and max(starts) == min(starts):
            candidates.append((frame_id, starts[0], len(starts)))
    if len(candidates) != 1:
        raise ValueError(f"Ambiguous/missing native scan timing: {candidates}")
    return candidates[0]


def prepare(bag, output):
    output.mkdir(parents=True, exist_ok=False)
    store = get_typestore(Stores.ROS2_JAZZY)
    types = store.types
    time_type = types["builtin_interfaces/msg/Time"]
    header_type = types["std_msgs/msg/Header"]
    vector_type = types["geometry_msgs/msg/Vector3"]
    quat_type = types["geometry_msgs/msg/Quaternion"]
    imu_type = types["sensor_msgs/msg/Imu"]
    cloud_type = types["sensor_msgs/msg/PointCloud2"]
    field_type = types["sensor_msgs/msg/PointField"]

    def header(ns, frame):
        sec, nano = divmod(int(ns), 10**9)
        return header_type(time_type(sec, nano), frame)

    queue = []
    ordinal = 0
    latest = 0
    last_written = -1
    frames = {}
    rows = []
    imu_times = []
    imu_arrivals = []
    accel_norms = []
    rejected = []
    bag_hash = hashlib.sha256()
    with bag.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            bag_hash.update(block)
    with Reader(bag) as reader, Writer(output / "rosbag2", version=9) as writer:
        connections = [c for c in reader.connections if c.topic in
                       [PREFIX + name for name in ("points", "lidar_packets", "imu_packets")]]
        if len(connections) != 3:
            raise ValueError("Exactly one points, lidar_packets and imu_packets connection required")
        input_store = get_typestore(Stores.EMPTY)
        for c in connections:
            input_store.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        pc_connection = writer.add_connection("/reference/points", cloud_type.__msgtype__, typestore=store)
        imu_connection = writer.add_connection("/reference/imu", imu_type.__msgtype__, typestore=store)

        def emit(until):
            nonlocal last_written
            while queue and queue[0][0] <= until:
                stamp, _, connection, data = heapq.heappop(queue)
                if stamp < last_written:
                    raise ValueError("Native clock reset or excessive message reorder; split recording")
                writer.write(connection, stamp, data)
                last_written = stamp

        def enqueue(stamp, connection, message):
            nonlocal ordinal, latest
            ordinal += 1
            heapq.heappush(queue, (stamp, ordinal, connection,
                                  store.serialize_cdr(message, connection.msgtype)))
            latest = max(latest, stamp)
            emit(latest - 500_000_000)

        frame_index = 0
        for c, arrival, raw in reader.messages(connections=connections):
            message = input_store.deserialize_ros1(raw, c.msgtype)
            if c.topic.endswith("lidar_packets"):
                for stamp, col, frame_id in lidar_columns(bytes(message.buf)):
                    frames.setdefault(frame_id, {})[col] = stamp
                while len(frames) > 5:
                    del frames[next(iter(frames))]
            elif c.topic.endswith("imu_packets"):
                _, acc_ns, gyro_ns, *values = decode_packet(bytes(message.buf))
                stamp = (acc_ns + gyro_ns) // 2
                acceleration = np.asarray(values[:3]) * 9.80665
                angular = np.deg2rad(values[3:])
                imu = imu_type(header(stamp, "front_ouster_imu"), quat_type(0., 0., 0., 1.),
                               np.asarray([-1.] + [0.] * 8), vector_type(*angular), np.zeros(9),
                               vector_type(*acceleration), np.zeros(9))
                if imu_times and stamp <= imu_times[-1]:
                    raise ValueError("Nonmonotonic raw IMU timestamps")
                imu_times.append(stamp)
                imu_arrivals.append(arrival)
                accel_norms.append(float(np.linalg.norm(acceleration)))
                enqueue(stamp, imu_connection, imu)
            else:
                frame_index += 1
                if message.header.frame_id != "front_ouster" or message.width != 1024 or message.height != 64:
                    raise ValueError("Unverified point cloud layout/frame")
                relative = point_view(message, "t")[0]
                original_header_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                try:
                    frame_id, start_ns, matches = match_scan(relative, frames)
                except ValueError as error:
                    rejected.append({"frame_index": frame_index, "reason": str(error)})
                    continue
                if rows and start_ns <= rows[-1]["native_start_ns"]:
                    raise ValueError("Duplicate/nonmonotonic cloud timestamps")
                # A scan is delivered only after its final point is acquired.
                end_ns = start_ns + int(relative.max())
                pc = cloud_type(header(start_ns, "front_ouster"), message.height, message.width,
                                [field_type(f.name, f.offset, f.datatype, f.count) for f in message.fields],
                                message.is_bigendian, message.point_step, message.row_step,
                                message.data, message.is_dense)
                enqueue(end_ns, pc_connection, pc)
                rows.append(dict(frame_index=frame_index, ros_sequence=message.header.seq,
                                 packet_frame_id=frame_id, native_start_ns=start_ns,
                                 native_end_ns=end_ns, original_header_ns=original_header_ns,
                                 original_bag_ns=arrival, exact_timing_columns=matches))
                if frame_index % 100 == 0:
                    print(f"{bag.name}: prepared {frame_index} frames", flush=True)
        emit(float("inf"))
    if not rows or len(imu_times) < 2:
        raise ValueError("Insufficient synchronized sensor input")
    with (output / "frames.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    physical = (np.asarray(imu_times) - imu_times[0]) * 1e-9
    recorded = (np.asarray(imu_arrivals) - imu_arrivals[0]) * 1e-9
    scale, intercept = np.polyfit(recorded, physical, 1)
    intervals = np.diff(imu_times) * 1e-9
    summary = dict(schema_version=1, source_bag=str(bag.resolve()), source_bag_sha256=bag_hash.hexdigest(),
                   estimator_inputs=[PREFIX + "points", PREFIX + "lidar_packets", PREFIX + "imu_packets"],
                   navigation_or_project_estimator_used=False, time_basis="Ouster native device clock, seconds",
                   scan_time_method="Exact integer agreement of packet column timestamp minus cloud t; scan-start header",
                   frames_seen=frame_index, frames_prepared=len(rows), rejected_frames=rejected,
                   imu_records=len(imu_times), median_imu_rate_hz=float(1 / np.median(intervals)),
                   imu_max_gap_seconds=float(intervals.max()),
                   native_seconds_per_recorded_bag_second=float(scale),
                   arrival_clock_fit_rms_seconds=float(np.sqrt(np.mean((physical - (scale * recorded + intercept))**2))),
                   median_acceleration_norm_mps2=float(np.median(accel_norms)),
                   duration_seconds=(rows[-1]["native_end_ns"] - rows[0]["native_start_ns"]) * 1e-9,
                   output_pose_frame="front_ouster; vehicle reference-point translation not surveyed",
                   imu_axes="Native Ouster sensor XYZ, parallel to point sensor XYZ; no project mounting rotation applied")
    (output / "preparation.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.bag, args.output)
