#!/usr/bin/env python3
"""Export the front Ouster's raw six-axis IMU, never receiver navigation data.

Run with uv run --with rosbags python scripts/extractOusterImuFromBag.py.
Packet arrival time determines availability; native device time determines
integration intervals. The driver's optional trailing zero byte is discarded.
"""
import argparse
import csv
import hashlib
import json
import math
import struct
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


def decode_packet(data):
    """Decode documented 48-byte legacy packets with optional zero padding."""
    if len(data) not in (48, 49) or (len(data) == 49 and data[-1] != 0):
        raise ValueError("Expected a 48-byte Ouster IMU packet or one trailing zero")
    values = struct.unpack("<QQQffffff", data[:48])
    if not all(math.isfinite(x) for x in values[3:]):
        raise ValueError("Nonfinite Ouster IMU measurement")
    return values


def multiply(a, b):
    return [[sum(x*y for x, y in zip(row, column)) for column in zip(*b)] for row in a]


def export(bag, output):
    root = Path(__file__).resolve().parents[1]
    mount_path = root / "config/mississippiLidarMount.json"
    mount = json.loads(mount_path.read_text())
    rotation = multiply(multiply(mount["additionalRotation"], mount["baseRotation"]), mount["imuToPointRotation"])
    topic = "/vehicle/lidar/front_ouster/imu_packets"
    rows = []
    point_frame = None
    with Reader(bag) as reader:
        connections = [c for c in reader.connections if c.topic in (topic, "/vehicle/lidar/front_ouster/points")]
        assert len(connections) == 2, "Front LiDAR points and raw IMU are required"
        store = get_typestore(Stores.EMPTY)
        for connection in connections:
            store.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        for connection, timestamp, data in reader.messages(connections=connections):
            if connection.topic != topic:
                if point_frame is None:
                    point_frame = store.deserialize_ros1(data, connection.msgtype).header.frame_id
                    assert point_frame == mount["pointFrame"], "Unverified point/IMU coordinate frame"
                continue
            message = store.deserialize_ros1(data, connection.msgtype)
            diagnostic, acc_time, gyro_time, *values = decode_packet(bytes(message.buf))
            acceleration = [sum(a*b for a, b in zip(row, values[:3]))*9.80665 for row in rotation]
            angular = [sum(a*b for a, b in zip(row, values[3:]))*math.pi/180 for row in rotation]
            rows.append([timestamp*1e-9, (acc_time+gyro_time)*.5e-9, acc_time, gyro_time, *acceleration, *angular])
    assert rows and point_frame is not None
    assert all(b[0] > a[0] and b[1] > a[1] for a, b in zip(rows, rows[1:])), "Nonmonotonic IMU clock"
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["arrivalTime", "deviceTime", "accelerometerTimeNs", "gyroscopeTimeNs", "specificX", "specificY", "specificZ", "gyroX", "gyroY", "gyroZ"])
        writer.writerows(rows)
    metadata = dict(schemaVersion=1, source="front-ouster-raw-imu", topic=topic, bag=str(bag),
                    records=len(rows), pointFrame=point_frame, referencePoseUsed=False,
                    units="seconds, m/s^2, rad/s", coordinates="stored_front_lidar_axes",
                    mountSha256=hashlib.sha256(mount_path.read_bytes()).hexdigest(),
                    csvSha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                    availability="ROS bag packet arrival; no future packets; physical integration uses native device time",
                    axesLimitation="Uses documented parallel Ouster IMU/sensor axes; per-unit accelerometer scale and bias are not surveyed")
    output.with_suffix(".json").write_text(json.dumps(metadata, indent=2)+"\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=Path("data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag"))
    parser.add_argument("--output", type=Path, default=Path("output/mississippi_lidar_imu/front_imu.csv"))
    args = parser.parse_args()
    export(args.bag, args.output)
