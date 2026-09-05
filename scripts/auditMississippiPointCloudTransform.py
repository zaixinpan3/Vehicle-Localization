#!/usr/bin/env python3
"""Export one original ROS cloud for a read-only MAT transform audit.

Run with ``uv run --with rosbags python scripts/auditMississippiPointCloudTransform.py``.
No source bag or MAT data is modified. XYZ uses original ROS row-major order.
"""
import argparse
import csv
import json
from importlib.metadata import version
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", type=int, default=260)
    parser.add_argument("--output", type=Path, default=Path("output/height_bias_diagnosis"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    stem = "raw_data_2024-06-07-12-09-31_0"
    raw = root / "data/raw/Missisipi"
    with (raw / f"gnss/{stem}_front_lidar_points.csv").open() as handle:
        row = next(r for r in csv.DictReader(handle) if int(r["frame_index"]) == args.frame)
    stamp = float(row["stamp_sec"])
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(raw / f"{stem}.bag") as reader:
        connections = [c for c in reader.connections if c.topic == "/vehicle/lidar/front_ouster/points"]
        candidates = []
        for connection, _, data in reader.messages(connections=connections,
                                                  start=int((stamp - 1) * 1e9),
                                                  stop=int((stamp + 1) * 1e9)):
            message = store.deserialize_ros1(data, connection.msgtype)
            message_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            candidates.append((abs(message_stamp - stamp), message_stamp, message))
        error, message_stamp, message = min(candidates, key=lambda item: item[0])
        assert error < 1e-5, "No matching original message."
        offsets = {field.name: field.offset for field in message.fields}
        assert all(field.datatype == 7 and field.count == 1
                   for field in message.fields if field.name in ["x", "y", "z"])
        dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                          "offsets": [offsets[n] for n in ["x", "y", "z"]],
                          "itemsize": message.point_step})
        assert not message.is_bigendian and message.row_step == message.width * message.point_step
        points = np.frombuffer(message.data, dtype=dtype)
        xyz = np.column_stack([points[n] for n in ["x", "y", "z"]]).astype("<f8")
        args.output.mkdir(parents=True, exist_ok=True)
        xyz.tofile(args.output / "original_ros_xyz.bin")
        metadata = {"frame": args.frame, "stamp": message_stamp, "timestamp_error_seconds": error,
                    "frame_id": message.header.frame_id, "height": message.height, "width": message.width,
                    "xyz_binary_format": "little-endian float64, N by 3, C row-major",
                    "rosbags_version": version("rosbags"), "numpy_version": version("numpy"),
                    "bag_has_tf": any(c.topic in ["/tf", "/tf_static"] for c in reader.connections)}
        (args.output / "original_ros_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
        print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
