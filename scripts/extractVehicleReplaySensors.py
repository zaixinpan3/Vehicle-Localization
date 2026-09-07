#!/usr/bin/env python3
"""Export recorded vehicle motion inputs for a localization replay.

Run with ``uv run --with rosbags python scripts/extractVehicleReplaySensors.py``.
Original bags and GNSS exports are read-only. No GNSS position is substituted
for vehicle speed or yaw rate. Values retain their recorded units and frames.
"""
from __future__ import annotations

import argparse
import csv
import json
from importlib.metadata import version
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


TOPICS = {
    "/vehicle/twist": "twist",
    "/vehicle/imu/data_raw": "imu",
    "/vehicle/steering_report": "steering",
    "/novatel/oem7/corrimu": "corrimu",
    "/novatel/oem7/inspva": "inspva",
}


def sensor_row(message, timestamp, kind):
    row = {
        "stamp_sec": message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
        "bag_time_sec": timestamp * 1e-9,
        "frame_id": message.header.frame_id,
    }
    if kind == "twist":
        for axis in "xyz":
            row[f"linear_{axis}_mps"] = getattr(message.twist.linear, axis)
            row[f"angular_{axis}_radps"] = getattr(message.twist.angular, axis)
    elif kind == "imu":
        for axis in "xyz":
            row[f"angular_{axis}_radps"] = getattr(message.angular_velocity, axis)
            row[f"acceleration_{axis}_mps2"] = getattr(message.linear_acceleration, axis)
    elif kind == "steering":
        row.update(speed_mps=message.speed, steering_wheel_angle_rad=message.steering_wheel_angle,
                   enabled=message.enabled, override=message.override,
                   calibration_fault=message.fault_calibration)
    elif kind == "inspva":
        row.update(gps_week=message.nov_header.gps_week_number,
                   gps_seconds=message.nov_header.gps_week_milliseconds * 1e-3)
        for field in ["latitude", "longitude", "height", "north_velocity",
                      "east_velocity", "up_velocity", "roll", "pitch", "azimuth"]:
            row[field] = getattr(message, field)
    else:
        row.update(gps_week=message.nov_header.gps_week_number,
                   gps_seconds=message.nov_header.gps_week_milliseconds * 1e-3,
                   imu_data_count=message.imu_data_count)
        for field in ["pitch_rate", "roll_rate", "yaw_rate", "lateral_acc",
                      "longitudinal_acc", "vertical_acc"]:
            row[field] = getattr(message, field)
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path,
                        default=Path("data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag"))
    parser.add_argument("--output-dir", type=Path,
                        default=Path("output/mississippi_20240607_120931_20260907/sensors"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = {kind: [] for kind in TOPICS.values()}
    with Reader(args.bag) as reader:
        selected = [c for c in reader.connections if c.topic in TOPICS]
        assert {c.topic for c in selected} == set(TOPICS), "Required sensor topic missing."
        store = get_typestore(Stores.EMPTY)
        types = {}
        for connection in selected:
            types.update(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        store.register(types)
        metadata = {"bag": str(args.bag), "bag_start_seconds": reader.start_time * 1e-9,
                    "bag_end_seconds": reader.end_time * 1e-9,
                    "rosbags_version": version("rosbags"),
                    "topics": [{"topic": c.topic, "type": c.msgtype, "count": c.msgcount}
                               for c in reader.connections]}
        for connection, timestamp, data in reader.messages(connections=selected):
            message = store.deserialize_ros1(data, connection.msgtype)
            kind = TOPICS[connection.topic]
            rows[kind].append(sensor_row(message, timestamp, kind))
    metadata["exports"] = {}
    for kind, records in rows.items():
        assert records, f"Empty sensor stream: {kind}"
        path = args.output_dir / f"{kind}.csv"
        with path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)
        metadata["exports"][kind] = {"file": path.name, "rows": len(records),
                                     "frame_ids": sorted({r["frame_id"] for r in records})}
    (args.output_dir / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata["exports"], indent=2))


if __name__ == "__main__":
    main()
