#!/usr/bin/env python3
"""Extract NovAtel GNSS/INS and LiDAR timing streams from a Mississippi ROS1 bag."""

from __future__ import annotations

import argparse
import csv
import json
from bisect import bisect_left
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


GNSS_TOPICS = {
    "bestpos": "/novatel/oem7/bestpos",
    "bestvel": "/novatel/oem7/bestvel",
    "inspva": "/novatel/oem7/inspva",
    "inspvax": "/novatel/oem7/inspvax",
    "odom": "/novatel/oem7/odom",
    "front_lidar_points": "/vehicle/lidar/front_ouster/points",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract GNSS/INS pose streams and front LiDAR frame timestamps from a ROS1 bag."
    )
    parser.add_argument(
        "--bag",
        type=Path,
        default=Path("data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag"),
        help="Path to the input ROS1 bag.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("data/raw/Missisipi/gnss"),
        help="Directory where CSV and manifest files will be written.",
    )
    parser.add_argument(
        "--frame-start",
        type=int,
        default=260,
        help="One-based LiDAR frame index used for the matched-pose preview export.",
    )
    parser.add_argument(
        "--frame-count",
        type=int,
        default=10,
        help="Number of LiDAR frames used for the matched-pose preview export.",
    )
    return parser.parse_args()


def stamp_to_sec(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def header_stamp_to_sec(message: object) -> float:
    return stamp_to_sec(message.header.stamp)


def bag_time_to_sec(timestamp_ns: int) -> float:
    return float(timestamp_ns) * 1.0e-9


def gps_milliseconds_to_sec(message: object) -> float:
    return float(message.nov_header.gps_week_milliseconds) * 1.0e-3


def status_value(field: object, attribute_name: str = "status") -> int:
    return int(getattr(field, attribute_name))


def build_typestore(reader: Reader, topics: set[str]):
    typestore = get_typestore(Stores.EMPTY)
    msg_types = {}
    for connection in reader.connections:
        if connection.topic in topics:
            msg_types.update(get_types_from_msg(connection.msgdef.data, connection.msgtype))
    typestore.register(msg_types)
    return typestore


def topic_connections(reader: Reader, topic: str):
    return [connection for connection in reader.connections if connection.topic == topic]


def read_topic_rows(reader: Reader, typestore, topic: str, row_builder):
    connections = topic_connections(reader, topic)
    rows = []
    for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
        message = typestore.deserialize_ros1(rawdata, connection.msgtype)
        rows.append(row_builder(message, timestamp_ns, len(rows) + 1))
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys()) if rows else ["empty"]
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def bestpos_row(message: object, timestamp_ns: int, row_index: int) -> dict:
    return {
        "index": row_index,
        "bag_time_sec": bag_time_to_sec(timestamp_ns),
        "stamp_sec": header_stamp_to_sec(message),
        "gps_week": int(message.nov_header.gps_week_number),
        "gps_seconds": gps_milliseconds_to_sec(message),
        "time_status": int(message.nov_header.time_status),
        "solution_status": status_value(message.sol_status),
        "position_type": int(message.pos_type.type),
        "latitude_deg": float(message.lat),
        "longitude_deg": float(message.lon),
        "ellipsoid_height_m": float(message.hgt),
        "undulation_m": float(message.undulation),
        "orthometric_height_m": float(message.hgt - message.undulation),
        "latitude_stdev_m": float(message.lat_stdev),
        "longitude_stdev_m": float(message.lon_stdev),
        "height_stdev_m": float(message.hgt_stdev),
        "diff_age_sec": float(message.diff_age),
        "solution_age_sec": float(message.sol_age),
        "num_satellites": int(message.num_svs),
        "num_solution_satellites": int(message.num_sol_svs),
    }


def bestvel_row(message: object, timestamp_ns: int, row_index: int) -> dict:
    return {
        "index": row_index,
        "bag_time_sec": bag_time_to_sec(timestamp_ns),
        "stamp_sec": header_stamp_to_sec(message),
        "gps_week": int(message.nov_header.gps_week_number),
        "gps_seconds": gps_milliseconds_to_sec(message),
        "time_status": int(message.nov_header.time_status),
        "solution_status": status_value(message.sol_status),
        "velocity_type": int(message.vel_type.type),
        "latency_sec": float(message.latency),
        "diff_age_sec": float(message.diff_age),
        "horizontal_speed_mps": float(message.hor_speed),
        "track_ground_deg": float(message.trk_gnd),
        "vertical_speed_mps": float(message.ver_speed),
    }


def inspva_row(message: object, timestamp_ns: int, row_index: int) -> dict:
    return {
        "index": row_index,
        "bag_time_sec": bag_time_to_sec(timestamp_ns),
        "stamp_sec": header_stamp_to_sec(message),
        "gps_week": int(message.nov_header.gps_week_number),
        "gps_seconds": gps_milliseconds_to_sec(message),
        "time_status": int(message.nov_header.time_status),
        "ins_status": status_value(message.status),
        "latitude_deg": float(message.latitude),
        "longitude_deg": float(message.longitude),
        "height_m": float(message.height),
        "north_velocity_mps": float(message.north_velocity),
        "east_velocity_mps": float(message.east_velocity),
        "up_velocity_mps": float(message.up_velocity),
        "roll_deg": float(message.roll),
        "pitch_deg": float(message.pitch),
        "azimuth_deg": float(message.azimuth),
    }


def inspvax_row(message: object, timestamp_ns: int, row_index: int) -> dict:
    return {
        "index": row_index,
        "bag_time_sec": bag_time_to_sec(timestamp_ns),
        "stamp_sec": header_stamp_to_sec(message),
        "gps_week": int(message.nov_header.gps_week_number),
        "gps_seconds": gps_milliseconds_to_sec(message),
        "time_status": int(message.nov_header.time_status),
        "ins_status": status_value(message.ins_status),
        "position_type": int(message.pos_type.type),
        "latitude_deg": float(message.latitude),
        "longitude_deg": float(message.longitude),
        "height_m": float(message.height),
        "undulation_m": float(message.undulation),
        "orthometric_height_m": float(message.height - message.undulation),
        "north_velocity_mps": float(message.north_velocity),
        "east_velocity_mps": float(message.east_velocity),
        "up_velocity_mps": float(message.up_velocity),
        "roll_deg": float(message.roll),
        "pitch_deg": float(message.pitch),
        "azimuth_deg": float(message.azimuth),
        "latitude_stdev_m": float(message.latitude_stdev),
        "longitude_stdev_m": float(message.longitude_stdev),
        "height_stdev_m": float(message.height_stdev),
        "north_velocity_stdev_mps": float(message.north_velocity_stdev),
        "east_velocity_stdev_mps": float(message.east_velocity_stdev),
        "up_velocity_stdev_mps": float(message.up_velocity_stdev),
        "roll_stdev_deg": float(message.roll_stdev),
        "pitch_stdev_deg": float(message.pitch_stdev),
        "azimuth_stdev_deg": float(message.azimuth_stdev),
        "time_since_update": int(message.time_since_update),
    }


def odom_row(message: object, timestamp_ns: int, row_index: int) -> dict:
    pose = message.pose.pose
    twist = message.twist.twist
    pose_cov = message.pose.covariance
    twist_cov = message.twist.covariance
    return {
        "index": row_index,
        "bag_time_sec": bag_time_to_sec(timestamp_ns),
        "stamp_sec": header_stamp_to_sec(message),
        "frame_id": message.header.frame_id,
        "child_frame_id": message.child_frame_id,
        "x_m": float(pose.position.x),
        "y_m": float(pose.position.y),
        "z_m": float(pose.position.z),
        "qx": float(pose.orientation.x),
        "qy": float(pose.orientation.y),
        "qz": float(pose.orientation.z),
        "qw": float(pose.orientation.w),
        "linear_x_mps": float(twist.linear.x),
        "linear_y_mps": float(twist.linear.y),
        "linear_z_mps": float(twist.linear.z),
        "angular_x_radps": float(twist.angular.x),
        "angular_y_radps": float(twist.angular.y),
        "angular_z_radps": float(twist.angular.z),
        "pose_cov_xx": float(pose_cov[0]),
        "pose_cov_yy": float(pose_cov[7]),
        "pose_cov_zz": float(pose_cov[14]),
        "pose_cov_rr": float(pose_cov[21]),
        "pose_cov_pp": float(pose_cov[28]),
        "pose_cov_yyaw": float(pose_cov[35]),
        "twist_cov_xx": float(twist_cov[0]),
        "twist_cov_yy": float(twist_cov[7]),
        "twist_cov_zz": float(twist_cov[14]),
    }


def lidar_row(message: object, timestamp_ns: int, row_index: int) -> dict:
    return {
        "frame_index": row_index,
        "bag_time_sec": bag_time_to_sec(timestamp_ns),
        "stamp_sec": header_stamp_to_sec(message),
        "frame_id": message.header.frame_id,
        "height": int(message.height),
        "width": int(message.width),
    }


def nearest_row(rows: list[dict], stamps: list[float], target_stamp: float) -> tuple[dict, float]:
    insert_idx = bisect_left(stamps, target_stamp)
    candidate_indices = [max(0, insert_idx - 1), min(len(rows) - 1, insert_idx)]
    nearest_idx = min(candidate_indices, key=lambda idx: abs(stamps[idx] - target_stamp))
    return rows[nearest_idx], rows[nearest_idx]["stamp_sec"] - target_stamp


def build_lidar_pose_matches(
    lidar_rows: list[dict],
    odom_rows: list[dict],
    inspva_rows: list[dict],
    frame_start: int,
    frame_count: int,
) -> list[dict]:
    odom_stamps = [row["stamp_sec"] for row in odom_rows]
    inspva_stamps = [row["stamp_sec"] for row in inspva_rows]
    frame_end = frame_start + frame_count - 1
    selected_lidar_rows = [
        row for row in lidar_rows if frame_start <= int(row["frame_index"]) <= frame_end
    ]
    matches = []
    for lidar in selected_lidar_rows:
        odom, odom_dt = nearest_row(odom_rows, odom_stamps, lidar["stamp_sec"])
        inspva, inspva_dt = nearest_row(inspva_rows, inspva_stamps, lidar["stamp_sec"])
        matches.append(
            {
                "frame_index": int(lidar["frame_index"]),
                "lidar_stamp_sec": float(lidar["stamp_sec"]),
                "nearest_odom_index": int(odom["index"]),
                "odom_stamp_sec": float(odom["stamp_sec"]),
                "odom_dt_sec": float(odom_dt),
                "odom_x_m": float(odom["x_m"]),
                "odom_y_m": float(odom["y_m"]),
                "odom_z_m": float(odom["z_m"]),
                "odom_qx": float(odom["qx"]),
                "odom_qy": float(odom["qy"]),
                "odom_qz": float(odom["qz"]),
                "odom_qw": float(odom["qw"]),
                "nearest_inspva_index": int(inspva["index"]),
                "inspva_stamp_sec": float(inspva["stamp_sec"]),
                "inspva_dt_sec": float(inspva_dt),
                "latitude_deg": float(inspva["latitude_deg"]),
                "longitude_deg": float(inspva["longitude_deg"]),
                "height_m": float(inspva["height_m"]),
                "roll_deg": float(inspva["roll_deg"]),
                "pitch_deg": float(inspva["pitch_deg"]),
                "azimuth_deg": float(inspva["azimuth_deg"]),
                "ins_status": int(inspva["ins_status"]),
            }
        )
    return matches


def summarize_topic(reader: Reader, topic: str) -> dict:
    connections = topic_connections(reader, topic)
    return {
        "topic": topic,
        "message_type": connections[0].msgtype,
        "message_count": int(sum(connection.msgcount for connection in connections)),
    }


def main() -> None:
    args = parse_args()
    bag_path = args.bag.resolve()
    output_dir = args.output_dir.resolve()
    assert bag_path.is_file(), f"Bag file not found: {bag_path}"
    output_dir.mkdir(parents=True, exist_ok=True)

    with Reader(bag_path) as reader:
        topics = set(GNSS_TOPICS.values())
        typestore = build_typestore(reader, topics)
        rows_by_name = {
            "bestpos": read_topic_rows(reader, typestore, GNSS_TOPICS["bestpos"], bestpos_row),
            "bestvel": read_topic_rows(reader, typestore, GNSS_TOPICS["bestvel"], bestvel_row),
            "inspva": read_topic_rows(reader, typestore, GNSS_TOPICS["inspva"], inspva_row),
            "inspvax": read_topic_rows(reader, typestore, GNSS_TOPICS["inspvax"], inspvax_row),
            "odom": read_topic_rows(reader, typestore, GNSS_TOPICS["odom"], odom_row),
            "front_lidar_points": read_topic_rows(reader, typestore, GNSS_TOPICS["front_lidar_points"], lidar_row),
        }
        matches = build_lidar_pose_matches(
            rows_by_name["front_lidar_points"],
            rows_by_name["odom"],
            rows_by_name["inspva"],
            args.frame_start,
            args.frame_count,
        )
        rows_by_name[f"front_lidar_pose_match_{args.frame_start}_{args.frame_start + args.frame_count - 1}"] = matches
        output_paths = {}
        for name, rows in rows_by_name.items():
            output_path = output_dir / f"{bag_path.stem}_{name}.csv"
            write_csv(output_path, rows)
            output_paths[name] = str(output_path)

        manifest = {
            "bag": str(bag_path),
            "output_dir": str(output_dir),
            "topics": {name: summarize_topic(reader, topic) for name, topic in GNSS_TOPICS.items()},
            "frame_match": {
                "frame_start": args.frame_start,
                "frame_count": args.frame_count,
                "frame_end": args.frame_start + args.frame_count - 1,
                "matched_rows": len(matches),
            },
            "outputs": output_paths,
        }
        manifest_path = output_dir / f"{bag_path.stem}_gnss_manifest.json"
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
