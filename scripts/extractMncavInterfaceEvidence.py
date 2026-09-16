#!/usr/bin/env python3
"""Read recorded receiver configuration and auxiliary vehicle channels.

uv run --offline --with rosbags --with numpy python scripts/extractMncavInterfaceEvidence.py
Raw bags are read-only; no external data is used as a calibration substitute.
"""
import csv
from dataclasses import fields, is_dataclass
import json
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/mncav_interface_audit_20260916"
TOPICS = ["/novatel/oem7/insconfig", "/novatel/oem7/heading2", "/novatel/oem7/insstdev",
          "/vehicle/steering_curvature", "/vehicle/wheel_speed_report", "/vehicle/ulc_report",
          "/vehicle/offset_angle_error", "/vehicle/pre_offset_angle_error", "/vehicle/pre_pre_offset_angle_error"]


def convert(value):
    if is_dataclass(value):
        return {f.name: convert(getattr(value, f.name)) for f in fields(value) if f.name != "__msgtype__"}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return [convert(x) for x in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def flatten(value, prefix=""):
    result = {}
    for key, item in value.items():
        name = prefix + key
        if isinstance(item, dict):
            result.update(flatten(item, name+"_"))
        elif not isinstance(item, list):
            result[name] = item
    return result


def main():
    for stamp in ["12-09-31", "12-11-24"]:
        bag = ROOT / f"data/raw/Missisipi/raw_data_2024-06-07-{stamp}_0.bag"
        dest = OUT / stamp
        dest.mkdir(parents=True, exist_ok=True)
        rows = {t: [] for t in TOPICS}
        with Reader(bag) as reader:
            selected = [c for c in reader.connections if c.topic in TOPICS]
            store = get_typestore(Stores.EMPTY)
            types = {}
            for c in selected:
                types.update(get_types_from_msg(c.msgdef.data, c.msgtype))
            store.register(types)
            inventory = [{"topic": c.topic, "type": c.msgtype, "count": c.msgcount} for c in reader.connections]
            for c, time, raw in reader.messages(connections=selected):
                msg = convert(store.deserialize_ros1(raw, c.msgtype))
                msg["bag_time_sec"] = time*1e-9
                if "header" in msg:
                    s = msg["header"]["stamp"]
                    msg["stamp_sec"] = s["sec"]+s["nanosec"]*1e-9
                rows[c.topic].append(msg)
        for topic, records in rows.items():
            if not records:
                continue
            name = topic.rsplit("/", 1)[1]
            if name == "insconfig":
                (dest / (name+".json")).write_text(json.dumps(records, indent=2)+"\n")
                print(stamp, name, json.dumps(records, indent=2))
            else:
                values = [flatten(r) for r in records]
                with (dest / (name+".csv")).open("w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=list(values[0]))
                    writer.writeheader()
                    writer.writerows(values)
        (dest / "manifest.json").write_text(json.dumps({"bag": str(bag.relative_to(ROOT)), "inventory": inventory,
            "exported": {k: len(v) for k, v in rows.items()}, "role": "Read-only recorded interface evidence"}, indent=2)+"\n")
        print(stamp, {k: len(v) for k, v in rows.items()})


if __name__ == "__main__":
    main()
