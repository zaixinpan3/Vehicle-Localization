"""Audit reference quality and BESTGNSSPOS availability in the recorded drive.

Run from the repository root:
uv run --offline --with numpy --with pyproj --with rosbags python \
    research/mncav_reference_roles_20260915/audit.py
"""
from collections import Counter
from pathlib import Path
import csv
import hashlib
import importlib.util
import json
import statistics

from rosbags.rosbag1 import Reader

ROOT = Path(__file__).resolve().parents[2]
STEM = "raw_data_2024-06-07-12-09-31_0"
BAG = ROOT / "data/raw/Missisipi" / (STEM + ".bag")


def main():
    result = {"bag": str(BAG.relative_to(ROOT)), "bag_bytes": BAG.stat().st_size,
              "csv_quality": {}}
    for name in ("inspva", "inspvax", "bestpos"):
        path = BAG.parent / "gnss" / (STEM + "_" + name + ".csv")
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        item = {"path": str(path.relative_to(ROOT)), "rows": len(rows),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        for field in ("ins_status", "position_type", "solution_status"):
            if field in rows[0]:
                item[field] = dict(sorted(Counter(row[field] for row in rows).items()))
                assert sum(item[field].values()) == len(rows)
        for field in ("latitude_stdev_m", "longitude_stdev_m", "height_stdev_m"):
            if field in rows[0]:
                values = [float(row[field]) for row in rows]
                item[field] = {"min": min(values), "median": statistics.median(values),
                               "max": max(values)}
        result["csv_quality"][name] = item

    spec = importlib.util.spec_from_file_location("extract", ROOT / "scripts/extractGnssFromBag.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ids, syncs = Counter(), Counter()
    topic = "/novatel/oem7/oem7raw"
    with Reader(BAG) as reader:
        # Preserve all connections, including repeated publisher connections.
        result["connections"] = [{"topic": c.topic, "type": c.msgtype, "messages": c.msgcount}
                                 for c in reader.connections]
        connections = [c for c in reader.connections if c.topic == topic]
        store = module.build_typestore(reader, {topic})
        for connection, _, raw in reader.messages(connections=connections):
            data = bytes(store.deserialize_ros1(raw, connection.msgtype).message_data)
            assert len(data) >= 6 and data[:3] in (b"\xaa\x44\x12", b"\xaa\x44\x13")
            # Both OEM7 binary header forms use a little-endian Ushort at offset 4.
            ids[int.from_bytes(data[4:6], "little")] += 1
            syncs[data[:3].hex()] += 1
        assert sum(ids.values()) == sum(c.msgcount for c in connections)
    result["raw_message_ids"] = dict(sorted(ids.items()))
    result["raw_header_forms"] = dict(sorted(syncs.items()))
    result["bestgnsspos"] = {
        "named_topic_connections": sum("bestgnsspos" in c["topic"].lower()
                                       for c in result["connections"]),
        "binary_message_id": 1429, "raw_messages": ids[1429],
        "scope": "Topic inventory and OEM7 header IDs only; no payload/CRC validation or PPK reconstruction."
    }
    output = Path(__file__).with_name("audit.json")
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"csv_rows": {k: v["rows"] for k, v in result["csv_quality"].items()},
                      "raw_messages": sum(ids.values()), "bestgnsspos": result["bestgnsspos"]}, indent=2))


if __name__ == "__main__":
    main()
