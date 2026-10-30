"""Export raw FAST-LIO inputs on the shared Ouster hardware clock.

LiDAR frame origins are recovered from raw column timestamps and the t field.
No GNSS pose, reference velocity or reconstructed orientation is read.
Generated binary streams are temporary and must be deleted after scoring.
"""

import argparse
import csv
import json
import struct
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from extractOusterImuFromBag import decode_packet

POINT = "/vehicle/lidar/front_ouster/points"
LIDAR = "/vehicle/lidar/front_ouster/lidar_packets"
IMU = "/vehicle/lidar/front_ouster/imu_packets"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    imu = []
    records = []
    groups = {}
    order = []
    residuals = []
    types = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    with Reader(a.bag) as reader:
        cs = [c for c in reader.connections if c.topic in [POINT, LIDAR, IMU]]
        assert len(cs) == 3
        store = get_typestore(Stores.EMPTY)
        for c in cs:
            store.register(get_types_from_msg(c.msgdef.data, c.msgtype))
        # Packet-topic delivery may lag cloud delivery. First recover clock metadata;
        # those columns are already represented by the corresponding complete cloud.
        packetCount = 0
        for c, arrival, raw in reader.messages(
            connections=[c for c in cs if c.topic == LIDAR]
        ):
            msg = store.deserialize_ros1(raw, c.msgtype)
            b = bytes(msg.buf)
            if len(b) == 12609 and b[-1] == 0:
                b = b[:-1]
            assert len(b) == 12608, "Expected legacy 64-ring Ouster packet"
            for j in range(16):
                stamp, column, frame, _ = struct.unpack_from("<QHHI", b, j * 788)
                if column >= 1024 or stamp == 0:
                    continue
                if frame not in groups:
                    groups[frame] = {}
                    order.append(frame)
                groups[frame][column] = stamp
            packetCount += 1
            if packetCount % 20000 == 0:
                print("PACKET CLOCKS", packetCount, flush=True)
        cursor = 0
        with (a.output / "scans.bin").open("wb") as out:
            for c, arrival, raw in reader.messages(
                connections=[c for c in cs if c.topic != LIDAR]
            ):
                msg = store.deserialize_ros1(raw, c.msgtype)
                if c.topic == IMU:
                    _, at, gt, *values = decode_packet(bytes(msg.buf))
                    imu.append(
                        [
                            0.5 * (at + gt) * 1e-9,
                            *[v * 9.80665 for v in values[:3]],
                            *[v * np.pi / 180 for v in values[3:]],
                        ]
                    )
                    continue
                dtype = np.dtype(
                    {
                        "names": [f.name for f in msg.fields],
                        "formats": [
                            ("<" if not msg.is_bigendian else ">") + types[f.datatype]
                            for f in msg.fields
                        ],
                        "offsets": [f.offset for f in msg.fields],
                        "itemsize": msg.point_step,
                    }
                )
                cloud = np.ndarray(
                    (msg.height, msg.width),
                    dtype=dtype,
                    buffer=msg.data,
                    strides=(msg.row_step, msg.point_step),
                )
                assert cloud.shape == (64, 1024)
                candidates = []
                for position in range(cursor, min(cursor + 4, len(order))):
                    frame = order[position]
                    columns = groups[frame]
                    origins = np.array(
                        [columns[j] - int(cloud["t"][0, j]) for j in columns],
                        dtype=np.int64,
                    )
                    origin = int(np.median(origins))
                    error = int(np.max(np.abs(origins - origin)))
                    candidates.append((error, position, frame, origin))
                error, position, frame, origin = min(candidates)
                assert error <= 1000, f"Column/t disagreement {error} ns"
                assert position == cursor, (
                    f"Unexplained dropped point-cloud frame {cursor}->{position}"
                )
                cursor = position + 1
                residuals.append(error)
                index = len(records) + 1
                header = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                points = np.column_stack(
                    [cloud[k].reshape(-1) for k in ["x", "y", "z", "intensity"]]
                    + [cloud["t"].reshape(-1) * 1e-6]
                ).astype("<f4")
                out.write(
                    struct.pack("<IddI", index, origin * 1e-9, header, len(points))
                )
                out.write(points.tobytes())
                records.append(
                    {
                        "frame": index,
                        "sensorFrame": frame,
                        "rosHeader": header,
                        "bagArrival": arrival * 1e-9,
                        "nativeBegin": origin * 1e-9,
                        "nativeEnd": (origin + int(cloud["t"].max())) * 1e-9,
                        "pointTimeSpanS": float(cloud["t"].max()) * 1e-9,
                        "clockConsistencyNs": error,
                    }
                )
                if index % 100 == 0:
                    print("HARDWARE EXPORT", index, flush=True)
    np.asarray(imu, dtype="<f8").tofile(a.output / "imu.bin")
    assert np.all(np.diff(np.asarray(imu)[:, 0]) > 0)
    with (a.output / "frames.csv").open("w") as f:
        w = csv.DictWriter(f, fieldnames=records[0])
        w.writeheader()
        w.writerows(records)
    summary = {
        "bag": str(a.bag),
        "frames": len(records),
        "imuSamples": len(imu),
        "maximumPointClockDisagreementNs": max(residuals),
        "inputClock": "Shared native Ouster column/IMU timestamps, no ROS arrival-time propagation",
        "orientationInput": False,
        "gnssInput": False,
        "referenceInput": False,
        "units": "XYZ m, point offset ms, IMU m/s^2 and rad/s, native time s",
    }
    (a.output / "hardware_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n"
    )
    print(summary, flush=True)


if __name__ == "__main__":
    main()
