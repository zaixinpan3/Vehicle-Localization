#!/usr/bin/env python3
"""Verify identification exports against raw ROS bags and audit INS status.

Run with: uv run --offline --with rosbags --with numpy python <this file>
Hashes cover the selected topic payload stream, not the entire large bag.
"""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
TOPICS = {'/novatel/oem7/inspva': 'inspva', '/vehicle/imu/data_raw': 'imu',
          '/vehicle/steering_report': 'steering', '/novatel/oem7/heading2': 'heading2',
          '/novatel/oem7/insstdev': 'insstdev'}


def main():
    reports = []
    for drive, folder in [('12-11-24', 'calibration_sensors'), ('12-09-31', 'sensors')]:
        path = ROOT / ('data/raw/Missisipi/raw_data_2024-06-07-'+drive+'_0.bag')
        streams, counts, hashes, first, status = {}, Counter(), {}, {}, Counter()
        heading_status, deviations, ins_intervals = Counter(), [], []
        origin_gps = None
        with Reader(path) as reader:
            selected = [c for c in reader.connections if c.topic in TOPICS]
            store = get_typestore(Stores.EMPTY)
            types = {}
            for c in selected:
                types.update(get_types_from_msg(c.msgdef.data, c.msgtype))
            store.register(types)
            for c, timestamp, raw in reader.messages(connections=selected):
                name = TOPICS[c.topic]
                m = store.deserialize_ros1(raw, c.msgtype)
                counts[name] += 1
                hashes.setdefault(name, hashlib.sha256()).update(
                    timestamp.to_bytes(8, 'little')+len(raw).to_bytes(8, 'little')+raw)
                if name not in first:
                    first[name] = dict(topic=c.topic, messageType=c.msgtype,
                        publisher=c.ext.callerid, frame=m.header.frame_id)
                if name == 'inspva':
                    status[int(m.status.status)] += 1
                    gps = m.nov_header.gps_week_number*604800+m.nov_header.gps_week_milliseconds/1000
                    if origin_gps is None:
                        origin_gps = gps
                    relative = gps-origin_gps
                    if not ins_intervals or ins_intervals[-1]['status'] != int(m.status.status):
                        ins_intervals.append(dict(status=int(m.status.status), startSeconds=relative,
                                                  endSeconds=relative, samples=0))
                    ins_intervals[-1]['endSeconds'] = relative
                    ins_intervals[-1]['samples'] += 1
                    row = [m.north_velocity, m.east_velocity, m.roll, m.pitch, m.azimuth]
                elif name == 'imu':
                    row = [m.angular_velocity.z, m.linear_acceleration.y]
                elif name == 'steering':
                    row = [m.steering_wheel_angle]
                elif name == 'heading2':
                    value = m.sol_status
                    heading_status[str(getattr(value, 'status', value))] += 1
                    continue
                else:
                    # Preserve a readable first-message description: field names
                    # differ between installed NovAtel ROS message revisions.
                    if len(deviations) == 0:
                        deviations.append(str(m))
                    continue
                streams.setdefault(name, []).append(row)
        fields = {'inspva': ['north_velocity', 'east_velocity', 'roll', 'pitch', 'azimuth'],
                  'imu': ['angular_z_radps', 'acceleration_y_mps2'],
                  'steering': ['steering_wheel_angle_rad']}
        checks = {}
        for name, columns in fields.items():
            csv_path = ROOT / 'output/mncav_wheel_only_20260916' / folder / (name+'.csv')
            with csv_path.open() as handle:
                exported = np.array([[float(row[key]) for key in columns]
                                     for row in csv.DictReader(handle)])
            raw_values = np.array(streams[name])
            assert exported.shape == raw_values.shape, (drive, name, exported.shape, raw_values.shape)
            error = float(np.max(np.abs(exported-raw_values)))
            assert error < 1e-10, (drive, name, error)
            checks[name] = dict(rows=len(exported), fields=columns, maxAbsoluteDifference=error,
                csvSha256=hashlib.sha256(csv_path.read_bytes()).hexdigest())
        reports.append(dict(drive=drive, bag=str(path.relative_to(ROOT)), bagBytes=path.stat().st_size,
            topicCounts=dict(counts), topicIdentity=first,
            topicStreamSha256={k:v.hexdigest() for k,v in hashes.items()},
            hashEncoding='For each topic: concatenated little-endian uint64 bag timestamp, uint64 payload length, ROS1 payload',
            inspvaStatusCounts=dict(status), headingSolutionStatusCounts=dict(heading_status),
            inspvaStatusIntervals=ins_intervals,
            firstInsstdevMessage=deviations, exportChecks=checks))
        print(drive, 'verified', checks, 'INS statuses', status, flush=True)
    (DEST/'raw_bag_audit.json').write_text(json.dumps(reports, indent=2)+'\n')


if __name__ == '__main__':
    main()
