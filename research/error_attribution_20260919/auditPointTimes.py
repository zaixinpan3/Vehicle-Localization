"""Read original scan fields without modifying the bag or inferring time units."""
import csv
import json
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

root = Path(__file__).resolve().parents[2]
raw = root / 'data/raw/Missisipi'
stem = 'raw_data_2024-06-07-12-09-31_0'
with (raw / 'gnss' / f'{stem}_front_lidar_points.csv').open() as handle:
    rows = list(csv.DictReader(handle))
store = get_typestore(Stores.ROS1_NOETIC)
results = []
with Reader(raw / f'{stem}.bag') as reader:
    connections = [c for c in reader.connections
                   if c.topic == '/vehicle/lidar/front_ouster/points']
    for frame in [92, 820]:
        stamp = float(rows[frame-1]['stamp_sec'])
        best = None
        for connection, _, data in reader.messages(connections=connections,
                start=int((stamp-.2)*1e9), stop=int((stamp+.2)*1e9)):
            message = store.deserialize_ros1(data, connection.msgtype)
            difference = abs(message.header.stamp.sec + message.header.stamp.nanosec*1e-9 - stamp)
            if best is None or difference < best[0]:
                best = difference, message
        assert best is not None and best[0] < 1e-5
        difference, message = best
        assert not message.is_bigendian
        assert message.row_step == message.width * message.point_step
        entry = dict(frame=frame, timestampDifferenceSeconds=difference,
                     fields=[dict(name=f.name, datatype=f.datatype, offset=f.offset,
                                  count=f.count) for f in message.fields])
        for field in message.fields:
            if field.name in ['t', 'time', 'timestamp']:
                formats = {6: '<u4', 7: '<f4', 8: '<f8'}
                dtype = np.dtype(dict(names=['t'], formats=[formats[field.datatype]],
                                      offsets=[field.offset], itemsize=message.point_step))
                values = np.frombuffer(message.data, dtype=dtype)['t']
                entry['pointTimeRaw'] = dict(field=field.name, min=float(values.min()),
                    max=float(values.max()), unit='not asserted from field name alone')
        results.append(entry)
out = root / 'output/error_attribution_20260919'
out.mkdir(parents=True, exist_ok=True)
(out / 'raw_time_fields.json').write_text(json.dumps(results, indent=2)+'\n')
print(json.dumps(results, indent=2))
