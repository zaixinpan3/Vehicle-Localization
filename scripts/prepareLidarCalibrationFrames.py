"""Export sparse independent-drive clouds for offline LiDAR/INS calibration.

The stored-axis rotation is identical to extractPointCloudsFromBag.m.
No reference pose or calibration is applied to the exported point coordinates.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json

import numpy as np
from scipy.io import savemat
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from prepareInspvaMappingPoses import prepare


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=root / 'data/raw/Missisipi/raw_data_2024-06-07-12-11-24_0.bag')
    parser.add_argument('--inspva', type=Path, default=root / 'output/mncav_bestpos_alignment_20260917/calibration_sources/inspva.csv')
    parser.add_argument('--output', type=Path, default=root / 'output/lidar_origin_20260922/calibration_inputs')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # Fixed calibration/validation windows selected for turning motion.
    selected = set(range(11, 172, 5)) | set(range(411, 612, 10))
    base = np.array([[.925216, -.367871, .093195], [.368647, .929524, .009468], [-.090110, .025595, .995625]])
    additional = np.array([[.931395, .364011, 0], [-.364011, .931395, 0], [0, 0, 1]])
    rotation = (additional @ base).astype(np.float32)
    store = get_typestore(Stores.ROS1_NOETIC)
    fields = []; files = []
    with Reader(args.bag) as reader:
        connection = [c for c in reader.connections if c.topic == '/vehicle/lidar/front_ouster/points']
        assert len(connection) == 1
        for index, (c, _, raw) in enumerate(reader.messages(connections=connection), 1):
            if index not in selected:
                continue
            msg = store.deserialize_ros1(raw, c.msgtype)
            assert not msg.is_bigendian and msg.row_step == msg.width * msg.point_step
            types = {1:'i1', 2:'u1', 3:'<i2', 4:'<u2', 5:'<i4', 6:'<u4', 7:'<f4', 8:'<f8'}
            dtype = np.dtype(dict(names=[f.name for f in msg.fields],
                                  formats=[types[f.datatype] for f in msg.fields],
                                  offsets=[f.offset for f in msg.fields], itemsize=msg.point_step))
            values = np.frombuffer(msg.data, dtype=dtype)
            xyz = np.column_stack([values[k] for k in ['x','y','z']]).astype(np.float32) @ rotation.T
            frame = {k:xyz[:,j].reshape(msg.height,msg.width) for j,k in enumerate(['x','y','z'])}
            for key in ['intensity','reflectivity','ambient']:
                frame[key] = values[key].astype(np.float32).reshape(msg.height,msg.width) if key in values.dtype.names else np.zeros((msg.height,msg.width), np.float32)
            frame['timestamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            path = args.output / f'frame_{index:04d}.mat'
            savemat(path, dict(frame=frame), do_compression=True)
            files.append(dict(frame=index, file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            fields.append(dict(frame_index=index,stamp_sec=frame['timestamp']))
    assert {r['frame_index'] for r in fields} == selected
    lidar = args.output / 'lidar_headers.csv'
    with lidar.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=['frame_index','stamp_sec']); writer.writeheader(); writer.writerows(fields)
    prepare(lidar, args.inspva, args.output / 'poses.csv')
    manifest = dict(bag=str(args.bag), inspva=str(args.inspva), files=files,
                    trainingFrameInterval=[11,171], validationFrameInterval=[411,611],
                    evaluationDriveUsed=False, storedAxisRotation=rotation.tolist())
    (args.output / 'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Exported {len(files)} independent-drive frames.')


if __name__ == '__main__':
    main()
