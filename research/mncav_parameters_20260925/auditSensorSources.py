#!/usr/bin/env python3
"""Audit MnCAV recording identity and timing without modifying source data.

Run from any directory with uv run --offline --with rosbags python <this file>.
The scatter diagnostic is not an identified white-noise covariance: quantized
moving-vehicle signals contain dynamics, vibration and temporal correlation.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent


def read_csv(path):
    with path.open() as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows):
    with path.open('w') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def main():
    rates, scatter, inputs = [], [], []
    for drive, folder, clock_name in [
        ('12-11-24', 'calibration_sensors',
         'output/mncav_wheel_only_20260916/calibration_sensors/inspva.clock.json'),
        ('12-09-31', 'sensors',
         'data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.clock.json'),
    ]:
        base = ROOT / 'output/mncav_wheel_only_20260916' / folder
        clock_path = ROOT / clock_name
        clock = json.loads(clock_path.read_text())
        inputs.append(clock_path)
        scale = clock['scale']
        ins = read_csv(base / 'inspva.csv')
        # Match the existing calibration's receiver-relative origin: first
        # INSPVA timestamp plus 0.5 receiver seconds.
        origin = float(ins[0]['stamp_sec']) + 0.5 / scale
        for name in ['imu', 'steering', 'wheel_speed_report', 'corrimu', 'inspva']:
            path = base / (name + '.csv')
            inputs.append(path)
            rows = read_csv(path)
            times = [float(row['stamp_sec']) for row in rows]
            differences = [scale * (b-a) for a, b in zip(times, times[1:]) if b > a]
            rates.append(dict(
                drive=drive, topicExport=name, samples=len(times),
                receiverMeanHz=(len(times)-1)/(scale*(times[-1]-times[0])),
                receiverMedianHz=1/statistics.median(differences),
                rosMeanHz=(len(times)-1)/(times[-1]-times[0]), clockScale=scale))
            if name == 'imu':
                for field in ['acceleration_x_mps2', 'acceleration_y_mps2', 'angular_z_radps']:
                    values = [float(row[field]) for row, time in zip(rows, times)
                              if 1 <= scale*(time-origin) <= 40]
                    diff2 = [a-2*b+c for a, b, c in zip(values, values[1:], values[2:])]
                    center = statistics.median(diff2)
                    proxy = (1.4826022185 / math.sqrt(6) *
                             statistics.median(abs(value-center) for value in diff2))
                    scatter.append(dict(drive=drive, field=field, samples=len(values),
                                        receiverStartSeconds=1, receiverEndSeconds=40,
                                        secondDifferenceScatterProxy=proxy))

    bag = ROOT / 'data/raw/Missisipi/raw_data_2024-06-07-12-09-31_0.bag'
    targets = ['/vehicle/imu/data_raw', '/novatel/oem7/corrimu',
               '/vehicle/lidar/front_ouster/points']
    identity = {}
    with Reader(bag) as reader:
        connections = [c for c in reader.connections if c.topic in targets]
        store = get_typestore(Stores.EMPTY)
        types = {}
        for connection in connections:
            types.update(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        store.register(types)
        for connection, _, raw in reader.messages(connections=connections):
            if connection.topic in identity:
                continue
            message = store.deserialize_ros1(raw, connection.msgtype)
            info = dict(topic=connection.topic, type=connection.msgtype,
                        publisher=connection.ext.callerid, frame=message.header.frame_id,
                        count=connection.msgcount)
            if connection.topic.endswith('data_raw'):
                for field in ['angular_velocity_covariance', 'linear_acceleration_covariance',
                              'orientation_covariance']:
                    info[field] = getattr(message, field).tolist()
            if connection.topic.endswith('points'):
                info.update(height=message.height, width=message.width,
                            fields=[field.name for field in message.fields])
            identity[connection.topic] = info
            if len(identity) == len(targets):
                break
    assert len(identity) == len(targets)
    config_path = ROOT / 'output/mncav_interface_audit_20260916/12-09-31/insconfig.json'
    inputs.append(config_path)
    config = json.loads(config_path.read_text())[0]
    identity['novatelRecordedConfiguration'] = dict(
        source=str(config_path.relative_to(ROOT)), imuType=config['imu_type'],
        exportedTranslations=config['number_of_translations'],
        exportedRotations=config['number_of_rotations'])
    (DEST / 'bag_sensor_identity.json').write_text(json.dumps(identity, indent=2)+'\n')
    write_csv(DEST / 'native_sensor_rates.csv', rates)
    write_csv(DEST / 'imu_scatter_diagnostic.csv', scatter)
    inputs.extend(ROOT / name for name in ['config/mncavVehicleParameters.json',
                  'config/mncavSensorParameters.json', 'config/mncavSensorConfig.m',
                  'config/lateralObserverConfig.m', 'tests/reference/mncavLateralObserverDesign.mat'])
    inputs.extend((ROOT / 'output/mncav_parameters_20260925').glob('*.pdf'))
    write_csv(DEST / 'source_hashes.csv', [dict(path=str(path.relative_to(ROOT)),
              bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
              for path in inputs])
    print(f'Audited {len(rates)} sensor streams and {len(scatter)} scatter diagnostics.')
    print('Recorded NovAtel IMU code:', config['imu_type'])


if __name__ == '__main__':
    main()
