#!/usr/bin/env python3
"""Simulator-side export of a measurement-only localization dataset.

Input truth is confined to this sensor-generation process. The output has no
pose/velocity truth, semantic labels, plant body states or fitted tire states.
LiDAR measurements are reused unchanged; paired runs vary the other sensors.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import numpy as np
from mncavVdbSensors import synthesize_sensors, SENSOR_COLUMNS


def prepare(args):
    root=Path(__file__).resolve().parents[2]
    original=json.loads((args.capture/'metadata.json').read_text())
    assert original['completed'] and not original['carlaPhysicsEnabled']
    cfg=original['parameters']['config']
    nominal=json.loads((root/'config/mncavVehicleParameters.json').read_text())
    assert cfg['vehicle']==nominal['vehicle'], 'Stale vehicle in source capture.'
    assert hashlib.sha256(args.truth.read_bytes()).hexdigest()==original['truthSha256'], 'Wrong plant trajectory for the LiDAR capture.'
    noise=json.loads((root/'config/mncavVdbSensorNoise.json').read_text())
    raw=np.genfromtxt(args.truth,delimiter=',',names=True)
    dt=cfg['sensors']['imuPeriodSeconds']
    rows=raw[np.isclose(raw['time']/dt,np.round(raw['time']/dt),atol=1e-7,rtol=0)]
    assert np.allclose(np.diff(rows['time']),dt)
    offset=json.loads((root/'config/mncavMotionOutputPoint.json').read_text())['forwardOffsetM']
    measured=synthesize_sensors(rows,cfg['sensors'],offset,args.seed,noise[args.profile])
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'points').mkdir()
    np.savetxt(args.output/'sensors.csv',measured,delimiter=',',header=','.join(SENSOR_COLUMNS),comments='')
    with (args.capture/'poses.csv').open() as f:
        frames=[{k:r[k] for k in ['frame_index','lidar_stamp_sec','points']} for r in csv.DictReader(f)]
    with (args.output/'frames.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['frame_index','lidar_stamp_sec','points']);writer.writeheader();writer.writerows(frames)
    for row in frames:
        name=f"{int(row['frame_index'])-1:06d}.bin"
        os.link(args.capture/'points'/name,args.output/'points'/name)
    (args.output/'calibration.json').write_bytes((args.capture/'calibration.json').read_bytes())
    wheelPath=root/'config/mncavWheelSpeedCalibration.json';wheel=json.loads(wheelPath.read_text())
    interface=dict(schemaVersion=2,effectiveRadiusM=wheel['effectiveRadiusM'],lagCompensationSeconds=[0.]*4,
        source='independent-mncav-wheel-calibration',sourceFile='config/mncavWheelSpeedCalibration.json',
        sourceSha256=hashlib.sha256(wheelPath.read_bytes()).hexdigest(),fitSequence=wheel['fitSequence'],
        interpretation='Independent recorded-drive radius prior; zero lag for synchronous synthetic encoder delivery. No plant state calibration.')
    (args.output/'sensor_interface.json').write_text(json.dumps(interface,indent=2)+'\n')
    metadata=dict(schemaVersion=2,sensorOnly=True,completed=True,carlaPhysicsEnabled=False,
        parameters=dict(config=cfg),outputPointForwardOffsetM=offset,
        noiseProfile=args.profile,seed=args.seed,sensorErrorModel=noise[args.profile],
        lidarNoise='Reused fixed CARLA ray-cast measurements with the original independent 0.015 m radial noise',
        initialization='No supplied pose or heading. Bootstrap from native noisy GNSS observations before evaluation.',
        sourceCaptureFingerprint=hashlib.sha256((args.capture/'metadata.json').read_bytes()).hexdigest())
    (args.output/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output),frames=len(frames),sensorSamples=len(measured),profile=args.profile,seed=args.seed)))


def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--truth',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profile',choices=['white','moderate'],default='moderate');p.add_argument('--seed',type=int,default=20261002)
    prepare(p.parse_args())

if __name__=='__main__':main()
