#!/usr/bin/env python3
"""Drive CARLA poses from a VDB 14DOF trajectory and capture real ray-cast LiDAR.

CARLA vehicle physics is disabled. Inertial, wheel and steering sensors are
sampled from VDB states with declared noise; CARLA's disabled-physics velocity
is never used. Truth, sensor samples and scan files are separate artifacts.
The predetermined test driver is external to the localization algorithm.
"""
import argparse
import csv
import hashlib
import json
import queue
from pathlib import Path

import carla
import numpy as np
from scipy.spatial.transform import Rotation

from captureCarlaTown10Drive import LIDAR_ATTRIBUTES, LIDAR_DTYPE, spawn_sensor

SAE_TO_PROJECT = np.diag([1., -1., -1.])
Z_FLIP = np.diag([1., 1., -1.])


def rotations(row):
    """SAE body->world and project body->world rotations."""
    r = Rotation.from_euler('xyz', [row['roll'], row['pitch'], row['yaw']]).as_matrix()
    return r, SAE_TO_PROJECT @ r @ SAE_TO_PROJECT


def synthesize_sensors(truth, cfg, offset, seed):
    """Generate IMU specific force, wheel encoder and steering measurements."""
    rng = np.random.default_rng(seed)
    n = len(truth)
    values = np.empty((n, 17))
    for k, row in enumerate(truth):
        r, rp = rotations(row)
        acceleration = SAE_TO_PROJECT @ np.array([row['ax'], row['ayRight'], row['azDown']])
        gravity_body = rp.T @ np.array([0., 0., -9.81])
        specific = acceleration - gravity_body + rng.normal(0., cfg['accelerometerNoiseStdMps2'], 3)
        omega = SAE_TO_PROJECT @ np.array([row['p'], row['q'], row['r']])
        omega += rng.normal(0., cfg['gyroNoiseStdRadps'], 3)
        wheels = np.array([row[f'omega{x}'] for x in ['FL', 'FR', 'RL', 'RR']])
        wheels += rng.normal(0., cfg['wheelNoiseStdRadps'], 4)
        # Recover equivalent center road-wheel angle from both actual wheel angles.
        tangents = np.tan([row['steerFL'], row['steerFR']])
        delta = -np.arctan(2. / np.sum(1. / tangents)) if np.all(np.abs(tangents) > 1e-8) else -np.mean(tangents)
        delta += rng.normal(0., cfg['steeringNoiseStdRad'], 1)[0]
        pos = np.array([row['x'], -row['y'], -row['zDown']]) + rp @ np.array([-offset, 0., 0.])
        gnss = pos[:2] + rng.normal(0., cfg['gnssNoiseStdM'], 2)
        values[k] = [row['time'], *specific, *omega, *wheels, delta, *gnss, cfg['gnssNoiseStdM']**2, 1., 0.]
    return values


def capture(args):
    params = json.loads(args.parameters.read_text());cfg = params['config']
    root = Path(__file__).resolve().parents[2]
    nominal = json.loads((root / 'config/mncavVehicleParameters.json').read_text())
    assert cfg['vehicle'] == nominal['vehicle'], 'Stale VDB vehicle parameters: rebuild the plant.'
    offset = json.loads((root / 'config/mncavMotionOutputPoint.json').read_text())['forwardOffsetM']
    dt = cfg['sensors']['imuPeriodSeconds']
    stride = int(round(cfg['sensors']['lidarPeriodSeconds'] / dt))
    assert stride >= 1 and np.isclose(stride * dt, cfg['sensors']['lidarPeriodSeconds'])
    raw = np.genfromtxt(args.trajectory, delimiter=',', names=True)
    ticks = raw[np.isclose(np.mod(raw['time'], dt), 0., atol=1e-7) | np.isclose(np.mod(raw['time'], dt), dt, atol=1e-7)]
    assert np.allclose(np.diff(ticks['time']), dt, atol=1e-7)
    if args.duration is not None:ticks = ticks[ticks['time'] <= args.duration + 1e-8]
    args.output.mkdir(parents=True, exist_ok=False)
    for folder in ['points', 'labels']:(args.output / folder).mkdir()
    sensors = synthesize_sensors(ticks, cfg['sensors'], offset, args.seed)
    np.savetxt(args.output / 'sensors.csv', sensors, delimiter=',', header=','.join([
        'time','specificX','specificY','specificZ','gyroX','gyroY','gyroZ',
        'omegaFL','omegaFR','omegaRL','omegaRR','roadWheelAngle','gnssX','gnssY','gnssVariance','gnssValid','reserved']), comments='')
    calib = {'schemaVersion': 1, 'calibration': {'rotation': np.eye(3).tolist(),
        'translation': [offset, 0., cfg['sensors']['lidarHeightAboveNominalRoadM'] - cfg['body']['cgHeightM']],
        'identifier': 'mncav-vdb-cg-roof-to-configured-output-v1'},
        'sourceFrame': 'carla_lidar_forward_left_up', 'targetFrame': 'mncav_configured_output_point'}
    (args.output / 'calibration.json').write_text(json.dumps(calib, indent=2))
    metadata = {'vehicleDynamics': 'Vehicle Dynamics Blockset Mncav14DOF', 'carlaPhysicsEnabled': False,
        'mode': 'synchronous replay of the independently integrated high-fidelity test plant',
        'truthInput': str(args.trajectory), 'truthSha256': hashlib.sha256(args.trajectory.read_bytes()).hexdigest(),
        'parameters': params, 'seed': args.seed, 'fixedDeltaSeconds': dt,
        'sensors': 'VDB specific-force/gyro/wheel states with seeded noise; real CARLA ray-cast LiDAR',
        'outputPointForwardOffsetM': offset, 'verticalDatum': 'VDB vertical displacement plus nominal CG height',
        'completed': False}
    (args.output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
    client = carla.Client(args.host, args.port);client.set_timeout(60.)
    world = client.get_world();assert 'Town10HD' in world.get_map().name
    original = world.get_settings();actors = [];lidar = None
    rows = []
    try:
        settings = world.get_settings();settings.synchronous_mode = True;settings.fixed_delta_seconds = dt
        settings.no_rendering_mode = True;world.apply_settings(settings)
        bp = world.get_blueprint_library().find('vehicle.lincoln.mkz')
        ego = world.spawn_actor(bp, carla.Transform(carla.Location(x=float(ticks[0]['x']), y=float(ticks[0]['y']), z=2.)))
        actors.append(ego);ego.set_simulate_physics(False)
        samples = queue.Queue()
        # One full scan per simulation step, recorded at 10 Hz after frame matching.
        attributes = dict(LIDAR_ATTRIBUTES);attributes['sensor_tick'] = '0.0'
        attributes['rotation_frequency'] = str(1. / dt)
        attributes['points_per_second'] = str(int(64 * 1024 / dt))
        lidar = spawn_sensor(world, 'sensor.lidar.ray_cast_semantic', attributes,
            carla.Transform(carla.Location(z=cfg['sensors']['lidarHeightAboveNominalRoadM'])), ego)
        actors.append(lidar);lidar.listen(samples.put)
        max_pose_error = 0.
        for step, row in enumerate(ticks):
            r, rp = rotations(row);rc = Z_FLIP @ r @ Z_FLIP
            cg = np.array([row['x'], row['y'], cfg['body']['cgHeightM'] - row['zDown']])
            actor_pos = cg + rc @ np.array([0., 0., -cfg['body']['cgHeightM']])
            transform = carla.Transform(carla.Location(*map(float, actor_pos)),
                carla.Rotation(roll=float(np.degrees(row['roll'])), pitch=float(np.degrees(row['pitch'])), yaw=float(np.degrees(row['yaw']))))
            assert np.max(np.abs(np.array(transform.get_matrix())[:3,:3] - rc)) < 1e-5
            ego.set_transform(transform);frame = world.tick()
            sample = samples.get(timeout=30.)
            while sample.frame < frame:sample = samples.get(timeout=30.)
            assert sample.frame == frame, (sample.frame, frame)
            actual = ego.get_transform().location
            max_pose_error = max(max_pose_error, float(np.linalg.norm(np.array([actual.x, actual.y, actual.z]) - actor_pos)))
            if step % stride:continue
            index = len(rows);points = np.frombuffer(sample.raw_data, dtype=LIDAR_DTYPE)
            xyz = np.column_stack([points['x'], -points['y'], points['z']]).astype(float)
            distance = np.linalg.norm(xyz, axis=1)
            noise = np.random.default_rng(args.seed + index + 100000).normal(0., cfg['sensors']['lidarRangeNoiseStdM'], len(xyz))
            xyz *= (1. + noise / np.maximum(distance, 1e-3))[:,None]
            xyz.astype('<f4').tofile(args.output / 'points' / f'{index:06d}.bin')
            with (args.output / 'labels' / f'{index:06d}.bin').open('wb') as f:
                points['tag'].astype('u1').tofile(f);points['instance'].astype('<u4').tofile(f)
            pos = np.array([cg[0], -cg[1], cg[2]]) + rp @ np.array([-offset, 0., 0.])
            q = Rotation.from_matrix(rp).as_quat(scalar_first=True)
            rows.append([index+1, row['time'], 'VDB_truth_scoring_only', 'mncav_configured_output_point', *pos, *q, frame, step, len(points)])
            if index % 100 == 0:print(f'scan {index}, t={row["time"]:.1f}s, points={len(points)}', flush=True)
        metadata.update(completed=True, scans=len(rows), maximumCarlaPoseErrorM=max_pose_error)
    finally:
        if lidar is not None:lidar.stop()
        for actor in reversed(actors):
            if actor.is_alive:actor.destroy()
        world.apply_settings(original)
        with (args.output / 'poses.csv').open('w', newline='') as f:
            writer=csv.writer(f);writer.writerow(['frame_index','lidar_stamp_sec','pose_source','pose_reference_point',
                'pose_x_m','pose_y_m','pose_z_m','pose_qw','pose_qx','pose_qy','pose_qz','carla_frame','step','points']);writer.writerows(rows)
        (args.output / 'metadata.json').write_text(json.dumps(metadata, indent=2))
    print(json.dumps({'scans': len(rows), 'maximumCarlaPoseErrorM': max_pose_error}))


def main():
    p=argparse.ArgumentParser();p.add_argument('--trajectory', type=Path, required=True)
    p.add_argument('--parameters', type=Path, required=True);p.add_argument('--output', type=Path, required=True)
    p.add_argument('--seed', type=int, default=20261002);p.add_argument('--duration', type=float)
    p.add_argument('--host', default='127.0.0.1');p.add_argument('--port', type=int, default=2000)
    capture(p.parse_args())

if __name__ == '__main__':main()
