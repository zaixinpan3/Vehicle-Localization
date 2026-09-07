#!/usr/bin/env python3
"""Validate CAN axes on the separate 12:11:24 drive and record nominal parameters.

uv run --with numpy --with scipy --with pandas python scripts/calibrateMncavReplayInputs.py
The first 1--40 receiver seconds calibrate constant offsets; later samples
validate them. Evaluation-drive GNSS never fits an online input correction.
Unidentifiable bicycle parameters are reported, not accepted as measurements.
"""
import argparse
import json
from importlib.metadata import version
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-folder', type=Path, default=Path(
        'output/mississippi_20240607_120931_20260907/calibration_sensors'))
    parser.add_argument('--output', type=Path, default=Path(
        'output/mississippi_20240607_120931_20260907/vehicle_parameters.json'))
    args = parser.parse_args()
    ins = pd.read_csv(args.sensor_folder/'inspva.csv')
    imu = pd.read_csv(args.sensor_folder/'imu.csv')
    steering = pd.read_csv(args.sensor_folder/'steering.csv')
    t = ins.gps_seconds.to_numpy()-ins.gps_seconds.iloc[0]
    dt = float(np.median(np.diff(t)))
    assert np.max(np.abs(np.diff(t)-dt)) < 1e-6, 'Expected regular receiver-time INS samples.'
    yaw = np.unwrap(np.pi/2-np.deg2rad(ins.azimuth.to_numpy()))
    vx = ins.east_velocity.to_numpy()*np.cos(yaw)+ins.north_velocity.to_numpy()*np.sin(yaw)
    vy = -ins.east_velocity.to_numpy()*np.sin(yaw)+ins.north_velocity.to_numpy()*np.cos(yaw)
    derivative = lambda x, order: savgol_filter(x, 51, 3, deriv=order, delta=dt)
    yaw_rate = derivative(yaw, 1)
    ax = derivative(vx, 1)-vy*yaw_rate
    ay = derivative(vy, 1)+vx*yaw_rate
    train = (t > 1) & (t < 40)
    validate = (t >= 40) & (t < t[-1]-.52)
    correction = {}
    for name, field, truth, sign in [
            ('longitudinalAcceleration', 'acceleration_x_mps2', ax, 1),
            ('lateralAcceleration', 'acceleration_y_mps2', ay, -1),
            ('yawRate', 'angular_z_radps', yaw_rate, 1)]:
        raw = np.interp(ins.stamp_sec, imu.stamp_sec, imu[field])
        offset = float(np.median(truth[train]-sign*raw[train]))
        fit = np.linalg.lstsq(np.c_[raw[train], np.ones(sum(train))], truth[train], rcond=None)[0]
        residual = sign*raw+offset-truth
        correction[name] = {'sign': sign, 'offset': offset,
                            'diagnostic_unconstrained_slope': float(fit[0]),
                            'validation_rmse': float(np.sqrt(np.mean(residual[validate]**2)))}
    mass, wheelbase, front_fraction = 2273., 3.089, .555
    lf, lr = wheelbase*(1-front_fraction), wheelbase*front_fraction
    delta = np.interp(ins.stamp_sec, steering.stamp_sec, steering.steering_wheel_angle_rad)/16.2
    design = np.c_[delta-(vy+lf*yaw_rate)/np.maximum(vx, 1),
                   -(vy-lr*yaw_rate)/np.maximum(vx, 1)]
    selected = (vx > 6) & (np.abs(yaw_rate) > .02) & (t > .52) & (t < t[-1]-.52)
    stiffness = np.linalg.lstsq(design[selected], mass*ay[selected], rcond=None)[0]
    yaw_acceleration = derivative(yaw, 2)
    moment = lf*design[:, 0]*stiffness[0]-lr*design[:, 1]*stiffness[1]
    inertia = np.linalg.lstsq(yaw_acceleration[selected, None], moment[selected], rcond=None)[0][0]
    report = {
        'software': {name: version(name) for name in ['numpy', 'scipy', 'pandas']},
        'calibration_sequence': 'raw_data_2024-06-07-12-11-24_0',
        'evaluation_sequence': 'raw_data_2024-06-07-12-09-31_0',
        'training_interval_seconds': [1, 40], 'validation_start_seconds': 40,
        'duration_seconds': float(t[-1]), 'input_correction': correction,
        'vehicle': {'mass': mass, 'lf': lf, 'lr': lr,
                    'yawInertia': mass*(5.189**2+2.022**2)/12,
                    'frontCorneringStiffness': 75000*mass/1575,
                    'rearCorneringStiffness': 56000*mass/1575},
        'steeringRatio': 16.2,
        'provenance': {
            'identity': 'UMN identifies MnCAV as a 2021 Chrysler Pacifica Hybrid.',
            'stock_specifications': '2273 kg EPA curb mass, 3.089 m wheelbase, 55.5/44.5 axle load split, 16.2 steering ratio; actual loaded MnCAV mass is unknown.',
            'lf_lr': 'Derived static unladen CG distances from published axle load fractions.',
            'yawInertia': 'Explicit uniform rectangular planform approximation, not a measured MnCAV inertia.',
            'cornering_stiffness': 'Unidentified nominal prior: existing generic model stiffnesses scaled by the stock mass ratio. Not a manufacturer specification.',
            'sensitivity': 'Repeat the experiment at 0.7 and 1.3 times nominal inertia and both axle stiffnesses; nominal mass and geometry remain fixed.',
            'input_offsets': 'Fixed signs plus median offsets from the independent calibration drive; no online truth injection. Residual road-bank and sensor-to-CG lever-arm errors remain.',
            'sources': [
                'https://www.cts.umn.edu/news-pubs/news/2021/august/mncav',
                'https://www.stellantisfleet.com/content/dam/fca-fleet/na/fleet/en_us/chrysler/2021/Pacifica/specifications/2021_CH_PacificaHybrid_Specifications.pdf']},
        'rejected_direct_identification': {
            'front_stiffness_N_per_rad': float(stiffness[0]),
            'rear_stiffness_N_per_rad': float(stiffness[1]),
            'yaw_inertia_kg_m2': float(inertia),
            'design_condition_number': float(np.linalg.cond(design[selected])),
            'used_in_observer': False,
            'reason': 'Nonphysical solution and unknown INS-to-CG reference point. This drive does not establish calibrated MnCAV bicycle parameters.'}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
