#!/usr/bin/env python3
"""Fit wheel conversion on the separate MnCAV 12:11:24 drive only.

uv run --offline --with numpy --with pandas --with scipy python scripts/calibrateMncavWheelSpeed.py
The 1--40 s fit and >=40 s validation are separate. Evaluation references
are exported for scoring, never for fitting or runtime wheel estimation.
"""
from pathlib import Path
from mncavParameters import replay_parameters
import hashlib
import json

import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from receiverClock import ensure_clock, convert_time, native_seconds

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "output/mncav_wheel_only_20260916"
INTERFACES = ROOT / "output/mncav_interface_audit_20260916"
OUT = BASE / "calibration"
WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]


def prepare(sequence, parameters):
    folder = BASE / ("calibration_sensors" if sequence == "12-11-24" else "sensors")
    ins_path = (folder / "inspva.csv" if sequence == "12-11-24" else
                ROOT / "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv")
    ins = pd.read_csv(ins_path)
    ins.columns = [x.replace("_deg", "").replace("_mps", "") for x in ins.columns]
    imu, steering = [pd.read_csv(folder / (name + ".csv")) for name in ["imu", "steering"]]
    wheel_path = folder / "wheel_speed_report.csv"
    wheel = pd.read_csv(wheel_path)
    clock = ensure_clock(ins_path)
    receiver = native_seconds(ins.gps_week, ins.gps_seconds)
    start = 0.0
    bridge = lambda stamps: convert_time(clock, stamps)
    if sequence == "12-09-31":
        poses = pd.read_csv(ROOT / "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_front_lidar_points.csv")
        start = float(bridge(poses.stamp_sec.iloc[0]))
    native = receiver - start
    yaw = np.pi / 2 - np.deg2rad(ins.azimuth)
    reference_vx = ins.east_velocity * np.cos(yaw) + ins.north_velocity * np.sin(yaw)
    # Strictly interior export times avoid inventing endpoint extrapolation.
    first = max(0.1, bridge(wheel.stamp_sec.iloc[0]) - start + .02)
    last = min(native[-1], bridge(imu.stamp_sec.iloc[-1]) - start,
               bridge(steering.stamp_sec.iloc[-1]) - start) - .02
    time = np.arange(np.ceil(first*100), np.floor(last*100)+1)/100
    sample = lambda table, key: np.interp(time, bridge(table.stamp_sec)-start, table[key])
    motion = pd.DataFrame({"time": time, "referenceVx": np.interp(time, native, reference_vx)})
    motion["steeringAngle"] = (sample(steering, "steering_wheel_angle_rad") - parameters["steeringWheelOffsetRad"]) / parameters["steeringRatio"]
    for name, raw in [("yawRate", "angular_z_radps"), ("longitudinalAcceleration", "acceleration_x_mps2")]:
        correction = parameters["input_correction"][name]
        motion[name] = correction["sign"] * sample(imu, raw) + correction["offset"]
    wheels = pd.DataFrame({"time": bridge(wheel.stamp_sec)-start, **{k: wheel[k] for k in WHEELS}})
    # Preserve only strictly ordered covered wheel timestamps; report all losses.
    ordered = np.r_[True, np.diff(wheels.time) > 0]
    wheels = wheels.loc[ordered].reset_index(drop=True)
    dest = OUT / sequence
    dest.mkdir(parents=True, exist_ok=True)
    motion.to_csv(dest / "motion.csv", index=False)
    wheels.to_csv(dest / "wheels.csv", index=False)
    pd.DataFrame({"time": native, "referenceVx": reference_vx}).to_csv(dest / "reference.csv", index=False)
    metadata = {"originalWheelPackets": len(wheel), "exportedWheelPackets": len(wheels),
                "nonIncreasingBridgePacketsRemoved": int((~ordered).sum()),
                "nonfiniteWheelValues": int((~np.isfinite(wheel[WHEELS])).sum().sum()),
                "receiverMedianWheelPeriodSeconds": float(np.median(np.diff(wheels.time))),
                "headerMedianWheelPeriodSeconds": float(np.median(np.diff(wheel.stamp_sec))),
                "maximumReceiverWheelGapSeconds": float(np.max(np.diff(wheels.time))),
                "wheelExportSha256": hashlib.sha256(wheel_path.read_bytes()).hexdigest(),
                "wheelTopic": "/vehicle/wheel_speed_report", "messageType": "dbw_fca_msgs/WheelSpeedReport",
                "units": "rad/s, confirmed from the message definition embedded in the original evaluation bag",
                "clock": clock}
    return motion, wheels, metadata


def main():
    parameters = replay_parameters()
    front = parameters["stock"]["frontTrackM"]
    rear = parameters["stock"]["rearTrackM"]
    track_y = np.array([front, -front, rear, -rear]) / 2
    wheelbase = parameters["vehicle"]["lf"] + parameters["vehicle"]["lr"]
    motion, wheels, calibration_metadata = prepare("12-11-24", parameters)
    time = motion.time.to_numpy()
    omega = np.column_stack([np.interp(time, wheels.time, wheels[k]) for k in WHEELS])
    r = motion.yawRate.to_numpy()
    delta = motion.steeringAngle.to_numpy()
    angle = np.zeros_like(omega)
    angle[:, :2] = np.arctan2((wheelbase*np.tan(delta))[:, None], wheelbase-track_y[:2]*np.tan(delta)[:, None])
    acceleration = motion.longitudinalAcceleration.to_numpy() + parameters["vehicle"]["lr"]*r*r
    mask = (time > 1) & (time < 40) & (motion.referenceVx.to_numpy() > 3)
    radius, lag, residuals = [], [], []
    for i in range(4):
        design = np.column_stack([omega[:, i]*np.cos(angle[:, i]), acceleration])
        target = motion.referenceVx.to_numpy() - r*track_y[i]
        fit = least_squares(lambda p: (design@p-target)[mask], [.36, .03],
                            bounds=([.32, 0], [.40, .2]), loss="soft_l1", f_scale=.025)
        assert fit.success
        radius.append(float(fit.x[0])); lag.append(float(fit.x[1]))
        residuals.append(float(np.sqrt(np.mean(fit.fun**2))))
    config = {"wheelOrder": WHEELS, "effectiveRadiusM": radius, "lagCompensationSeconds": lag,
              "fitSequence": "raw_data_2024-06-07-12-11-24_0", "fitReceiverIntervalSeconds": [1, 40],
              "fitSamples": int(mask.sum()), "fitWheelRmseMps": residuals,
              "method": "Bounded robust radius and acceleration-coefficient fit; independent calibration drive only",
              "limitations": "Equivalent planar conversion, not measured tire radius or physical latency. GNSS/INS velocity used offline for calibration; none at runtime. Common slip, installation geometry and transfer error remain.",
              "geometrySource": "https://www.stellantisfleet.com/content/dam/fca-fleet/na/fleet/en_us/chrysler/2021/Pacifica/specifications/2021_CH_PacificaHybrid_Specifications.pdf"}
    (ROOT / "config/mncavWheelSpeedCalibration.json").write_text(json.dumps(config, indent=2)+"\n")
    _, _, evaluation_metadata = prepare("12-09-31", parameters)
    report = {"calibration": calibration_metadata, "evaluation": evaluation_metadata, "configuration": config,
              "runtimeInputs": "Four wheel rates, steering angle, corrected IMU yaw rate and longitudinal acceleration"}
    (OUT / "input_audit.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
