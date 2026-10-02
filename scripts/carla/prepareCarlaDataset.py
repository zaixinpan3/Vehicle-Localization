#!/usr/bin/env python3
"""Convert a CARLA capture into the right-handed inputs of the MATLAB pipeline.

Frames (all metres, radians unless named otherwise):

* world: CARLA world with y negated (x east-ish, y north-ish, z up, right-handed);
* body: vehicle x forward, y left, z up, origin at the vehicle reference point;
* LiDAR: sensor x forward, y left, z up (CARLA sensor frame with y negated).

The vehicle reference point is the point on the body x axis whose lateral
velocity is smallest in turns (the kinematic rear-axle point). It is estimated
once, on the mapping drive, by least squares over turning samples and then
fixed for every drive; the pose table, odometry and LiDAR calibration all use
it. This is a vehicle-geometry calibration, not an online input.

Outputs in ``--output``:

* ``points/NNNNNN.bin``: float32 [x y z] per point in the LiDAR frame, with
  Gaussian range noise (``--range-noise``, seeded per sweep);
* ``labels/NNNNNN.bin``: uint8 semantic tag then uint32 instance per point
  (evaluation labels only);
* ``poses.csv``: ``readFramePoseTable`` columns for the reference point at
  each sweep (CARLA ground truth);
* ``motion.csv``: 50 Hz wheel-speed and gyro odometry inputs plus ground truth;
* ``calibration.json``: LiDAR-to-reference frame calibration.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

import carla

LIDAR_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos", "<f4"),
                        ("instance", "<u4"), ("tag", "<u4")])
FLIP = np.diag([1.0, -1.0, 1.0])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--capture", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-offset", type=float, default=None,
                        help="Reference point x in the CARLA actor frame (m); estimated from this capture if omitted")
    parser.add_argument("--range-noise", type=float, default=0.015, help="LiDAR range noise standard deviation (m)")
    parser.add_argument("--wheel-speed-noise", type=float, default=0.02, help="Wheel speed noise (m/s)")
    parser.add_argument("--wheel-speed-scale", type=float, default=1.002, help="Wheel speed scale factor")
    parser.add_argument("--seed", type=int, default=20261001)
    return parser.parse_args()


def read_csv(path):
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    return {key: np.array([row[key] for row in rows]) for key in rows[0]}


def as_float(column):
    out = np.full(column.shape, np.nan)
    filled = column != ""
    out[filled] = column[filled].astype(float)
    return out


def carla_rotation(roll, pitch, yaw):
    """Rotation matrix of a CARLA transform (left-handed), via CARLA itself."""
    transform = carla.Transform(carla.Location(), carla.Rotation(pitch=float(pitch), yaw=float(yaw), roll=float(roll)))
    return np.array(transform.get_matrix())[:3, :3]


def right_handed(rotation_lh):
    return FLIP @ rotation_lh @ FLIP


def quaternion(rotation):
    """[w x y z] of a proper rotation matrix (Shepperd)."""
    m = rotation
    trace = np.trace(m)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            q = [(m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s]
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    return q / np.linalg.norm(q) * (1 if q[0] >= 0 else -1)


def body_states(ticks):
    """Right-handed world rotation, body velocity and yaw rate per step."""
    n = len(ticks["step"])
    rotations = np.zeros((n, 3, 3))
    body_velocity = np.zeros((n, 3))
    yaw_rate = np.zeros(n)
    for k in range(n):
        r = right_handed(carla_rotation(float(ticks["roll_deg"][k]), float(ticks["pitch_deg"][k]),
                                        float(ticks["yaw_deg"][k])))
        rotations[k] = r
        v_world = FLIP @ np.array([float(ticks["vx"][k]), float(ticks["vy"][k]), float(ticks["vz"][k])])
        body_velocity[k] = r.T @ v_world
        # CARLA angular velocity is left-handed degrees per second about world axes.
        w_world = -FLIP @ np.radians([float(ticks["wx_degps"][k]), float(ticks["wy_degps"][k]),
                                      float(ticks["wz_degps"][k])])
        yaw_rate[k] = (r.T @ w_world)[2]
    return rotations, body_velocity, yaw_rate


def estimate_reference_offset(body_velocity, yaw_rate):
    """x of the body-axis point with least lateral velocity in turns."""
    speed = np.hypot(body_velocity[:, 0], body_velocity[:, 1])
    use = (np.abs(yaw_rate) > 0.08) & (speed > 2.0)
    offset = -np.sum(body_velocity[use, 1] * yaw_rate[use]) / np.sum(yaw_rate[use] ** 2)
    residual = body_velocity[use, 1] + yaw_rate[use] * offset
    return float(offset), int(np.count_nonzero(use)), float(np.sqrt(np.mean(residual ** 2)))


def main():
    args = parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "points").mkdir()
    (args.output / "labels").mkdir()
    metadata = json.loads((args.capture / "metadata.json").read_text())
    ticks = read_csv(args.capture / "ticks.csv")
    frames = read_csv(args.capture / "lidar_frames.csv")
    rotations, body_velocity, yaw_rate = body_states(ticks)
    estimate = estimate_reference_offset(body_velocity, yaw_rate)
    offset = estimate[0] if args.reference_offset is None else args.reference_offset
    print(f"reference offset estimate {estimate[0]:.4f} m over {estimate[1]} samples, "
          f"lateral residual {estimate[2]:.4f} m/s; using {offset:.4f} m", flush=True)
    mount = metadata["lidar"]["mount"]
    calibration = {
        "schemaVersion": 1,
        "calibration": {"rotation": np.eye(3).tolist(),
                        "translation": [mount["x"] - offset, -mount["y"], mount["z"]],
                        "identifier": f"carla-town10-{metadata['vehicle']}-roof-lidar"},
        "sourceFrame": "carla_semantic_lidar_right_handed",
        "targetFrame": "vehicle_kinematic_reference_point",
        "coordinateConvention": "p_reference = rotation * p_stored + translation; metres, forward/left/up",
        "referenceOffsetInActorFrameM": offset,
        "referenceOffsetEstimate": {"value": estimate[0], "samples": estimate[1],
                                    "lateralVelocityResidualMps": estimate[2]},
    }
    (args.output / "calibration.json").write_text(json.dumps(calibration, indent=2))

    # Reference-point world pose at every step (right-handed world).
    position = np.stack([as_float(ticks["x"]), -as_float(ticks["y"]), as_float(ticks["z"])], axis=1)
    reference = position + rotations[:, :, 0] * offset
    step_of = {int(s): k for k, s in enumerate(ticks["step"].astype(int))}

    rng = np.random.default_rng(args.seed)
    with open(args.output / "poses.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame_index", "lidar_stamp_sec", "pose_source", "pose_reference_point",
                         "pose_x_m", "pose_y_m", "pose_z_m", "pose_qw", "pose_qx", "pose_qy", "pose_qz",
                         "carla_frame", "step", "points"])
        for i in range(len(frames["lidar_index"])):
            index = int(frames["lidar_index"][i])
            k = step_of[int(frames["step"][i])]
            lidar_rotation = right_handed(carla_rotation(float(frames["sensor_roll_deg"][i]),
                                                         float(frames["sensor_pitch_deg"][i]),
                                                         float(frames["sensor_yaw_deg"][i])))
            assert np.abs(lidar_rotation - rotations[k]).max() < 1e-5, "Sensor and vehicle attitudes disagree"
            q = quaternion(rotations[k])
            writer.writerow([index + 1, frames["time_s"][i], "CARLA_ground_truth", "vehicle_kinematic_reference_point",
                             f"{reference[k, 0]:.6f}", f"{reference[k, 1]:.6f}", f"{reference[k, 2]:.6f}",
                             f"{q[0]:.12f}", f"{q[1]:.12f}", f"{q[2]:.12f}", f"{q[3]:.12f}",
                             frames["frame"][i], frames["step"][i], frames["points"][i]])
            raw = np.load(args.capture / "lidar" / f"{index:06d}.npy")
            xyz = np.stack([raw["x"], -raw["y"], raw["z"]], axis=1).astype(np.float64)
            r = np.linalg.norm(xyz, axis=1)
            noise = np.random.default_rng(args.seed + index).normal(0.0, args.range_noise, len(r))
            xyz *= (1.0 + noise / np.maximum(r, 1e-3))[:, None]
            xyz.astype("<f4").tofile(args.output / "points" / f"{index:06d}.bin")
            with open(args.output / "labels" / f"{index:06d}.bin", "wb") as labels:
                labels.write(raw["tag"].astype("<u1").tobytes())
                labels.write(raw["instance"].astype("<u4").tobytes())

    # Odometry at every step: wheel speed of the reference point and gyro yaw rate.
    longitudinal = body_velocity[:, 0]
    lateral_reference = body_velocity[:, 1] + yaw_rate * offset
    wheel = args.wheel_speed_scale * longitudinal + rng.normal(0.0, args.wheel_speed_noise, len(longitudinal))
    gyro = -as_float(ticks["imu_gz"])  # IMU z is left-handed
    with open(args.output / "motion.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["step", "time_s", "wheel_speed_mps", "gyro_yaw_rate_radps",
                         "gt_longitudinal_mps", "gt_lateral_reference_mps", "gt_yaw_rate_radps",
                         "steer_fl_deg", "steer_fr_deg"])
        for k in range(len(longitudinal)):
            writer.writerow([ticks["step"][k], ticks["time_s"][k], f"{wheel[k]:.6f}", f"{gyro[k]:.8f}",
                             f"{longitudinal[k]:.6f}", f"{lateral_reference[k]:.6f}", f"{yaw_rate[k]:.8f}",
                             ticks["steer_fl_deg"][k], ticks["steer_fr_deg"][k]])
    summary = {"capture": str(args.capture), "sweeps": len(frames["lidar_index"]), "steps": len(longitudinal),
               "rangeNoiseM": args.range_noise, "wheelSpeedNoiseMps": args.wheel_speed_noise,
               "wheelSpeedScale": args.wheel_speed_scale, "seed": args.seed, "referenceOffsetM": offset,
               "gyroBiasRadps": float(np.mean(gyro[np.abs(yaw_rate) < 1e-3] - yaw_rate[np.abs(yaw_rate) < 1e-3]))
               if np.any(np.abs(yaw_rate) < 1e-3) else None,
               "gyroMinusTruthRmsRadps": float(np.sqrt(np.nanmean((gyro - yaw_rate) ** 2))),
               "maxLateralSpeedAtReferenceMps": float(np.max(np.abs(lateral_reference)))}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
