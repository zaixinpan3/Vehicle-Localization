#!/usr/bin/env python3
"""Run independent GPU GLIM on prepared bags and export frame-indexed references.

Dependencies: numpy, scipy, json5. GLIM runs in its official Docker image.
The source/config directory supplies versioned upstream default parameters.
Generated data, maps and third-party dependencies are never staged as code.
"""
import argparse
import csv
import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

import json5
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.io import savemat


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


IMPLEMENTATION_SHA256 = sha256(Path(__file__))


def configure(upstream, overrides, destination):
    destination.mkdir(exist_ok=False)
    selected = json.loads(overrides.read_text())
    for source in sorted(upstream.glob("*.json")):
        parameters = json5.loads(source.read_text())
        for section, update in selected.get(source.name, {}).items():
            parameters[section].update(update)
        (destination / source.name).write_text(json.dumps(parameters, indent=2) + "\n")
    validate_time_contract(destination)
    return {p.name: sha256(p) for p in destination.glob("*.json")}


def validate_time_contract(configuration):
    """GLIM ROS converts our UINT32 t to seconds before TimeKeeper.

    Applying another nanosecond scale silently disables practical deskew.
    This runner consumes only bags from prepareOfflineReferenceBag.py, whose
    cloud time representation is UINT32 nanoseconds and scan-start relative.
    """
    sensors = json.loads((configuration / "config_sensors.json").read_text())["sensors"]
    if sensors.get("global_shutter_lidar") is not False:
        raise ValueError("Prepared spinning Ouster scans require global_shutter_lidar=false for deskew")
    if sensors.get("autoconf_perpoint_times") is not False:
        raise ValueError("Prepared Ouster timing requires explicit relative-time settings")
    if sensors.get("perpoint_relative_time") is not True:
        raise ValueError("Prepared Ouster t must remain relative to scan-start headers")
    if sensors.get("perpoint_time_scale") != 1.0:
        raise ValueError("GLIM ROS already converts UINT32 t from ns to seconds; perpoint_time_scale must be 1.0")


def export_reference(prepared, output, estimator=None, log_name="glim.log"):
    path = output / "dump/traj_lidar.txt"
    trajectory = np.loadtxt(path, ndmin=2)
    if trajectory.shape[1] != 8 or not np.isfinite(trajectory).all():
        raise ValueError("Malformed TUM trajectory")
    if len(trajectory) < 2 or not np.all(np.diff(trajectory[:, 0]) > 0):
        raise ValueError("Too few estimates or nonmonotonic trajectory")
    norms = np.linalg.norm(trajectory[:, 4:8], axis=1)
    if np.max(np.abs(norms - 1)) > 1e-3:
        raise ValueError("Invalid orientation quaternions")
    times = trajectory[:, 0]
    preparation = json.loads((prepared / "preparation.json").read_text())
    with (prepared / "frames.csv").open() as stream:
        frames = list(csv.DictReader(stream))
    prepared_count = len(frames)
    known = {int(frame["frame_index"]): frame for frame in frames}
    rejected = {int(frame["frame_index"]): frame for frame in preparation["rejected_frames"]}
    if set(known) & set(rejected) or set(known) | set(rejected) != set(range(1, preparation["frames_seen"] + 1)):
        raise ValueError("Prepared/rejected frame identities do not reconcile with the original recording")
    frames = [known.get(i, {"frame_index": i, **rejected.get(i, {})}) for i in range(1, preparation["frames_seen"] + 1)]
    rows = []
    for frame in frames:
        stamp = int(frame["native_start_ns"]) * 1e-9 if "native_start_ns" in frame else float("nan")
        index = int(np.searchsorted(times, stamp)) if np.isfinite(stamp) else 0
        candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
        best = min(candidates, key=lambda i: abs(times[i] - stamp))
        exact = np.isfinite(stamp) and abs(times[best] - stamp) < 1e-5
        row = dict(frame_index=int(frame["frame_index"]), native_time_sec=stamp,
                   original_header_time_sec=int(frame["original_header_ns"]) * 1e-9 if "original_header_ns" in frame else float("nan"),
                   original_bag_time_sec=int(frame["original_bag_ns"]) * 1e-9 if "original_bag_ns" in frame else float("nan"),
                   estimated=int(exact), reference_kind="pseudo_ground_truth",
                   pose_frame="front_ouster", quality="estimated_unverified" if exact else "not_estimated")
        if not np.isfinite(stamp):
            row["quality"] = "timing_rejected_not_estimated"
        for key in ["x_m", "y_m", "z_m", "qx", "qy", "qz", "qw", "roll_rad", "pitch_rad", "yaw_rad"]:
            row[key] = ""
        if exact:
            pose = trajectory[best]
            for key, value in zip(["x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"], pose[1:]):
                row[key] = float(value)
            angles = Rotation.from_quat(pose[4:]).as_euler("xyz")
            row.update(zip(["roll_rad", "pitch_rad", "yaw_rad"], map(float, angles)))
        rows.append(row)
    with (output / "reference_poses.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    shutil.copyfile(path, output / "reference_lidar.tum")
    dt = np.diff(times)
    steps = np.linalg.norm(np.diff(trajectory[:, 1:4], axis=0), axis=1)
    speeds = steps / dt
    log = (output / log_name).read_text(errors="replace")
    fatal_terms = ["CUDA error", "out of memory", "indeterminant linear system", "corruption", "segmentation fault",
                   "an exception was caught", "inconsistent arguments"]
    issues = [term for term in fatal_terms if term.lower() in log.lower()]
    if issues:
        for row in rows:
            if row["estimated"]:
                row["quality"] = "solver_issue_do_not_score"
        with (output / "reference_poses.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    positions = np.full((len(rows), 3), np.nan)
    quaternions = np.full((len(rows), 4), np.nan)
    for index, row in enumerate(rows):
        if row["estimated"]:
            positions[index] = [row[key] for key in ["x_m", "y_m", "z_m"]]
            quaternions[index] = [row[key] for key in ["qx", "qy", "qz", "qw"]]
    savemat(output / "reference_poses.mat", {
        "frameIndex": np.array([r["frame_index"] for r in rows]),
        "nativeTimeSec": np.array([r["native_time_sec"] for r in rows]),
        "originalHeaderTimeSec": np.array([r["original_header_time_sec"] for r in rows]),
        "originalBagTimeSec": np.array([r["original_bag_time_sec"] for r in rows]),
        "estimatedMask": np.array([bool(r["estimated"]) for r in rows]),
        "positionMeters": positions, "quaternionXYZW": quaternions,
        "poseFrame": "front_ouster", "referenceKind": "pseudo_ground_truth",
        "solverIssueDetected": bool(issues), "externalAccuracyVerified": False,
        "navigationPoseUsedForEstimation": preparation.get("receiver_used_for_sensor_calibration", False),
    }, do_compression=True, oned_as="column")
    cautions = {term: log.lower().count(term.lower()) for term in
                ["IMU prediction is not good", "IMU data are noisy", "small overlap",
                 "insufficient IMU", "time stamp is too old", "an exception was caught"]}
    if estimator is None:
        configuration = output / "config/config.json"
        selected = json.loads(configuration.read_text()) if configuration.exists() else {}
        frontend = str(selected)
        estimator = "GLIM continuous-time LiDAR global mapping" if "odometry_ct" in frontend else "GLIM GPU range-inertial global mapping"
    summary = dict(reference_kind="pseudo_ground_truth", estimator=estimator,
                   frames_prepared=prepared_count, frames_output=len(rows), frames_estimated=sum(r["estimated"] for r in rows),
                   missing_frame_indices=[r["frame_index"] for r in rows if not r["estimated"]],
                   source_frames_seen=preparation["frames_seen"],
                   input_timing_rejections=preparation["rejected_frames"],
                   trajectory_records=len(trajectory), path_length_m=float(steps.sum()),
                   maximum_step_speed_mps=float(speeds.max()),
                   maximum_estimate_interval_sec=float(dt.max()),
                   logged_solver_issues=issues, external_accuracy_verified=False,
                   warning_counts=cautions,
                   global_coordinate_frame="arbitrary local map; no geodetic anchors; consult estimator configuration for gravity convention",
                   output_reference_point="front LiDAR origin; not a surveyed vehicle/INS origin",
                   navigation_pose_used_for_estimation=preparation.get("receiver_used_for_sensor_calibration", False),
                   receiver_used_for_sensor_calibration=preparation.get("receiver_used_for_sensor_calibration", False),
                   qualification="Review solver issues, coverage, dynamic objects, calibration and independent validation before scoring fine accuracy",
                   hashes={p.name: sha256(p) for p in [output / "reference_poses.csv", output / "reference_lidar.tum"]})
    if "imu_calibration" in preparation:
        summary["imu_calibration"] = preparation["imu_calibration"]
        summary["imu_calibration_sha256"] = preparation["imu_calibration_sha256"]
    (output / "quality.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def run(args):
    docker = (["sudo", "-n"] if args.sudo else []) + [args.docker]
    if args.docker_host:
        docker += ["-H", args.docker_host]
    image = subprocess.check_output(docker + ["image", "inspect", args.image,
                                            "--format", "{{if .RepoDigests}}{{index .RepoDigests 0}}{{else}}{{.Id}}{{end}}"], text=True).strip()
    status_path = args.output_root / "batch_status.json"
    args.output_root.mkdir(parents=True, exist_ok=True)
    statuses = json.loads(status_path.read_text()) if status_path.exists() else []
    for prep in sorted(args.prepared_root.glob("*/*/preparation.json")):
        prepared = prep.parent.resolve()
        if args.only and args.only not in str(prepared):
            continue
        relative = prepared.relative_to(args.prepared_root.resolve())
        output = args.output_root.resolve() / relative
        if (output / "quality.json").exists():
            continue
        if (output / "run.json").exists():
            previous = json.loads((output / "run.json").read_text())
            if previous.get("status") == "failed":
                print(f"{relative}: preserving failed run; use a new output root to retry", flush=True)
                continue
        output.mkdir(parents=True, exist_ok=False)
        config_hashes = configure(args.glim_config_root, args.overrides, output / "config")
        gpu_args = ["--gpus", "all"]
        if args.gpu_driver_dir:
            gpu_args = ["--device", "/dev/nvidia0", "--device", "/dev/nvidiactl",
                        "--device", "/dev/nvidia-uvm",
                        "-v", f"{args.gpu_driver_dir.resolve()}:/hostgpu:ro",
                        "-e", "LD_LIBRARY_PATH=/hostgpu:/usr/local/cuda/lib64:/usr/local/lib:/opt/ros/jazzy/lib"]
        command = docker + ["run", "--rm", "--network", "none", "--ipc", "private", "--shm-size", "2g"] + gpu_args + [
                            "-v", f"{prepared}/rosbag2:/input:ro",
                            "-v", f"{output}:/result", "-e", "HOME=/tmp",
                            image, "ros2", "run", "glim_ros", "glim_rosbag", "/input",
                            "--ros-args", "-p", "config_path:=/result/config",
                            "-p", "auto_quit:=true", "-p", "dump_path:=/result/dump"]
        record = dict(prepared=str(prepared), output=str(output), image_digest=image,
                      implementation_sha256=IMPLEMENTATION_SHA256,
                      config_hashes=config_hashes, override_sha256=sha256(args.overrides), command=command,
                      started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"), status="running")
        (output / "run.json").write_text(json.dumps(record, indent=2) + "\n")
        print(f"Running {relative}", flush=True)
        start = time.monotonic()
        with (output / "glim.log").open("w") as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
        # The official image's entrypoint sources a workspace under /root.
        # Restore only this run's output ownership after the root container exits.
        subprocess.run(docker + ["run", "--rm", "--network", "none", "--entrypoint", "chown",
                                  "-v", f"{output}:/result", image, "-R",
                                  f"{os.getuid()}:{os.getgid()}", "/result"], check=True)
        record.update(exit_code=result.returncode, elapsed_seconds=time.monotonic() - start)
        try:
            if result.returncode:
                raise RuntimeError(f"GLIM exited {result.returncode}; see glim.log")
            quality = export_reference(prepared, output)
            record.update(status="completed", quality=quality)
        except Exception as error:
            record.update(status="failed", error=str(error))
        (output / "run.json").write_text(json.dumps(record, indent=2) + "\n")
        statuses.append(record)
        status_path.write_text(json.dumps(statuses, indent=2) + "\n")
        print(f"{relative}: {record['status']} in {record['elapsed_seconds']:.1f}s", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument("--prepared-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--glim-config-root", type=Path, required=True)
    parser.add_argument("--overrides", type=Path, default=root / "config/offlineReferenceConfig.json")
    parser.add_argument("--docker", default="docker")
    parser.add_argument("--docker-host")
    parser.add_argument("--sudo", action="store_true")
    parser.add_argument("--gpu-driver-dir", type=Path,
                        help="Optional isolated NVIDIA driver libraries when container toolkit is unavailable")
    parser.add_argument("--image", default="koide3/glim_ros2:jazzy_cuda13.1")
    parser.add_argument("--only", help="Process paths containing this substring")
    run(parser.parse_args())
