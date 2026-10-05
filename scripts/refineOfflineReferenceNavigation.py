#!/usr/bin/env python3
"""Batch SE(3) smoothing of independent LiDAR references with NovAtel anchors.

Requires a separately recorded timing/extrinsic calibration. Receiver anchors
are observations with error, not ground truth. Optional held-out time blocks
remove both position and attitude anchors, including a one-second guard band.
The project localization estimator is never called. Dependencies: rosbags,
numpy, scipy, json5. All poses refer to the front LiDAR, not a vehicle axle.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat, savemat
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation, Slerp

from offlineReferenceNavigation import read_navigation
from runOfflineReference import export_reference, sha256

IMPLEMENTATION_SHA256 = sha256(Path(__file__))


def integrate_gyro(times, gyro_times, gyro, bias):
    """Integrate body-frame angular velocity on the physical device clock."""
    if times[0] < gyro_times[0] or times[-1] > gyro_times[-1]:
        raise ValueError("Gyro integration cannot extrapolate beyond acquired data")
    result = []
    for begin, end in zip(times[:-1], times[1:]):
        selected = gyro_times[(gyro_times > begin) & (gyro_times < end)]
        nodes = np.r_[begin, selected, end]
        samples = np.column_stack([np.interp(nodes, gyro_times, gyro[:, k]) for k in range(3)]) - bias
        increments = (samples[1:] + samples[:-1]) * .5 * np.diff(nodes)[:, None]
        rotation = Rotation.identity()
        for increment in increments:
            rotation = rotation * Rotation.from_rotvec(increment)
        result.append(rotation.as_quat())
    return Rotation.from_quat(np.asarray(result))


def solve_poses(trajectory, targets, anchor_mask, translation_sigma=.03, rotation_sigma=np.deg2rad(.05), max_nfev=200,
                gyro_relative=None, gyro_sigma=np.deg2rad(.02)):
    """Optimize world-frame pose corrections using adjacent SE(3) factors."""
    if min(translation_sigma, rotation_sigma, gyro_sigma) <= 0:
        raise ValueError("Motion factor sigmas must be positive")
    count = len(trajectory)
    initial_rotation = Rotation.from_quat(trajectory[:, 4:])
    initial_position = trajectory[:, 1:4]
    measurements = initial_rotation[:-1].inv().apply(np.diff(initial_position, axis=0))
    relative_rotation = initial_rotation[:-1].inv() * initial_rotation[1:]
    anchors = np.flatnonzero(anchor_mask)
    if len(anchors) < 3:
        raise ValueError("At least three usable navigation anchors required")
    edge_size = 6 if gyro_relative is None else 9
    total_rows = edge_size * (count - 1) + 6 * len(anchors)
    sparsity = lil_matrix((total_rows, count * 6), dtype=np.int8)
    for i in range(count - 1):
        sparsity[edge_size*i:edge_size*i+edge_size, 6*i:6*i+12] = 1
    start = edge_size * (count - 1)
    for j, i in enumerate(anchors):
        sparsity[start + 6*j:start + 6*j+6, 6*i:6*i+6] = 1

    def poses(x):
        delta = x.reshape(count, 6)
        return initial_position + delta[:, :3], Rotation.from_rotvec(delta[:, 3:]) * initial_rotation

    def residual(x):
        position, rotation = poses(x)
        increment_position = (rotation[:-1].inv().apply(np.diff(position, axis=0)) - measurements) / translation_sigma
        increment_rotation = (relative_rotation.inv() * rotation[:-1].inv() * rotation[1:]).as_rotvec() / rotation_sigma
        receiver_position = position[anchors] + rotation[anchors].apply(targets["lever_lidar_to_ins"])
        absolute_position = (receiver_position - targets["position"][anchors]) / targets["sigma"][anchors]
        absolute_rotation = (targets["rotation"][anchors].inv() * rotation[anchors]).as_rotvec() / targets["attitude_sigma"][anchors]
        edges = [increment_position, increment_rotation]
        if gyro_relative is not None:
            scale = gyro_sigma * np.sqrt(np.diff(trajectory[:, 0]) / .1)
            edges.append((gyro_relative.inv() * rotation[:-1].inv() * rotation[1:]).as_rotvec() / scale[:, None])
        terms = [np.column_stack(edges).ravel()]
        terms.append(np.column_stack([absolute_position, absolute_rotation]).ravel())
        return np.concatenate(terms)

    fit = least_squares(residual, np.zeros(count * 6), jac_sparsity=sparsity.tocsr(),
                        method="trf", loss="soft_l1", f_scale=2., x_scale="jac",
                        tr_options=dict(atol=1e-8, btol=1e-8, maxiter=5000),
                        max_nfev=max_nfev, ftol=1e-9, xtol=1e-9, gtol=1e-5)
    position, rotation = poses(fit.x)
    output = np.column_stack([trajectory[:, 0], position, rotation.as_quat()])
    return output, dict(success=bool(fit.success), message=fit.message, evaluations=fit.nfev,
                        initial_quadratic_cost=float(np.sum(residual(np.zeros(count * 6))**2)/2),
                        robust_initial_cost=float(4*np.sum(np.sqrt(1+(residual(np.zeros(count * 6))/2)**2)-1)),
                        robust_final_cost=float(fit.cost),
                        optimality=float(fit.optimality), anchor_count=len(anchors),
                        increment_translation_sigma_m=translation_sigma, increment_rotation_sigma_deg=float(np.rad2deg(rotation_sigma)),
                        calibrated_gyro_factors=gyro_relative is not None)


def refine(args):
    calibration = json.loads(args.calibration.read_text())
    noise = json.loads(args.config.read_text())
    model_fields = [k for k in noise if k.endswith(("_m", "_deg", "_100ms", "_seconds"))]
    if any(not np.isfinite(noise[k]) or noise[k] <= 0 for k in model_fields):
        raise ValueError("Measurement-model sigmas, intervals and guards must be finite and positive")
    if not calibration.get("success") or calibration.get("bound_hit"):
        raise ValueError("Successful calibration without a physical search-bound hit is required")
    if not np.isfinite(np.r_[calibration["rotation_lidar_to_ins_xyzw"], calibration["lever_lidar_to_ins_m"],
                             calibration["nav_query_offset_seconds"]]).all():
        raise ValueError("Calibration contains nonfinite parameters")
    preparation = json.loads((args.prepared / "preparation.json").read_text())
    if Path(preparation["source_bag"]).resolve() != args.bag.resolve():
        raise ValueError("Navigation bag must match the prepared sensor recording")
    source_quality = json.loads((args.reference / "quality.json").read_text())
    if source_quality.get("logged_solver_issues"):
        raise ValueError("Cannot refine a LiDAR reference with solver issues")
    nav = read_navigation(args.bag)
    trajectory = np.loadtxt(args.reference / "reference_lidar.tum", ndmin=2)
    query = trajectory[:, 0] + calibration["nav_query_offset_seconds"]
    covered = (query >= nav["times"][0]) & (query <= nav["times"][-1])
    trajectory, query = trajectory[covered], query[covered]
    nav_rotation = Slerp(nav["times"], Rotation.from_quat(nav["quaternion"]))(query)
    extrinsic = Rotation.from_quat(calibration["rotation_lidar_to_ins_xyzw"])
    lever = np.asarray(calibration["lever_lidar_to_ins_m"])
    target_position = np.column_stack([np.interp(query, nav["times"], nav["xyz"][:, k]) for k in range(3)])
    target_rotation = nav_rotation * extrinsic
    usable = np.interp(query, nav["times"], nav["good"].astype(float)) > .999
    elapsed = trajectory[:, 0] - trajectory[0, 0]
    withheld, guarded = np.zeros(len(query), dtype=bool), np.zeros(len(query), dtype=bool)
    for begin, end in args.holdout:
        if not 0 <= begin < end <= elapsed[-1]:
            raise ValueError("Holdout must lie within the covered trajectory")
        withheld |= (elapsed >= begin) & (elapsed <= end)
        guarded |= (elapsed >= begin - noise["holdout_guard_seconds"]) & (elapsed <= end + noise["holdout_guard_seconds"])
    anchors = usable & ~guarded
    # Held-out navigation must also be excluded from initialization. Attitude
    # fixes the map rotation without an ill-conditioned straight-road fit.
    world_rotation = (target_rotation[anchors] * Rotation.from_quat(trajectory[anchors, 4:]).inv()).mean()
    position = world_rotation.apply(trajectory[:, 1:4])
    orientation = world_rotation * Rotation.from_quat(trajectory[:, 4:])
    translation = np.median((target_position - orientation.apply(lever) - position)[anchors], axis=0)
    trajectory[:, 1:4], trajectory[:, 4:] = position + translation, orientation.as_quat()
    # Roughly 1 Hz avoids pretending that correlated 50 Hz INS solutions are
    # fifty independent satellite measurements per second.
    candidates = np.flatnonzero(anchors)
    anchors[:] = False
    last = -np.inf
    for index in candidates:
        if trajectory[index, 0] - last >= noise["receiver_anchor_interval_seconds"]:
            anchors[index], last = True, trajectory[index, 0]
    sigma = np.column_stack([np.interp(query, nav["times"], nav["sigma"][:, k]) for k in range(3)])
    sigma = np.sqrt(sigma**2 + noise["receiver_position_floor_m"]**2)
    # Isotropic attitude precision is conservative under the unspecified
    # receiver output-body convention and uncertain mounting rotation.
    attitude_sigma = np.column_stack([np.interp(query, nav["times"], nav["attitude_sigma"][:, k]) for k in range(3)])
    attitude_sigma[:] = np.maximum(np.max(attitude_sigma, axis=1)[:, None], np.deg2rad(noise["receiver_attitude_floor_deg"]))
    targets = dict(position=target_position, rotation=target_rotation, sigma=sigma,
                   attitude_sigma=attitude_sigma, lever_lidar_to_ins=lever)
    gyro_relative, gyro_metadata = None, None
    if args.gyro_factor:
        nav_r = Rotation.from_quat(nav["quaternion"])
        rate_times = (nav["times"][1:] + nav["times"][:-1]) * .5
        rates = (nav_r[:-1].inv() * nav_r[1:]).as_rotvec() / np.diff(nav["times"])[:, None]
        rates = savgol_filter(rates, 25, 2, axis=0)
        expected = extrinsic.inv().apply(np.column_stack([np.interp(query, rate_times, rates[:, k]) for k in range(3)]))
        measured = np.column_stack([np.interp(trajectory[:, 0], nav["gyro_times"], nav["gyro"][:, k]) for k in range(3)])
        # The one-second guard exceeds the differentiated attitude filter's
        # support. No hidden receiver orientation enters the bias estimate.
        bias = np.median((measured - expected)[usable & ~guarded], axis=0)
        gyro_relative = integrate_gyro(trajectory[:, 0], nav["gyro_times"], nav["gyro"], bias)
        gyro_metadata = dict(gyro_bias_lidar_radps=bias.tolist(),
                             bias_estimation="Median raw gyro minus INS attitude rate, only outside held-out blocks and guards",
                             physical_clock_used=True, receiver_holdout_guard_seconds=noise["holdout_guard_seconds"])
    if args.gyro_factor and noise["holdout_guard_seconds"] < .5:
        raise ValueError("Gyro bias estimation requires a guard wider than the attitude differentiation filter support")
    result, solver = solve_poses(trajectory, targets, anchors, max_nfev=args.max_nfev, gyro_relative=gyro_relative,
        translation_sigma=noise["increment_translation_sigma_m"], rotation_sigma=np.deg2rad(noise["increment_rotation_sigma_deg"]),
        gyro_sigma=np.deg2rad(noise["gyro_increment_sigma_deg_at_100ms"]))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "dump").mkdir()
    np.savetxt(args.output / "dump/traj_lidar.txt", result, fmt="%.12f")
    (args.output / "navigation.log").write_text(json.dumps(solver, indent=2) + "\n")
    quality = export_reference(args.prepared, args.output, estimator="Independent LiDAR SE3 batch smoother with calibrated NovAtel anchors", log_name="navigation.log")
    disagreement = np.linalg.norm(result[:, 1:4] + Rotation.from_quat(result[:, 4:]).apply(lever) - target_position, axis=1)
    yaw_disagreement = (target_rotation.inv() * Rotation.from_quat(result[:, 4:])).magnitude()
    diagnostic = {}
    for name, mask in [("anchors", anchors), ("withheld", withheld & usable), ("all_receiver", usable)]:
        diagnostic[name] = dict(samples=int(mask.sum()),
                                position_discrepancy_rmse_m=float(np.sqrt(np.mean(disagreement[mask]**2))) if mask.any() else None,
                                orientation_discrepancy_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(yaw_disagreement[mask]**2)))) if mask.any() else None)
    initial_disagreement = np.linalg.norm(trajectory[:, 1:4] + Rotation.from_quat(trajectory[:, 4:]).apply(lever) - target_position, axis=1)
    baseline_mask = withheld & usable
    diagnostic["initial_same_withheld_frames"] = dict(samples=int(baseline_mask.sum()),
        position_discrepancy_rmse_m=float(np.sqrt(np.mean(initial_disagreement[baseline_mask]**2))) if baseline_mask.any() else None,
        alignment="Same attitude and translation initialization; all held-out navigation excluded")
    quality.update(navigation_pose_used_for_estimation=True, receiver_used_for_sensor_calibration=True, solver=solver,
                   global_coordinate_frame="WGS84 ENU at first INSPVA position", enu_origin_lla=nav["origin"].tolist(),
                   reference_accuracy_certified=False, reference_inputs_independent_of_project_localization=True,
                   receiver_orientation_convention="NovAtel output body: X right, Y forward, Z up (configured body not surveyed)",
                   receiver_comparison=diagnostic, clock_model=nav["clock"],
                   calibration=calibration, calibration_sha256=sha256(args.calibration),
                   initial_trajectory_sha256=sha256(args.reference / "reference_lidar.tum"),
                   initial_reference_path=str(args.reference.resolve()),
                   source_bag_sha256=preparation["source_bag_sha256"],
                   heldout_blocks_elapsed_seconds=args.holdout,
                   holdout_calibration_sequence_disjoint=calibration.get("calibration_sequence") != args.bag.stem,
                   gyro_integration=gyro_metadata,
                   implementation_sha256=IMPLEMENTATION_SHA256,
                   measurement_model=noise, measurement_model_sha256=sha256(args.config),
                   qualification="Provisional reference; fitted receiver discrepancies cannot establish absolute accuracy. Use withheld blocks only for independent receiver diagnostics; shared-sensor errors remain.")
    if not solver["success"]:
        quality["qualification"] += " Solver convergence not reached; do not use this candidate as the preferred reference."
        quality["logged_solver_issues"].append("Navigation batch solver did not converge")
    receiver_poses = np.column_stack([result[:, 0], result[:, 1:4] + Rotation.from_quat(result[:, 4:]).apply(lever),
                                     (Rotation.from_quat(result[:, 4:]) * extrinsic.inv()).as_quat()])
    np.savetxt(args.output / "reference_receiver_origin.tum", receiver_poses, fmt="%.12f")
    forward_left = Rotation.from_quat(receiver_poses[:, 4:]) * Rotation.from_euler("z", 90, degrees=True)
    forward_poses = np.column_stack([receiver_poses[:, :4], forward_left.as_quat()])
    np.savetxt(args.output / "reference_forward_left_up_at_receiver_origin.tum", forward_poses, fmt="%.12f")
    mat = {k: v for k, v in loadmat(args.output / "reference_poses.mat").items() if not k.startswith("__")}
    with (args.output / "reference_poses.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    flags = np.zeros((len(rows), 3), dtype=bool)
    for index, row in enumerate(rows):
        stamp = float(row["native_time_sec"])
        j = int(np.argmin(np.abs(result[:, 0] - stamp)))
        if bool(int(row["estimated"])) and abs(result[j, 0] - stamp) < 1e-5:
            flags[index] = [anchors[j], withheld[j], usable[j]]
            if not usable[j]:
                row["quality"] = "estimated_receiver_ins_free"
            if not solver["success"]:
                row["quality"] = "solver_issue_do_not_score"
        row.update(navigation_anchor_used=int(flags[index, 0]), navigation_withheld=int(flags[index, 1]),
                   receiver_ins_good=int(flags[index, 2]))
    with (args.output / "reference_poses.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    mat.update(navigationPoseUsedForEstimation=True, enuOriginLLA=nav["origin"],
               receiverOriginSurveyed=False, calibrationSurveyed=False,
               receiverOriginTUM=receiver_poses, calibrationLidarToInsQuaternionXYZW=extrinsic.as_quat(),
               receiverOrientationConvention="NovAtel body X right Y forward Z up",
               forwardLeftUpAtReceiverOriginTUM=forward_poses,
               navigationAnchorUsed=flags[:, 0, None], receiverWithheldMask=flags[:, 1, None],
               receiverINSGoodMask=flags[:, 2, None], solverIssueDetected=not solver["success"],
               calibrationLidarToInsLeverMeters=lever)
    savemat(args.output / "reference_poses.mat", mat, do_compression=True)
    np.savetxt(args.output / "navigation_usage.csv", np.column_stack([result[:, 0], anchors, withheld, usable, disagreement]),
               delimiter=",", header="native_time_sec,anchor_used,withheld,ins_good,position_discrepancy_m", comments="", fmt="%.9f")
    quality["hashes"] = {name: sha256(args.output / name) for name in
                         ["reference_lidar.tum", "reference_poses.csv", "reference_poses.mat", "navigation_usage.csv"]}
    (args.output / "quality.json").write_text(json.dumps(quality, indent=2) + "\n")
    print(json.dumps(quality, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ["bag", "prepared", "reference", "calibration", "output"]:
        parser.add_argument("--" + key, type=Path, required=True)
    parser.add_argument("--holdout", nargs=2, type=float, action="append", default=[], metavar=("START_SEC", "END_SEC"))
    parser.add_argument("--max-nfev", type=int, default=200)
    parser.add_argument("--gyro-factor", action="store_true", help="Fuse physical-clock gyro increments; estimate its bias outside holdouts")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parents[1] / "config/offlineReferenceNavigationConfig.json")
    refine(parser.parse_args())
