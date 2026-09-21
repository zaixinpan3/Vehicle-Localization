"""Audit frame 307 timing independently of the matching pose.

Run from the repository root with:
uv run --offline --with numpy --with matplotlib --with pyproj \
    python research/frame307_matching_diagnosis_20260921/analyze_clock.py

Alternate clock bridges are diagnostic hypotheses, not replacement ground truth.
No fit below uses a spatial pose, a map residual, or the matching result.
"""
from pathlib import Path
import csv
import hashlib
import json
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyproj import Proj, Transformer

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from prepareInspvaMappingPoses import quaternion_from_rpy, interpolate_quaternions

OUT = ROOT / "output/frame307_matching_diagnosis_20260921"
OUT.mkdir(parents=True, exist_ok=True)
STEM = "raw_data_2024-06-07-12-09-31_0"
RAW = ROOT / "data/raw/Missisipi/gnss"


def read(name):
    return np.genfromtxt(name, delimiter=",", names=True, dtype=None, encoding="utf-8")


def export(name, rows):
    with (OUT / name).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def robust_clock(ros, receiver, center, half_width):
    """Local affine timestamp-only fit with median/MAD residual rejection."""
    selected = np.abs(ros - center) <= half_width
    x, y = ros[selected] - center, receiver[selected]
    keep = np.ones(len(x), dtype=bool)
    for _ in range(10):
        slope, offset = np.polyfit(x[keep], y[keep], 1)
        residual = y - (slope * x + offset)
        median = np.median(residual)
        mad = np.median(np.abs(residual - median))
        updated = np.abs(residual - median) <= max(0.001, 3 * 1.4826 * mad)
        if np.array_equal(keep, updated):
            break
        keep = updated
    slope, offset = np.polyfit(x[keep], y[keep], 1)
    return float(offset), dict(nativeSamples=len(x), retainedSamples=int(keep.sum()),
                               receiverSecondsPerRosSecond=float(slope))


def main():
    ins_path = RAW / f"{STEM}_inspva.csv"
    lidar_path = RAW / f"{STEM}_front_lidar_points.csv"
    pose_path = RAW / f"{STEM}_front_lidar_inspva_pose_1_1170.csv"
    ins, lidar, poses = read(ins_path), read(lidar_path), read(pose_path)
    verification = json.loads((OUT / "verification.json").read_text())
    ros = ins["stamp_sec"] - ins["stamp_sec"][0]
    receiver = ins["gps_seconds"] - ins["gps_seconds"][0]
    query = lidar["stamp_sec"] - ins["stamp_sec"][0]
    original_time = np.interp(query, ros, receiver)
    assert np.max(np.abs(original_time - original_time[0] - poses["receiver_time_sec"])) < 1e-10
    k = 306
    ref = np.asarray(verification["referencePose"])
    match = np.asarray(verification["matchingPose"])
    rot = np.array([[np.cos(ref[2]), -np.sin(ref[2])], [np.sin(ref[2]), np.cos(ref[2])]])
    east, north = Transformer.from_crs(4326, 32615, always_xy=True).transform(
        ins["longitude_deg"], ins["latitude_deg"])
    xyz = np.column_stack((east, north, ins["height_m"]))
    gamma = np.asarray(Proj("EPSG:32615").get_factors(ins["longitude_deg"], ins["latitude_deg"]).meridian_convergence)
    quaternions = quaternion_from_rpy(np.deg2rad(ins["roll_deg"]), -np.deg2rad(ins["pitch_deg"]),
                                    np.deg2rad(90 - ins["azimuth_deg"] + gamma))

    def native_pose(t):
        upper = int(np.searchsorted(receiver, t, side="right"))
        fraction = (t - receiver[upper - 1]) / (receiver[upper] - receiver[upper - 1])
        point = (1 - fraction) * xyz[upper - 1] + fraction * xyz[upper]
        q = interpolate_quaternions(quaternions[[upper - 1]], quaternions[[upper]], np.array([fraction]))[0]
        yaw = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        return np.r_[point[:2], yaw]

    assert np.max(np.abs(native_pose(original_time[k]) - ref)) < 1e-8
    candidates = {"original_piecewise_bridge": original_time[k]}
    fit_details = {}
    for radius in [1, 2, 3]:
        candidates[f"neighbor_scan_radius_{radius}"] = np.interp(
            query[k], query[[k - radius, k + radius]], original_time[[k - radius, k + radius]])
    for width in [0.3, 0.5, 1.0, 2.0]:
        name = f"robust_native_halfwidth_{width}"
        candidates[name], fit_details[name] = robust_clock(ros, receiver, query[k], width)
    rows = []
    for name, t in candidates.items():
        p = native_pose(t)
        shift = (p[:2] - ref[:2]) @ rot
        error = (match[:2] - p[:2]) @ rot
        rows.append(dict(method=name, receiverTimeFromFirstScan=t-original_time[0],
                         timeShiftMs=1000*(t-original_time[k]), referenceX=p[0], referenceY=p[1],
                         referenceYaw=p[2], referenceForwardShiftM=shift[0], referenceLeftShiftM=shift[1],
                         unchangedMatchErrorM=float(np.linalg.norm(error)),
                         unchangedMatchForwardErrorM=error[0], unchangedMatchLeftErrorM=error[1],
                         unchangedMatchYawErrorDeg=float(np.rad2deg(match[2]-p[2]))))
    export("clock_controls.csv", rows)
    nominal = native_pose(candidates["neighbor_scan_radius_1"])
    controls = read(OUT / "controls.csv")
    ablations = []
    for r in controls:
        ablations.append(dict(variant=r["variant"], originalErrorM=r["positionErrorM"],
                              clockDiagnosticErrorM=float(np.hypot(r["x"]-nominal[0], r["y"]-nominal[1])),
                              accepted=int(r["accepted"]), rank=int(r["rank"])))
    export("ablation_clock_comparison.csv", ablations)
    native_rows = []
    for j in np.flatnonzero(np.abs(ros-query[k])<0.3):
        native_rows.append(dict(index=int(ins["index"][j]), rosTimeFromInsOrigin=ros[j],
            receiverTimeFromInsOrigin=receiver[j], rosIntervalMs=1000*(ros[j]-ros[j-1]),
            receiverIntervalMs=1000*(receiver[j]-receiver[j-1]),
            bagMinusHeaderMs=1000*(ins["bag_time_sec"][j]-ins["stamp_sec"][j])))
    export("native_clock_neighborhood.csv", native_rows)
    neighborhood = []
    for j in range(295-1, 316-1):
        t = np.interp(query[j], query[[j-1,j+1]], original_time[[j-1,j+1]])
        neighborhood.append(dict(frame=j+1, originalTime=original_time[j]-original_time[0],
            neighborTime=t-original_time[0], clockDifferenceMs=1000*(t-original_time[j])))
    export("scan_clock_neighborhood.csv", neighborhood)
    speed = np.hypot(np.interp(original_time[k], receiver, ins["east_velocity_mps"]),
                     np.interp(original_time[k], receiver, ins["north_velocity_mps"]))
    audit = dict(frame=307, originalPositionErrorM=verification["positionErrorM"],
        originalForwardErrorM=float(((match[:2]-ref[:2])@rot)[0]),
        originalLeftErrorM=float(((match[:2]-ref[:2])@rot)[1]),
        nativeHorizontalSpeedMps=float(speed),
        neighborClockTimeShiftMs=rows[1]["timeShiftMs"],
        referenceDisplacementM=float(np.linalg.norm(nominal[:2]-ref[:2])),
        unchangedMatchingErrorAgainstNeighborClockM=rows[1]["unchangedMatchErrorM"],
        robustClockFits=fit_details,
        limitations=["Timestamp-only alternate bridges are sensitivity controls, not certified timing calibration.",
                     "The existing map and matching output are fixed; no corrected-map experiment was run.",
                     "INSPVA is a recorded reference, not independent ground truth; same-drive map remains in use.",
                     "No point-time deskew, extrinsic calibration, perception, or production algorithm was changed."])
    (OUT / "clock_audit.json").write_text(json.dumps(audit, indent=2)+"\n")
    make_figure(native_rows, ref, match, nominal, rot, ablations)
    paths = [ins_path, lidar_path, pose_path,
             ROOT / "output/saved_perception_inspva_20260915/experiment.mat",
             ROOT / "output/saved_perception_inspva_20260915/calls.csv",
             ROOT / "output/mississippi_mapping_inspva_20260915/probability_cloud.mat",
             ROOT / "output/mississippi_mapping_inspva_20260915/feature_observations.mat"]
    paths += sorted(OUT.glob("*.csv")) + sorted(OUT.glob("*.json")) + sorted(OUT.glob("*.png"))
    manifest = [dict(path=str(p.relative_to(ROOT)), bytes=p.stat().st_size,
                     sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in paths if p.name!="artifact_manifest.json"]
    (OUT / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps(audit, indent=2))
    for row in rows:
        print(row["method"], "time shift ms", row["timeShiftMs"], "error cm", row["unchangedMatchErrorM"]*100)


def make_figure(native_rows, ref, match, nominal, rot, ablations):
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), layout="constrained")
    ax = axes[0, 0]
    indices = [r["index"] for r in native_rows]
    ax.plot(indices, [r["rosIntervalMs"] for r in native_rows], "o-", label="ROS header interval")
    ax.plot(indices, [r["receiverIntervalMs"] for r in native_rows], label="INSPVA receiver interval")
    ax.set(xlabel="Native INSPVA sample", ylabel="Interval (ms)", title="A. Header timing has a local 54.93 ms gap")
    ax.legend(); ax.grid(alpha=.25)
    ax = axes[0, 1]
    points = np.stack([ref[:2], nominal[:2], match[:2]])
    local = (points-ref[:2]) @ rot * 100
    colors = ["#c33", "#287b3c", "#2661ad"]
    labels = ["Original reference", "Neighbor-clock reference (diagnostic)", "Unchanged map match"]
    for i in range(3):
        ax.scatter(local[i, 0], local[i, 1], c=colors[i], s=65, label=labels[i])
    ax.plot(local[[0, 2], 0], local[[0, 2], 1], color="#c33", linestyle="--")
    ax.plot(local[[1, 2], 0], local[[1, 2], 1], color="#287b3c")
    original_error = np.linalg.norm(match[:2]-ref[:2])*100
    control_error = np.linalg.norm(match[:2]-nominal[:2])*100
    ax.set(xlabel="Forward from original reference (cm)", ylabel="Left (cm)",
           title=f"B. Fixed result: {original_error:.2f} cm vs original; {control_error:.2f} cm vs timing control")
    ax.axis("equal"); ax.grid(alpha=.25); ax.legend(fontsize=8)
    ax = axes[1, 0]
    observations = read(OUT / "landmark_observations.csv")
    for target in np.unique(observations["target"]):
        selected = observations["target"]==target
        ax.plot(observations["frame"][selected], observations["forwardOffsetFromMapM"][selected]*100,
                "o-", label=f"Map component {target}")
    ax.axvline(307, color="black", linestyle="--")
    ax.set(xlabel="Frame", ylabel="Observed centroid minus map (forward cm)",
           title="C. Different landmarks shift together at frame 307")
    ax.legend(fontsize=8); ax.grid(alpha=.25)
    ax = axes[1, 1]
    x = np.arange(4)
    ax.bar(x-.18, [r["originalErrorM"]*100 for r in ablations[:4]], .36, label="Original reference")
    ax.bar(x+.18, [r["clockDiagnosticErrorM"]*100 for r in ablations[:4]], .36, label="Neighbor-clock diagnostic")
    ax.set(xticks=x, xticklabels=["All classes", "No curb", "No pole", "No sign"],
           ylabel="Position difference (cm)", title="D. Dropping a feature class does not remove the original spike")
    ax.legend(fontsize=8); ax.grid(axis="y", alpha=.25)
    fig.suptitle("Mississippi frame 307: historical geometric D2D match and independent clock sensitivity", fontsize=14)
    fig.savefig(OUT / "diagnosis.png", dpi=160)
    fig.savefig(OUT / "diagnosis.pdf")
    plt.close(fig)


if __name__ == "__main__":
    main()
