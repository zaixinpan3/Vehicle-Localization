#!/usr/bin/env python3
"""Build the evaluation and mapping reference of the 12-09-31 drive from receiver data.

Run from the repository root, in this order:
    uv run --offline --with numpy --with pyproj python scripts/buildReceiverReference.py detect
    matlab -batch "judgeReceiverContradictions"
    uv run --offline --with numpy --with pyproj python scripts/buildReceiverReference.py build

Rule (user decision of 2026-10-01, restated 2026-10-10): every value of the
reference comes from the receiver. Where the receiver contradicts itself,
vehicle sensors may decide which receiver quantity is trusted, but no vehicle
sensor value enters the reference. Where the receiver merely disagrees with
the vehicle sensors, the receiver stands. LiDAR is not used, not even as an
arbiter, because the map is built from the reference. Only the current
corrected trajectory is kept; superseded versions and the maps built from them
are deleted (user decision of 2026-10-10).

Self-contradiction tested here: the INSPVA position increment of each 20 ms
epoch against the integral of the INSPVA velocity of the same messages. The
two quantities refer to points of the vehicle that are not identical and the
receiver's grid velocity carries a small constant scale offset, so the test
first fits, on the whole drive, the rigid relation
    position speed - velocity speed, along  = c_a + r_y * yaw rate
    position speed - velocity speed, left   = c_l - r_x * yaw rate
(a lever arm r between the velocity point and the position point plus two
constants), and the velocity of the position point is modeled as the INSPVA
velocity plus these terms. What remains after the model is the contradiction:
stretches where the positions slew against the velocity at a constant rate for
about a second at a time. Candidates are stretches whose model-corrected
residual rate, averaged over 0.5 s, exceeds 0.06 m/s for at least 0.2 s;
neighbours closer than 1.0 s are merged and candidates whose position
excursion stays below 0.05 m are dropped.

Stage ``detect`` writes the candidates and the per-sample consistency table.
scripts/judgeReceiverContradictions.m then decides per candidate, from the
vehicle's dead-reckoned track, whether the receiver position or the receiver
velocity is trusted (verdict per segment: velocity, position, undecided), and
whether the modeled or the raw velocity integral is used. Stage ``build``
writes the corrected native reference and the corrected frame poses:
  Position. Out-and-back single-epoch glitches are repaired by interpolation.
    In a velocity-trusted segment the positions follow the integral of the
    trusted velocity, anchored on the receiver position at both ends of the
    segment; the closure (the position change the integral does not explain)
    is spread linearly over the segment. Positions outside segments, in
    position-trusted and in undecided segments are unchanged.
  Heading. On straight driving where the arbiter rejected sideslip, the
    receiver course replaces the azimuth (Gaussian 2 s smoothing of the
    course-minus-azimuth offset over supported samples); the correction is
    interpolated between supported samples and held beyond the first and
    last. Samples whose velocity is distrusted (position-trusted or undecided
    segments) do not enter the estimate.
  Tier of each mapping frame: A when the receiver reports INS RTK fixed with
    at least 5 solution satellites and at most 0.10 m horizontal standard
    deviation, the heading has straight support and the position is
    consistent or was resolved by the arbiter; B as A without the solution
    quality; C when the heading is bridged or extrapolated or the
    contradiction is undecided.
Outputs go to output/receiver_reference/. The reference is refined receiver
data, not surveyed truth.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys

import numpy as np
from pyproj import Proj

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/receiver_reference"
GNSS = ROOT / "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_"
POSES = Path(str(GNSS) + "front_lidar_synchronized_pose_1_1170.csv")
NATIVE = ROOT / "output/receiver_synchronized_inputs/native_reference.csv"
NATIVE_METADATA = ROOT / "output/receiver_synchronized_inputs/reference_metadata.json"

MOVING_SPEED = 3.0           # m/s, samples that enter the model fit and the test
CLIP_SIGMAS = 3.0            # robust fit of the lever-arm model
BOX_SECONDS = 0.5            # averaging window of the residual rate
RATE_THRESHOLD = 0.06        # m/s, model-corrected residual rate that counts as a contradiction
MINIMUM_DURATION = 0.2       # s, shortest stretch above the threshold
MERGE_GAP = 1.0              # s, neighbours closer than this form one candidate
PAD_SECONDS = 0.1            # s, margin added at both ends of a candidate
MINIMUM_EXCURSION = 0.05     # m, smallest position excursion that is a material contradiction
GLITCH_STEP = 0.03           # m in one 20 ms epoch not explained by the trusted velocity
MINIMUM_SPEED = 5.0          # m/s, straight-driving support for the heading
MAXIMUM_YAW_RATE = 1.0       # deg/s, straight-driving support
SMOOTHING_SECONDS = 2.0      # Gaussian scale of the heading estimate
MINIMUM_SUPPORT = 25.0       # kernel weight, about 0.5 s of straight samples
FIXED_TYPE = 56              # NovAtel INS_RTKFIXED
MINIMUM_SOLUTION_SATELLITES = 5
MAXIMUM_FIXED_STD = 0.10     # m


def read_rows(path):
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def write_rows(path, rows, fieldnames=None):
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames or list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_receiver():
    """Native INSPVA record on the receiver clock, its grid velocity and yaw rate."""
    inspva = np.genfromtxt(str(GNSS) + "inspva.csv", delimiter=",", names=True)
    native = np.genfromtxt(NATIVE, delimiter=",", names=True)
    assert len(inspva) == len(native), "Native reference and INSPVA log differ in length."
    time = native["time"]
    assert np.max(np.abs(np.diff(time) - np.diff(inspva["gps_seconds"]))) < 1e-8, "Native reference is not on the INSPVA epochs."
    factors = Proj("EPSG:32615").get_factors(inspva["longitude_deg"], inspva["latitude_deg"])
    gamma = np.deg2rad(np.asarray(factors.meridian_convergence))
    scale = np.asarray(factors.meridional_scale)
    east, north = inspva["east_velocity_mps"], inspva["north_velocity_mps"]
    velocity = scale[:, None] * np.column_stack((np.cos(gamma) * east - np.sin(gamma) * north,
                                                 np.sin(gamma) * east + np.cos(gamma) * north))
    position = np.column_stack((native["x"], native["y"]))
    yaw = np.unwrap(native["psi"])
    box = int(round(BOX_SECONDS / np.median(np.diff(time))))
    yaw_rate = np.convolve(np.gradient(yaw, time), np.ones(box) / box, mode="same")
    return dict(inspva=inspva, native=native, time=time, position=position, velocity=velocity,
                yaw=yaw, yaw_rate=yaw_rate, box=box)


def epoch_frames(r):
    """Unit vectors along and to the left of the mean velocity of each epoch."""
    v = (r["velocity"][:-1] + r["velocity"][1:]) / 2
    speed = np.hypot(v[:, 0], v[:, 1])
    along = v / np.maximum(speed, 1e-6)[:, None]
    left = np.column_stack((-along[:, 1], along[:, 0]))
    return v, speed, along, left


def fit_model(r):
    """Robust least squares of the residual rates on the yaw rate (moving samples only)."""
    dt = np.diff(r["time"])
    v, speed, along, left = epoch_frames(r)
    residual = np.diff(r["position"], axis=0) - v * dt[:, None]
    rate_along = (residual * along).sum(1) / dt
    rate_left = (residual * left).sum(1) / dt
    w = (r["yaw_rate"][:-1] + r["yaw_rate"][1:]) / 2
    moving = speed > MOVING_SPEED

    def robust(y):
        mask = moving.copy()
        for _ in range(6):
            design = np.column_stack((np.ones(mask.sum()), w[mask]))
            c = np.linalg.lstsq(design, y[mask], rcond=None)[0]
            e = y - (c[0] + c[1] * w)
            s = float(np.std(e[mask]))
            mask = moving & (np.abs(e) < CLIP_SIGMAS * s)
        return c, s, int(mask.sum())

    (c_a, r_y), s_a, n_a = robust(rate_along)
    (c_l, minus_r_x), s_l, n_l = robust(rate_left)
    model = dict(alongConstantMps=float(c_a), leftConstantMps=float(c_l),
                 leverArmBodyM=[float(-minus_r_x), float(r_y)],
                 inlierRateSdMps=[float(s_a), float(s_l)], inlierEpochs=[n_a, n_l],
                 movingEpochs=int(moving.sum()), description=(
                     "position speed minus velocity speed: along = c_a + r_y*yawRate, left = c_l - r_x*yawRate; "
                     "yaw rate in rad/s from the receiver azimuth; lever arm [r_x, r_y] in the body frame"))
    corrected_along = rate_along - (c_a + r_y * w)
    corrected_left = rate_left - (c_l + minus_r_x * w)
    return model, dict(dt=dt, speed=speed, along=along, left=left, yaw_rate=w, moving=moving,
                       rate_along=rate_along, rate_left=rate_left,
                       corrected_along=corrected_along, corrected_left=corrected_left,
                       model_velocity=modeled_velocity(r, model))


def modeled_velocity(r, model):
    """Velocity of the position point: INSPVA velocity plus the fitted rigid relation, per sample."""
    v = r["velocity"]
    speed = np.hypot(v[:, 0], v[:, 1])
    along = v / np.maximum(speed, 1e-6)[:, None]
    left = np.column_stack((-along[:, 1], along[:, 0]))
    r_x, r_y = model["leverArmBodyM"]
    w = r["yaw_rate"]
    return v + (model["alongConstantMps"] + r_y * w)[:, None] * along + (model["leftConstantMps"] - r_x * w)[:, None] * left


def detect(r, model, e):
    """Candidate contradictions from the model-corrected residual rate."""
    time, dt, box = r["time"], e["dt"], r["box"]
    magnitude = np.hypot(np.convolve(e["corrected_along"], np.ones(box) / box, mode="same"),
                         np.convolve(e["corrected_left"], np.ones(box) / box, mode="same"))
    quiet_sd = None
    flag = (magnitude > RATE_THRESHOLD) & e["moving"]
    edges = np.diff(np.concatenate(([0], flag.astype(int), [0])))
    starts, stops = np.where(edges == 1)[0], np.where(edges == -1)[0]
    step = float(np.median(dt))
    minimum = int(round(MINIMUM_DURATION / step))
    spans = [[s, p] for s, p in zip(starts, stops) if p - s >= minimum]
    merged = []
    for s, p in spans:
        if merged and time[s] - time[merged[-1][1] - 1] <= MERGE_GAP:
            merged[-1][1] = p
        else:
            merged.append([s, p])
    pad = int(round(PAD_SECONDS / step))
    candidates = []
    cum_along = np.concatenate(([0.0], np.cumsum(e["corrected_along"] * dt)))
    cum_left = np.concatenate(([0.0], np.cumsum(e["corrected_left"] * dt)))
    for s, p in merged:
        s, p = max(0, s - pad), min(len(dt), p + pad)
        excursion = float(np.max(np.hypot(cum_along[s:p + 1] - cum_along[s], cum_left[s:p + 1] - cum_left[s])))
        if excursion < MINIMUM_EXCURSION:
            continue
        candidates.append(dict(first=s, last=p, excursion=excursion,
                               netAlong=float(cum_along[p] - cum_along[s]), netLeft=float(cum_left[p] - cum_left[s])))
    outside = e["moving"].copy()
    for c in candidates:
        outside[c["first"]:c["last"]] = False
    quiet_sd = [float(np.std(np.convolve(e["corrected_along"], np.ones(box) / box, mode="same")[outside])),
                float(np.std(np.convolve(e["corrected_left"], np.ones(box) / box, mode="same")[outside]))]
    return candidates, magnitude, quiet_sd


def frame_rows():
    rows = read_rows(POSES)
    frame_time = np.array([float(row["receiver_time_sec"]) for row in rows])
    return rows, frame_time


def stage_detect():
    OUT.mkdir(parents=True, exist_ok=True)
    r = load_receiver()
    model, e = fit_model(r)
    candidates, magnitude, quiet_sd = detect(r, model, e)
    rows, frame_time = frame_rows()
    time = r["time"]
    candidate_of_epoch = np.zeros(len(e["dt"]), dtype=int)
    table = []
    for k, c in enumerate(candidates, start=1):
        candidate_of_epoch[c["first"]:c["last"]] = k
        begin, end = float(time[c["first"]]), float(time[c["last"]])
        frames = np.where((frame_time >= begin) & (frame_time <= end))[0]
        table.append(dict(candidate=k, beginSeconds=f"{begin:.6f}", endSeconds=f"{end:.6f}",
                          firstEpoch=c["first"] + 1, lastEpoch=c["last"] + 1,
                          firstFrame=int(rows[frames[0]]["frame_index"]) if frames.size else "",
                          lastFrame=int(rows[frames[-1]]["frame_index"]) if frames.size else "",
                          durationSeconds=f"{end - begin:.3f}", peakExcursionM=f"{c['excursion']:.4f}",
                          netAlongM=f"{c['netAlong']:.4f}", netLeftM=f"{c['netLeft']:.4f}",
                          meanSpeedMps=f"{float(np.mean(e['speed'][c['first']:c['last']])):.3f}",
                          meanAbsYawRateDegps=f"{float(np.rad2deg(np.mean(np.abs(e['yaw_rate'][c['first']:c['last']])))):.3f}"))
    write_rows(OUT / "contradiction_candidates.csv", table) if table else (OUT / "contradiction_candidates.csv").write_text(
        "candidate,beginSeconds,endSeconds,firstEpoch,lastEpoch,firstFrame,lastFrame,durationSeconds,peakExcursionM,netAlongM,netLeftM,meanSpeedMps,meanAbsYawRateDegps\n")
    mv = e["model_velocity"]
    samples = []
    for k in range(len(time)):
        j = min(k, len(e["dt"]) - 1)
        samples.append(dict(time=repr(float(time[k])), x=repr(float(r["position"][k, 0])), y=repr(float(r["position"][k, 1])),
                            psi=repr(float(r["yaw"][k])), vxRaw=repr(float(r["velocity"][k, 0])), vyRaw=repr(float(r["velocity"][k, 1])),
                            vxModel=repr(float(mv[k, 0])), vyModel=repr(float(mv[k, 1])),
                            yawRateRadps=f"{float(r['yaw_rate'][k]):.6f}", speedMps=f"{float(np.hypot(*r['velocity'][k])):.4f}",
                            residualAlongMps=f"{float(e['corrected_along'][j]):.5f}", residualLeftMps=f"{float(e['corrected_left'][j]):.5f}",
                            residualMagnitude05sMps=f"{float(magnitude[j]):.5f}", candidate=int(candidate_of_epoch[j]),
                            insStatus=int(r["native"]["ins_status"][k])))
    write_rows(OUT / "consistency_samples.csv", samples)
    summary = dict(model=model, thresholds=dict(movingSpeedMps=MOVING_SPEED, clipSigmas=CLIP_SIGMAS, boxSeconds=BOX_SECONDS,
                                                rateThresholdMps=RATE_THRESHOLD, minimumDurationSeconds=MINIMUM_DURATION,
                                                mergeGapSeconds=MERGE_GAP, padSeconds=PAD_SECONDS, minimumExcursionM=MINIMUM_EXCURSION),
                   quietResidualRateSd05sMps=quiet_sd, candidates=len(candidates),
                   candidateSeconds=float(sum(float(c["endSeconds"]) - float(c["beginSeconds"]) for c in table)),
                   inputs={str(p.relative_to(ROOT)): sha256(p) for p in (Path(str(GNSS) + "inspva.csv"), NATIVE, POSES)})
    (OUT / "consistency_model.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=1))
    for row in table:
        print("candidate %2s  %8s-%8s s  %5s s  excursion %s m  net along %s left %s  frames %s-%s" % (
            row["candidate"], row["beginSeconds"][:7], row["endSeconds"][:7], row["durationSeconds"], row["peakExcursionM"],
            row["netAlongM"], row["netLeftM"], row["firstFrame"], row["lastFrame"]))


def stage_build():
    r = load_receiver()
    model, e = fit_model(r)
    time, dt = r["time"], e["dt"]
    judge = json.loads((OUT / "judge_summary.json").read_text())
    candidates = read_rows(OUT / "contradiction_candidates.csv")
    segments = read_rows(OUT / "position_velocity_segments.csv")
    runs = read_rows(OUT / "straight_runs.csv")
    verdict = {int(s["candidate"]): s["verdict"] for s in segments}
    assert set(verdict) == {int(c["candidate"]) for c in candidates}, "Judge verdicts do not cover the candidates."
    assert all(v in ("velocity", "position", "undecided") for v in verdict.values())
    use_model = judge["velocityIntegral"]["chosen"] == "modeled"
    velocity = e["model_velocity"] if use_model else r["velocity"]
    inspva, native = r["inspva"], r["native"]

    # Single-epoch out-and-back glitches: physically impossible motion.
    p = r["position"].copy()
    original = p.copy()
    step = np.diff(p, axis=0) - dt[:, None] * (velocity[:-1] + velocity[1:]) / 2
    size = np.hypot(step[:, 0], step[:, 1])
    repaired = []
    for k in range(1, len(time) - 1):
        out, back = step[k - 1], step[k]
        if size[k - 1] > GLITCH_STEP and size[k] > GLITCH_STEP and np.hypot(*(out + back)) < GLITCH_STEP / 3:
            w = (time[k] - time[k - 1]) / (time[k + 1] - time[k - 1])
            p[k] = (1 - w) * p[k - 1] + w * p[k + 1]
            repaired.append(k)

    # Velocity-trusted segments: integral of the trusted velocity, anchored at both ends.
    rebuilt = []
    for c in candidates:
        k = int(c["candidate"])
        if verdict[k] != "velocity":
            continue
        first, last = int(c["firstEpoch"]) - 1, int(c["lastEpoch"]) - 1
        increments = dt[first:last, None] * (velocity[first:last] + velocity[first + 1:last + 1]) / 2
        path = p[first] + np.vstack(([0.0, 0.0], np.cumsum(increments, axis=0)))
        closure = p[last] - path[-1]
        ramp = ((time[first:last + 1] - time[first]) / (time[last] - time[first]))[:, None]
        corrected = path + ramp * closure
        moved = float(np.max(np.hypot(*(corrected - p[first:last + 1]).T)))
        p[first:last + 1] = corrected
        rebuilt.append(dict(candidate=k, beginSeconds=float(time[first]), endSeconds=float(time[last]), samples=last - first + 1,
                            closureM=[float(closure[0]), float(closure[1])], closureNormM=float(np.hypot(*closure)),
                            closureRateMps=float(np.hypot(*closure) / (time[last] - time[first])), maximumMoveM=moved))
    changed = np.hypot(*(p - original).T) > 0
    drift = np.vstack(([0.0, 0.0], np.cumsum(np.diff(p, axis=0) - dt[:, None] * (velocity[:-1] + velocity[1:]) / 2, axis=0)))

    # Heading from the receiver's own direction of travel on straight driving.
    speed = np.hypot(inspva["east_velocity_mps"], inspva["north_velocity_mps"])
    course = np.rad2deg(np.arctan2(inspva["east_velocity_mps"], inspva["north_velocity_mps"]))
    offset = (inspva["azimuth_deg"] - course + 180) % 360 - 180
    rate = np.gradient(np.unwrap(np.deg2rad(inspva["azimuth_deg"])), time)
    rate = np.rad2deg(np.convolve(rate, np.ones(25) / 25, mode="same"))
    straight = (speed >= MINIMUM_SPEED) & (np.abs(rate) <= MAXIMUM_YAW_RATE)
    distrusted = np.zeros_like(straight)
    for c in candidates:
        if verdict[int(c["candidate"])] != "velocity":
            distrusted |= (time >= float(c["beginSeconds"])) & (time <= float(c["endSeconds"]))
    sideslip_kept = np.zeros_like(straight)
    for run in runs:
        if run["sideslipRejected"] not in ("1", "true", "True"):
            sideslip_kept |= (time >= float(run["beginSeconds"])) & (time <= float(run["endSeconds"]))
    usable = straight & ~distrusted & ~sideslip_kept
    estimate = np.zeros_like(time)
    support = np.zeros_like(time)
    for k in range(len(time)):
        kernel = usable * np.exp(-0.5 * ((time - time[k]) / SMOOTHING_SECONDS) ** 2)
        support[k] = kernel.sum()
        if support[k] >= MINIMUM_SUPPORT:
            estimate[k] = np.sum(kernel * offset) / support[k]
    supported = support >= MINIMUM_SUPPORT
    correction = np.interp(time, time[supported], estimate[supported])

    # Solution quality for the tiers.
    inspvax = np.genfromtxt(str(GNSS) + "inspvax.csv", delimiter=",", names=True)
    best = np.genfromtxt(str(GNSS) + "bestpos.csv", delimiter=",", names=True)
    origin = inspva["gps_seconds"][0] - time[0]
    std_time = inspvax["gps_seconds"] - origin
    std = np.hypot(inspvax["latitude_stdev_m"], inspvax["longitude_stdev_m"])
    best_time = best["gps_seconds"] - origin
    best_type = best["position_type"].astype(int)
    best_satellites = best["num_solution_satellites"].astype(int)

    rows, frame_time = frame_rows()
    frame_correction = np.interp(frame_time, time, correction)
    frame_supported = np.interp(frame_time, time, supported.astype(float)) >= 1.0
    first_supported, last_supported = time[supported][0], time[supported][-1]
    spans = [(float(c["beginSeconds"]), float(c["endSeconds"]), verdict[int(c["candidate"])]) for c in candidates]
    flags = []
    worst_check = 0.0
    for row, t, degrees, heading_ok in zip(rows, frame_time, frame_correction, frame_supported):
        lower, upper = int(row["inspva_lower_index"]) - 1, int(row["inspva_upper_index"]) - 1
        fraction = float(row["interpolation_fraction"])
        source = (1 - fraction) * original[lower] + fraction * original[upper]
        worst_check = max(worst_check, abs(source[0] - float(row["pose_x_m"])), abs(source[1] - float(row["pose_y_m"])))
        moved = (1 - fraction) * p[lower] + fraction * p[upper]
        if changed[lower] or changed[upper]:
            row["pose_x_m"], row["pose_y_m"] = repr(float(moved[0])), repr(float(moved[1]))
        shift = float(np.hypot(moved[0] - source[0], moved[1] - source[1]))
        lo, hi = np.searchsorted(time, t - 1.0), min(np.searchsorted(time, t + 1.0), len(time) - 1)
        disagreement = float(np.hypot(*(drift[hi] - drift[lo])))
        sigma = float(std[max(0, np.searchsorted(std_time, t, side="right") - 1)])
        j = max(0, np.searchsorted(best_time, t, side="right") - 1)
        verdicts = [v for b, en, v in spans if b <= t <= en]
        if verdicts:
            resolution = {"velocity": "rebuilt_from_velocity", "position": "kept_position", "undecided": "undecided"}[verdicts[0]]
        else:
            resolution = "consistent"
        position_ok = resolution != "undecided"
        heading_source = "straight_course" if heading_ok else (
            "bridged" if first_supported <= t <= last_supported else "extrapolated")
        fixed = best_type[j] == FIXED_TYPE and best_satellites[j] >= MINIMUM_SOLUTION_SATELLITES and sigma <= MAXIMUM_FIXED_STD
        usable_frame = heading_ok and position_ok
        tier = "A" if usable_frame and fixed else ("B" if usable_frame else "C")
        flags.append(dict(frame=int(row["frame_index"]), time=f"{t:.6f}", headingCorrectionDeg=f"{degrees:.6f}",
                          headingSource=heading_source, positionResolution=resolution, positionShiftM=f"{shift:.4f}",
                          positionType=int(best_type[j]), solutionSatellites=int(best_satellites[j]),
                          reportedHorizontalStdM=f"{sigma:.4f}", disagreement2sM=f"{disagreement:.4f}", tier=tier))
        half = np.deg2rad(degrees) / 2
        w, x, y, z = (float(row[name]) for name in ("pose_qw", "pose_qx", "pose_qy", "pose_qz"))
        c, s = np.cos(half), np.sin(half)  # left-multiply by a rotation about the vertical axis
        row["pose_qw"], row["pose_qx"], row["pose_qy"], row["pose_qz"] = (
            repr(float(c * w - s * z)), repr(float(c * x - s * y)), repr(float(c * y + s * x)), repr(float(c * z + s * w)))
    assert worst_check < 1e-6, "The pose table no longer matches interpolation of the receiver samples."
    write_rows(OUT / "corrected_poses.csv", rows)
    with open(OUT / "corrected_native_reference.csv", "w", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(["time", "x", "y", "z", "psi", "ins_status"])
        for k in range(len(time)):
            writer.writerow([repr(float(time[k])), repr(float(p[k, 0])), repr(float(p[k, 1])), repr(float(native["z"][k])),
                             repr(float(native["psi"][k] + np.deg2rad(correction[k]))), float(native["ins_status"][k])])
    write_rows(OUT / "reference_flags.csv", flags)

    def count(key):
        values = sorted({f[key] for f in flags})
        return {value: sum(1 for f in flags if f[key] == value) for value in values}

    def tier_spans(value):
        out, start, previous = [], None, None
        for f in flags:
            t = float(f["time"])
            if f["tier"] == value and start is None:
                start = t
            if f["tier"] != value and start is not None:
                out.append([round(start, 2), round(previous, 2)])
                start = None
            previous = t
        if start is not None:
            out.append([round(start, 2), round(previous, 2)])
        return out

    by_source = {name: np.array([float(f["headingCorrectionDeg"]) for f in flags if f["headingSource"] == name])
                 for name in ("straight_course", "bridged", "extrapolated")}
    shifts = np.array([float(f["positionShiftM"]) for f in flags])
    native_shift = np.hypot(*(p - original).T)
    summary = dict(
        rule="Every value from the receiver; vehicle sensors decide only which receiver quantity is trusted where the receiver contradicts itself",
        model=model, velocityIntegral=judge["velocityIntegral"],
        thresholds=dict(rateThresholdMps=RATE_THRESHOLD, minimumDurationSeconds=MINIMUM_DURATION, mergeGapSeconds=MERGE_GAP,
                        minimumExcursionM=MINIMUM_EXCURSION, glitchStepM=GLITCH_STEP, minimumSpeedMps=MINIMUM_SPEED,
                        maximumYawRateDegPerS=MAXIMUM_YAW_RATE, smoothingSeconds=SMOOTHING_SECONDS, minimumSupport=MINIMUM_SUPPORT,
                        fixedPositionType=FIXED_TYPE, minimumSolutionSatellites=MINIMUM_SOLUTION_SATELLITES, maximumFixedStdM=MAXIMUM_FIXED_STD),
        candidates=[dict(candidate=int(c["candidate"]), beginSeconds=float(c["beginSeconds"]), endSeconds=float(c["endSeconds"]),
                         peakExcursionM=float(c["peakExcursionM"]), verdict=verdict[int(c["candidate"])]) for c in candidates],
        verdicts={v: sum(1 for c in candidates if verdict[int(c["candidate"])] == v) for v in ("velocity", "position", "undecided")},
        rebuiltSegments=rebuilt, repairedGlitchEpochsSeconds=[round(float(time[k]), 3) for k in repaired],
        straightRunsWithSideslipRejected=f"{sum(r['sideslipRejected'] in ('1', 'true', 'True') for r in runs)} of {len(runs)}",
        frames=len(flags), tiers=count("tier"), tierTimeSpans={k: tier_spans(k) for k in "ABC"},
        headingSources=count("headingSource"), positionResolutions=count("positionResolution"),
        headingCorrectionDeg={name: dict(median=float(np.median(v)), minimum=float(v.min()), maximum=float(v.max()))
                              for name, v in by_source.items() if v.size},
        nativeSamplesWithStraightSupport=int(supported.sum()), usableStraightSamples=int(usable.sum()),
        nativeSamplesMoved=int((native_shift > 0).sum()), maximumNativeMoveM=float(native_shift.max()),
        framesWithChangedPosition=int((shifts > 0).sum()), maximumFramePositionChangeM=float(shifts.max()),
        poseTableInterpolationCheckMaximumM=worst_check,
        valueSources=["INSPVA, INSPVAX and BESTPOS of the 12-09-31 drive", "native INSPVA reference on the receiver clock"],
        arbiterInputs=["output/receiver_reference/position_velocity_segments.csv (verdict per candidate)",
                       "output/receiver_reference/straight_runs.csv (sideslip verdict per run)",
                       "output/receiver_reference/judge_summary.json (velocity integral choice)"],
        changedQuantities=["yaw on every frame", "horizontal position inside velocity-trusted segments",
                           "horizontal position at repaired single-epoch glitches"],
        scope="Reference refined from receiver data with vehicle sensors as arbiters only; not surveyed truth")
    (OUT / "reference_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    clock = json.loads(NATIVE_METADATA.read_text())["clock"]
    metadata = dict(clock=clock, samples=len(time), role="Evaluation and mapping reference; never an observer measurement or reset",
                    coordinates="EPSG:32615, ellipsoidal height, unwrapped grid yaw", builder="scripts/buildReceiverReference.py",
                    inputs={str(q.relative_to(ROOT)): sha256(q) for q in (Path(str(GNSS) + "inspva.csv"), Path(str(GNSS) + "inspvax.csv"),
                                                                           Path(str(GNSS) + "bestpos.csv"), NATIVE, POSES)},
                    outputs={name: sha256(OUT / name) for name in ("corrected_native_reference.csv", "corrected_poses.csv", "reference_flags.csv")})
    (OUT / "reference_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps({k: summary[k] for k in ("verdicts", "rebuiltSegments", "tiers", "nativeSamplesMoved", "maximumNativeMoveM",
                                                "framesWithChangedPosition", "maximumFramePositionChangeM", "headingCorrectionDeg")}, indent=1))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("stage", choices=["detect", "build"])
    stage = parser.parse_args().stage
    if stage == "detect":
        stage_detect()
    else:
        for name in ("judge_summary.json", "position_velocity_segments.csv", "straight_runs.csv", "contradiction_candidates.csv"):
            if not (OUT / name).exists():
                sys.exit(f"{name} is missing; run the detect stage and judgeReceiverContradictions first.")
        stage_build()
