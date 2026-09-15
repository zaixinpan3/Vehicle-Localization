#!/usr/bin/env python3
"""Audit the median tradeoff and signal geometry; no calibration is applied.

uv run --offline --with numpy --with matplotlib python
research/mncav_inspva_median_20260915/analyze_signals.py
"""
from pathlib import Path
import csv
import hashlib
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "output/mncav_inspva_median_20260915"
DEST = Path(__file__).resolve().parent


def stats(values):
    return {"mean": float(np.mean(values)), "median": float(np.median(values)),
            "rmse": float(np.sqrt(np.mean(values**2))),
            "q25": float(np.percentile(values, 25)), "q75": float(np.percentile(values, 75))}


def course_heading(path, evaluation):
    ins = np.genfromtxt(path, delimiter=",", names=True, encoding=None)
    suffix = "_deg" if evaluation else ""
    az = np.deg2rad(ins["azimuth"+suffix])
    suffix = "_mps" if evaluation else ""
    east, north = ins["east_velocity"+suffix], ins["north_velocity"+suffix]
    vx, vy = np.sin(az)*east+np.cos(az)*north, -np.cos(az)*east+np.sin(az)*north
    time = ins["gps_seconds"]-ins["gps_seconds"][0]
    rate = np.gradient(np.unwrap(np.pi/2-az), time)
    mask = (vx > 5) & (np.abs(rate) < .03)
    beta = np.rad2deg(np.arctan2(vy, vx))
    result = {"path": str(path.relative_to(ROOT)), "selection": "INS forward speed >5 m/s and absolute INS yaw rate <.03 rad/s",
              "durationSeconds": float(time[-1]), "samples": int(mask.sum()),
              "courseMinusHeadingDeg": stats(beta[mask]), "lateralVelocityMps": stats(vy[mask]),
              "interpretation": "Planar course versus reported INS attitude; not an identified vehicle sideslip or extrinsic calibration"}
    return result


def main():
    sampled = np.genfromtxt(OUT / "motion_audit.csv", delimiter=",", names=True)
    frames = np.genfromtxt(OUT / "paired_frames.csv", delimiter=",", names=True)
    time = np.arange(11690)*.01
    a = {name: np.interp(time, sampled["time"], sampled[name]) for name in sampled.dtype.names}
    moving = a["measuredVx"] >= 5
    straight = moving & (np.abs(a["measuredYawRate"]) < .03)
    signal = {}
    for name, mask in [("all_uniform", np.ones(len(time), bool)), ("moving", moving),
                       ("straight_moving", straight), ("straight_moving_INS_good", straight & (a["insGood"] > .999))]:
        entry = {"samples": int(mask.sum())}
        for measured, reference in [("measuredVx", "insVx"), ("estimatedVy", "insVy"),
                                    ("measuredAx", "insDerivedAx"), ("measuredAy", "insDerivedAy")]:
            entry[measured] = {"measuredMean": float(np.mean(a[measured][mask])),
                               "referenceMean": float(np.mean(a[reference][mask])),
                               "difference": stats((a[measured]-a[reference])[mask])}
        for field in ["observerForwardVelocityError", "observerLeftVelocityError", "observerHeadingErrorRad"]:
            entry[field] = stats(a[field][mask])
        signal[name] = entry
    # Inspect temporal shifts without estimating or applying a clock correction.
    lag_rows = []
    safe = straight & (time >= .5) & (time <= time[-1]-.5)
    for lag in [-.5, -.2, -.1, 0, .1, .2, .5]:
        residual = a["estimatedVy"]-np.interp(time+lag, time, a["insVy"])
        lag_rows.append({"shiftSeconds": lag, "samples": int(safe.sum()), **stats(residual[safe])})
    sequences = [course_heading(ROOT / "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv", True),
                 course_heading(ROOT / "output/mississippi_20240607_120931_20260907/calibration_sensors/inspva.csv", False)]
    # These regressions describe compatibility with a small frame rotation and
    # rigid-body point offset. They do not identify or apply physical extrinsics.
    residual = a["insVy"]-a["estimatedVy"]
    train, later = moving & (time <= 60), moving & (time > 60)
    regressions = []
    for intercept in [False, True]:
        matrix = np.column_stack((a["measuredVx"], a["measuredYawRate"]))
        if intercept:
            matrix = np.column_stack((matrix, np.ones(len(time))))
        coefficient = np.linalg.lstsq(matrix[train], residual[train], rcond=None)[0]
        regressions.append({"model": "insVy-estimatedVy = delta*vx + ell_x*r"+(" + c" if intercept else ""),
                            "trainingSamples": int(train.sum()), "laterSamples": int(later.sum()),
                            "deltaRad": float(coefficient[0]), "deltaDeg": float(np.rad2deg(coefficient[0])),
                            "ellXM": float(coefficient[1]), "interceptMps": float(coefficient[2]) if intercept else 0,
                            "laterResidualBeforeMps": stats(residual[later]),
                            "laterResidualAfterMps": stats(residual[later]-matrix[later]@coefficient),
                            "physicalParametersIdentified": False, "correctionApplied": False})
    with (OUT / "controls.csv").open(newline="") as stream:
        controls = list(csv.DictReader(stream))
    primary = next(r for r in controls if r["variant"] == "actual" and r["population"] == "native_full")
    assert len(frames) == 1084
    observed = frames["observerErrorM"]
    assert abs(np.median(observed)-float(primary["medianM"])) < 1e-12
    assert abs(np.sqrt(np.mean(observed**2))-float(primary["rmseM"])) < 1e-12
    with (OUT / "raw_error_bins.csv").open(newline="") as stream:
        bins = list(csv.DictReader(stream))
    assert sum(int(r["frames"]) for r in bins) == len(frames)
    assert abs(sum(float(r["rawSquaredErrorSum"]) for r in bins)-np.sum(frames["rawErrorM"]**2)) < 1e-10
    assert abs(sum(float(r["observerSquaredErrorSum"]) for r in bins)-np.sum(observed**2)) < 1e-10
    checked = 0
    for row in bins:
        lower, upper = float(row["rawErrorLowerM"]), float(row["rawErrorUpperM"])
        mask = (frames["rawErrorM"] >= lower) & (frames["rawErrorM"] < upper)
        assert np.sum(mask) == int(row["frames"])
        assert abs(np.median(observed[mask])-float(row["observerMedianM"])) < 1e-12
        assert abs(np.mean(observed[mask]>frames["rawErrorM"][mask])-float(row["fractionWorse"])) < 1e-12
        checked += 1
    result = {"uniformSignalStatistics": signal, "lateralTimeShiftDiagnostics": lag_rows,
              "twoDriveCourseHeadingDiagnostics": sequences, "exploratoryGeometryRegressions": regressions,
              "checks": {"primaryRmseAndMedianReproduced": True, "binsVerified": checked,
                         "squaredErrorSumsReconcile": True, "referenceUsedInDiagnosticRegression": True,
                         "diagnosticRegressionAppliedToObserver": False,
                         "productionParametersChanged": False}}
    (OUT / "signal_diagnostics.json").write_text(json.dumps(result, indent=2)+"\n")
    plt.rcParams.update({"font.size": 10})
    fig, axs = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for field, label in [("rawErrorM", "Original LiDAR"), ("observerErrorM", "Observer")]:
        axs[0, 0].plot(np.sort(frames[field])*100, np.arange(1, len(frames)+1)/len(frames), label=label)
    axs[0, 0].set(xlim=(0, 40), xlabel="Position discrepancy (cm)", ylabel="Fraction of accepted frames", title="Large-error tail improves; small-error fraction worsens")
    axs[0, 0].legend();axs[0, 0].grid(alpha=.25)
    x = np.arange(4)
    for offset, key, label in [(-.18, "rawMedianM", "Original LiDAR"), (.18, "observerMedianM", "Observer")]:
        axs[0, 1].bar(x+offset, [100*float(r[key]) for r in bins], .36, label=label)
    axs[0, 1].set(xticks=x, xticklabels=["<5 cm\nN=274", "5–10 cm\nN=394", "10–20 cm\nN=274", "≥20 cm\nN=142"],
                  ylabel="Within-bin median discrepancy (cm)", title="Descriptive bins defined by original LiDAR error")
    axs[0, 1].legend();axs[0, 1].grid(axis="y", alpha=.25)
    axs[1, 0].plot(time, a["estimatedVy"], label="Lateral observer output")
    axs[1, 0].plot(time, a["insVy"], label="INS-implied planar lateral velocity")
    axs[1, 0].set(xlabel="Receiver time (s)", ylabel="Lateral velocity (m/s)", title="Persistent mismatch relative to the INS frame")
    axs[1, 0].legend(loc="lower right");axs[1, 0].grid(alpha=.25)
    raw_median = np.median(frames["rawErrorM"])*100
    gain_rows = [next(r for r in controls if r["variant"] == name and r["population"] == "native_full")
                 for name in ["kp_2", "actual", "kp_8", "kp_12", "kp_16", "kp_24"]]
    gains = [2, 4, 8, 12, 16, 24]
    axs[1, 1].plot(gains, [100*float(r["medianM"]) for r in gain_rows], "o-", label="Median")
    axs[1, 1].plot(gains, [100*float(r["rmseM"]) for r in gain_rows], "o-", label="RMSE")
    axs[1, 1].axhline(raw_median, color="gray", linestyle="--", label="Raw LiDAR median")
    axs[1, 1].set(xlabel="Position correction gain kp (1/s)", ylabel="Discrepancy (cm)", title="Diagnostic gain sensitivity; no setting deployed")
    axs[1, 1].legend();axs[1, 1].grid(alpha=.25)
    fig.savefig(OUT / "median_diagnosis.png", dpi=160)
    fig.savefig(OUT / "median_diagnosis.pdf")
    plt.close(fig)
    artifact_paths = list(OUT.glob("*")) + [ROOT / x["path"] for x in sequences]
    artifact_paths += [ROOT / "output/mncav_inspva_observer_20260915" / name
                      for name in ["experiment.mat", "motion_gap_experiment.mat", "native_reference.csv", "frame_errors.csv"]]
    manifest = []
    for path in sorted(artifact_paths):
        if path.is_file():
            with path.open("rb") as stream:
                digest = hashlib.file_digest(stream, "sha256").hexdigest()
            manifest.append({"path": str(path.relative_to(ROOT)), "bytes": path.stat().st_size, "sha256": digest})
    (DEST / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps({"straightSignal": signal["straight_moving"], "twoDrives": sequences, "checks": result["checks"]}, indent=2))


if __name__ == "__main__":
    main()
