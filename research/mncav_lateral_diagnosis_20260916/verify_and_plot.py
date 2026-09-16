#!/usr/bin/env python3
"""Independently verify the lateral diagnosis and export a standalone figure.

uv run --offline --with numpy --with matplotlib python
research/mncav_lateral_diagnosis_20260916/verify_and_plot.py
"""
from pathlib import Path
import csv
import hashlib
import json
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
OUT = ROOT / "output/mncav_lateral_diagnosis_20260916"


def main():
    s = np.genfromtxt(OUT / "signals.csv", delimiter=",", names=True)
    controls = list(csv.DictReader((OUT / "controls.csv").open()))
    report = json.loads((OUT / "summary.json").read_text())
    v = json.loads((ROOT / "config/mncavVehicleParameters.json").read_text())["vehicle"]
    moving = s["vx"] >= 5
    straight = moving & (np.abs(s["measuredR"]) < .03)
    masks = {"all": np.ones(len(s), bool), "moving": moving, "straight": straight,
             "turning": moving & ~straight, "moving_first60": moving & (s["time"] <= 60),
             "moving_later": moving & (s["time"] > 60)}
    baseline = {r["population"]: r for r in controls if r["variant"] == "actual"}
    for group, mask in masks.items():
        error = s["estimatedVy"][mask]-s["referenceVy"][mask]
        row = baseline[group]
        assert int(row["samples"]) == mask.sum()
        for key, actual in [("rmseMps", np.sqrt(np.mean(error**2))),
                            ("medianAbsMps", np.median(np.abs(error))), ("biasMps", np.mean(error))]:
            assert abs(float(row[key])-actual) < 1e-12
    cf, cr, m = v["frontCorneringStiffness"], v["rearCorneringStiffness"], v["mass"]
    moment = v["lr"]*cr-v["lf"]*cf
    pred = -(cf+cr)/(m*s["vx"][moving])*s["referenceVy"][moving] + moment/(m*s["vx"][moving])*s["measuredR"][moving] + cf/m*s["steering"][moving]
    inverse = (moment*s["measuredR"][moving]+cf*s["vx"][moving]*s["steering"][moving]-m*s["vx"][moving]*s["measuredAy"][moving])/(cf+cr)
    assert np.max(np.abs(pred-s["predictedAyAtReferenceVy"][moving])) < 1e-11
    assert np.max(np.abs(inverse-s["algebraicVy"][moving])) < 1e-11
    dm = np.column_stack((s["plantVyMismatch"], s["plantRMismatch"]))
    dy = np.column_stack((s["outputAyMismatch"], s["outputRMismatch"]))
    injection = np.column_stack((s["L11"]*dy[:, 0]+s["L12"]*dy[:, 1],
                                s["L21"]*dy[:, 0]+s["L22"]*dy[:, 1]))
    flow = np.column_stack((s["observerFlowAtReferenceVy"]-s["referenceVyDot"],
                            s["observerFlowAtReferenceR"]-s["referenceRDot"]))
    reconstruction = np.max(np.abs((dm-injection-flow)[moving]))
    assert reconstruction < 1e-11
    details = {"rows": len(s), "successfulReplayVariants": len(controls)//len(masks),
               "failedSynthesis": report["failures"], "populations": {k: int(a.sum()) for k, a in masks.items()},
               "independentMetricChecksPassed": True, "outputInversionChecksPassed": True,
               "errorEquationResidual": float(reconstruction),
               "straightPlantMismatchMean": np.mean(dm[straight], axis=0).tolist(),
               "straightInnovationContributionMean": np.mean(-injection[straight], axis=0).tolist(),
               "straightTotalForcingMean": np.mean(flow[straight], axis=0).tolist(),
               "straightKinematicResidualMeanMps2": float(np.mean((s["measuredAy"]-s["measuredR"]*s["vx"]-s["referenceVyDot"])[straight])),
               "straightAlgebraicVelocityRmseMps": float(np.sqrt(np.mean((s["algebraicVy"]-s["referenceVy"])[straight]**2))),
               "straightMasterDynamicMeanDifferenceMps": float(np.mean((s["estimatedVy"]-s["dynamicVy"])[straight])),
               "statement": "Descriptive fixed-drive diagnostics; no parameter fit, deployment or independent validation."}
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    ax = axes[0, 0]
    ax.plot(s["time"], s["referenceVy"], label="INSPVA reference", lw=1.3)
    ax.plot(s["time"], s["estimatedVy"], label="Actual lateral observer", lw=1)
    ax.plot(s["time"], s["dynamicVy"], label="Hidden bicycle branch", lw=.8, alpha=.7)
    ax.set(title="The master follows the bicycle branch", xlabel="Time (s)", ylabel="Lateral velocity (m/s)")
    ax.legend(fontsize=8)
    ax = axes[0, 1]
    for name, label in [("predictedAyAtReferenceVy", "Model output at reference vy"), ("measuredAy", "Actual accelerometer input"), ("referenceAy", "INS-derived acceleration")]:
        val = np.where(straight, s[name], np.nan)
        ax.plot(s["time"], val, label=label, lw=1)
    ax.set(title="Straight motion: reference state conflicts with model output", xlabel="Time (s)", ylabel="Lateral acceleration (m/s²)")
    ax.legend(fontsize=8)
    ax = axes[1, 0]
    selected = ["actual", "gain_10", "tires_07_resynth", "weight_vy_100_resynth", "oracle_motion"]
    values = [float(next(r["rmseMps"] for r in controls if r["variant"] == n and r["population"] == "moving")) for n in selected]
    ax.barh(["Actual", "10 × LPV gain", "0.7 × stiffness; new gain", "100 × vy design weight", "Reference motion inputs*"], values)
    ax.set(xlabel="Moving-sample lateral velocity RMSE (m/s)", title="Gain / parameter probes retain the main discrepancy", xlim=(0, .32))
    ax.invert_yaxis()
    for i, value in enumerate(values):
        ax.text(value+.004, i, f"{value:.3f}", va="center", fontsize=9)
    ax.text(.02, -.19, "* Oracle ay, yaw rate, vx, ax; no reference vy injection", transform=ax.transAxes, fontsize=8)
    ax = axes[1, 1]
    init = np.genfromtxt(OUT / "initial_state_control.csv", delimiter=",", names=True)
    for name, label in [("referenceVy", "INSPVA reference"), ("referenceInitializedVy", "Observer initialized at reference"), ("originalVy", "Original observer")]:
        ax.plot(init["elapsedSeconds"], init[name], "o-", label=label)
    ax.set(title="A correct initial velocity is rapidly pulled away", xlabel="Elapsed time after t = 20 s (s)", ylabel="Lateral velocity (m/s)")
    ax.legend(fontsize=8)
    for ax in axes.flat:
        ax.grid(alpha=.2)
    fig.suptitle("MnCAV lateral observer diagnosis — fixed INSPVA benchmark", fontsize=15)
    fig.savefig(OUT / "lateral_diagnosis.png", dpi=180)
    fig.savefig(OUT / "lateral_diagnosis.pdf")
    plt.close(fig)
    (OUT / "independent_checks.json").write_text(json.dumps(details, indent=2)+"\n")
    for name in ["summary.json", "controls.csv", "initial_state_control.csv", "independent_checks.json"]:
        shutil.copy2(OUT / name, DEST / name)
    paths = [ROOT / "scripts/analyzeMncavLateralDiagnosis.m", Path(__file__),
             ROOT / report["source"], ROOT / "output/mncav_inspva_median_20260915/motion_audit.csv",
             ROOT / "config/mncavVehicleParameters.json", ROOT / "config/lateralObserverConfig.m",
             ROOT / "localization/lateralObserver/runLateralVelocityObserver.m",
             ROOT / "localization/lateralObserver/designLateralObserverGains.m",
             ROOT / "localization/lateralObserver/lateralBicycleModel.m"]
    paths += [p for p in OUT.iterdir() if p.is_file() and p.name not in ["partial.mat", "run_log.txt"]]
    manifest = []
    for p in paths:
        with p.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        manifest.append({"path": str(p.relative_to(ROOT)), "sha256": digest, "bytes": p.stat().st_size})
    (DEST / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps(details, indent=2))


if __name__ == "__main__":
    main()
