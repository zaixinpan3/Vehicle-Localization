#!/usr/bin/env python3
"""Independently score the fresh matching-only experiment and export its plot.

uv run --offline --with numpy --with pandas --with matplotlib python research/mississippi_matching_only_20260919/analyze_results.py
"""
from pathlib import Path
import hashlib
import json
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
OUT = ROOT / "output/mississippi_matching_only_20260919"
MODES = ("recursive", "referenceSeed")


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def score(calls):
    dx = calls.x.to_numpy() - calls.referenceX.to_numpy()
    dy = calls.y.to_numpy() - calls.referenceY.to_numpy()
    yaw = np.rad2deg(wrap(calls.psi.to_numpy() - calls.referencePsi.to_numpy()))
    error = np.hypot(dx, dy)
    a = calls.referencePsi.to_numpy()
    forward = dx * np.cos(a) + dy * np.sin(a)
    left = -dx * np.sin(a) + dy * np.cos(a)
    return dict(samples=len(calls), positionRmseM=float(np.sqrt(np.mean(error**2))),
                positionMedianM=float(np.median(error)),
                positionP95M=float(np.percentile(error, 95, method="hazen")),
                positionMaximumM=float(error.max()),
                headingRmseDeg=float(np.sqrt(np.mean(yaw**2))),
                mapXRmseM=float(np.sqrt(np.mean(dx**2))),
                mapYRmseM=float(np.sqrt(np.mean(dy**2))),
                forwardRmseM=float(np.sqrt(np.mean(forward**2))),
                leftRmseM=float(np.sqrt(np.mean(left**2))),
                meanForwardErrorM=float(np.mean(forward)), meanLeftErrorM=float(np.mean(left)),
                fractionAtMost10cm=float(np.mean(error <= .1)),
                fractionAtMost20cm=float(np.mean(error <= .2)),
                fractionAtMost30cm=float(np.mean(error <= .3)))


def audit_prediction(calls, mode):
    output = calls[["x", "y", "psi"]].to_numpy()
    prediction = calls[["predictedX", "predictedY", "predictedPsi"]].to_numpy()
    reference = calls[["referenceX", "referenceY", "referencePsi"]].to_numpy()
    offset = np.array([.5, -.4, np.deg2rad(2)])
    if mode == "referenceSeed":
        expected = reference + offset
    else:
        dead = pd.read_csv(OUT / mode / "dead_reckoning.csv")[["x", "y", "psi"]].to_numpy()
        a = dead[:-1, 2]
        displacement = np.diff(dead[:, :2], axis=0)
        forward = displacement[:, 0] * np.cos(a) + displacement[:, 1] * np.sin(a)
        left = -displacement[:, 0] * np.sin(a) + displacement[:, 1] * np.cos(a)
        a = output[:-1, 2]
        expected = np.empty_like(prediction)
        expected[0] = reference[0] + offset
        expected[1:, 0] = output[:-1, 0] + forward * np.cos(a) - left * np.sin(a)
        expected[1:, 1] = output[:-1, 1] + forward * np.sin(a) + left * np.cos(a)
        expected[1:, 2] = output[:-1, 2] + np.diff(dead[:, 2])
    position_residual = float(np.max(abs(expected[:, :2] - prediction[:, :2])))
    heading_residual = float(np.max(abs(wrap(expected[:, 2] - prediction[:, 2]))))
    assert position_residual < 1e-7 and heading_residual < 1e-12
    rejected = ~(calls.accepted.to_numpy(bool) | calls.directionalAccepted.to_numpy(bool))
    assert np.array_equal(output[rejected], prediction[rejected])
    return dict(maximumPredictionCoordinateResidualM=position_residual,
                maximumPredictionHeadingResidualRad=heading_residual,
                rejectedOutputsEqualInitialGuesses=True)


def main():
    metrics = pd.read_csv(OUT / "metrics.csv")
    report = json.loads((OUT / "summary.json").read_text())
    assert not report["metadata"]["globalFusionObserverUsed"]
    assert not report["metadata"]["finePerceptionUsed"]
    data, checks, rows = {}, {}, []
    metric_residual = 0.0
    for mode in MODES:
        calls = pd.read_csv(OUT / mode / "calls.csv")
        data[mode] = calls
        assert np.array_equal(calls.frame, np.arange(1, 1171))
        assert np.all(np.diff(calls.timeSeconds) > 0)
        accepted = calls.accepted.to_numpy(bool)
        directional = calls.directionalAccepted.to_numpy(bool)
        assert not np.any(directional), "Directional-only events require a separate evaluation population."
        metadata = json.loads((OUT / mode / "metadata.json").read_text())
        assert metadata["mode"] == mode and metadata["perceptionRerun"]
        assert metadata["perceptionMode"] == "coarseProbabilityCloud" and not metadata["finePerceptionUsed"]
        displacement = calls[["x", "y"]].to_numpy() - calls[["referenceX", "referenceY"]].to_numpy()
        np.testing.assert_allclose(np.linalg.norm(displacement, axis=1), calls.positionErrorM, rtol=0, atol=1e-8)
        np.testing.assert_allclose(np.rad2deg(wrap(calls.psi - calls.referencePsi)), calls.yawErrorDeg,
                                   rtol=0, atol=1e-10)
        information = np.array([
            [calls.informationXX, calls.informationXY, calls.informationXPsi],
            [calls.informationXY, calls.informationYY, calls.informationYPsi],
            [calls.informationXPsi, calls.informationYPsi, calls.informationPsiPsi]]).transpose(2, 0, 1)
        eigenvalues = np.linalg.eigvalsh(information[accepted])
        assert np.isfinite(eigenvalues).all() and eigenvalues.min() > 0
        checks[mode] = dict(accepted=int(accepted.sum()), rejected=int((~accepted).sum()),
                            directionalAccepted=int(directional.sum()),
                            reasons={str(k): int(v) for k, v in calls.reason.value_counts().items()},
                            minimumAcceptedInformationEigenvalue=float(eigenvalues.min()),
                            predictionAudit=audit_prediction(calls, mode))
        for population, selected in [("accepted_matching_measurements", accepted),
                                     ("all_outputs_with_prediction", np.ones(len(calls), bool))]:
            recomputed = score(calls.loc[selected])
            row = metrics.loc[(metrics["mode"] == mode) & (metrics.population == population)].iloc[0]
            for name, value in recomputed.items():
                if name in row.index:
                    metric_residual = max(metric_residual, abs(float(row[name]) - value))
            rows.append(dict(mode=mode, population=population, **recomputed))
        calls.loc[accepted].nlargest(15, "positionErrorM").to_csv(
            OUT / f"{mode}_largest_accepted_errors.csv", index=False)
    assert metric_residual < 1e-8
    # Identical evaluation scans prevent acceptance differences from explaining
    # the effect of changing initialization in this diagnostic comparison.
    common = data["recursive"].accepted.to_numpy(bool) & data["referenceSeed"].accepted.to_numpy(bool)
    for mode in MODES:
        rows.append(dict(mode=mode, population="accepted_in_both_modes", **score(data[mode].loc[common])))
    previous = pd.read_csv(ROOT / "output/mncav_coarse_localization_20260918/matching/calls.csv")
    columns = [name for name in previous.columns if not name.endswith("Ms")]
    pd.testing.assert_frame_equal(data["recursive"][columns], previous[columns], check_exact=True)
    checks.update(metricRowsVerified=len(metrics), maximumMetricResidual=metric_residual,
                  jointlyAcceptedFrames=int(common.sum()),
                  previousRecursiveNonTimingResultsExactlyReproduced=True,
                  csvPoseToleranceM=1e-8,
                  percentileMethod="Hazen (MATLAB prctile); per-mode replay summaries use linear percentiles")
    summary = pd.DataFrame(rows)
    summary.to_csv(DEST / "independent_metrics.csv", index=False)
    (DEST / "independent_checks.json").write_text(json.dumps(checks, indent=2) + "\n")
    shutil.copy2(OUT / "metrics.csv", DEST / "metrics.csv")
    shutil.copy2(OUT / "summary.json", DEST / "run_summary.json")
    manifest = []
    for relative in ["scripts/runMississippiMapMatchingExperiment.m", "scripts/replayMississippiLocalization.m",
                     "config/distributionRegistrationConfig.m", "config/coarseSemanticProbabilityCloudConfig.m",
                     "output/mississippi_mapping_inspva_20260915/probability_cloud.mat",
                     "output/mississippi_matching_only_20260919/experiment.mat"]:
        path = ROOT / relative
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        manifest.append(dict(path=relative, sha256=digest))
    (DEST / "artifact_hashes.json").write_text(json.dumps(manifest, indent=2) + "\n")
    plot(data)
    print(summary.to_string(index=False))
    print(json.dumps(checks, indent=2))


def plot(data):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), constrained_layout=True)
    styles = [("recursive", "Recursive initial guess", "#155B9A"),
              ("referenceSeed", "Reference-assisted initial guess (diagnostic)", "#D77B13")]
    for mode, label, color in styles:
        c = data[mode]
        selected = c.accepted.to_numpy(bool)
        error = np.where(selected, c.positionErrorM.to_numpy() * 100, np.nan)
        yaw = np.where(selected, c.yawErrorDeg.to_numpy(), np.nan)
        axes[0].plot(c.frame, error, color=color, lw=1, alpha=.85, label=label)
        axes[1].plot(c.frame, yaw, color=color, lw=1, alpha=.85)
        values = np.sort(c.loc[selected, "positionErrorM"].to_numpy() * 100)
        axes[2].plot(values, np.arange(1, len(values) + 1) / len(values) * 100, color=color, lw=2)
    c = data["recursive"]
    rejected = ~c.accepted.to_numpy(bool)
    axes[0].scatter(c.loc[rejected, "frame"], c.loc[rejected, "positionErrorM"] * 100,
                    s=13, marker="x", color="#A5A5A5", label="Recursive prediction after rejection")
    axes[0].set(ylabel="Horizontal error (cm)", xlabel="Mississippi frame (1–1170)")
    axes[0].legend(loc="upper right", fontsize=9)
    axes[1].set(ylabel="Heading error (degrees)", xlabel="Mississippi frame (1–1170)")
    axes[2].set(xlabel="Accepted matching position error (cm)", ylabel="Cumulative fraction (%)",
                ylim=(0, 100), xlim=(0, None))
    for axis in axes:
        axis.grid(alpha=.22)
    fig.suptitle("Raw coarse map matching vs. recorded INSPVA pose\n"
                 "No global pose fusion; same-drive map includes query observations", fontsize=13)
    for extension in ("png", "pdf"):
        fig.savefig(OUT / f"matching_errors.{extension}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
