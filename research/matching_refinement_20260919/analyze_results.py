#!/usr/bin/env python3
"""Independently verify pose errors, causal predictions, tail counts and plots.

uv run --offline --with numpy --with pandas --with matplotlib python research/matching_refinement_20260919/analyze_results.py
"""
from pathlib import Path
import json
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
OUT = ROOT / "output/matching_refinement_20260919"


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def errors(c):
    return np.hypot(c.x - c.referenceX, c.y - c.referenceY).to_numpy()


def score(c):
    e = errors(c)
    over = e > 1
    run = best = 0
    for value in over:
        run = run + 1 if value else 0
        best = max(run, best)
    return dict(samples=len(e), positionRmseM=float(np.sqrt(np.mean(e**2))),
                positionP95M=float(np.percentile(e, 95, method="hazen")),
                positionMaximumM=float(max(e)), aboveOneMeterFrames=int(over.sum()),
                longestAboveOneMeterRunFrames=best)


def audit_predictions(c, folder, mode):
    state = c[["x", "y", "psi"]].to_numpy()
    prediction = c[["predictedX", "predictedY", "predictedPsi"]].to_numpy()
    reference = c[["referenceX", "referenceY", "referencePsi"]].to_numpy()
    expected = reference + [.5, -.4, np.deg2rad(2)]
    if mode == "recursive":
        d = pd.read_csv(folder / "dead_reckoning.csv")[["x", "y", "psi"]].to_numpy()
        delta = np.diff(d[:, :2], axis=0)
        forward = delta[:, 0] * np.cos(d[:-1, 2]) + delta[:, 1] * np.sin(d[:-1, 2])
        left = -delta[:, 0] * np.sin(d[:-1, 2]) + delta[:, 1] * np.cos(d[:-1, 2])
        a = state[:-1, 2]
        expected[1:, 0] = state[:-1, 0] + forward * np.cos(a) - left * np.sin(a)
        expected[1:, 1] = state[:-1, 1] + forward * np.sin(a) + left * np.cos(a)
        expected[1:, 2] = a + np.diff(d[:, 2])
    assert np.max(abs(expected[:, :2] - prediction[:, :2])) < 1e-7
    assert np.max(abs(wrap(expected[:, 2] - prediction[:, 2]))) < 1e-12
    rejected = ~(c.accepted.astype(bool) | c.directionalAccepted.astype(bool))
    np.testing.assert_array_equal(state[rejected], prediction[rejected])


def main():
    rows, checks, data = [], {}, {}
    baseline = pd.read_csv(ROOT / "output/mississippi_matching_only_20260919/recursive/calls.csv")
    failed_ndt = pd.read_csv(ROOT / "output/stability_ndt_20260919/overlap_full/recursive/calls.csv")
    rows.extend([dict(method="previous_geometric_all_outputs", **score(baseline)),
                 dict(method="withdrawn_ndt_all_outputs", **score(failed_ndt))])
    for mode in ("recursive", "referenceSeed"):
        folder = OUT / "validated" / mode
        c = pd.read_csv(folder / "calls.csv")
        meta = json.loads((folder / "metadata.json").read_text())
        w = pd.read_csv(folder / "source_windows.csv")
        np.testing.assert_array_equal(c.frame, np.arange(1, 1171))
        assert meta["perceptionMode"] == "coarseProbabilityCloud" and meta["perceptionRerun"]
        assert not meta["finePerceptionUsed"]
        assert w.scans.max() == 3 and w.spanSeconds.max() <= .25
        np.testing.assert_allclose(errors(c), c.positionErrorM, atol=1e-8, rtol=0)
        audit_predictions(c, folder, mode)
        accepted = c.accepted.astype(bool)
        I = np.array([[c.informationXX, c.informationXY, c.informationXPsi],
                      [c.informationXY, c.informationYY, c.informationYPsi],
                      [c.informationXPsi, c.informationYPsi, c.informationPsiPsi]]).transpose(2, 0, 1)
        eigenvalues = np.linalg.eigvalsh(I[accepted])
        assert np.isfinite(eigenvalues).all() and eigenvalues.min() > 0
        assert not c.directionalAccepted.any()
        for population, subset in (("all_outputs", c), ("accepted", c[accepted])):
            rows.append(dict(method=f"current_{mode}_{population}", **score(subset)))
        checks[mode] = dict(accepted=int(accepted.sum()), rejected=int((~accepted).sum()),
                            minimumInformationEigenvalue=float(eigenvalues.min()),
                            maximumSourceAgeSeconds=float(w.spanSeconds.max()),
                            causalPredictionAudit=True)
        data[mode] = c
    c = data["recursive"]
    common = c.accepted.astype(bool) & baseline.accepted.astype(bool)
    rows.extend([dict(method="previous_common_accepted", **score(baseline[common])),
                 dict(method="current_common_accepted", **score(c[common]))])
    assert score(c)["positionRmseM"] < score(baseline)["positionRmseM"]
    assert score(c)["aboveOneMeterFrames"] == 0
    assert c.accepted.sum() >= baseline.accepted.sum()
    official = pd.read_csv(OUT / "observer_final/metrics.csv")
    observer_data = {}
    for scenario in official.scenario.unique():
        a = pd.read_csv(OUT / "observer_final" / f"{scenario}_poses.csv")
        m = score(a)
        source = official[(official.scenario == scenario) & (official.population == "full")].iloc[0]
        for field in ("positionRmseM", "positionP95M", "positionMaximumM"):
            assert abs(m[field] - source[field]) < 1e-8
        assert m["aboveOneMeterFrames"] == 0, (scenario, m)
        rows.append(dict(method=f"observer_{scenario}", **m))
        observer_data[scenario] = a
    assert score(observer_data["both"])["positionRmseM"] < score(observer_data["gnss_only"])["positionRmseM"]
    checks.update(noReferenceBasedClipping=True, scoring="All outputs retained; rejected matching predictions included",
                  informationCalibrated=False, sharedMapAndReceiverReference=True)
    metrics = pd.DataFrame(rows)
    metrics.to_csv(DEST / "independent_metrics.csv", index=False)
    (DEST / "independent_checks.json").write_text(json.dumps(checks, indent=2) + "\n")
    for name in ("metrics.csv", "summary.json"):
        shutil.copy2(OUT / "validated" / name, DEST / ("matching_" + name))
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), constrained_layout=True)
    for label, a, color in (("Previous geometric matcher", baseline, "#9C9C9C"),
                            ("Current matcher (all frames)", c, "#1565C0")):
        axes[0].plot(a.frame, errors(a), lw=1, color=color, label=label)
    rejected = ~c.accepted.astype(bool)
    axes[0].scatter(c.frame[rejected], errors(c)[rejected], marker="x", color="black", label="Prediction after rejection")
    axes[0].set(xlabel="Mississippi frame", ylabel="Position discrepancy (m)")
    for scenario in ("both", "lidar_only", "gnss_only"):
        a = observer_data[scenario]
        axes[1].plot(a.time, errors(a), lw=1, label=scenario)
    for scenario in ("gnss_outage", "lidar_outage", "both_outage", "alternating"):
        a = observer_data[scenario]
        axes[2].plot(a.time, errors(a), lw=1, label=scenario)
    axes[2].axvspan(40, 60, color="gray", alpha=.1)
    for axis in axes:
        axis.axhline(1, color="red", ls="--", lw=.8, label="1 m")
        axis.grid(alpha=.2)
        axis.legend(fontsize=8, ncols=3)
    for axis in axes[1:]:
        axis.set(xlabel="Receiver time (s)", ylabel="Position discrepancy (m)")
    fig.suptitle("Coarse-pillar localization: full traces including unavailable-measurement intervals\n"
                 "Same-drive map and shared INSPVA reference; no trajectory alignment or error clipping")
    fig.savefig(OUT / "validated_comparison.png", dpi=170)
    fig.savefig(OUT / "validated_comparison.pdf")
    plt.close(fig)
    print(metrics.to_string(index=False))


if __name__ == "__main__":
    main()
