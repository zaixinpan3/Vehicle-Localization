#!/usr/bin/env python3
"""Render recorded diagnostic results and a local analytic delay response."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/mncav_error_diagnosis_20260914"


def read(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf8")


def save(fig, name):
    fig.savefig(OUT / (name + ".png"), dpi=170)
    fig.savefig(OUT / (name + ".pdf"))
    plt.close(fig)


def main():
    ref = read(OUT / "reference_comparison.csv")
    metrics = read(OUT / "complete_ablation_metrics.csv")
    audit = json.loads((OUT / "reference_audit.json").read_text())
    t = ref["time"]
    odom = np.column_stack([ref["odomX"], ref["odomY"]])
    pva = np.column_stack([ref["pvaX"], ref["pvaY"]])
    estimate = np.column_stack([ref["observerX"], ref["observerY"]])
    odom_error = np.linalg.norm(estimate - odom, axis=1)
    pva_error = np.linalg.norm(estimate - pva, axis=1)
    difference = np.linalg.norm(odom - pva, axis=1)
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), constrained_layout=True)
    axes[0].plot(t, odom_error, lw=1, label="Observer vs mixed ODOM")
    axes[0].plot(t, pva_error, lw=1, alpha=.85, label="Observer vs INSPVA")
    axes[0].set_title("Identical observer output; two recorded reference constructions")
    axes[0].legend(loc="upper left", ncol=2)
    axes[0].set_xlim(t[0], t[-1])
    for ax, bounds in zip(axes[1:], [(86, 90), (107, 111)]):
        ax.plot(t, odom_error, label="Observer vs mixed ODOM")
        ax.plot(t, pva_error, label="Observer vs INSPVA")
        ax.plot(t, difference, "k--", lw=1, label="Reference disagreement")
        ax.set_xlim(*bounds)
        ax.set_title("Reference-source transition and observer response")
    axes[1].legend(loc="upper right", fontsize=9)
    for ax in axes:
        for a, b in audit["bestposMatchingIntervals"]:
            ax.axvspan(a, b, color="tab:red", alpha=.12)
        ax.set_ylabel("Position discrepancy (m)")
        ax.set_xlabel("Receiver elapsed time (s)")
        ax.grid(alpha=.2)
    fig.suptitle("Shaded: ODOM positions match BESTPOS. INSPVA is not independent truth.")
    save(fig, "reference_source_diagnosis")

    frequency = np.linspace(.001, 4, 120000)
    z = 2j * np.pi * frequency
    peaks = []
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    names = ["baseline", "zero_auxiliary", "zero_lateral", "theta_15", "delay_075ms", "delay_zero"]
    labels = ["Baseline", "Auxiliary gain = 0", "Lateral output = 0", "Theta = 1.5", "Delay = 75 ms", "Delay = 0 ms"]
    rows = [metrics[metrics["scenario"] == name][0] for name in names]
    y = np.arange(len(names))
    axes[0].barh(y-.17, [r["positionRmseM"] for r in rows], .32, label="Mixed ODOM reference")
    axes[0].barh(y+.17, [r["pvaPositionRmseM"] for r in rows], .32, label="INSPVA reference")
    axes[0].set_yticks(y, labels); axes[0].invert_yaxis()
    axes[0].set_xlabel("Whole-sequence position RMSE (m)")
    axes[0].set_title("Frozen matching: one downstream change per run")
    axes[0].legend(loc="lower right", fontsize=9)
    for theta, delay in [(2, .15), (2, .075), (2, 0), (1.5, .15)]:
        polynomial = 3*theta*z*z + 3*theta*theta*z + 1.5*theta**3
        response = np.abs(polynomial*np.exp(-delay*z)/(z**3+polynomial*np.exp(-delay*z)))
        k = int(np.argmax(response))
        peaks.append(dict(theta=theta, delaySeconds=delay, peakGain=float(response[k]), frequencyHz=float(frequency[k])))
        axes[1].plot(frequency, response, label=f"Theta {theta:g}, delay {1000*delay:g} ms")
    axes[1].axhline(1, color="k", ls="--", lw=.7)
    axes[1].set(xlim=(0, 3), xlabel="Frequency (Hz)", ylabel="Position noise amplitude gain",
                title="Local transfer: course rate = 0, auxiliary = 0, W = I")
    axes[1].legend(fontsize=9)
    for ax in axes: ax.grid(alpha=.2)
    save(fig, "delay_gain_diagnosis")

    calls = read(ROOT / "output/mncav_full_localization_20260914/matching/calls.csv")
    native_pva = read(OUT / "pva_reference.csv")
    mask = (calls["timeSeconds"] >= t[0]) & (calls["timeSeconds"] <= t[-1])
    scan_t = calls["timeSeconds"][mask]
    common_reference = np.column_stack([np.interp(scan_t, native_pva["time"], native_pva[n]) for n in ["x", "y"]])
    common_observer = np.column_stack([np.interp(scan_t, t, estimate[:,j]) for j in range(2)])
    common_matcher = np.column_stack([calls["x"][mask], calls["y"][mask]])
    accepted_t = calls["timeSeconds"][calls["accepted"] == 1]
    query = t-.15
    right = np.clip(np.searchsorted(accepted_t, query), 1, len(accepted_t)-1)
    gap = accepted_t[right]-accepted_t[right-1]
    long_gap = gap>.2
    details = dict(peaks=peaks, commonScanCount=int(mask.sum()),
        commonScanPvaMatcherRmseM=float(np.sqrt(np.mean(np.sum((common_matcher-common_reference)**2, axis=1)))),
        commonScanPvaObserverRmseM=float(np.sqrt(np.mean(np.sum((common_observer-common_reference)**2, axis=1)))),
        samplesReconstructedAcrossGapOver02Seconds=int(long_gap.sum()),
        theirMixedOdomSquaredErrorShare=float(np.sum(odom_error[long_gap]**2)/np.sum(odom_error**2)),
        note="Gap-conditioned association is descriptive, not causal attribution. Transfer is a frozen scalar approximation validated separately.")
    (OUT / "signal_analysis.json").write_text(json.dumps(details, indent=2)+"\n")
    print(json.dumps(details, indent=2))


if __name__ == "__main__":
    main()
