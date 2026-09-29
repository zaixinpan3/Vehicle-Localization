"""Summarize actual replay artifacts; do not infer independent-drive accuracy."""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def metrics(table):
    errors = table.errorM.to_numpy()
    operational = table.loc[table.frame > 1]
    maximum = operational.loc[operational.errorM.idxmax()]
    return dict(samples=len(table), maximumFrame=int(maximum.frame),
                maximumErrorM=float(maximum.errorM),
                rmseM=float(np.sqrt(np.mean(errors**2))),
                p95M=float(np.quantile(errors, .95, method="hazen")),
                full=int(table.accepted.sum()), directional=int(table.directional.sum()))


def main():
    before = pd.read_csv(ROOT / "research/root_cause_matching_20260929/finalSurface.csv")
    after = pd.read_csv(HERE / "final_raw.csv")
    verify = pd.read_csv(HERE / "final_raw_verification.csv")
    tests = pd.read_csv(HERE / "final_tests.csv")
    analyzer = pd.read_csv(HERE / "code_analysis.csv")
    assert len(before) == len(after) == len(verify) == 1170
    assert verify.identicalCurrentCloud.all() and verify.identicalWindowCloud.all()
    assert verify.maximumPoseDifference.max() < 1e-7
    assert tests.passed.all() and not tests.failed.any() and not tests.incomplete.any()
    assert not analyzer.findings.any()
    assert before.errorM.iloc[0] == after.errorM.iloc[0]
    baseline, final = metrics(before), metrics(after)
    cached = pd.read_csv(HERE / "productionAll.csv")
    assert np.max(np.abs(after[["x", "y", "psi"]].values - cached[["x", "y", "psi"]].values)) < 1e-7
    control_rows = []
    for name, mask in [("productionEven", lambda f: (f % 2 == 1)),
                       ("productionQuarter", lambda f: ((f - 1) % 4 != 0))]:
        control = pd.read_csv(HERE / f"{name}.csv")
        held = mask(control.frame) & (control.frame > 1)
        row = metrics(control.loc[held])
        row.update(variant=name, baseline=metrics(before.loc[held]),
                   scope="Point-landmark observation model excludes scored frames; original curb map and grouping scaffold still reuse the drive.")
        control_rows.append(row)
    summaries = []
    for path in sorted(HERE.glob("*_summary.csv")):
        table = pd.read_csv(path)
        if "variant" in table and "maximumErrorM" in table:
            summaries.extend(table.to_dict("records"))
    pd.DataFrame(summaries).to_csv(HERE / "all_replay_summaries.csv", index=False)
    runtime = pd.read_csv(HERE / "paired_matching_runtime_summary.csv").iloc[0].to_dict()
    summary = dict(baselineCommit="0ac9acc76da7f5565dcbc01313c2de65250a0d37", frames=1170,
                   initialErrorM=float(after.errorM.iloc[0]), baseline=baseline, final=final,
                   maximumReductionPercent=100 * (1 - final["maximumErrorM"] / baseline["maximumErrorM"]),
                   rmseReductionPercent=100 * (1 - final["rmseM"] / baseline["rmseM"]),
                   p95ReductionPercent=100 * (1 - final["p95M"] / baseline["p95M"]),
                   unchangedCurrentClouds=int(verify.identicalCurrentCloud.sum()),
                   unchangedWindowClouds=int(verify.identicalWindowCloud.sum()),
                   maximumRawCachedPoseDifference=float(verify.maximumPoseDifference.max()),
                   testsPassed=len(tests), testSuites=21, codeAnalyzerFiles=len(analyzer),
                   heldOutPointObservationControls=control_rows, pairedRuntime=runtime,
                   completedReplayLabels=len(summaries),
                   scope="Reused Mississippi route, same-drive map. Configured first frame excluded only from maximum. No online fine labels or reference XY/yaw resets. No independent-drive or surveyed-truth claim.",
                   runtimeScope="Alternating-order pairs including crop, conditioning and solve; concurrent external MATLAB workload observed, not a real-time guarantee.")
    (HERE / "final_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    after.sort_values("errorM", ascending=False).loc[lambda t: t.frame > 1].head(20).to_csv(HERE / "final_largest_errors.csv", index=False)
    merged = after[["frame", "errorM"]].merge(before[["frame", "errorM"]], on="frame", suffixes=("After", "Before"))
    merged.to_csv(HERE / "route_comparison.csv", index=False)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), constrained_layout=True)
    for table, label, color in [(before, "Previous production", "#939da7"), (after, "View-conditioned map", "#006d77")]:
        table = table.loc[table.frame > 1]
        axes[0].plot(table.frame, 100 * table.errorM, lw=1, color=color, label=label)
        values = np.sort(100 * table.errorM.to_numpy())
        axes[1].plot(values, np.arange(1, len(values) + 1) / len(values), color=color, label=label)
    axes[0].set(xlabel="Mississippi frame", ylabel="Horizontal discrepancy (cm)",
                title="Same-drive causal replay; configured first frame omitted from this figure")
    axes[1].set(xlabel="Horizontal discrepancy (cm)", ylabel="Empirical fraction", xlim=(0, 32))
    for ax in axes:
        ax.grid(alpha=.2)
        ax.legend()
    fig.savefig(HERE / "route_comparison.png", dpi=160)
    fig.savefig(HERE / "route_comparison.pdf")
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
