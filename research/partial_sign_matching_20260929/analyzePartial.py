"""Validate the adopted matching change and export static route comparisons."""
from pathlib import Path
import hashlib
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

study = Path(__file__).resolve().parent
root = study.parents[1]
baseline = pd.read_csv(root / "research/matching_objective_20260929/final_raw.csv")
adopted = pd.read_csv(study / "final_raw.csv")
cached = pd.read_csv(study / "production.csv")
verification = pd.read_csv(study / "final_raw_verification.csv")
tests = pd.read_csv(study / "final_tests.csv")
targeted = pd.read_csv(study / "targeted_tests.csv").rename(columns={"Name": "test", "Passed": "passed", "Failed": "failed", "Incomplete": "incomplete"})
unique_tests = pd.concat([tests[["test", "passed", "failed", "incomplete"]], targeted[["test", "passed", "failed", "incomplete"]]]).drop_duplicates("test", keep="last")
analysis = pd.read_csv(study / "code_analysis.csv")
assert len(adopted) == len(cached) == len(verification) == 1170
assert verification.identicalCurrentCloud.all() and verification.identicalWindowCloud.all()
assert verification.maximumPoseDifference.max() < 1e-7
assert len(tests) in (311, 312) and len(targeted) == 9 and len(unique_tests) == 312
assert unique_tests.passed.all() and not unique_tests.failed.any() and not unique_tests.incomplete.any()
assert len(analysis) == 6 and not analysis.findings.any()
pairs = pd.read_csv(study / "production_frame178/pairs.csv").set_index("source")
assert pairs.loc[22, "partialSignSurface"] == 1 and pairs.loc[22, "weight"] > 0
assert not pairs.loc[[21, 30], "partialSignSurface"].any()
legacy = pd.read_csv(study / "legacyParity.csv")
legacy_reference = pd.read_csv(root / "research/root_cause_matching_20260929/finalSurface.csv")
legacy_delta = float(np.max(np.abs(legacy[["x", "y", "psi"]].to_numpy()-legacy_reference[["x", "y", "psi"]].to_numpy())))
assert legacy_delta < 1e-7
def metrics(frame):
    scored = frame[frame.frame > 1]
    peak = scored.loc[scored.errorM.idxmax()]
    frame178 = frame.loc[frame.frame == 178, "errorM"]
    return {"maximumFrame": int(peak.frame), "maximumM": float(peak.errorM), "rmseM": float(np.sqrt(np.mean(frame.errorM**2))),
            # MATLAB prctile uses midpoint plotting positions (Hazen).
            "p95M": float(np.percentile(frame.errorM, 95, method="hazen")), "frame178M": float(frame178.iloc[0]) if len(frame178) else None}
summary = {"baseline": metrics(baseline), "adopted": metrics(adopted), "frames": 1170,
           "frame178ReductionPercent": float(100*(1-adopted.errorM.iloc[177]/baseline.errorM.iloc[177])),
           "fullUpdates": int(adopted.accepted.sum()), "directionalUpdates": int(adopted.directional.sum()),
           "unchangedCurrentClouds": 1170, "unchangedWindowClouds": 1170,
           "rawCachedMaximumPoseDifference": float(verification.maximumPoseDifference.max()),
           "legacyMapMaximumPoseDifference": legacy_delta, "uniquePassingTests": 312, "completeSuiteRunTests": len(tests),
           "finalTargetedTests": 9, "codeAnalyzerFiles": 6, "codeAnalyzerFindings": 0,
           "sign22RetainedNormalConstraint": True, "queryFineLabelsUsed": False}
summary["maximumReductionPercent"] = 100*(1-summary["adopted"]["maximumM"]/summary["baseline"]["maximumM"])
even = pd.read_csv(study / "evenMap.csv")
old_even = pd.read_csv(root / "research/matching_objective_20260929/productionEven.csv")
excluded = (even.frame % 2 == 1) & (even.frame > 1)
summary["evenPointMapExcludedOddQueries"] = {"count": int(excluded.sum()), "baseline": metrics(old_even[excluded]), "adopted": metrics(even[excluded])}
timing = pd.read_csv(study / "paired_matching_runtime_summary.csv").iloc[0].to_dict()
summary["pairedTiming"] = timing
rows = []
for path in sorted(study.glob("*_summary.csv")):
    data = pd.read_csv(path)
    if "maximumErrorM" in data:
        row = data.iloc[0].to_dict(); row["resultCsv"] = path.name.replace("_summary", ""); rows.append(row)
pd.DataFrame(rows).to_csv(study / "route_comparison.csv", index=False)
(study / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

fig, axes = plt.subplots(2, 1, figsize=(10, 6.4), constrained_layout=True)
axes[0].plot(baseline.frame.iloc[1:], baseline.errorM.iloc[1:]*100, label="Previous matching", color="#64748b", linewidth=.85)
axes[0].plot(adopted.frame.iloc[1:], adopted.errorM.iloc[1:]*100, label="Partial-sign matching", color="#168a7b", linewidth=.85)
axes[0].set(xlabel="Frame", ylabel="Horizontal discrepancy (cm)", title="Mississippi, same-recording development replay")
axes[0].legend()
select = (adopted.frame >= 170) & (adopted.frame <= 185)
axes[1].plot(baseline.frame[select], baseline.errorM[select]*100, ".-", color="#64748b", label="Previous matching")
axes[1].plot(adopted.frame[select], adopted.errorM[select]*100, ".-", color="#168a7b", label="Partial-sign matching")
axes[1].axvline(178, linestyle="--", color="#d44939", linewidth=.8)
axes[1].set(xlabel="Frame", ylabel="Horizontal discrepancy (cm)", title="The confirmed sign remains in the objective")
axes[1].legend()
fig.savefig(study / "route_comparison.png", dpi=180)
fig.savefig(study / "route_comparison.pdf")
plt.close(fig)

inputs = ["output/root_cause_matching_20260929/finalSurface_sources.mat", "output/mississippi_mapping_calibrated/view_conditioned_cloud.mat", "research/matching_objective_20260929/final_raw.csv"]
def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()
(study / "input_hashes.json").write_text(json.dumps({name: digest(root/name) for name in inputs}, indent=2) + "\n")
(study / "artifact_hashes.json").write_text(json.dumps({str(p.relative_to(study)): digest(p) for p in sorted(study.rglob("*")) if p.is_file() and p.name != "artifact_hashes.json"}, indent=2) + "\n")
print(json.dumps(summary, indent=2))
