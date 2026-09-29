"""Audit temporal shape compatibility and summarize actual route controls."""
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
baseline = pd.read_csv(study / "baseline.csv")
previous = pd.read_csv(root / "research/partial_sign_matching_20260929/final_raw.csv")
adopted = pd.read_csv(study / "final_raw.csv")
cached = pd.read_csv(study / "shape50.csv")
verification = pd.read_csv(study / "final_raw_verification.csv")
audit = pd.read_csv(study / "shape50_audit.csv")
tests = pd.read_csv(study / "final_tests.csv")
analysis = pd.read_csv(study / "code_analysis.csv")
assert len(adopted) == len(cached) == len(verification) == len(audit) == 1170
assert verification.identicalOriginalCurrentCloud.all() and verification.identicalAdoptedWindowCloud.all()
assert verification.maximumPoseDifference.max() < 1e-7
assert len(tests) == 322 and tests.passed.all() and not tests.failed.any() and not tests.incomplete.any()
assert len(analysis) == 3 and not analysis.findings.any()
baseline_delta = float(np.max(np.abs(baseline[["x", "y", "psi"]].to_numpy()-previous[["x", "y", "psi"]].to_numpy())))
assert baseline_delta < 1e-7
assert pd.read_csv(study / "baseline_audit.csv").identicalBaselineSources.all()
assert audit.shapeRejectedPairs.sum() > 0
def metrics(frame):
    peak = frame.loc[frame.loc[frame.frame > 1, "errorM"].idxmax()]
    return {"maximumFrame": int(peak.frame), "maximumM": float(peak.errorM),
            "rmseM": float(np.sqrt(np.mean(frame.errorM**2))), "p95M": float(np.percentile(frame.errorM, 95, method="hazen")),
            "frame178M": float(frame.errorM.iloc[177]), "fullUpdates": int(frame.accepted.sum()), "directionalUpdates": int(frame.directional.sum())}
summary = {"baseline": metrics(baseline), "adopted": metrics(adopted), "frames": 1170,
           "rawCachedMaximumPoseDifference": float(verification.maximumPoseDifference.max()),
           "disabledGateBaselineMaximumPoseDifference": baseline_delta,
           "unchangedCurrentClouds": 1170, "exactAdoptedRawWindowClouds": 1170,
           "changedWindowCloudFrames": int((audit.identicalBaselineSources == 0).sum()),
           "shapeRejectedPairTests": int(audit.shapeRejectedPairs.sum()),
           "framesWithShapeRejectedPairs": int((audit.shapeRejectedPairs > 0).sum()),
           "positionCompatiblePairTests": int(audit.positionCompatiblePairs.sum()),
           "baselineComponentOccurrences": int(audit.baselineComponents.sum()), "adoptedComponentOccurrences": int(audit.components.sum()),
           "passingTests": len(tests), "codeAnalyzerFiles": 3, "codeAnalyzerFindings": 0,
           "pairedWindowTiming": pd.read_csv(study / "paired_window_runtime_summary.csv").iloc[0].to_dict(),
           "shapeThreshold": .5, "shapeVarianceFloorM2": .0004,
           "additionalShapeStatisticsInSourceCloud": False,
           "queryFineLabelsUsed": False}
assert 0 <= summary["changedWindowCloudFrames"] <= 1170
assert summary["adoptedComponentOccurrences"] == verification.components.sum()
assert np.max(np.abs(adopted[["x", "y", "psi"]].to_numpy()-cached[["x", "y", "psi"]].to_numpy())) < 1e-7
summary["baselineSourceCounts"] = {name: int(baseline[name].sum()) for name in ["poles", "signs", "curbs"]}
summary["adoptedSourceCounts"] = {name: int(cached[name].sum()) for name in ["poles", "signs", "curbs"]}
original_sign = pd.read_csv(study / "shape50_originalSignResidual.csv")
summary["shapeGateWithOriginalSignResidual"] = metrics(original_sign)
summary["rmseDifferenceMm"] = 1000*(summary["adopted"]["rmseM"]-summary["baseline"]["rmseM"])
summary["maximumDifferenceMm"] = 1000*(summary["adopted"]["maximumM"]-summary["baseline"]["maximumM"])
rows = []
for path in sorted(study.glob("*_summary.csv")):
    frame = pd.read_csv(path)
    if "maximumErrorM" in frame:
        row = frame.iloc[0].to_dict(); row["resultCsv"] = path.name.replace("_summary", ""); rows.append(row)
pd.DataFrame(rows).to_csv(study / "route_comparison.csv", index=False)
(study / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

fig, axes = plt.subplots(2, 1, figsize=(10, 6.4), constrained_layout=True)
axes[0].plot(baseline.frame.iloc[1:], baseline.errorM.iloc[1:]*100, label="Previous temporal association", color="#64748b", linewidth=.85)
axes[0].plot(adopted.frame.iloc[1:], adopted.errorM.iloc[1:]*100, label="Center and shape association", color="#168a7b", linewidth=.85)
axes[0].set(xlabel="Frame", ylabel="Horizontal discrepancy (cm)", title="Mississippi: association consistency repair, peak essentially unchanged")
axes[0].legend()
axes[1].plot(audit.frame, audit.shapeRejectedPairs, color="#bb4a37", linewidth=.9)
axes[1].set(xlabel="Frame", ylabel="Rejected candidate pair tests", title="Position-compatible candidates rejected by covariance shape")
fig.savefig(study / "route_comparison.png", dpi=180)
fig.savefig(study / "route_comparison.pdf")
plt.close(fig)

fig, ax = plt.subplots(figsize=(5, 4.5), constrained_layout=True)
angle = np.linspace(0, 2*np.pi, 400)
circle = np.array([np.cos(angle), np.sin(angle)])
for covariance, color, label in [(np.diag([.04, .001]), "#4057a6", "Acquisition 1"), (np.diag([.001, .04]), "#168a7b", "Acquisition 2")]:
    points = 2*np.sqrt(covariance)@circle
    ax.plot(*points, color=color, label=label, linewidth=2)
ax.plot(0, 0, "+", color="#bb4a37", markersize=8)
ax.set(aspect="equal", xlabel="X (m)", ylabel="Y (m)", title="Same center, conflicting probability-cloud shapes")
ax.legend(loc="upper right")
fig.savefig(study / "shape_conflict.png", dpi=180)
fig.savefig(study / "shape_conflict.pdf")
plt.close(fig)

def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()
inputs = ["output/root_cause_matching_20260929/finalSurface_sources.mat", "output/mississippi_mapping_calibrated/view_conditioned_cloud.mat",
          "output/line_direction_matching_20260928/sources.mat", "research/partial_sign_matching_20260929/final_raw.csv"]
(study / "input_hashes.json").write_text(json.dumps({name: digest(root/name) for name in inputs}, indent=2) + "\n")
(study / "artifact_hashes.json").write_text(json.dumps({str(p.relative_to(study)): digest(p) for p in sorted(study.rglob("*"))
    if p.is_file() and p.name != "artifact_hashes.json"}, indent=2) + "\n")
print(json.dumps(summary, indent=2))
