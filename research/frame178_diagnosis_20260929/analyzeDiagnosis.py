"""Validate and summarize the frame-178 counterfactuals without opening a GUI."""
from pathlib import Path
import hashlib
import json
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

study = Path(__file__).resolve().parent
root = study.parents[1]
controls = pd.read_csv(study / "controls.csv").set_index("variant")
mechanisms = pd.read_csv(study / "mechanism_controls.csv").set_index("variant")
objective = pd.read_csv(study / "objectives.csv").set_index("pose")
acquisitions = pd.read_csv(study / "sign_acquisitions.csv")
sign = json.loads((study / "sign_summary.json").read_text())
detail = json.loads((study / "mechanism_summary.json").read_text())
validation = json.loads((study / "validation.json").read_text())
reference_pairs = pd.read_csv(study / "reference_pairs.csv").set_index("source")
production_pairs = pd.read_csv(study / "production_pairs.csv").set_index("source")
alternate_pairs = pd.read_csv(study / "reference_seed_frozen_pairs.csv").set_index("source")
target = np.array(sign["mapCenterBody"])
track = acquisitions[acquisitions.pooledSource == 22]
assert len(controls) + len(mechanisms) == validation["controlledRegistrations"] == 31
assert mechanisms.loc["iterations_400", "errorM"] == controls.loc["production", "errorM"]
assert mechanisms.loc["drop22_preserve_other_weights", "errorM"] < .02
assert controls.loc["drop_sign_22", "errorM"] < .01
assert objective.loc["production", "cost"] < objective.loc["reference", "cost"]
assert track.frame.tolist() == list(range(174, 179))
assert sign["frame178LocalFineSignCount"] == 158
assert validation["codeAnalyzerFindings"] == 0
changed = production_pairs.index[production_pairs.target != alternate_pairs.target].tolist()
summary = {
    "baselineCommit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
    "frame": 178, "errorM": float(controls.loc["production", "errorM"]),
    "bodyErrorM": controls.loc["production", ["longitudinalM", "lateralM"]].tolist(),
    "incomingPredictionErrorM": detail["predictionErrorM"],
    "sign22DisplacementM": float(np.linalg.norm(np.array(sign["source22Mean"]) - target)),
    "sign22FineSubsetDisplacementM": float(np.linalg.norm(np.array(detail["sign22FineCenterOracle"]) - target)),
    "sign22FineFractionAcrossFiveScans": sign["source22AllFramesFineFraction"],
    "sign22CauchyInfluenceAtReference": float(reference_pairs.loc[22, "robustWeight"] / reference_pairs.loc[22, "weight"]),
    "sign22CauchyInfluenceAtProduction": float(production_pairs.loc[22, "robustWeight"] / production_pairs.loc[22, "weight"]),
    "weightPreservingAblationErrorM": float(mechanisms.loc["drop22_preserve_other_weights", "errorM"]),
    "ordinaryRemovalErrorM": float(controls.loc["drop_sign_22", "errorM"]),
    "fineSubsetCenterOracleErrorM": float(mechanisms.loc["sign22_fine_center_oracle", "errorM"]),
    "poleFullWeightDiagnosticErrorM": float(controls.loc["pool_add_pole_full", "errorM"]),
    "referenceMotionPoolErrorM": float(controls.loc["reference_motion_pool", "errorM"]),
    "referenceSeedFixedMapErrorM": float(controls.loc["reference_seed_frozen", "errorM"]),
    "curbSourcesChangingTargetAcrossBasins": changed,
    "productionChanged": False,
    "scope": "Fixed-frame same-recording diagnostic interventions, not deployable frame-specific rules or an independent-drive validation.",
}
(study / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

points = pd.read_csv(study / "sign_points.csv")
points = points[(points.frame == 178) & (np.hypot(points.x178-target[0], points.y178-target[1]) < 1.5)]
fig, axes = plt.subplots(1, 2, figsize=(11, 4.4), constrained_layout=True)
for fine, color, label in [(0, "#9ca3af", "Other selected returns"), (1, "#168a7b", "Original fine sign labels")]:
    p = points[points.fineSign == fine]
    axes[0].scatter(p.x178, p.y178, s=12, c=color, label=label, alpha=.65)
axes[0].scatter(*target, marker="*", s=150, color="black", label="Conditioned map center")
for source, color in [(21, "#2563eb"), (22, "#d44939"), (30, "#9333ea")]:
    pair = reference_pairs.loc[source]
    xy = pair[["sourceMeanXY_1", "sourceMeanXY_2"]].to_numpy(dtype=float)
    axes[0].scatter(*xy, marker="x", s=90, color=color, linewidth=2, label=f"Pooled sign {source}")
    axes[0].plot([target[0], xy[0]], [target[1], xy[1]], color=color, alpha=.6)
axes[0].set(xlabel="Forward, frame-178 body axes (m)", ylabel="Left (m)", title="Partial sign observations share one map center")
axes[0].set_aspect("equal")
axes[0].legend(fontsize=7, loc="lower right")
labels = ["Production", "400 iterations", "Fine-only sign-22 center*", "Reference-motion pooling*", "Current pole admitted", "Sign-22 force removed**"]
errors = [summary["errorM"], mechanisms.loc["iterations_400", "errorM"], summary["fineSubsetCenterOracleErrorM"], summary["referenceMotionPoolErrorM"], summary["poleFullWeightDiagnosticErrorM"], summary["weightPreservingAblationErrorM"]]
bars = axes[1].barh(labels, np.array(errors)*100, color=["#d44939"]*3+["#64748b", "#2563eb", "#168a7b"])
axes[1].invert_yaxis()
axes[1].bar_label(bars, fmt="%.2f", padding=3, fontsize=8)
axes[1].set(xlabel="Horizontal discrepancy (cm)", title="Fixed-frame counterfactuals", xlim=(0, 19))
axes[1].text(0, -.18, "* Reference-assisted diagnostic. ** Other residual weights preserved.\nAll ablations are retrospective diagnostics; no production change.", transform=axes[1].transAxes, fontsize=7)
fig.savefig(study / "diagnosis.png", dpi=180)
fig.savefig(study / "diagnosis.pdf")
plt.close(fig)

inputs = ["output/matching_objective_20260929/productionAll.mat", "output/root_cause_matching_20260929/finalSurface_sources.mat", "output/mississippi_mapping_calibrated/view_conditioned_cloud.mat", "output/root_cause_matching_20260929/map_point_indices.mat"]
def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()
(study / "input_hashes.json").write_text(json.dumps({name: digest(root/name) for name in inputs}, indent=2) + "\n")
(study / "artifact_hashes.json").write_text(json.dumps({p.name: digest(p) for p in sorted(study.iterdir()) if p.is_file() and p.name != "artifact_hashes.json"}, indent=2) + "\n")
print(json.dumps(summary, indent=2))
