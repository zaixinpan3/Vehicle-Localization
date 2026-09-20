"""Independently score complete replays and audit reference-free seed recursion."""
from pathlib import Path
import json
import hashlib
import h5py
import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = Path(__file__).resolve().parents[2]
OUT = REPO / "output/reference_free_785_20260920"
REPORT = Path(__file__).resolve().parent
MODES = ("matching_prediction", "fusion_prediction")


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def score(name, population, frame, pose, reference, mask):
    error = np.linalg.norm(pose[mask, :2] - reference[mask, :2], axis=1)
    yaw = np.rad2deg(wrap(pose[mask, 2] - reference[mask, 2]))
    assert error.size and np.isfinite(error).all() and np.isfinite(yaw).all()
    return dict(method=name, population=population, samples=int(mask.sum()),
                positionRmseM=float(np.sqrt(np.mean(error**2))),
                medianM=float(np.median(error)), p95M=float(np.quantile(error, .95, method="hazen")),
                maximumM=float(error.max()), worstFrame=int(frame[mask][error.argmax()]),
                headingRmseDeg=float(np.sqrt(np.mean(yaw**2))),
                over50cm=int((error > .5).sum()), over1m=int((error > 1).sum()))


def main():
    old = pd.read_csv(OUT / "historical_fused.csv")
    reference = old[["referenceX", "referenceY", "referencePsi"]].to_numpy()
    frames = old.frame.to_numpy()
    assert np.array_equal(frames, np.arange(1, 1170))
    all_frames = np.ones(len(old), dtype=bool)
    historic = pd.read_csv(OUT / "historical_matching.csv")
    old_valid = historic.accepted.to_numpy(bool)
    motion = pd.read_csv(OUT / "motion.csv")
    bootstrap = pd.read_csv(OUT / "bootstrap_candidates.csv")
    winner = bootstrap.loc[bootstrap.accepted.astype(bool)].sort_values("similarity", ascending=False).iloc[0]
    with h5py.File(REPO / "output/mncav_bestpos_alignment_20260917/experiment.mat") as saved:
        gnss = saved["data/gnss/position"][:, 0]
        offset = saved["cfg/gnss/outputPoint/bodyOffset"][...].ravel()
    yaw = bootstrap.initialPsi.to_numpy()
    rotation_offset = np.column_stack((np.cos(yaw)*offset[0]-np.sin(yaw)*offset[1],
                                      np.sin(yaw)*offset[0]+np.cos(yaw)*offset[1]))
    bootstrap_seed_difference = np.abs(gnss-rotation_offset-bootstrap[["initialX","initialY"]].to_numpy()).max()
    assert bootstrap_seed_difference < 1e-8
    assert np.max(np.abs(np.rad2deg(yaw)-np.arange(-180, 180, 10))) < 1e-10
    checks = dict(frames=len(old), historicalAccepted=int(old_valid.sum()),
                  bootstrapSeedCorrectionDifference=float(bootstrap_seed_difference),
                  bootstrapSelected=int(winner.candidate),
                  bootstrapInitialHeadingDeg=float(np.rad2deg(winner.initialPsi)),
                  bootstrapPositionErrorM=float(np.linalg.norm(winner[["x", "y"]].to_numpy(float) - reference[0, :2])),
                  bootstrapHeadingErrorDeg=float(np.rad2deg(wrap(winner.psi-reference[0, 2]))))
    rows = []
    fused_curves = {}
    matching_curves = {}
    for name in ("historical_fused", "historical_gnss_only"):
        df = pd.read_csv(OUT / f"{name}.csv")
        pose = df[["x", "y", "psi"]].to_numpy()
        rows.append(score(name, "all_frames", frames, pose, reference, all_frames))
        fused_curves[name] = np.linalg.norm(pose[:, :2] - reference[:, :2], axis=1)
    assert abs(rows[0]["positionRmseM"] - .0784742734612255) < 1e-8
    old_pose = historic[["x", "y", "psi"]].to_numpy()
    rows.append(score("historical_matching", "accepted", frames, old_pose, reference, old_valid))
    matching_curves["historical_matching"] = np.linalg.norm(old_pose[:, :2] - reference[:, :2], axis=1)
    for mode in MODES:
        calls = pd.read_csv(OUT / mode / "matching.csv")
        fused = pd.read_csv(OUT / mode / "fused.csv")
        gnss = pd.read_csv(OUT / mode / "gnss_only.csv")
        assert np.array_equal(calls.frame, frames)
        assert np.max(np.abs(calls.time - motion.time)) < 1e-9
        pose = calls[["x", "y", "psi"]].to_numpy()
        fused_pose = fused[["x", "y", "psi"]].to_numpy()
        # Reconstruct every seed from sensor motion and the previous estimated
        # output. This check needs no reference array.
        previous = (pose if mode == "matching_prediction" else fused_pose)[:-1]
        dt = np.diff(motion.time)
        yaw_step = dt * (motion.yawRate.to_numpy()[:-1] + motion.yawRate.to_numpy()[1:]) / 2
        vx = motion.longitudinalSpeed.to_numpy()
        vy = motion.lateralVelocity.to_numpy()
        vx = (vx[:-1] + vx[1:]) / 2
        vy = (vy[:-1] + vy[1:]) / 2
        yaw = previous[:, 2] + yaw_step / 2
        distance = dt * np.sinc(yaw_step / (2*np.pi))
        expected = np.column_stack((previous[:, 0] + distance*(np.cos(yaw)*vx-np.sin(yaw)*vy),
                                    previous[:, 1] + distance*(np.sin(yaw)*vx+np.cos(yaw)*vy),
                                    wrap(previous[:, 2]+yaw_step)))
        seeds = calls[["predictedX", "predictedY", "predictedPsi"]].to_numpy()
        difference = np.abs(expected-seeds[1:]).max()
        assert difference < 3e-8, (mode, difference)
        assert np.max(np.abs(seeds[0] - winner[["initialX", "initialY", "initialPsi"]].to_numpy(float))) < 1e-8
        valid = calls.accepted.to_numpy(bool)
        assert np.array_equal((fused["mode"].to_numpy(int) & 2) != 0, valid)
        assert np.max(np.abs(pose[~valid] - seeds[~valid]), initial=0) < 1e-8
        I = np.zeros((len(calls), 3, 3))
        for i, j, suffix in ((0,0,"XX"),(0,1,"XY"),(0,2,"XPsi"),(1,1,"YY"),(1,2,"YPsi"),(2,2,"PsiPsi")):
            I[:,i,j] = I[:,j,i] = calls[f"information{suffix}"]
        assert np.linalg.eigvalsh(I[valid]).min() > 0
        checks[mode] = dict(accepted=int(valid.sum()), rejected=int((~valid).sum()),
                            maximumSeedReconstructionDifference=float(difference))
        for suffix, trajectory in (("fused", fused_pose), ("gnss_only", gnss[["x","y","psi"]].to_numpy()),
                                   ("matching_with_prediction_fallback", pose)):
            rows.append(score(f"{mode}/{suffix}", "all_frames", frames, trajectory, reference, all_frames))
        rows.append(score(f"{mode}/matching", "accepted", frames, pose, reference, valid))
        common = valid & old_valid
        rows.append(score(f"{mode}/matching", "common_accepted_with_historical", frames, pose, reference, common))
        rows.append(score(f"historical_matching_vs_{mode}", "common_accepted_with_historical", frames, old_pose, reference, common))
        fused_curves[mode] = np.linalg.norm(fused_pose[:, :2]-reference[:, :2], axis=1)
        curve = np.linalg.norm(pose[:, :2]-reference[:, :2], axis=1)
        curve[~valid] = np.nan
        matching_curves[mode] = curve
    metrics = pd.DataFrame(rows)
    metrics.to_csv(OUT / "metrics.csv", index=False)
    metrics.to_csv(REPORT / "metrics.csv", index=False)
    for folder in (OUT, REPORT):
        (folder / "audit.json").write_text(json.dumps(checks, indent=2) + "\n")
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    names = {"historical_fused":"Historical fusion (reference-seeded matching)",
             "historical_gnss_only":"Historical GNSS only", "matching_prediction":"No reference seeds: matching feedback",
             "fusion_prediction":"No reference seeds: fusion feedback", "historical_matching":"Historical accepted matching"}
    colors = {"historical_fused":"tab:blue", "historical_matching":"tab:blue",
              "historical_gnss_only":"tab:orange", "matching_prediction":"tab:green",
              "fusion_prediction":"tab:red"}
    for ax, curves, title in ((axes[0], fused_curves, "Fused position: all 1,169 frames"),
                              (axes[1], matching_curves, "Raw matching: accepted poses only; rejected gaps retained")):
        for name, error in curves.items():
            ax.plot(old.time, error, label=names[name], color=colors[name], linewidth=1)
        ax.set_ylabel("Position discrepancy (m)")
        ax.set_title(title)
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    axes[1].set_xlabel("Time (s)")
    fig.savefig(OUT / "comparison.png", dpi=180)
    fig.savefig(OUT / "comparison.pdf")
    # Hash independent technical artifacts only; no archive weekly/monthly
    # documents enter this manifest. Large runtime/data artifacts stay local.
    inputs = [REPO / name for name in (
        "output/mncav_bestpos_alignment_20260917/experiment.mat",
        "output/saved_perception_inspva_20260915/experiment.mat",
        "output/mississippi_mapping_inspva_20260915/probability_cloud.mat")]
    outputs = [p for p in OUT.rglob("*") if p.is_file() and "historical_runtime" not in p.parts]
    sources = [p for p in REPORT.iterdir() if p.suffix in (".m", ".py")]
    manifest = []
    for p in inputs + outputs + sources:
        digest = hashlib.file_digest(p.open("rb"), "sha256").hexdigest()
        manifest.append(dict(path=str(p.relative_to(REPO)), bytes=p.stat().st_size, sha256=digest,
                             role="input" if p in inputs else ("source" if p in sources else "output")))
    (REPORT / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(metrics.to_string(index=False))
    print(json.dumps(checks, indent=2))


if __name__ == "__main__":
    main()
