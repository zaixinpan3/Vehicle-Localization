"""Retain compact LIO comparisons and plots, never per-frame numerical arrays."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scoreTrajectory import score


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--temporary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    a = parser.parse_args()
    rows = []
    for path in sorted(a.output.glob("drive*_v*.json")):
        metrics = json.loads(path.read_text())
        rows.append(
            {
                key: metrics[key]
                for key in [
                    "drive",
                    "mode",
                    "voxelM",
                    "frames",
                    "completedFrames",
                    "complete",
                    "positionRmseM",
                    "positionP95M",
                    "positionMaximumM",
                    "headingRmseDeg",
                    "processingMeanMs",
                    "processingP95Ms",
                    "mapPointsMaximum",
                    "maximumScanEndOffsetMs",
                ]
            }
            | {"trial": path.stem}
        )
    with (a.output / "metrics.csv").open("w") as out:
        writer = csv.DictWriter(out, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    comparisons = []
    for drive in [1, 2]:
        old = (
            a.output.parent
            / "mississippi_baselines_20261030"
            / f"drive{drive}_metrics.csv"
        )
        with old.open() as f:
            for row in csv.DictReader(f):
                if (
                    row["method"] == "proposed"
                    and row["scenario"] in ["both", "no_gnss"]
                    and row["window"] == ("after_2s" if drive == 1 else "all")
                ):
                    comparisons.append(
                        {
                            "drive": drive,
                            "method": "Proposed " + row["scenario"],
                            "frames": row["frames"],
                            "rmseM": row["rmseM"],
                            "maximumM": row["maximumM"],
                            "headingRmseDeg": row["headingRmseDeg"],
                        }
                    )
        for mode in ["native", "prior"]:
            m = next(row for row in rows if row["trial"] == f"drive{drive}_{mode}_v0.5")
            comparisons.append(
                {
                    "drive": drive,
                    "method": "FAST-LIO2 " + mode,
                    "frames": m["frames"],
                    "rmseM": m["positionRmseM"],
                    "maximumM": m["positionMaximumM"],
                    "headingRmseDeg": m["headingRmseDeg"],
                }
            )
    with (a.output / "comparison.csv").open("w") as out:
        writer = csv.DictWriter(out, fieldnames=comparisons[0])
        writer.writeheader()
        writer.writerows(comparisons)
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), layout="constrained")
    for col, drive in enumerate([1, 2]):
        for mode, color in [("native", "#b34a2b"), ("prior", "#20639b")]:
            _, curve = score(
                drive,
                a.temporary,
                a.temporary / f"drive{drive}" / f"drive{drive}_{mode}_v0.5.csv",
            )
            row = 0 if mode == "native" else 1
            axes[row, col].plot(curve["time"], curve["error"], color=color, lw=1)
            axes[row, col].set(
                title=f"Drive {drive}: FAST-LIO2 {mode}",
                xlabel="Receiver time (s)",
                ylabel="Planar position error (m)",
            )
            axes[row, col].grid(alpha=0.25)
    fig.savefig(a.output / "lio_errors.png", dpi=180)
    plt.close(fig)
    print(json.dumps(comparisons, indent=2))


if __name__ == "__main__":
    main()
