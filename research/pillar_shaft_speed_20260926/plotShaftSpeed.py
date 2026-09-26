"""Plot measured steady replay latency; bar tops are medians, dots are p95."""

import csv
from pathlib import Path
from statistics import median

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from summarizeShaftSpeed import percentile


def main():
    folder = Path(__file__).resolve().parent
    variants = ["baseline", "serial", "threads4", "threads8"]
    labels = ["Before", "Optimized\n1 worker", "Optimized\n4 workers", "Optimized\n8 workers"]
    colors = ["#7f8c8d", "#df983d", "#247fa1", "#4a9975"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
    for ax, dataset in zip(axes, ["Mississippi", "Downtown"]):
        with (folder / f"{dataset.lower()}_steady_paired.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        grouped = {name: [float(r["milliseconds"]) for r in rows if r["variant"] == name]
                   for name in variants}
        centers = [median(grouped[name]) for name in variants]
        tails = [percentile(grouped[name], .95) for name in variants]
        ax.bar(range(4), centers, color=colors, width=.65)
        ax.scatter(range(4), tails, color="#273746", marker="D", s=22, label="95th percentile")
        for index, (center, tail) in enumerate(zip(centers, tails)):
            ax.plot([index, index], [center, tail], color="#273746", linewidth=1)
            ax.text(index, center - 8, f"{center:.1f}", ha="center", va="top", fontsize=10,
                    color="white" if index>=2 else "#15232b", fontweight="bold")
        frame_count = len({r["frame"] for r in rows})
        ax.set_title(f"{dataset}: {frame_count} frames, two passes")
        ax.set_xticks(range(4), labels, fontsize=9)
        ax.set_axisbelow(True)
        ax.grid(axis="y", alpha=.22)
        ax.spines[["right", "top"]].set_visible(False)
    axes[0].set_ylabel("Perception time per frame [ms]")
    axes[1].legend(frameon=False, loc="upper right")
    fig.suptitle("Same search and outputs, less repeated work and bounded CPU parallelism", fontsize=12)
    fig.text(.5, .015, "Ryzen 7 7800X3D · MATLAB R2026a · warm continuous blocks · reversed order on pass 2\n"
             "Frame I/O, path switching, output comparisons, and plotting are outside the timed region.",
             ha="center", fontsize=8, color="#444444")
    fig.tight_layout(rect=(0, .09, 1, .95))
    fig.savefig(folder / "runtime_comparison.png", dpi=170)
    plt.close(fig)


if __name__ == "__main__":
    main()
