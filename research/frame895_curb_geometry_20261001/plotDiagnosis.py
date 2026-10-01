"""Render the recorded frame-895 diagnostics without opening a desktop window."""
from pathlib import Path
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["svg.hashsalt"] = "frame895-covariance-diagnosis"

HERE = Path(__file__).resolve().parent
OUTPUT = HERE.parents[1] / "output/frame895_curb_geometry_20261001"


def rows(name):
    with (HERE / name).open() as stream:
        return list(csv.DictReader(stream))


fig, axes = plt.subplots(3, 1, figsize=(10, 11), constrained_layout=True)
fine = np.loadtxt(OUTPUT / "frame895_fine_points.csv", delimiter=",")
source = rows("source_directions.csv")
source = [r for r in source if r["centerVariant"] == "1" and r["transportVariant"] == "1"]
axes[0].scatter(fine[:, 0], fine[:, 1], s=10, color="#16856a", label="Frozen fine curb points, frame 895")
axes[0].plot([float(r["x"]) for r in source], [float(r["y"]) for r in source],
             "o-", ms=4, label="Five-scan coarse source centers")
x = np.linspace(5, 13, 60)
axes[0].plot(x, -2.17900445732 + np.tan(np.deg2rad(4.13912323871113)) * (x - 8.9442810857675),
             "--", label="Published map-685 principal axis")
axes[0].set(xlim=(5, 17), ylim=(-2.6, -1.4), xlabel="Reference-body longitudinal X (m)",
            ylabel="Lateral Y (m)", title="Detected support can remain geometrically inconsistent")
axes[0].legend(fontsize=9, loc="upper left")

cov = rows("map_covariance.csv")
colors = ["#ba3d32", "#16856a", "#b38415", "#777777"]
labels = ["Published predictive covariance", "Within-acquisition shape", "Stable-offset prior", "Reference-mean uncertainty"]
for r, color, label in zip(cov, colors, labels):
    # Common origin and unit longitudinal scale compare angles, not uncertainty sizes.
    angle = float(r["angleDeg"])
    axes[1].plot([-4, 4], [-4*np.tan(np.deg2rad(angle)), 4*np.tan(np.deg2rad(angle))],
                 color=color, label=f"{label}: {angle:.3f} deg")
axes[1].set(xlabel="Common-origin longitudinal offset (m)", ylabel="Principal-axis lateral offset (m)",
            title="Anisotropic uncertainty rotates the published axis by 0.656 degrees")
axes[1].legend(fontsize=9)

controls = rows("map_shape_controls.csv")
chosen = [controls[0], controls[4], controls[5]]
labels = ["Production", "Within-acquisition map shape", "Same + reference transport\n(offline oracle)"]
positions = [100*float(r["errorM"]) for r in chosen]
lateral = [100*abs(float(r["lateralM"])) for r in chosen]
xx = np.arange(3)
axes[2].bar(xx-.18, positions, .36, label="Position discrepancy")
axes[2].bar(xx+.18, lateral, .36, label="Absolute lateral discrepancy")
for at, value in zip(xx-.18, positions):
    axes[2].text(at, value+.3, f"{value:.2f}", ha="center", fontsize=10)
axes[2].set(xticks=xx, xticklabels=labels, ylabel="Discrepancy against recorded reference (cm)",
            ylim=(0, 19), title="Isolated frame-895 controls; not a full-route performance claim")
axes[2].legend()
for ax in axes:
    ax.grid(alpha=.2)
    ax.set_axisbelow(True)
svg_path = HERE / "diagnosis.svg"
fig.savefig(svg_path, metadata={"Date": None})
svg_path.write_text("\n".join(line.rstrip() for line in svg_path.read_text().splitlines()) + "\n")
fig.savefig(OUTPUT / "diagnosis.png", dpi=150)
