"""Summarize observed populations without treating startup as a LiDAR pose."""
import csv
import json
import math
import statistics
from pathlib import Path

DEST = Path(__file__).resolve().parent
RUNS = {
    "concatenated_baseline": Path("output/lidar_origin_20260922/coarse_pipeline/matching"),
    "confirmed_three_frames": Path("output/temporal_perception_20260922/matching"),
    "confirmed_five_frames": Path("output/temporal_perception_20260922/five_frame_matching"),
}


def read(path):
    with path.open() as source:
        return list(csv.DictReader(source))


def quantile(values, fraction):
    values = sorted(values)
    coordinate = (len(values) - 1) * fraction
    low = math.floor(coordinate)
    high = math.ceil(coordinate)
    return values[low] + (coordinate - low) * (values[high] - values[low])


def rmse(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def write(path, rows):
    with path.open("w", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main():
    populations = {name: read(folder / "calls.csv") for name, folder in RUNS.items()}
    common = set.intersection(*[
        {int(r["frame"]) for r in rows if r["accepted"] == "1"}
        for rows in populations.values()
    ])
    metrics = []
    curves = []
    for name, rows in populations.items():
        errors = [float(r["positionErrorM"]) for r in rows]
        full = [r for r in rows if r["accepted"] == "1"]
        events = [r for r in rows if r["accepted"] == "1" or r["directionalAccepted"] == "1"]
        worst = max(rows, key=lambda r: float(r["positionErrorM"]))
        metrics.append(dict(
            variant=name, frames=len(rows), fullPoses=len(full),
            directionalPoses=len(events) - len(full), noMeasurement=len(rows) - len(events),
            allFrameTrajectoryRmseM=rmse(errors),
            emittedFullPoseRmseM=rmse([float(r["positionErrorM"]) for r in full]),
            emittedAnyPoseRmseM=rmse([float(r["positionErrorM"]) for r in events]),
            commonFullPoseFrames=len(common),
            commonFullPoseRmseM=rmse([float(r["positionErrorM"]) for r in rows if int(r["frame"]) in common]),
            medianM=statistics.median(errors), p95M=quantile(errors, .95), maximumM=max(errors),
            maximumFrame=int(worst["frame"]), frame959M=errors[958], frame959Status=rows[958]["reason"],
            totalMedianMs=statistics.median(float(r["totalMs"]) for r in rows),
            totalP95Ms=quantile([float(r["totalMs"]) for r in rows], .95),
        ))
    for k in range(1170):
        row = dict(frame=k + 1)
        for name, rows in populations.items():
            row[name + "ErrorM"] = rows[k]["positionErrorM"]
            row[name + "Status"] = rows[k]["reason"]
        curves.append(row)
    write(DEST / "comparison.csv", metrics)
    write(DEST / "error_curves.csv", curves)
    selected = read(RUNS["confirmed_three_frames"] / "source_windows.csv")
    write(DEST / "source_windows.csv", selected)
    assert all(int(r["minimumSupport"]) >= 2 for r in selected if int(r["components"]) > 0)
    observer = read(Path("output/temporal_perception_20260922/observer/metrics.csv"))
    write(DEST / "observer_smoke.csv", observer)
    summary = dict(
        activeVariant="confirmed_three_frames", metrics=metrics,
        baselineSourceRevision="24f6cd34c91278cad65bfd7b628fbebeb93ee82e",
        initialization="First temporal frame emits no LiDAR measurement; replay prediction is retained.",
        observerScope="Integration smoke test with matching-prediction startup; differs from former first-LiDAR initialization.",
        limitations="Same-drive map including query frames; sequence-fitted origin, reference tilt, offline synchronized motion and existing biased initialization. Five-frame selection uses this evaluation sequence.",
        knownRegression="Three-frame trajectory RMSE slightly exceeds concatenation; persistent false detections and incorrect map correspondences are not resolved by temporal confirmation alone.",
    )
    (DEST / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
