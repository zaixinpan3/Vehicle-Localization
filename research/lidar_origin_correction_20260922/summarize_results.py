"""Summarize both executed calibration candidates without dropping bad frames."""
from pathlib import Path
import csv
import json

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
EXPERIMENTS = {
    "identity_baseline": "output/receiver_clock_20260921/coarse_pipeline",
    "selected_sequence_fit": "output/lidar_origin_20260922/coarse_pipeline",
    "independent_drive_fit": "output/lidar_origin_20260922/independent_pipeline",
}


def main():
    rows = []
    calls_by_experiment = {}
    observer_rows = []
    for name, folder in EXPERIMENTS.items():
        path = ROOT / folder
        matching = json.loads((path / "matching/summary.json").read_text())
        with (path / "matching/calls.csv").open() as stream:
            calls = list(csv.DictReader(stream))
        assert [int(row["frame"]) for row in calls] == list(range(1, 1171))
        calls_by_experiment[name] = calls
        peak = max(calls, key=lambda row: float(row["positionErrorM"]))
        with (path / "observer/metrics.csv").open() as stream:
            metrics = list(csv.DictReader(stream))
        observer_rows.extend(dict(experiment=name, **row) for row in metrics)
        fused = next(row for row in metrics if row["scenario"] == "both" and row["population"] == "full")
        rows.append(dict(experiment=name, folder=folder, frames=len(calls), accepted=matching["accepted"],
            rawRmseM=matching["allFramePositionRmseM"], rawAcceptedRmseM=matching["acceptedPositionRmseM"],
            rawP95M=matching["positionP95M"], rawMaximumM=matching["positionMaximumM"], rawPeakFrame=int(peak["frame"]),
            rawFrame94M=float(calls[93]["positionErrorM"]), fusedSamples=int(fused["samples"]),
            fusedRmseM=float(fused["positionRmseM"]), fusedP95M=float(fused["positionP95M"]),
            fusedMaximumM=float(fused["positionMaximumM"]),
            finePerceptionUsed=json.loads((path / "matching/metadata.json").read_text())["finePerceptionUsed"]))
    write_csv("comparison.csv", rows)
    write_csv("observer_scenarios.csv", observer_rows)
    curve = [dict(frame=index+1, **{name: float(calls[index]["positionErrorM"])
             for name, calls in calls_by_experiment.items()}) for index in range(1170)]
    write_csv("raw_error_curve.csv", curve)
    (DEST / "comparison.json").write_text(json.dumps(rows, indent=2) + "\n")
    print(json.dumps(rows, indent=2))


def write_csv(name, rows):
    with (DEST / name).open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
