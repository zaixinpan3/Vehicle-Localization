#!/usr/bin/env python3
"""Independently verify saved MATLAB metric populations using Python stdlib."""
from pathlib import Path
import csv
import hashlib
import json
import math
import statistics

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "output/mncav_inspva_observer_20260915"
DESTINATION = Path(__file__).resolve().parent


def read(name):
    with (OUTPUT / name).open(newline="") as stream:
        return list(csv.DictReader(stream))


def check(values, row):
    assert len(values) == int(row["samples"]) and all(map(math.isfinite, values))
    expected = {
        "positionRmseM": math.sqrt(math.fsum(x*x for x in values)/len(values)),
        "positionMedianM": statistics.median(values),
        "positionMaximumM": max(values),
        "fractionAtMost5cm": sum(x <= .05 for x in values)/len(values),
        "fractionAtMost10cm": sum(x <= .1 for x in values)/len(values),
    }
    for field, value in expected.items():
        assert abs(value-float(row[field])) < 1e-9, (row, field, value)


def main():
    original, improved = read("frame_errors.csv"), read("motion_gap_frame_errors.csv")
    metrics, uniform = read("motion_gap_metrics.csv"), read("motion_gap_uniform_errors.csv")
    assert len(original) == len(improved) == 3510 and len(uniform) == 11690
    assert [(r["mode"], r["frame"], r["fullMeasurement"]) for r in original] == [
        (r["mode"], r["frame"], r["fullMeasurement"]) for r in improved]
    checked = 0
    columns = {"lidar_measurement": "lidarMeasurementErrorM",
               "lidar_linear_reconstruction": "lidarReconstructionErrorM",
               "motion_aided_observer": "observerErrorM",
               "motion_gap_reconstruction": "motionGapReconstructionErrorM",
               "observer_with_motion_gaps": "observerWithMotionGapsErrorM"}
    uniform_columns = {"lidar_linear_reconstruction": "lidarLinearErrorM",
                       "motion_aided_observer": "originalObserverErrorM",
                       "motion_gap_reconstruction": "motionReconstructionErrorM",
                       "observer_with_motion_gaps": "motionGapObserverErrorM"}
    for row in metrics:
        pop, method, mode = row["population"], row["method"], row["mode"]
        if pop.startswith("native_"):
            source = improved if method in ["motion_gap_reconstruction", "observer_with_motion_gaps"] else original
            chosen = [r for r in source if r["mode"] == mode and
                      (pop == "native_all" or (r["fullMeasurement"] == "1") == (pop == "native_full"))]
            check([float(r[columns[method]]) for r in chosen], row)
            checked += 1
        elif mode == "per_frame_zero":
            chosen = [r for r in uniform if pop == "uniform_all" or
                      (float(r["time"]) <= 60) == (pop == "uniform_first60")]
            check([float(r[uniform_columns[method]]) for r in chosen], row)
            checked += 1
    matching_path = ROOT / "output/saved_perception_inspva_20260915/calls.csv"
    with matching_path.open(newline="") as stream:
        calls = {(r["mode"], r["frame"]): r for r in csv.DictReader(stream)}
    difference = 0
    for row in original:
        call = calls[(row["mode"], row["frame"])]
        assert row["fullMeasurement"] == call["fullPose"]
        if row["fullMeasurement"] == "1":
            difference = max(difference, abs(float(row["lidarMeasurementErrorM"])-float(call["pvaErrorM"])))
        elif call["measurementType"] == "rejected":
            assert math.isnan(float(row["lidarMeasurementErrorM"]))
    assert difference < 1e-8
    result = {"checkedMetricRows": checked, "checkedStatisticsPerRow": 5,
              "matchedFrameModeRows": len(original), "uniformZeroModeRows": len(uniform),
              "allFrameModesAndAcceptancePreserved": True,
              "maximumOriginalMatchingErrorCsvDifferenceM": difference,
              "statisticToleranceM": 1e-9, "gainsSelected": False}
    (DESTINATION / "independent_checks.json").write_text(json.dumps(result, indent=2)+"\n")
    names = ["experiment.mat", "motion_gap_experiment.mat", "motion_gap_ablation.mat", "diagnostics.mat",
             "native_reference.csv", "reference_metadata.json", "metrics.csv", "motion_gap_metrics.csv",
             "frame_errors.csv", "motion_gap_frame_errors.csv", "motion_gap_uniform_errors.csv",
             "gains.json", "summary.json", "motion_gap_summary.json", "diagnostics.json",
             "tests.csv", "gap_tests.csv", "code_checks.json", "motion_gap_comparison.png", "motion_gap_comparison.pdf"]
    manifest = []
    paths = [(OUTPUT / name, "Output") for name in names]
    paths.extend((ROOT / name, "Frozen input") for name in [
        "output/mncav_zero_delay_20260914/experiment.mat",
        "output/saved_perception_inspva_20260915/experiment.mat",
        "output/saved_perception_inspva_20260915/calls.csv",
        "output/mississippi_mapping_inspva_20260915/probability_cloud.mat"])
    for path, role in paths:
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
        manifest.append({"path": str(path.relative_to(ROOT)), "role": role, "bytes": path.stat().st_size, "sha256": digest})
    (DESTINATION / "operational_artifact_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
