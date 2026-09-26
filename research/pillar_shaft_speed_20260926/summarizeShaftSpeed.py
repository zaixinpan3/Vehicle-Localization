"""Summarize archived replay CSVs without requiring third-party packages."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median


def number(row: dict[str, str], key: str) -> float:
    value = row.get(key, "nan")
    if value.lower() == "true":
        return 1.0
    if value.lower() == "false":
        return 0.0
    return float(value)


def total(rows: list[dict[str, str]], key: str) -> float:
    return sum(value for row in rows if math.isfinite(value := number(row, key)))


def ratio(numerator: float, denominator: float) -> float | None:
    return numerator / denominator if denominator else None


def percentile(values: list[float], fraction: float) -> float:
    """Use explicitly documented linear interpolation of order statistics."""
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = math.floor(index)
    high = math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def summarize(rows: list[dict[str, str]]) -> dict:
    # Timing repeats do not count as extra frames or reference observations.
    unique = {}
    for row in rows:
        unique.setdefault((row["dataset"], row["frame"]), row)
    quality = list(unique.values())
    elapsed = [number(row, "milliseconds") for row in rows]
    totals = {
        key: int(total(quality, key))
        for key in (
            "baselineCount", "candidateCount", "sharedCount", "lostCount",
            "addedCount", "legacyCount", "legacyRetained", "fineCount",
            "fineMatched", "baselineFineMatched",
        )
    }
    result = {
        "uniqueFrames": len(quality),
        "timingSamples": len(rows),
        **totals,
        "poleRetention": ratio(totals["sharedCount"], totals["baselineCount"]),
        "poleJaccard": ratio(
            totals["sharedCount"],
            totals["baselineCount"] + totals["candidateCount"] - totals["sharedCount"],
        ),
        "referenceRecall": ratio(totals["fineMatched"], totals["fineCount"]),
        "medianMs": median(elapsed),
        "p95Ms": percentile(elapsed, 0.95),
        "maximumMs": max(elapsed),
    }
    for key in (
        "maskExact", "sourceExact", "componentsExact", "nonPoleMasksExact",
        "nonPoleComponentsExact",
    ):
        values = [number(row, key) for row in rows]
        checked = [value for value in values if math.isfinite(value)]
        result[key + "Checks"] = len(checked)
        result[key + "Failures"] = sum(value != 1.0 for value in checked)
    for key in (
        "maximumMeanError", "maximumCovarianceError", "maximumProbabilityError",
        "maximumMixtureWeightError",
    ):
        result[key] = max(number(row, key) for row in rows)
    if "baselineMilliseconds" in rows[0]:
        baseline = [number(row, "baselineMilliseconds") for row in rows]
        result["baselineMedianMs"] = median(baseline)
        result["baselineP95Ms"] = percentile(baseline, 0.95)
        result["medianPairedSavingMs"] = median(
            old - new for old, new in zip(baseline, elapsed)
        )
        result["medianPairedSpeedup"] = median(
            old / new for old, new in zip(baseline, elapsed)
        )
        result["medianRuntimeReduction"] = 1 - median(elapsed) / median(baseline)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path(__file__).parent)
    args = parser.parse_args()
    groups: dict[str, list[dict[str, str]]] = {}
    for path in sorted(args.directory.glob("*_paired.csv")):
        with path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                key = f"{path.stem}/{row['variant']}"
                groups.setdefault(key, []).append(row)
    for path in sorted(args.directory.glob("*_batch_[0-9][0-9][0-9].csv")):
        key = path.stem.rsplit("_batch_", 1)[0] + "/fullCapture"
        with path.open(newline="") as handle:
            groups.setdefault(key, []).extend(csv.DictReader(handle))
    output = {
        "units": "Milliseconds per frame; pillar occurrences, not manual truth",
        "percentileConvention": "Linear interpolation at (n-1)*p in sorted timings",
        "qualityCounting": "Each dataset/frame counts once per group; all timing repeats are checked",
        "experiments": {key: summarize(rows) for key, rows in groups.items()},
    }
    (args.directory / "summary.json").write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output, indent=2))


if __name__ == "__main__":
    main()
