"""Independently aggregate the recorded run and identify technical artifacts."""
from datetime import datetime
from zoneinfo import ZoneInfo
from pathlib import Path
import csv
import hashlib
import json
import shutil

import numpy as np


STUDY = Path(__file__).resolve().parent
ROOT = STUDY.parent.parent
OUTPUT = ROOT / "output/support_full_localization_20260930"


def read_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def write(name, value):
    (STUDY / name).write_text(json.dumps(value, indent=2) + "\n")


def main():
    inputs = json.loads((OUTPUT / "source_inputs.json").read_text())
    for name, expected in inputs["sha256"].items():
        assert digest(ROOT / name) == expected, name
    shutil.copyfile(OUTPUT / "source_inputs.json", STUDY / "source_inputs.json")
    for source, target in [
        ("matching/calls.csv", "raw_matching.csv"),
        ("matching/metadata.json", "raw_matching_metadata.json"),
        ("matching/summary.json", "raw_matching_summary.json"),
        ("observer/alignment_controls.csv", "alignment_controls.csv"),
    ]:
        shutil.copyfile(OUTPUT / source, STUDY / target)
    traces = read_rows(STUDY / "frame_errors.csv")
    metrics = read_rows(STUDY / "metrics.csv")
    for row in metrics:
        samples = [item for item in traces if item["scenario"] == row["scenario"]]
        population = row["population"]
        if population == "after_2_seconds":
            samples = [item for item in samples if float(item["time"]) >= 2]
        elif population == "outage_40_60":
            samples = [item for item in samples if 40 <= float(item["time"]) < 60]
        elif population == "recovery_60_70":
            samples = [item for item in samples if 60 <= float(item["time"]) < 70]
        else:
            assert population == "all"
        values = np.array([float(item["positionErrorM"]) for item in samples])
        yaw = np.array([float(item["yawErrorDeg"]) for item in samples])
        actual = [np.sqrt(np.mean(values**2)), np.median(values),
                  np.percentile(values, 95, method="hazen"), np.max(values),
                  np.sqrt(np.mean(yaw**2)), np.max(np.abs(yaw)), np.mean(values <= .1)]
        keys = ["positionRmseM", "medianM", "p95M", "maximumM", "yawRmseDeg",
                "yawMaximumDeg", "fractionWithin10cm"]
        assert np.max(np.abs(actual - np.array([float(row[key]) for key in keys]))) < 1e-11
        assert len(samples) == int(row["frames"])
        assert samples[int(np.argmax(values))]["frame"] == row["maximumFrame"]
    raw = read_rows(STUDY / "raw_matching.csv")
    cached = read_rows(ROOT / "research/support_matching_repair_20260930/final_default.csv")
    assert len(raw) == len(cached) == 1170
    differences = np.abs(np.array([[float(x[k]) for k in ("x", "y", "psi")] for x in raw]) -
                         np.array([[float(x[k]) for k in ("x", "y", "psi")] for x in cached]))
    assert np.max(differences) < 2e-8
    assert all(a["accepted"] == b["accepted"] and a["directionalAccepted"] == b["directional"]
               for a, b in zip(raw, cached))
    validation = json.loads((STUDY / "validation.json").read_text())
    assert validation["runtimeChecksPassed"] and validation["passedTests"] == 179
    assert validation["tests"] == 180 and not validation["allTestsPassed"]
    assert not validation["lateralGainBoundCheckResolved"]
    analysis = read_rows(STUDY / "code_analysis.csv")
    assert len(analysis) == 5 and all(row["findings"] == "0" for row in analysis)
    summary = json.loads((STUDY / "summary.json").read_text())
    assert summary["rawFrames"] == 1170 and summary["observerFrames"] == 1169
    assert summary["excludedFrames"] == 1170 and summary["closedLoopRematching"]
    write("independent_checks.json", {
        "metricRowsVerified": len(metrics), "frameRows": len(traces),
        "sourceInputHashesUnchanged": len(inputs["sha256"]),
        "rawCachedMaximumPoseDifference": np.max(differences, axis=0).tolist(),
        "rawCachedAcceptanceIdentical": True, "factoryAnalyzerFiles": len(analysis),
        "runtimeChecksPassed": True, "allUnitTestsPassed": False,
        "failedUnitTestRetained": validation["failedTests"],
    })
    independent_inputs = [
        "output/support_full_localization_20260930/source_inputs.json",
        "output/support_full_localization_20260930/working_tree_inputs.patch",
        "output/support_full_localization_20260930/matching/report.mat",
        "output/support_full_localization_20260930/observer/experiment.mat",
        "output/receiver_synchronized_inputs/bestpos.csv",
        "output/receiver_synchronized_inputs/native_reference.csv",
        "output/mississippi_mapping_calibrated/view_conditioned_cloud.mat",
        "research/support_matching_repair_20260930/final_default.csv",
    ]
    write("input_hashes.json", {name: digest(ROOT / name) for name in independent_inputs})
    write("findings.json", {
        "recorded_at": datetime.now(ZoneInfo("America/Chicago")).isoformat(),
        "implementation_head": inputs["head"],
        "outcome": "Completed fresh raw replay and seven full observer scenarios; numerical gain-bound unit failure retained.",
        "production_change": "Raw replay delegates map cropping to selectLocalProbabilityCloud to align landmark-view and height sidecars.",
        "gain_or_fusion_tuning": False, "rawFrames": 1170, "observerFrames": 1169,
        "excludedFrame": 1170, "sourceVersionsUnchangedDuringRun": len(inputs["sha256"]),
        "runtime_validation": validation,
        "metrics": metrics,
        "paired_comparisons": read_rows(STUDY / "paired_matching.csv"),
        "frame895": read_rows(STUDY / "frame895.csv"),
        "frame895_diagnosis": "21 separate controls reproduce 15.8718 cm, isolate historical translation and a curb group; pole is present. Reference interventions are offline oracles.",
        "limitations": [
            "Same-recording map/query and INS-assisted BESTPOS versus shared INSPVA reference",
            "Zero processing delay and offline bracket alignment; measured raw runtime not injected",
            "Raw runtime median 180.182 ms under concurrent machine load; no 10 Hz throughput claim",
            "LiDAR-only observer is worse than its matched poses; both-absolute-source outage peaks near 1.97 m",
            "One pre-existing lateral gain-bound numerical check remains failed; not an all-pass regression",
            "Pre-existing uncommitted lateral code/tests used and fingerprinted, excluded from this scoped commit",
        ],
    })
    artifacts = sorted(path for path in STUDY.iterdir() if path.is_file() and path.name != "artifact_hashes.json")
    artifacts.append(ROOT / "scripts/replayMississippiLocalization.m")
    write("artifact_hashes.json", {str(path.relative_to(ROOT)): digest(path) for path in artifacts})
    print(json.dumps({"metricsVerified": len(metrics), "sourceHashes": len(inputs["sha256"]),
                      "tests": "179/180 passed; one known numerical bound failure retained",
                      "runtimeChecksPassed": True, "artifactHashes": len(artifacts)}, indent=2))


if __name__ == "__main__":
    main()
