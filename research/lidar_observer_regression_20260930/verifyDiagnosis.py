"""Independently check exported attribution and baseline scoring with NumPy."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent


def main():
    d = np.genfromtxt(DEST/"decomposition.csv", delimiter=",", names=True)
    m = np.genfromtxt(DEST/"motion_inputs.csv", delimiter=",", names=True)
    summary = json.loads((DEST/"summary.json").read_text())
    paired = np.genfromtxt(ROOT/"research/support_full_localization_20260930/paired_matching.csv",
                          delimiter=",", names=True, dtype=None, encoding="utf-8")
    old = paired[(paired["scenario"] == "lidar_only") & (paired["population"] == "after_2_seconds")][0]
    controls = list(csv.DictReader((DEST/"controls.csv").open()))
    after = d["time"] >= d["time"][0]+2
    accepted = after & (d["fullMatch"] == 1)
    error = np.hypot(d["errorX"], d["errorY"])
    assert len(d) == len(m) == 1169 and accepted.sum() == 1121
    assert np.array_equal(d["frame"], m["frame"])
    assert abs(np.sqrt(np.mean(error[accepted]**2))-old["observerRmseM"]) < 1e-12
    assert np.argmax(np.where(after, error, -np.inf))+1 == 847
    closure = max(np.max(np.abs(d["initial"+axis]+d["match"+axis]+d["motion"+axis]-d["error"+axis])) for axis in "XY")
    split = max(np.max(np.abs(d["filter"+axis]+d["quadrature"+axis]+d["input"+axis]-d["motion"+axis])) for axis in "XY")
    nested = max(np.max(np.abs(sum(m[name+axis] for name in ["heading", "longitudinal", "lateral", "referenceKinematics"])-d["input"+axis])) for axis in "XY")
    assert closure < 1e-7 and split < 1e-12 and nested < 1e-12
    assert summary["baselineReplayMaximumDifference"] == 0 and len(controls) == 22
    for row in controls:
        assert float(row["baselineReplayDifference"]) == 0
        if row["control"] == "baseline" and row["population"] == "paired_full_matches":
            assert abs(float(row["positionRmseM"])-old["observerRmseM"]) < 1e-12
    inputs = json.loads((DEST/"input_hashes.json").read_text())
    for path, expected in inputs.items():
        with (ROOT/path).open("rb") as stream:
            assert hashlib.file_digest(stream, "sha256").hexdigest() == expected, path
    sources = json.loads((ROOT/"research/support_full_localization_20260930/source_inputs.json").read_text())["sha256"]
    for path, expected in sources.items():
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == expected, path
    validation = dict(checksPassed=True, frames=len(d), pairedFrames=int(accepted.sum()), controlMetricRows=len(controls),
                      baselinePairedMatchingRmseM=float(old["matchingRmseM"]), baselinePairedObserverRmseM=float(old["observerRmseM"]),
                      additiveClosureMaximumM=float(closure), motionSplitMaximumM=float(split), inputSplitMaximumM=float(nested),
                      baselineRuntimeExactlyReproduced=True, unchangedPriorSourceHashes=len(sources), checkedInputHashes=len(inputs),
                      productionAlgorithmOrConfigurationChanged=False,
                      scope="Executed diagnostic assertions and independent exports check. No new complete closed-loop run or production retuning; the earlier unrelated lateral gain-bound test failure is not resolved by this study.")
    (DEST/"validation.json").write_text(json.dumps(validation, indent=2)+"\n")
    print(json.dumps(validation, indent=2))


if __name__ == "__main__":
    main()
