"""Summarize recorded candidate-cell diagnostics without treating labels as truth."""

import csv
import json
from collections import Counter
from pathlib import Path

FOLDER = Path(__file__).resolve().parent
STAGES = {
    0: "initial owner count/span",
    1: "seed support",
    2: "initial tilt",
    3: "cylinder count",
    4: "local window contrast",
    5: "gap-separated run count",
    6: "run owner count",
    7: "total/robust/owner height",
    8: "radial RMS",
    9: "transverse density peak",
    10: "shaft found; final footprint omitted",
}


def read(name):
    with (FOLDER / name).open(newline="") as stream:
        return list(csv.DictReader(stream))


def number(row, key):
    return float(row[key])


def yes(row, key):
    return number(row, key) != 0


def count(rows, key):
    return sum(yes(row, key) for row in rows)


def total(rows, key):
    return int(sum(number(row, key) for row in rows))


def stage_counts(rows):
    values = Counter(int(row["stage"]) for row in rows)
    return {STAGES[k]: values[k] for k in sorted(values)}


def main():
    rows = read("reference_cell_causes.csv")
    fine = [r for r in rows if yes(r, "fine")]
    default_miss = [r for r in fine if not yes(r, "previous")]
    subset_miss = [r for r in fine if not yes(r, "subsetAccepted")]
    coarse = [r for r in rows if yes(r, "baseline")]
    coarse_miss = [r for r in coarse if not yes(r, "subsetAccepted")]
    measured = [r for r in default_miss if yes(r, "densityCoreEvaluated")]
    isolation = [r for r in measured if yes(r, "failIsolation")]
    other_gates = [k for k in rows[0] if k.startswith("fail") and k != "failIsolation"]
    isolation_only = [r for r in isolation if not any(yes(r, k) for k in other_gates)]

    sweeps = read("gate_sweep_frames.csv")
    aggregates = []
    for variant in dict.fromkeys(r["variant"] for r in sweeps):
        group = [r for r in sweeps if r["variant"] == variant]
        t = {key: total(group, key) for key in group[0] if key not in ("variant", "frame")}
        aggregates.append({"variant": variant, "frames": len(group), **t,
                           "fineRecallExact": t["fineShared"] / t["fineCount"],
                           "fineRecallTolerant": t["fineRecallTol"] / t["fineCount"],
                           "coarsePrecisionExact": t["baselineShared"] / t["candidateCount"],
                           "coarseRecallExact": t["baselineShared"] / t["baselineCount"],
                           "coarsePrecisionTolerant": t["baselinePrecisionTolCount"] / t["candidateCount"],
                           "coarseRecallTolerant": t["baselineRecallTol"] / t["baselineCount"]})
    with (FOLDER / "gate_sweep_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, aggregates[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(aggregates)

    full = json.loads((FOLDER.parent / "pole_subset_20260925" / "full_summary.json").read_text())
    pole = full["pole"]
    audit = read("footprint_losses.csv")
    relaxed_losses = [r for r in audit if r["variant"] == "contrast3"]
    probes = read("missed_case_counterfactuals.csv")
    absent = {(r["frame"], r["pillar"]) for r in probes if r["variant"] == "base" and not yes(r, "found")}
    recovered = {}
    for variant in dict.fromkeys(r["variant"] for r in probes):
        recovered[variant] = [f'{r["frame"]}/{r["pillar"]}' for r in probes
                              if r["variant"] == variant and yes(r, "found")
                              and (r["frame"], r["pillar"]) in absent]

    summary = {
        "units": "frame/pillar occurrences, not unique physical poles",
        "referenceType": "algorithmic outputs; no manual ground truth",
        "fullFrames": full["frames"],
        "fullCoarseAgreement": {
            "referenceCells": pole["Base"], "experimentCells": pole["Cand"],
            "exactShared": pole["Shared"],
            "exactMissingOldCells": pole["Base"] - pole["Shared"],
            "exactNewUnmatchedCells": pole["Cand"] - pole["Shared"],
            "tolerantMatchedOldCells": pole["RecallTol"],
            "tolerantMissingOldCells": pole["Base"] - pole["RecallTol"],
            "tolerantMatchedNewCells": pole["SharedTol"],
            "tolerantNewUnmatchedCells": pole["Cand"] - pole["SharedTol"],
            "tolerantPrecision": pole["precisionTol"], "tolerantRecall": pole["recallTol"],
            "tolerance": "independent 8-neighbor matches, 0.6 m/axis, not one-to-one object matching",
        },
        "sampleFrames": len({r["frame"] for r in rows}),
        "sampleReferenceUnionCells": len(rows),
        "fineCells": len(fine),
        "defaultFineMatched": count(fine, "previous"),
        "subsetFineMatched": count(fine, "subsetAccepted"),
        "subsetRecoversDefaultFineMisses": count(default_miss, "subsetAccepted"),
        "subsetLosesPreviouslyMatchedFineCells": count(subset_miss, "previous"),
        "coarseCells": len(coarse),
        "coarseExactMissing": len(coarse_miss),
        "coarseExactMissingWithFineSupport": count(coarse_miss, "fine"),
        "coarseExactMissingWithoutFineSupport": len(coarse_miss) - count(coarse_miss, "fine"),
        "coarseMissingFurthestStage": stage_counts(coarse_miss),
        "defaultFineMisses": {
            "total": len(default_miss),
            "countOrHeightPrefilter": len(default_miss) - len(measured),
            "measuredIsolationFailed": len(isolation),
            "isolationOnly": len(isolation_only),
            "otherCoreGatesWithoutIsolation": sum(not yes(r, "failIsolation") and not yes(r, "defaultCore") for r in measured),
            "footprintOmissions": count(default_miss, "defaultFootprintDrop"),
            "isolationOnlyMinimum": min(number(r, "coreIsolation") for r in isolation_only),
            "isolationOnlyMaximum": max(number(r, "coreIsolation") for r in isolation_only),
            "fineOriginalOwnPoints": total(default_miss, "fineOriginalOwnCount"),
            "fineGroundRemovedPoints": total(default_miss, "fineGroundRemovedCount"),
            "cellsWithGroundRemovedFinePoints": count(default_miss, "fineGroundRemovedCount"),
        },
        "fineOriginalOwnPoints": total(fine, "fineOriginalOwnCount"),
        "fineGroundRemovedPoints": total(fine, "fineGroundRemovedCount"),
        "fineCellsWithGroundRemovedPoints": count(fine, "fineGroundRemovedCount"),
        "subsetFineMisses": {
            "total": len(subset_miss),
            "furthestStage": stage_counts(subset_miss),
            "withAcceptedOneCellNeighbor": sum(number(r, "nearestAcceptedCellMeters") <= .6 * 2**.5 + 1e-10 for r in subset_miss),
        },
        "contrastRelaxationFineLosses": {
            "total": len(relaxed_losses),
            "shaftFoundButFootprintOmitted": count(relaxed_losses, "shaftFound"),
            "shaftNoLongerFound": len(relaxed_losses) - count(relaxed_losses, "shaftFound"),
        },
        "ownEvidenceRescuedByCounterfactual": recovered,
        "caveats": [
            "Fine reference is algorithmic and can miss real poles or label non-poles.",
            "No fine support is not proof of a coarse false positive.",
            "Furthest successful gate summarizes a search; relaxing one gate may expose another.",
            "Trace extrema can arise from different hypotheses; longest-run metrics share one run.",
            "Density core values are placeholders when the whole-count/height prefilter failed.",
            "A neighboring accepted cell does not by itself prove the same pole was retained.",
        ],
    }
    assert (len(rows), len(fine), len(default_miss), len(subset_miss)) == (980, 306, 46, 10)
    assert len(isolation_only) == 31 and len(isolation) == 33
    assert sum((len(default_miss) - len(measured), len(isolation),
                summary["defaultFineMisses"]["otherCoreGatesWithoutIsolation"],
                count(default_miss, "defaultFootprintDrop"))) == len(default_miss)
    assert len(coarse_miss) == 333 and count(coarse_miss, "fine") == 3
    assert not recovered["search48x12"]
    assert all(not (r["coarsePrecisionTolerant"] >= .8 and r["coarseRecallTolerant"] >= .8) for r in aggregates)
    (FOLDER / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"frames": summary["sampleFrames"], "referenceCells": len(rows),
                      "defaultFineMisses": len(default_miss), "subsetFineMisses": len(subset_miss),
                      "variants": len(aggregates), "assertions": "passed"}))


if __name__ == "__main__":
    main()
