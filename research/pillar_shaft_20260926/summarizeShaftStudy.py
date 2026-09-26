"""Summarize shaft identification experiments without asserting manual accuracy."""
import csv
import json
import math
from pathlib import Path
from statistics import median

FOLDER = Path(__file__).resolve().parent


def read(name):
    with (FOLDER / name).open(newline="") as stream:
        return list(csv.DictReader(stream))


def total(rows, key):
    return sum(float(row[key]) for row in rows if math.isfinite(float(row[key])))


def main():
    full = read("full_pipeline.csv")
    downtown = read("downtown_pipeline.csv")
    timing = read("paired_timing.csv")
    summary = {"units": "frame/pillar occurrences; algorithmic references, not manual truth"}
    for name, rows in [("Mississippi", full), ("Downtown", downtown)]:
        values = {k: int(total(rows, k)) for k in ["legacyCount", "candidateCount", "retainedLegacy", "fineCount", "fineMatched", "legacyFineMatched"]}
        values["frames"] = len(rows)
        values["referenceRecall"] = values["fineMatched"] / max(values["fineCount"], 1)
        values["legacyReferenceRecall"] = values["legacyFineMatched"] / max(values["fineCount"], 1)
        values["captureMedianMs"] = median(float(r["milliseconds"]) for r in rows)
        if name == "Mississippi":
            base = int(total(rows, "coarseCount"));shared = int(total(rows, "coarseShared"))
            values.update(coarseReferenceCount=base, coarseExactPrecision=shared / values["candidateCount"],
                          coarseExactRecall=shared / base,
                          coarseTolerantPrecision=total(rows, "coarseMatchedNewTolerant") / values["candidateCount"],
                          coarseTolerantRecall=total(rows, "coarseMatchedOldTolerant") / base)
        else:
            values["subsetCount"] = int(total(rows, "subsetCount"))
            values["subsetReferenceMatched"] = int(total(rows, "subsetFineMatched"))
        assert values["retainedLegacy"] == values["legacyCount"]
        summary[name] = values
    summary["pairedTiming"] = {k: median(float(r[k]) for r in timing) for k in ["legacyMs", "subsetMs", "shaftMs"]}
    summary["pairedTiming"]["shaftMinusLegacyMedianMs"] = median(float(r["shaftMs"]) - float(r["legacyMs"]) for r in timing)
    summary["validation"] = json.loads((FOLDER / "validation.json").read_text())
    variants = []
    for path in sorted(FOLDER.glob("*_frames.csv")):
        rows = read(path.name)
        variants.append({"variant": path.stem.removesuffix("_frames"), "frames": len(rows),
                         "rawModeCells": int(total(rows, "candidates")),
                         "fineMatchedBeforeSelection": int(total(rows, "fineMatched")),
                         "fineReferenceCells": int(total(rows, "fine")),
                         "developmentMedianMs": median(float(r["seconds"]) * 1000 for r in rows)})
    summary["developmentVariants"] = variants
    (FOLDER / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "developmentVariants"}, indent=2))


if __name__ == "__main__":
    main()
