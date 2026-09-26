"""Compare subset evidence against algorithmic references, never manual truth."""
from pathlib import Path
import csv
import json
import statistics

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def selected(rows, name):
    return {(int(r["frame"]), int(r["pillarIndices"]) - 1)
            for r in rows if r[name] == "1"}


def dilate(keys):
    return {(f, 100 * (c // 100 + dx) + c % 100 + dy)
            for f, c in keys for dx in (-1, 0, 1) for dy in (-1, 0, 1)
            if 0 <= c // 100 + dx < 100 and 0 <= c % 100 + dy < 100}


def agreement(candidate, reference):
    shared = len(candidate & reference)
    p, r = shared / max(len(candidate), 1), shared / max(len(reference), 1)
    pt = len(candidate & dilate(reference)) / max(len(candidate), 1)
    rt = len(reference & dilate(candidate)) / max(len(reference), 1)
    return dict(candidate=len(candidate), reference=len(reference), shared=shared,
                precision=p, recall=r, f1=2*p*r/max(p+r, 1e-15),
                precisionTol=pt, recallTol=rt, f1Tol=2*pt*rt/max(pt+rt, 1e-15))


def main():
    rows = list(csv.DictReader((HERE / "native_pillars.csv").open()))
    keys = {name: selected(rows, name) for name in ("accepted", "baseline", "fine", "previous")}
    report = {"frames": len({r["frame"] for r in rows}),
              "references": "Frozen 0.3 m coarse and offline fine algorithms; not manual labels.",
              "tolerance": "One Chebyshev cell: 0.6 m per axis, up to 0.849 m diagonal.",
              "subsetVsCoarse": agreement(keys["accepted"], keys["baseline"]),
              "subsetVsFine": agreement(keys["accepted"], keys["fine"]),
              "previousVsFine": agreement(keys["previous"], keys["fine"]),
              "previousVsCoarse": agreement(keys["previous"], keys["baseline"]),
              "retainedPrevious": len(keys["accepted"] & keys["previous"]),
              "added": len(keys["accepted"] - keys["previous"]),
              "removed": len(keys["previous"] - keys["accepted"])}
    measurements = []
    for label, group in [
        ("fine_supported", [r for r in rows if r["accepted"] == r["fine"] == "1"]),
        ("accepted_without_fine_reference", [r for r in rows if r["accepted"] == "1" and r["fine"] == "0"]),
    ]:
        fields = ["score", "supportCount", "ownCount", "height", "robustHeight", "radialRms",
                  "maximumGap", "contrast", "peakContrast", "radius", "ownSupportFraction"]
        item = dict(group=label, count=len(group))
        for field in fields:
            if field in rows[0]:
                item[field] = statistics.median(float(r[field]) for r in group)
        measurements.append(item)
    report["groupMedians"] = measurements
    full_path = ROOT / "output/coarse_lattice_20260924/subsetFull_vs_baseline.csv"
    if full_path.exists():
        full = list(csv.DictReader(full_path.open()))
        summary = dict(frames=len(full), medianMs=1000*statistics.median(float(r["elapsedCand"]) for r in full))
        for channel in ("curb", "pole", "trafficSign", "road"):
            totals = {name: sum(int(r[channel + name]) for r in full)
                      for name in ("Base", "Cand", "Shared", "SharedTol", "RecallTol")}
            p, recall = totals["Shared"]/totals["Cand"], totals["Shared"]/totals["Base"]
            pt, rt = totals["SharedTol"]/totals["Cand"], totals["RecallTol"]/totals["Base"]
            summary[channel] = dict(**totals, precision=p, recall=recall, f1=2*p*recall/(p+recall),
                                    precisionTol=pt, recallTol=rt, f1Tol=2*pt*rt/(pt+rt))
        (HERE / "full_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    thresholds = []
    for threshold in (0, .2, .3, .4, .45, .5, .55, .6, .7):
        candidate = {(int(r["frame"]), int(r["pillarIndices"])-1) for r in rows
                     if r["accepted"] == "1" and float(r["score"]) >= threshold}
        for label in ("baseline", "fine"):
            thresholds.append(dict(threshold=threshold, referenceName=label,
                                   **agreement(candidate, keys[label])))
    with (HERE / "score_threshold_diagnostic.csv").open("w") as f:
        writer = csv.DictWriter(f, fieldnames=list(thresholds[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(thresholds)
    (HERE / "sample_summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
