#!/usr/bin/env python3
"""Describe candidate geometry and exploratory gates on development frames only.

Labels come from original fine pole points. Frames after 780 are discarded
before parsing feature or label values. Gate searches describe development
tradeoffs; they are not held-out validation or a calibrated classifier.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def numeric(value):
    if value.lower() in {"true", "false"}:
        return float(value.lower() == "true")
    return float(value)


def development_table(path):
    with path.open(newline="") as source:
        rows = [row for row in csv.DictReader(source) if float(row["frame"]) <= 780]
    if not rows:
        raise ValueError("No development frames in feature export")
    return {key: np.array([numeric(row[key]) for row in rows]) for key in rows[0]}


def auc(values, positive):
    finite = np.isfinite(values)
    v, y = values[finite], positive[finite]
    positives, negatives = int(y.sum()), int((~y).sum())
    if not positives or not negatives:
        return None
    order = np.argsort(v, kind="stable")
    v, y = v[order], y[order]
    lower_negative, concordance = 0, 0.0
    for low, high in zip(np.r_[0, np.flatnonzero(np.diff(v)) + 1],
                         np.r_[np.flatnonzero(np.diff(v)) + 1, len(v)]):
        group_positive = int(y[low:high].sum())
        group_negative = high - low - group_positive
        concordance += group_positive * (lower_negative + .5 * group_negative)
        lower_negative += group_negative
    return float(concordance / (positives * negatives))


def main():
    folder = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=folder / "candidate_features.csv")
    parser.add_argument("--output", type=Path, default=folder)
    args = parser.parse_args()
    data = development_table(args.csv)
    frames = development_table(args.csv.with_name(args.csv.stem + "_frames.csv"))
    total_points = int(frames["finePointCount"].sum())
    total_pillars = int(frames["finePillarCount"].sum())
    candidate = data["candidate"].astype(bool)
    weights = data["fineOwnPointCount"]
    positive = weights > 0

    def metrics(keep):
        matched = int((keep & positive).sum())
        count = int(keep.sum())
        covered = int(weights[keep].sum())
        return {"candidatePillars": count, "extraPillars": count - matched,
                "coveredFinePillars": matched, "coveredFinePoints": covered,
                "pointCoverage": covered / total_points,
                "pillarRecall": matched / total_pillars,
                "extraPillarFraction": (count - matched) / count if count else 0}

    excluded = {"frame", "pillarIndex", "development", "occupied", "fineOwnPointCount",
                "fineOwnOffGroundPointCount", "fineOwner", "candidate", "contextSource"}
    features = []
    gates = []
    for name, values in data.items():
        if name in excluded or "axisXY" in name or "minimumZ" in name or "maximumZ" in name or "axisZ" in name:
            continue
        finite = candidate & np.isfinite(values)
        if not np.any(finite):
            continue
        row = {"feature": name, "aucHigherIsFine": auc(values[candidate], positive[candidate])}
        for label, mask in (("fine", finite & positive), ("extra", finite & ~positive)):
            for percentile, value in zip((10, 50, 90), np.quantile(values[mask], [.1, .5, .9]) if np.any(mask) else [np.nan] * 3):
                row[f"{label}P{percentile}"] = float(value)
        features.append(row)
        thresholds = np.unique(np.quantile(values[finite], np.linspace(0, 1, 41)))
        for threshold in thresholds:
            for operator in (">=", "<="):
                keep = finite & ((values >= threshold) if operator == ">=" else (values <= threshold))
                gates.append({"conditions": [{"feature": name, "operator": operator,
                                               "threshold": float(threshold)}], **metrics(keep)})
    features.sort(key=lambda row: abs((row["aucHigherIsFine"] or .5) - .5), reverse=True)

    # Restrict pair exploration to interpretable context and support measures.
    pair_names = ["coreIsolation", "pointScore", "assigned_score",
                  "context_balancedMaximumStd25", "context_coreFraction25_75",
                  "context_supportDensityContrast75", "context_ownerRadialRms",
                  "context_growthDimension25_75", "context_ownerCenterOffset",
                  "context_minimumHalfFraction25_75"]
    prepared = {}
    for name in pair_names:
        if name not in data:
            continue
        v = data[name]
        finite = candidate & np.isfinite(v)
        if not finite.any():
            continue
        prepared[name] = []
        for threshold in np.unique(np.quantile(v[finite], np.linspace(.05, .95, 10))):
            for operator in (">=", "<="):
                keep = finite & ((v >= threshold) if operator == ">=" else (v <= threshold))
                condition = {"feature": name, "operator": operator, "threshold": float(threshold)}
                prepared[name].append((condition, keep))

    targets = (.80, .90, .95)
    best = {}

    def consider(rule, category):
        criteria = [(target, 0) for target in targets] + [(.90, .80), (.95, .80), (.95, .90)]
        for target, pillar_target in criteria:
            if rule["pointCoverage"] < target or rule["pillarRecall"] < pillar_target:
                continue
            key = f"{category}_coverage{target:.2f}"
            if pillar_target:
                key += f"_pillar{pillar_target:.2f}"
            rank = (rule["extraPillars"], -rule["coveredFinePoints"], len(rule["conditions"]))
            if key not in best or rank < best[key][0]:
                best[key] = (rank, rule)

    for rule in gates:
        consider(rule, "single")
    names = list(prepared)
    for i, first in enumerate(names):
        for second in names[i + 1:]:
            for a, keep_a in prepared[first]:
                for b, keep_b in prepared[second]:
                    consider({"conditions": [a, b], **metrics(keep_a & keep_b)}, "pair")
    result = {"split": "Development only: frame <= 780",
              "finePointDenominator": total_points, "finePillarDenominator": total_pillars,
              "developmentFrames": len(frames["frame"]),
              "baseline": metrics(candidate),
              "legacy": metrics(data["legacy"].astype(bool)),
              "availableSupportCeilings": {
                  "assignedModes": metrics(candidate & (data["assigned_found"] == 1)),
                  "contextAvailable": metrics(candidate & np.isfinite(data["context_coreFraction25_75"])),
              },
              "exploratoryBestRules": {key: value[1] for key, value in best.items()},
              "limitation": "Exploratory development fitting only; holdout labels were not analyzed."}
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "development_feature_distributions.csv").open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=list(features[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(features)
    with (args.output / "development_single_gates.csv").open("w", newline="") as destination:
        columns = ["feature", "operator", "threshold"] + [key for key in gates[0] if key != "conditions"]
        writer = csv.DictWriter(destination, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for rule in gates:
            writer.writerow({**rule["conditions"][0], **{key: value for key, value in rule.items() if key != "conditions"}})
    (args.output / "development_feature_summary.json").write_text(json.dumps(result, indent=2) + "\n")
    selected_rule = best.get("pair_coverage0.95_pillar0.80")
    if selected_rule:
        keep = candidate.copy()
        for condition in selected_rule[1]["conditions"]:
            values = data[condition["feature"]]
            keep &= np.isfinite(values) & ((values >= condition["threshold"])
                                          if condition["operator"] == ">=" else (values <= condition["threshold"]))
        examples = []
        groups = [
            ("fine_supported_rejected_by_pair", positive & candidate & ~keep, weights, 8),
            ("fine_absent_accepted_by_pair", ~positive & keep, data["assigned_score"], 8),
            ("fine_supported_low_whole_core_fraction", positive & keep & (data["coreFraction"] < .55), weights, 6),
            ("fine_supported_missing_from_candidates", positive & ~candidate, weights, 6),
        ]
        columns = ["frame", "pillarIndex", "fineOwnPointCount", "fineOwnOffGroundPointCount",
                   "legacy", "assigned_found", "assigned_score", "pointScore", "coreFraction", "coreIsolation",
                   "assigned_ownCount", "assigned_ownHeight", "assigned_height", "assigned_radius",
                   "context_coreFraction25_75", "context_balancedMaximumStd25", "context_tiltDegrees",
                   "context_ownerCenterOffset", "context_ownerRadialRms"]
        for label, mask, ranking, count in groups:
            indices = np.flatnonzero(mask)
            indices = indices[np.argsort(-np.nan_to_num(ranking[indices]))[:count]]
            examples.extend({"case": label, **{name: float(data[name][i]) for name in columns}} for i in indices)
        with (args.output / "development_review_examples.csv").open("w", newline="") as destination:
            writer = csv.DictWriter(destination, fieldnames=["case"] + columns, lineterminator="\n")
            writer.writeheader()
            writer.writerows(examples)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
