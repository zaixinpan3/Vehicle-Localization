#!/usr/bin/env python3
"""Test shallow statistical classifiers with blocked development-only validation.

This explores separability; no trained model is installed in production. All
models use the same predeclared five temporal folds and a 20-frame purge around
each validation block. Later holdout rows are discarded before label parsing.
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from sklearn.ensemble import GradientBoostingClassifier, RandomForestClassifier
from sklearn.tree import DecisionTreeClassifier, export_text

from analyzeCandidateDistributions import development_table


SEED = 6260926
BASE_FEATURES = [
    "pointScore", "lineScore", "coreFraction", "coreHeight", "coreIsolation", "corePointCount",
    "pillarZRange", "pointCount", "wholeTilt", "wholeRadialStd", "wholeHeightStd",
    "assigned_score", "assigned_supportCount", "assigned_ownCount", "assigned_ownHeight",
    "assigned_height", "assigned_robustHeight", "assigned_radialRms", "assigned_maximumGap",
    "assigned_radius", "assigned_peakContrast", "assigned_excessCount", "assigned_radialSignificance",
    "assigned_heightCoverage", "assigned_fitScaleCount",
    "context_coreFraction25_40", "context_coreFraction25_75", "context_supportFraction25",
    "context_supportFraction75", "context_densityContrast25_40", "context_densityContrast25_75",
    "context_supportDensityContrast75", "context_growthDimension25_75", "context_wideMaximumStd25",
    "context_wideMinimumStd25", "context_wideAspect25", "context_balancedMaximumStd25",
    "context_balancedAspect25", "context_minimumHalfFraction25_75", "context_minimumQuarterFraction25_75",
    "context_minimumQuarterSupport", "context_maximumSupportGap", "context_supportCoverage20",
    "context_supportedFraction25_75", "context_halfAxisDisplacement", "context_halfSlopeDifference",
    "context_halfMaximumRms", "context_tiltDegrees", "context_ownerSupportCount", "context_ownerSupportHeight",
    "context_ownerRadialRms", "context_ownerRadiusFraction", "context_ownerCenterOffset",
    "context_ownerSupportFraction25", "context_ownerCoverage20", "context_ownerMaximumGap",
]


def prepare(values, medians=None):
    missing = ~np.isfinite(values)
    if medians is None:
        medians = np.array([np.median(column[np.isfinite(column)]) if np.isfinite(column).any() else 0
                            for column in values.T])
    return np.c_[np.where(missing, medians, values), missing.astype(float)], medians


def weights(labels, fine_points):
    # Preserve sparse positive pillars while mildly weighting their point mass.
    result = np.ones(len(labels))
    if labels.any():
        median = max(float(np.median(fine_points[labels])), 1)
        result[labels] = np.sqrt(1 + fine_points[labels] / median)
        result[labels] *= (~labels).sum() / result[labels].sum()
    return result


def model_factory(name):
    if name.startswith("tree"):
        return DecisionTreeClassifier(max_depth=int(name[-1]), min_samples_leaf=8, random_state=SEED)
    if name == "forest4":
        return RandomForestClassifier(n_estimators=64, max_depth=4, min_samples_leaf=6,
                                      max_features=.7, random_state=SEED, n_jobs=1)
    if name.startswith("boost"):
        return GradientBoostingClassifier(n_estimators=60, learning_rate=.05, max_depth=int(name[-1]),
                                          min_samples_leaf=10, random_state=SEED)
    raise ValueError(name)


def main():
    folder = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=folder / "candidate_features.csv")
    parser.add_argument("--output", type=Path, default=folder)
    args = parser.parse_args()
    data = development_table(args.csv)
    frames = development_table(args.csv.with_name(args.csv.stem + "_frames.csv"))
    eligible = data["candidate"] == 1
    data = {name: value[eligible] for name, value in data.items()}
    y = data["fineOwnPointCount"] > 0
    fine_points = data["fineOwnPointCount"]
    point_denominator = int(frames["finePointCount"].sum())
    pillar_denominator = int(frames["finePillarCount"].sum())
    blocks = np.array_split(np.unique(frames["frame"]), 5)
    folds = []
    for block in blocks:
        validate = np.isin(data["frame"], block)
        train = (data["frame"] < block.min() - 20) | (data["frame"] > block.max() + 20)
        folds.append((train, validate))

    def metrics(score, threshold):
        keep = score >= threshold
        matched = int(np.count_nonzero(keep & y))
        selected = int(keep.sum())
        covered = int(fine_points[keep].sum())
        return {"threshold": float(threshold), "selected": selected, "extra": selected - matched,
                "matchedFinePillars": matched, "coveredFinePoints": covered,
                "pointCoverage": covered / point_denominator, "pillarRecall": matched / pillar_denominator,
                "precision": matched / selected if selected else 0}

    def frontier(score, point_target, pillar_target):
        for threshold in np.unique(score)[::-1]:
            result = metrics(score, threshold)
            if result["pointCoverage"] >= point_target and result["pillarRecall"] >= pillar_target:
                return result
        return None

    results = []
    scores = {}
    trained = {}
    for use_range in (False, True):
        features = [name for name in BASE_FEATURES if name in data] + (["range"] if use_range else [])
        X = np.column_stack([data[name] for name in features])
        for model_name in ("tree3", "tree4", "forest4", "boost2", "boost3"):
            name = model_name + ("_range" if use_range else "_no_range")
            score = np.full(len(y), np.nan)
            for train, validate in folds:
                xtrain, medians = prepare(X[train])
                xvalidate, _ = prepare(X[validate], medians)
                model = model_factory(model_name)
                model.fit(xtrain, y[train], sample_weight=weights(y[train], fine_points[train]))
                score[validate] = model.predict_proba(xvalidate)[:, 1]
            assert np.isfinite(score).all()
            xall, medians = prepare(X)
            model = model_factory(model_name)
            model.fit(xall, y, sample_weight=weights(y, fine_points))
            training_score = model.predict_proba(xall)[:, 1]
            scores[name] = score
            trained[name] = (model, features, medians)
            for point_target in (.90, .95):
                rule = frontier(score, point_target, .80)
                if rule is None:
                    continue
                training = metrics(training_score, rule["threshold"])
                results.append({"model": name, "withRange": use_range, "features": len(features),
                                "pointTarget": point_target, "pillarTarget": .80, **rule,
                                "trainingExtra": training["extra"], "trainingPointCoverage": training["pointCoverage"],
                                "trainingPillarRecall": training["pillarRecall"]})
            print(name, "complete", flush=True)

    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "development_classifier_comparison.csv").open("w", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=list(results[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(results)
    primary = min((row for row in results if row["pointTarget"] == .95),
                  key=lambda row: (row["extra"], -row["coveredFinePoints"]))
    model, features, medians = trained[primary["model"]]
    names = features + [name + "__missing" for name in features]
    importance = sorted(zip(names, model.feature_importances_), key=lambda pair: -pair[1])
    summary = {"developmentOnly": True, "seed": SEED, "folds": 5, "purgeRawFrames": 20,
               "pointDenominator": point_denominator, "pillarDenominator": pillar_denominator,
               "blocks": [{"first": int(block[0]), "last": int(block[-1])} for block in blocks],
               "bestPrimaryByDevelopmentOOF": primary,
               "featureImportance": [{"feature": name, "importance": float(value)}
                                     for name, value in importance if value > 0],
               "features": features,
               "limitations": ["Thresholds and model selection reuse development OOF scores; holdout validation remains necessary.",
                               "Candidate generation misses remain in denominators; models cannot recover absent modes.",
                               "Fine reference is algorithm output, not independently annotated physical-pole truth."]}
    (args.output / "development_classifier_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    with (args.output / "development_classifier_predictions.csv").open("w", newline="") as destination:
        writer = csv.writer(destination, lineterminator="\n")
        writer.writerow(["frame", "pillarIndex", "fineOwnPointCount"] + list(scores))
        for i in range(len(y)):
            writer.writerow([int(data["frame"][i]), int(data["pillarIndex"][i]), int(fine_points[i])]
                            + [float(score[i]) for score in scores.values()])
    for name, (fitted, features, _) in trained.items():
        if isinstance(fitted, DecisionTreeClassifier):
            (args.output / f"development_{name}.txt").write_text(
                export_text(fitted, feature_names=features + [name + "__missing" for name in features], decimals=5))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
