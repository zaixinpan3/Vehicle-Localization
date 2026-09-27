# Precision-first coarse pole selection — 2026-09-27

The user explicitly prioritizes avoiding false selected pillars and accepts
reference-point coverage below the preceding 80% floor. This change raises
the default score threshold from `0.1488746496646483` to
`0.9371209151674477`. It supersedes the high-coverage operating policy at
commit `6df904cfed15c72e09cabb3a39c2c620522faae1`.

The 0.6 m pillar grid, candidate point-distribution measurements, continuous
partial-shaft support, 143-feature scoring model and all 250 trees are
unchanged. The model JSON changes only its decision-threshold metadata.
Original 0.3 m fine pole points are offline references; inference neither
loads references nor creates a fine XY grid. A pillar remains false only
when it contains **zero** original reference points. Coverage retains all
original reference points in the original coarse ROI denominator.

## Selection policy and its limitations

`selectPrecisionThreshold.py` selects threshold candidates from the existing
five purged prefix folds: Mississippi frames 1–780, Downtown selected frames
at or before 360; 20/10-frame purge margins; original training seed 927.
This task does not retrain the score model.

The deployed threshold maximizes the worse dataset's reference-point coverage
subject to **at most 5% pooled out-of-fold false selection in each dataset**.
This leaves an empirical margin below the 10% requested upper limit. There
is no requirement to improve old coverage and no 80% coverage floor.
Every prefix fold must retain at least one selection. The 5% margin is an
engineering choice, not a confidence bound; adjacent frames are correlated.

| Prefix out-of-fold policy | Mississippi false / coverage | Downtown false / coverage |
|---|---:|---:|
| Previous high-coverage default | 38.38% / 96.44% | 45.51% / 90.71% |
| At most 10% pooled false selection | 8.35% / 78.58% | 9.96% / 54.24% |
| **At most 5% pooled, deployed** | **4.62% / 71.27%** | **4.85% / 43.25%** |
| At most 5% pooled and 10% in every fold | 3.50% / 61.02% | 2.70% / 32.76% |

The initial stricter every-fold guard is retained in
`initial_guarded_probe.json`. Its Downtown suffix coverage is only 16.10%
with 1/16 false selections. The pooled 5% candidate retains 38.61% with
3/40 false selections. Both meet 10% in aggregate on that suffix, so the
pooled candidate is promoted to retain more usable points. Threshold values
come from prefix scores, but this final policy choice considers the reused
suffix tradeoff. **These suffixes are not independent test data.**

The pooled 10% candidate has 11/70 = 15.71% false selections on the Downtown
suffix. Merely selecting the prefix operating point nearest 10% does not
leave sufficient margin there. Some individual prefix folds of the deployed
policy also exceed 10%, with a maximum of 20%; these appear in
`prefix_fold_metrics.csv`. There is no per-frame, per-block or future-scene
precision guarantee. The small Downtown suffix denominator of 40 selections
limits the strength of its aggregate percentage.

## Raw replay results

The paired replay compares the revised default against the immediately
preceding high-coverage default, on the same raw frames and frozen fine points.

| Reused temporal suffix | False selection, previous → revised | Point coverage, previous → revised |
|---|---:|---:|
| Mississippi, 390 frames | 910/1810 = 50.28% → **20/397 = 5.04%** | 65732/69592 = 94.45% → **43411/69592 = 62.38%** |
| Downtown, 43 frames | 170/313 = 54.31% → **3/40 = 7.50%** | 6393/7113 = 89.88% → **2746/7113 = 38.61%** |

The missed reference-point fractions on the suffixes are therefore **37.62%**
and **61.39%**. This is a substantial coverage cost, particularly Downtown,
and is reported alongside the precision improvement.
The suffixes contain 77/390 and 12/43 frames, respectively, with reference
pole points but no selected pole pillar at all; downstream localization
availability is not established by the aggregate precision result.

| Full replay, including fitted prefixes | False selection, previous → revised | Point coverage, previous → revised |
|---|---:|---:|
| Mississippi, 1170 frames | 40.84% → **30/1222 = 2.45%** | 97.42% → **70.48%** |
| Downtown, 128 frames | 38.73% → **3/275 = 1.09%** | 94.07% → **58.33%** |

Full replay includes training frames and must not be used as an independent
generalization estimate. Counts represent frame/pillar occurrences, not
distinct tracked physical poles. No reference-negative physical object is
relabeled to improve these results.

## Validation and reproduction

`replayPrecisionPriority` checks every selected set against frozen score
predictions, verifies that the stricter selection is a subset of the previous
selection, preserves ground/filter counts, and compares non-pole component
fields. Globally normalized mixture weights are the documented exception,
since they depend on the number and weights of pole components.
`validatePrecisionPriority` passes all existing 210 perception regression
tests, with zero failures or incomplete cases. Factory MATLAB Code Analyzer
reports no findings on the changed configuration and new MATLAB scripts.
Completed per-frame measurements and test outcomes are stored in this folder;
`summarizePrecisionPriority.py` refuses to produce a completed summary if any
count, denominator, selection, or test check fails.
The completed 1298-frame replay passes all those checks with unchanged
reference denominators and exact agreement with frozen score selections.

```sh
python research/pole_precision_priority_20260927/selectPrecisionThreshold.py
```

```matlab
setupVehicleLocalization;
addpath('research/pole_precision_priority_20260927');
validatePrecisionPriority;
replayPrecisionPriority;
checkPrecisionPriorityCode;
```

```sh
python research/pole_precision_priority_20260927/summarizePrecisionPriority.py
```

The selector requires local feature tables and the stored out-of-fold score
array in `output/pole_geometry_20260927/`. The raw replay requires original
`data/raw/MissisipiPointClouds.mat`, `data/raw/downTownPointClouds.mat` and frozen
reference caches in `output/pole_precision_20260927/`. Downtown uses the same
128 frames, `unique([1:5:539 round(linspace(1,539,24))])`. Large caches and raw
recordings remain outside the commit. MATLAB R2026a and Python/NumPy execute
the checks; this task requires no model training or Python inference at runtime.

No new runtime benchmark is claimed. Candidate generation and tree scoring
are unchanged, so the higher threshold should not be interpreted as a major
perception speed improvement. No downstream localization improvement has been
measured. The remaining task is to recover reference coverage while preserving
precision, and to verify the policy on an independent recording.
