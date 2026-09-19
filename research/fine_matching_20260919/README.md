# Fresh fine-perception matching experiment

Date: 2026-09-19. Production source revision examined:
`48d043b083ccc2fb44c19726655223b1d5054484`.

## Results

Fresh fine point perception does **not** improve the complete matching trajectory
with the current distribution construction and matcher. All 1170 frames were
processed, followed by five complete sequential replay arms (5850 matching
calls). XY discrepancy is measured directly against recorded INSPVA, without
trajectory alignment, clipping or removing difficult frames.

| Source and initialization | XY RMSE (m) | Median (m) | P95 (m) | Maximum (m) | Accepted / 1170 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Coarse, own preceding output | 0.182535 | 0.115818 | 0.328015 | 0.844666 | 1166 |
| Fresh fine, own preceding output | 0.200994 | 0.107084 | 0.436496 | 1.016281 | 1164 |
| Coarse with binary semantic quality, own preceding output | 0.181713 | 0.117350 | 0.325636 | 0.845425 | 1167 |
| Coarse, frozen coarse initial guesses | 0.182535 | 0.115818 | 0.328015 | 0.844666 | 1166 |
| Fresh fine, frozen coarse initial guesses | 0.181344 | 0.104670 | 0.343478 | 0.885137 | 1164 |

P95 uses the Hazen quantile convention. These all-output metrics include motion
predictions on rejected frames. Counting only accepted measurements gives
coarse/fine RMSE of 0.182346/0.194881 m. On the same 1161 frames accepted by both
recursive arms, RMSE is 0.179412/0.194967 m. Thus the fine trajectory's poorer
RMSE is not explained just by its additional rejected measurements. Recursive
yaw RMSE changes from 0.582579 to 0.829709 degrees; errors above 0.5 m increase
from 31 to 41 frames.

The median improves, and fine is better on 619 frames. However, 112 frames
worsen by more than 0.1 m while 74 improve by more than 0.1 m. In the recursive
comparison, frame 92 improves from 0.844666 to 0.318069 m and frame 959 from
0.661315 to approximately 0.246 m. These local gains do not eliminate the
sequence's tail errors.

The new fine maximum is frame 809. The fine recursive prediction differs from
the frozen coarse prediction by 0.7694 m in XY. The same fine source window
gives 1.016281 m error from the recursive seed, versus 0.2807 m from the frozen
coarse seed. The former has similarity 0.481758, rank three and 60 matched
components and passes acceptance. At frame 808 coarse recovers to 0.2146 m
while fine remains at 0.8851 m; the differing predictions subsequently enter
different solutions. Fine errors remain between 0.89 and 0.97 m at frames
810--816, including rejected frames 814--816. This is concrete evidence of
initialization sensitivity and inadequate rejection of an erroneous solution,
not proof of the physical correctness of individual correspondences.

With common initial guesses the fine RMSE improvement is only 0.00119 m
(0.65 percent), while its P95 and maximum are worse. The coarse binary-quality
control has RMSE 0.181713 m, so simply assigning probability one to accepted
labels does not reproduce the fine recursive deterioration. Fine filtering
also changes the Gaussian centers, covariances, class coverage and occupancy:
median per-frame curb/pole/sign components change from 69/7/3 to 29/3/3, and
median matched window components from 228 to 105. This experiment does not
separate these individual effects, establish which semantic labels are more
accurate, or show that fine perception is inherently unsuitable for matching.

The supported decision is to retain the coarse online input and focus further
matching work on correspondence/constraint consistency and recovery from
misleading local solutions. Replacing the current coarse input with the
current fine input alone is not a demonstrated accuracy improvement. No
production behavior is changed by this diagnostic experiment.

See `metrics.csv`, `checks.json`, `diagnostics.json` and `selected_frames.csv`.
The full comparison plot is
`output/fine_matching_20260919/fine_vs_coarse.png` (also PDF).

## Experimental contract

This is a temporary research replay. The online `localizeLidarFrame` entry point
and all production defaults remain coarse-only. No fine-perception, map,
registration, motion, or observer parameter is retuned for this experiment.
No global fusion observer is involved.

Every one of the 1170 original Mississippi frames is processed afresh through
`perceiveFrame` with `executionMode="offline"`. This produces both the ordinary
whole-pillar probability cloud and independent fine point masks for curb, pole
and traffic sign. Four disjoint contiguous partitions run in independent MATLAB
processes with two computational threads each. This parallelizes stateless
perception only; each matching trajectory is replayed sequentially.

`buildFineMatchingCloud` takes the selected **raw-frame points**, applies the
same stored-point calibration and known recorded tilt as the coarse path, and
computes population means/covariances in the existing 0.9 m output lattice.
Covariance bounds, regularization and occupancy transfer match coarse settings.
It uses no ring layout, reference XY/yaw, registered map observations or future
scan. Every accepted binary fine label has semantic probability one. The
three-scan source window, independent wheel/gyro/lateral transport, fixed map,
map crop, stored stability weights, solver and acceptance gates are unchanged.

The primary comparison is **coarse versus fine recursive matching**. Each arm
starts at the same reference-plus-`[0.5 m,-0.4 m,2 degrees]` initial guess and
then predicts from its own preceding accepted pose and independent odometry.
Rejected matches produce that arm's motion prediction and remain in all-output
metrics. A separate `commonInitialGuess` control gives coarse and fine the
original coarse prediction at every frame to distinguish source effects from
feedback into later initial guesses. This control is diagnostic, not an
independent localization trajectory.

A third recursive arm retains coarse geometry and labels but sets accepted
semantic probabilities to one. It controls for the binary-probability policy
used with fine masks; occupancy and within-class normalization remain active.
This quality-only control is not a proposed production configuration.

## Verification and artifacts

The adapter's four class-based MATLAB tests check exclusion of unselected
outliers, analytical population covariance, calibration followed by tilt
exactly once, empty masks, and invariance to unorganized point order. They run
through the MATLAB MCP session. The experiment compares all fresh coarse
Gaussian means, covariances and semantic memberships with the frozen baseline,
and asserts that coarse matching reproduces the original trajectory. All four
adapter tests pass. Fresh coarse means/covariances exactly equal the cached
baseline on every frame (maximum differences zero); independent Python checks
confirm both coarse replay trajectories within 1e-7 of the recorded poses.
MATLAB Code Analyzer reports zero messages for the four MATLAB research files.

An additional frame-92 coordinate audit compares fresh fine selected points
with the same frame's registered map-building observations: counts are 103
curb, 60 pole and 124 traffic-sign points in both products, with maximum global
coordinate difference 9.32e-10 m. These map observations are used only for
verification, never as matching source input in this experiment.

`analyze.py` independently verifies all-frame coverage, each recursive prediction,
fallback outputs on rejected frames, window size/age, error calculations and
positive accepted information. It reports all outputs, accepted measurements
and the identical common-accepted population. It exports a complete error trace
and cumulative distribution without trajectory alignment or error clipping.

Large caches, per-frame trajectories and plots are in
`output/fine_matching_20260919/`. Compact summaries, selected-frame tables,
checks and the research scripts are tracked here. Perception timing is recorded
under concurrent load; coarse timing in replay tables is inherited from the
baseline. This is not a controlled speed benchmark.

## Reproduction

Run `cacheInputs(k,4)` for each `k=1,2,3,4` in separate MATLAB processes after
adding the repository and this research directory to the MATLAB path. Each
process writes only its own input partition. After all four finish:

```matlab
setupVehicleLocalization;
addpath('research/fine_matching_20260919');
results=runtests('research/fine_matching_20260919/fineMatchingCloudTest.m');
assert(all([results.Passed]));
writetable(table(results),'output/fine_matching_20260919/adapter_tests.csv');
runExperiment;
```

```bash
uv run --offline --with numpy --with pandas --with matplotlib python research/fine_matching_20260919/analyze.py
```

The map is the unchanged
`output/mississippi_mapping_inspva_20260915/probability_cloud.mat` as projected
in the validated baseline cache. Independent motion and reference timestamps
come from `output/matching_refinement_20260919/validated/recursive/report.mat`.
The map includes observations from the query drive; reference INSPVA is not
independent surveyed truth, and recorded tilt is assumed known. Results are
development comparisons on this drive, not independent accuracy guarantees.
