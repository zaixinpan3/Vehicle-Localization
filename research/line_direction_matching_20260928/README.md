# Stabilize map matching with local curb direction

The deployed change reduces Mississippi frame 601 from **85.16 cm to 20.54 cm**
horizontal error and from **4.532 to 0.775 degrees** heading error. It uses the
same 0.6 m perception selections, five-scan source window, calibrated map and
recursive wheel/gyro/lateral motion. No reference position, fine point labels,
manual correspondence or frame-specific condition is supplied to the new factor.

## Change

`prepareSemanticRegistrationGeometry` previously used curb line-normal position
residuals. Short biased support could rotate toward a displaced pole even when
its selected curb means showed a different line direction. The solver now also
compares the local source line direction with the map component direction.

For a unit source tangent t, map normal n and candidate yaw rotation R, the added
residual is `n' * R * t / sigma`. Its yaw derivative is analytic and its translation
derivative is zero. Squaring makes the factor invariant to tangent/normal sign.
It enters both same-class target assignment and the robust Gauss-Newton residual,
so association and pose optimization use consistent evidence. Existing semantic
balancing, temporal support, robust loss, information-coordinate transforms and
class-consistency gates remain in effect.

`sourceLineDirections` estimates the tangent from already selected source means:

- 4 m neighborhood, at least three distinct positive-quality component centers;
- at least 2.4 m span and principal variance ratio at least nine;
- curb/facade only; no extra point selection or fine perception;
- repeated identical centers cannot create additional directional support;
- an engineering angular scale of 1 degree, not calibrated measurement noise.

The factor adds yaw information without constraining translation along a straight
road. Sparse or non-collinear neighborhoods get no direction factor. New default
configurations enable it through `distributionRegistrationConfig.lineDirection`.
Explicitly disabling that group, or loading an older configuration without it,
preserves the old geometric objective for reproducible historical experiments.
`lineDirectionUsed` and `lineDirectionResidual` are exported per correspondence.
These are correlated geometric features derived from existing data, not new
independent sensor observations; exported information remains uncalibrated.

## Full-route validation

All 1,170 raw Mississippi scans were replayed recursively from one initial
reference offset of [0.5 m, -0.4 m, 2 degrees]. Current MnCAV gains were synthesized
fresh; subsequent predictions use wheel speed, corrected gyro and lateral velocity.
Recorded INS tilt is supplied as before. No global GNSS/LiDAR pose fusion is run.
The position/yaw reference is used only for initialization and evaluation.

| Metric | Previous production | New production |
|---|---:|---:|
| Frame 601 position error | 85.159 cm | **20.538 cm** |
| Frame 601 heading error | 4.532 deg | **0.775 deg** |
| All-output position RMSE | 16.609 cm | **14.079 cm** |
| All-output position P95 | 33.733 cm | **26.375 cm** |
| Heading RMSE | 0.6241 deg | **0.4334 deg** |
| Maximum after startup frame | 85.159 cm | **56.304 cm** |
| Full-pose matches | 1,150 | 1,148 |
| Directional updates | 19 | 15 |
| No measurement update | 1 | 7 |
| Median total compute time | 143.668 ms | 143.638 ms |
| Median window + matching time | 11.431 ms | 12.472 ms |

Frame 601 is a full-pose accepted measurement, not a rejected prediction counted
as a successful match. Position error falls by 75.88%. The all-output maximum is
64.031 cm because the unchanged initial-offset frame has no measurement yet.
The six additional missing updates are rejected for class inconsistency; no final
run call fails to converge. Candidate rejection is preserved rather than relaxing
acceptance thresholds to claim a higher success rate.

All outputs, including predictions, are included; no trajectory alignment or
outlier removal is applied. P95 uses linear interpolation. At a 1e-6 m change
threshold, 547 frames improve and 586 worsen; this is an aggregate/tail improvement,
not a claim that every pose improves. Total timings are observational sequential
workstation runs, not an isolated speed benchmark. The roughly 144 ms median still
misses a 100 ms/10 Hz budget. Perception and matching timings retain cold calls and
exclude disk reads, gain synthesis and offline preparation.

## Alternatives and selection

- Radius 3/4/6 m and angular scale 1/2/3/5/10 degrees were screened at frame 601.
- Causal horizons 3/5/8/10/15 and scales 0.25/0.5/1/2 degrees were also screened.
  Longer windows did not consistently help and were not deployed.
- Full recursive comparisons on identical raw sources reproduced the baseline
  within 9.4e-10. Direction in residuals alone yielded 13.749 cm RMSE at 1 degree
  and 14.784 cm at 0.5 degree, but each had ten nonconverged calls.
- Using direction in association as well yields 14.079 cm RMSE, removes those
  nonconverged cases and accepts 1,148 full poses. This coherent association/solve
  variant is selected despite its slightly higher aggregate error than the
  residual-only variant. No convergence or class gate was weakened.
- A fresh full raw replay of residual-only production is retained locally as
  `production_direction_only`; the selected fresh replay is `production`.
- An initial synthetic test assumed that suppressing yaw bias must always reduce
  position bias. That assumption failed: with an inconsistent point landmark,
  translation can absorb more bias as yaw improves. The retained failed prototype
  test CSV documents this limitation; final tests check the actual yaw, geometric
  observability and derivative contracts rather than promising universal accuracy.
  Initial toolbox-dependent `range` and test-fixture path errors were corrected.

The existing drive, including frame 601 and the complete route, was used to select
and validate this change. The map contains query observations and perception
models/calibration also used this drive. This is same-drive engineering validation,
not an untouched test or independent physical accuracy measurement. The remaining
20.54 cm frame-601 error is not eliminated: the earlier source/map pole-center
inconsistency is still present. No map recalibration or pose-reference correction
is claimed. Downtown and global-fusion observer performance were not replayed.

## Checks and reproduction

**79 tests pass**, zero failed or incomplete, across the new direction suite and
five existing registration suites. Checks cover sparse/nonlinear neighborhoods,
duplicate invariance, legacy-off equivalence, invalid scale rejection, straight-road
null directions, known SE(2) recovery at large coordinates, biased-pole yaw behavior,
direction-consistent target selection, finite-difference frozen-cost gradients,
physical information, class/height behavior and position-aided compatibility.
Factory MATLAB analysis reports zero findings in 12 implementation/test/study files.
Python compilation and scoped whitespace checks pass.

The independent analyzer verifies all 1,170 outputs, unchanged reference epochs,
XY/wrapped-yaw arithmetic, full-run prototype/production agreement (maximum pose
coordinate difference 1.03e-8), all 10 original run input hashes, executed source
hashes, and 124 unchanged perception/mapping/configuration files. Frame-601 pair
export independently reproduces the deployed pose. Pre-existing unrelated observer
working changes were preserved and executed as in the baseline; their source hashes
remain recorded rather than being included in this implementation commit.

From the repository root (dependencies/data from the baseline must be present):

```sh
python research/line_direction_matching_20260928/preparePrototype.py
matlab -batch "addpath('research/line_direction_matching_20260928'); screenDirection; screenWindow; evaluateDirection; evaluateAssociation;"
matlab -batch "addpath('research/line_direction_matching_20260928'); runProduction; inspectProduction601; checkCode;"
uv run --offline --with numpy --with pandas --with matplotlib python research/line_direction_matching_20260928/analyzeProduction.py
```

`runProduction` refuses to overwrite a completed result. It uses installed YALMIP
and SeDuMi in the sibling `RobustVehicleLocalization/external` directory. Prototype
copies are generated from the pre-change Git revision; they are not runtime source.
The independent analyzer consumes the recorded source manifest captured before the
run. Reproduction of the exact historical motion also requires the unchanged
working observer source identified by the baseline manifests/local patch.

`summary.json`, complete production CSVs, control CSVs, tests, code analysis,
frame-601 correspondences and `correction.png`/`.pdf` provide reviewable evidence.
Large MAT caches, logs, temporary solver copies, input data and solver dependencies
remain under their normal local paths and are excluded from Git. Perception models,
original fine reference masks and the map are unchanged.
