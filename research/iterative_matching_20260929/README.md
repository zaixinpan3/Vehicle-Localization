# Iterative Mississippi worst-frame matching study

The deployed change reduces the largest post-initialization horizontal error
from **0.396434602 m (frame 854)** to **0.300607691 m (frame 178)**, a **24.17%**
reduction. Full-route RMSE changes from **0.132390519 m** to **0.126034268 m**;
MATLAB `prctile` P95 changes from **0.242411397 m** to **0.223979462 m**.
There are 1,153 full updates and 10 directional updates, versus 1,152 and 10.
The configured first-frame offset is 0.640312424 m in both runs: it is excluded
only from the operational maximum, and included in RMSE/P95. No claim that the
literal maximum including initialization is 0.3006 m is intended.

## Scope and evaluation protocol

Baseline commit: `6b600c56fde32107371fec127e5e78826aa0bca2`.
All experiments use Mississippi's 1,170 frames, the existing calibrated map,
the same initial state, and separately recorded cumulative wheel/gyro motion.
Pose is recursively propagated from the last accepted full/directional event;
there are no intermediate reference XY/yaw resets. Reference poses are used
for scoring and explicitly labeled diagnostic reference-seed controls only.
The existing roll/pitch projection is retained. Downtown was not evaluated.

This is development on a repeatedly inspected route and map, **not an
independent holdout evaluation**. Its fine perception labels are imperfect
reference labels, not exhaustive physical ground truth. No reference pose,
label, frame number, or special coordinate enters the deployed algorithm.

`replayFinalRoute.m` recomputes every raw scan and asserts exact equality of
all 1,170 current probability clouds **and** windowed sources against the
pre-study production replay. Thus the 0.6 m lattice, pillar masks, whole-pillar
moments, historical confirmation and learned pole model are unchanged.
The previously measured pole reference-empty fraction remains 36/1,542
(2.3346%); reference point coverage remains 153,665/194,300 (79.0865%). Curb and
sign detections likewise remain unchanged; no new all-class accuracy claim is
inferred from those pole metrics.

## Findings and changes

1. Frame 854's merged sign representation pulls the coarse solution forward.
   Fine refinement reaches 10.14 cm, but the old 15 cm trust gate retains the
   39.64 cm coarse result. Initializing at the reference pose gives the same
   biased result. Removing all merging or globally relaxing the trust gate
   creates worse peaks elsewhere (up to 82.20 cm). The new gate requires a
   matched pole to protect the coarse basin; a sign-only merged centroid
   cannot veto refinement.
2. Hard assignment to a single component makes nearby split map landmarks
   artificially decisive. `softPointAssociationTarget` moment-matches nearby
   same-class ambiguous targets, including **between-component covariance**.
   Geometry compatibility is tempered (2.5); stored map priors and optional
   relative-height penalties are not tempered. Radius is 1.5 m. A posterior
   at least 0.9 on the original MAP target preserves the sharp target.
3. Two distinct matched pole groups, each containing one original map
   component, restore hard fine association. Informative position-aid
   hypotheses also use a sharp fine solve. Missing, invalid-flag or uncertain
   aid leaves the LiDAR-only objective unchanged. Aid contributes no residual
   or information force. Relative-height evidence can still resolve aliases.
4. Curb directions are useful but are not equally reliable. Frame 885 improves
   from 36.83 to 10.71 cm with directions disabled; frame 178 becomes much
   worse with directions disabled. The adopted angular standard deviation is
   `hypot(1 degree, 0.25*sqrt(local minor/major variance))`, retaining the
   existing support/span/anisotropy checks.
5. Correspondences now expose the actual source mean, effective map mean and
   map covariance used by the residual. Original target/source indices are
   representatives when a canonical group is retained; they are not the
   effective blended means. Both indices are remapped to the original clouds.

## Rejected alternatives and stopping evidence

Recorded CSV screens cover map/source merge radii, trust radii, pole cutoffs,
window lengths, map prior flattening, line direction strength/locality,
current-only geometry or directions, independent-anchor rules, soft
association temperatures/radii/confidence, and combined variants.
`variant_summary.csv` consolidates replay labels; some labels are cleanup or
repeat controls, not distinct parameter sets or independent experiments.

- Global no-merge: peak 82.20 cm. Larger fine trust radius: 48.59 cm.
- Lower pole cutoffs 0.87/0.85/0.82: peak about 38.76 cm, with increasing
  reference-empty fractions (2.49%/2.65%/3.41%). Not deployed.
- Three/four/six-frame windows with the final association model: peaks
  82.20/38.87/31.19 cm. The five-frame production window is retained.
- Positive whole-pillar intensity weighting for signs, keeping masks fixed
  and every point's weight nonzero: peaks 31.07/30.86/30.57 cm. This experiment
  does not improve the peak and its representation change is not deployed.
- Geometry noise 0.05/0.15/0.20/0.30 m: peaks 30.31/33.84/35.77/38.09 cm.
- Neighboring soft-association profiles raise the peak or trade it for lower
  mean error. The retained peak is 30.06 cm; it is not a claim of global
  optimality or an impossibility result for future algorithms.
- Admitting high-score singleton poles at stability 0.1/0.2/0.4 improves frame
  178 (26.83 cm in the 0.94/0.1 profile), but moves the peak to frame 893/894
  at 30.91--32.08 cm. The current detector is not changed in these tests.
- A subsequent admission gate requires an already confirmed pole or sign in
  the source. This is a source-class corroboration heuristic, not a formal
  observability guarantee. It reduces the peak to 29.71 cm: only **3.53 mm**
  below deployment. The stronger temporal contract is retained in preference
  to this marginal gain, consistent with the user's low-false-selection
  priority. Both rejected admission families remain reproducible research
  scripts, with no production source-window change.

The stopping decision is practical: broad geometric changes caused regressions,
local sweeps plateaued, and the final new mechanism offers less than 1 cm of
peak improvement while weakening temporal confirmation. It does not establish
that a different map, sensor model or future algorithm cannot do better.

The remaining frame 178 reaches the same 30.06 cm error from the reference
seed. Single-scan geometry gives 20.31 cm, but globally discarding history is
unstable. Disabling direction yields 65.02 cm. Its pooled source has 33 curb
components and three signs, with no confirmed pole. The current scan does
contain one selected pole (30/38 fine points covered, zero reference-empty
pole pillars); temporal confirmation suppresses it. The viewer also measures
159/159 sign points and 115/119 curb points covered, with zero reference-empty
pillars for either class on this frame. Detection, temporal admission and map
association must therefore be distinguished when interpreting the display.

## Validation

- Fresh full-route raw perception and recursive registration completed;
  current clouds and windowed sources are bitwise equal to the baseline.
- Final raw poses agree with the cleaned cached replay to below 1e-9.
- **130 tests pass** under the deployed configuration in ten matching/window
  suites, including eight new ambiguity/anchor/aid tests. Nine changed
  implementation/config/test files have zero factory Code Analyzer findings.
- Two existing relative-height assertions already failed at baseline: one
  required a coarse-retention flag even when refinement stayed at the same
  midpoint; the other assumed an arbitrary planar alias instead of checking
  equality with height disabled. They now test the intended invariants.
- Canonical-pyramid tests that explicitly demonstrate hard-assignment failure
  now disable soft association in that control. Tests of the old trust gate
  explicitly use hard refinement; exact recovery assertions remain intact.
- An initial isolated-profile harness lost the repository root after changing
  directory. A later profile-shadow test also did not reliably preserve the
  profile through test setup. Those intermediate results are not final
  validation. `checkFinalImplementation` runs the actual deployed config
  directly. This caught and fixed height/position-aid interactions before the
  final 130-test pass.
- Matching timing in CSVs is affected by simultaneous local experiments and
  other workloads. No runtime improvement or real-time guarantee is claimed.

## Reproduction and artifacts

With the local recorded inputs available:

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/iterative_matching_20260929');
replayFinalRoute;
checkFinalImplementation;
```

```bash
uv run --offline --with numpy --with pandas --with matplotlib python \
  research/iterative_matching_20260929/summarizeStudy.py
```

`final_raw.csv`, `final_largest_errors.csv`, `final_summary.json`,
`final_config.json`, `final_tests.csv` and `code_analysis.csv` describe the
production result. `cleanSoft_frame178/` contains reproducible controls and
actual effective correspondence geometry. `route_comparison.png`/`.pdf` and
`view178/` are visualization exports. Niri window 170 displays all 65,536
original points with colors directly applied, size 4, no overlay points and a
nearby 0.6 m grid floor.

Historical rejected prototypes were deliberately removed from production.
To reproduce those screens, use a detached checkout of the baseline commit,
apply `experimental_solver.patch`, copy this research directory, and invoke
its corresponding `run*Variants` script. `baselineMatchingConfig` restores the
frozen baseline settings regardless of newer defaults. The patch contains
experimental code, **not an additional deployed change**. Newer sign/singleton
and final sensitivity scripts use the cleaned solver and saved profile.
Do not apply the prototype patch to the deployed tree.

Local, deliberately uncommitted data dependencies are:

- `data/raw/MissisipiPointClouds.mat`, pose CSV from `featureMapBuildConfig`;
- `output/mississippi_mapping_calibrated/probability_cloud.mat`;
- `output/line_direction_matching_20260928/sources.mat` (independent motion),
  `production/report.mat` (timestamps, initial state and scoring references);
- `output/pole_boundary_recovery_20260929/replay.mat` (baseline sources/clouds);
- `output/iterative_matching_20260929/` (MAT profiles, replays, test and logs).

Full raw data, binary dependencies and large MAT intermediates are not
published. Unrelated observer, vehicle simulation, lattice-study, instruction,
README and `reference/` workspace edits are excluded from this change.
