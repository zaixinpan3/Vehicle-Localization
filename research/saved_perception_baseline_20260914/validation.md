# Full saved-perception map-matching experiment

## Material passport

Date: September 14, 2026. Mode: executed experiment and validation.
Verification: VERIFIED for the completed 4680 calls, exact 21-case historical
reproduction, independent metric calculations and 46 passing regression tests.
This study uses actual saved fine-perception data and the existing fixed map.
Neither lateral nor global localization observer is run.

## Inputs and fixed protocol

- All 1170 saved frames in `output/mississippi_perception_video_20260912/feature_observations.mat`.
- Existing 1331-component map in `output/mississippi_mapping_20260912/probability_cloud.mat`.
- Curb, pole and traffic-sign points; existing fine-point-to-Gaussian adapter,
  0.9 m XY cells, unchanged covariance/quality safeguards and registration settings.
- Four modes fixed before execution: per-frame offsets [0.5 m,-0.4 m,2 deg],
  its negative, zero; and one-initialization LiDAR-only recursive prediction.
- Every mode processes every frame, with no RNG, best-start selection,
  reference-error gate, perception rerun or map rebuild.

The saved points have global coordinates. `buildSavedFeatureProbabilityCloud`
uses the recorded pose to invert that storage transformation; all 1170
round-trip errors are zero. Those poses were also involved in constructing
this map. This is in-sample map consistency with known attitude/calibration,
not independent physical localization accuracy.

For the first three modes, reference-relative initialization on every frame
is deliberately a local-registration diagnostic. For the recursive mode,
only the first pose plus the positive offset initializes the trajectory.
Subsequent predictions use constant map-frame velocity and yaw rate computed
from the latest two full accepted matches. Directional events correct only
observable directions; they do not update the predictor velocity. Rejected
frames propagate prediction. There is no subsequent reference reset, no
motion-sensor input, and no observer. This is a deliberately simple predictor,
not a claim that every possible LiDAR-only odometry scheme fails.

The prior all-frame cache supplies scan clock metadata only; no prior
matching output enters the registration or prediction. The existing INSPVA
CSV supplies position evaluation. Heading is evaluated against recorded
mapping yaw for both reference labels. INSPVA is from the same receiver,
not independent ground truth. No reference trajectory is aligned to results.

## Full-pose measurement results

Statistics below use full accepted measurements only, with acceptance counts
shown explicitly. A directional event is not a full-pose measurement.

| Per-frame initialization | Full / directional / rejected | Mapping-pose median / RMSE (cm) | INSPVA median / RMSE (cm) | Heading RMSE (deg) |
|---|---:|---:|---:|---:|
| Positive offset | 1074 / 0 / 96 | 9.30 / 18.00 | 9.71 / 23.53 | 0.526 |
| Negative offset | 1088 / 0 / 82 | 9.69 / 21.71 | 10.08 / 25.99 | 0.566 |
| Zero offset | 1092 / 1 / 77 | 7.20 / 13.85 | 7.68 / 22.70 | 0.387 |

Full acceptance rates are 91.79%, 92.99% and 93.33%, respectively.
For the positive/negative/zero modes, respectively, full-measurement INSPVA
P95 is 42.51/48.15/36.56 cm; maximum is about 135.99 cm in each. The
fractions within 5 cm are 20.02/21.14/28.75%, and within 10 cm are
51.40/49.54/62.82%. Initialization affects both accepted populations and
local solutions; this table is not a paired comparison on a common subset.

The full-frame `all_outputs` metrics are retained separately in `metrics.csv`.
They include the unchanged seed on rejection for diagnostic modes and the
predicted pose on rejection for recursive mode. In particular, zero-offset
rejection retains the reference seed; its zero mapping discrepancy is NOT a
successful matching measurement. Do not use that fallback to claim accuracy.
Rejected solver candidates are separately recorded, while actual measurement
pose fields remain NaN. The full record is `output/.../calls.csv` and the
compact per-frame error/acceptance export is `frame_results.csv`.

There are genuinely centimeter-scale median discrepancies, but this
full-frame matching experiment does not establish a few-centimeter RMSE.
The earlier 18.85 cm continuous-input figure used a different coarse
perception product, motion-aided initialization and interpolated 100 Hz
population. It is not an identical-input comparison to this fine-feature,
per-frame-seeded experiment.

## Reference sensitivity and recursive failure

The difference between reference definitions is material. For example,
frame 1086's accepted solution differs from the recorded mapping pose by
about 2.81 cm but from INSPVA by about 130.34 cm. Other largest INSPVA
errors occur near frames 1078, 1090 and 1091. These overlap the previously
identified mixed ODOM/INSPVA source discrepancy. They cannot be attributed
entirely to registration failure, and proximity to the mapping pose does
not establish physical accuracy. No map correction was attempted here.

The recursive mode loses tracking: 162 full events, 220 directional events
and 788 rejections. Its last full event is frame 216, already displaced
14.46 m from INSPVA. Full-run output RMSE is about 301.82 m and final
error 840.91 m. The first errors over 1 m and 5 m occur at frames 96
(9.500 s) and 189 (18.807 s); the latter is directional. No reset removes
the failure. Its accepted-only median of 7.65 cm is dominated by early
successful tracking and must not be presented as a full-route result.
These data show the inadequacy of this predictor/registration combination;
they do not prove impossibility of LiDAR-only localization.

## Boundary bug and verification

The first batch stopped after its 200-frame checkpoint when exactly one
correspondence caused `repelem(weights.*robust,2)` to return a 1-by-2 row.
The whitened Jacobian had two rows, requiring a 2-by-1 weight column.
`registerSemanticProbabilityCloud` now uses `repelem(...,2,1)`. This makes
one-pair overlap return `insufficientOverlap`, not crash or accept a pose.

A minimal three-component fixture reproduces the pre-fix dimension error;
its new regression test verifies rejection, unchanged predicted pose, one
correspondence, finite information and no exported measurement after the
fix. All 46 geometric/information/repeatability tests pass afterward.
The original 800 checkpoint calls reproduce exactly except timings. Seven
historical source clouds and 21 complete registration result structures
also match exactly. The fix changes no accepted-path tuning or thresholds.

The completed batch took 101.126 s for feature adaptation and matching
loops, excluding input loading, MAT persistence, original perception and
mapping. This is a single offline timing, not a sensor-to-pose real-time
benchmark. Per-call times are retained. Three MATLAB files have zero factory
Code Analyzer findings. Independent Python calculations reproduce every
RMSE and median within 1e-12 m, verify four complete frame sequences and
NaN rejected measurements. CDF and error plots were visually inspected.
A temporary replot command initially resolved paths relative to `/tmp`;
executing the unchanged plot code in repository context corrected this
postprocessing issue without rerunning registration.

## Statistical checks and reproduction

All 11 checks were considered: Simpson's paradox (modes/references/populations
separate); ecological fallacy (no every-frame inference from median);
Berkson's paradox and collider bias (acceptance-conditioned results identified);
base-rate neglect (counts and acceptance rates shown); regression to mean
(no selection of extreme frames); survivorship bias (all frames and recursive
failure retained); look-elsewhere effect (all four preselected modes reported);
forking paths (no post-result tuning or best-start selection);
correlation/causation (reference disagreement not assigned a unique physical
cause); reverse causality (per-frame reference seeding disclosed, recursive
prediction does not use future/reference poses). Samples and mapping/query
frames are dependent. No statistical significance or independent-drive claim.

```matlab
setupVehicleLocalization;
report = runSavedPerceptionMatchingBaseline;
results = runtests({'tests/geometricRegistrationTest.m', ...
    'tests/registrationInformationTest.m','tests/repeatabilityRegistrationTest.m'});
assertSuccess(results);
```

The output folder contains all calls, full solver/event structures, adapted
source clouds, exact configuration objects, references, plots and before/after
test results. Small metrics, frame errors, checks and input hashes are
versioned beside this report. Original datasets and generated MAT/PDF/PNG
files remain local and are exported separately to the archive.


## September 15 follow-up: why median and RMSE differ

This analysis uses the unchanged accepted full-pose rows in
`frame_results.csv`; no matching is rerun. For each row, let e be its planar
position discrepancy. Median sorts e and takes its middle value; RMSE is
sqrt(sum(e^2)/N), so a small number of large discrepancies can dominate.
A 1 m discrepancy contributes as much squared error as 400 discrepancies
of 5 cm each; this compares squared-error sums, not equal-size datasets.

For the zero-offset mode, N=1092, INSPVA median=7.6768 cm and RMSE=22.7011 cm.
Exactly 22 accepted frames (2.0147%) have discrepancy at least 1 m and
contribute 61.0531% of the total squared error. The largest ceil(0.05*N)=55
frames contribute 74.6095%. These subsets overlap; their contributions
must not be added. The low median describes the center, while RMSE exposes
the much larger errors. Neither is an arithmetic mistake.

The previously identified mapping-source intervals contain frames
878--887 and 1078--1097. Of those 30 frames, 24 are accepted in this mode.
Those 24 account for 61.6921% of INSPVA squared error but only 1.2965% of
squared discrepancy from recorded mapping poses. This is strong reference
sensitivity, not a decomposition of physical error or proof of which
reference is correct. At frame 1086, discrepancies are about 130.34 cm
from INSPVA and 2.81 cm from its mapping pose. Rebuilding with a consistent
pose source has not been executed by this follow-up.

There are still genuine discrepancies relative to the existing map-pose
reference: median 7.2024 cm, RMSE 13.8464 cm, with 49 accepted frames at
least 30 cm away. Thus reference switching does not explain every large
matching discrepancy. Positive-offset results and both reference definitions
are retained in `error_concentration_20260915.csv`. Its groups are defined
by e>=1 m, the largest ceil(0.05*N) e values, and the fixed frame intervals
above. The squared-error share is sum(e_subset^2)/sum(e_all^2). Counts and
full metrics retain the same acceptance set; no rejected frame is converted
into a measurement and no large error is removed from the reported RMSE.
