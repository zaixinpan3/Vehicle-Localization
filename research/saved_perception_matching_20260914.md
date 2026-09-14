# Pose and information from saved Mississippi perception

Date: 2026-09-14. The existing `geometricD2D` solver was executed against the
1170-frame map constructed on 2026-09-12, using saved fine-perception results.
The experiment produced full SE(2) measurement events and their physical
3-by-3 information matrices. Registration parameters and map geometry were not
changed. The solver implementation was at commit
`696e26f5b4825889d0081a5cd7eb72a8ae53006a`.

## Inputs and method

- Observations: `output/mississippi_perception_video_20260912/feature_observations.mat`.
- Fixed Gaussian map: `output/mississippi_mapping_20260912/probability_cloud.mat`.
- Source frames chosen before executing: 28, 214, 458, 600, 855, 943, 1137.
- Three initial offsets per frame: [0.5 m, -0.4 m, 2 degrees], its negative,
  and zero. No randomness, threshold tuning, or discarded trials.
- Existing `distributionRegistrationConfig`, with default XY height mode.

The saved feature points already have global coordinates. The new reusable
experiment entry point `scripts/matchSavedPerception.m` subtracts the recorded
position and applies inverse recorded yaw to recover gravity-aligned local XY.
This preserves the calibration and tilt already applied during collection.
The forward/inverse round-trip check returned zero error for all seven frames.

Per semantic class, the adapter aggregates local fine feature points into
0.9 m XY cells using the ROI and covariance safeguards from
`coarseSemanticProbabilityCloudConfig`. It uses population covariance plus
0.0001 square meters regularization, eigenvalues clipped to [0.01, 4] square
meters, unit semantic quality for the saved class labels, and occupancy quality
min(1, point count / 6). Mixture weights are uniform. This explicitly identified
fine-point adapter is not the online pillar-only perception product. Neither
fine nor coarse perception was rerun, and the map was not refitted. Full cached
source clouds and actual configuration objects are saved for inspection.

## Actual results

Twenty of 21 registrations were accepted as full-pose measurements; no
partial-direction event was accepted. Frame 855 with the positive initial
offset was rejected as `inconsistentClasses` (0.8652 m, 1.8700 degrees from the
recorded pose). Its pose remains a rejected solver candidate and is not exported
as a measurement. All runs, including this rejection, are in
[the case table](results/saved_perception_matching_20260914/cases.csv).

Among accepted runs, median/max XY differences from recorded poses were
0.0953353/0.2421952 m; median/max absolute yaw differences were
0.1721309/0.7239283 degrees. Acceptance sets and local minima differ by start,
including frame 458 (0.0670, 0.2259, 0.0072 m for the three starts).
Total solver time across 21 calls was 0.748071 seconds, including a first-call
cost of 0.232527 seconds. This is a single batch timing, not a real-time benchmark.

Frame 28, positive initial offset, provides a concrete measurement:

```text
pose = [484200.37930074934 m, 4977499.95369199 m, -2.125905570872644 rad]
information =
[13.020218777991  -2.463498135024    27.511724212714
 -2.463498135024 14.098184397570   -24.279579310920
 27.511724212714 -24.279579310920 2028.300363946203]
```

This run used 26 source components, converged in six iterations, had similarity
0.901162 and observable rank three, and reduced the 0.640312 m / 2 degree
initial discrepancy to 0.088183 m / 0.122787 degrees. The smallest physical
information eigenvalue was 11.015518; Cholesky factorization succeeded.

The matrix acts on additive map-coordinate [delta X, delta Y, delta psi],
with translation in meters and yaw in radians. It is the final robust composite
Gaussian Gauss-Newton information, including semantic quality, map repeatability,
robust influence, and cross terms. It is not an empirically calibrated inverse
pose-error covariance. Numeric eigenvalues depend on these mixed coordinates.
Events preserve acquisition timestamps; arrival time is explicitly marked as a
placeholder because a delivery-delay replay was not performed.

## Validation and limits

- `registrationInformationTest`, `repeatabilityRegistrationTest`, and
  `geometricRegistrationTest`: 45 passed, zero failed/incomplete.
- All exported event matrices were finite, symmetric, and positive
  semidefinite; every full-pose event additionally passed Cholesky factorization.
- All source Gaussian covariances passed the existing cloud schema validator.
- Code Analyzer with factory settings reported zero findings in the new script.
  The initial default-settings invocation reported a missing personal settings
  file and fell back to defaults; the factory invocation removed that environment
  warning without changing code.

These frames contributed to the fixed map, and their recorded poses were used
to invert the saved global coordinates. This is an in-sample perturbation and
interface consistency check, not independent localization accuracy or a test of
online coarse perception. A production assessment needs local coarse clouds,
an independent mapping pass, and an actual prediction source. The tested initial
poses are controlled offsets, not observer outputs. No observer replay, map
update, information calibration, or global relocalization was performed.

## Reproduction and artifacts

From the repository root in MATLAB:

```matlab
addpath(pwd); setupVehicleLocalization;
report = matchSavedPerception( ...
    'output/mississippi_perception_video_20260912/feature_observations.mat', ...
    'output/mississippi_mapping_20260912/probability_cloud.mat', ...
    'output/saved_perception_matching_20260914');
```

The output directory contains `matching_results.mat` (report, full solver
results, measurement events, source clouds, and configurations),
`matching_results.json`, `frame_28_measurement.mat` (variable `measurement`),
`frame_28_measurement.json`, `tests.mat/json`, `code_analysis.mat/json`, and
`manifest.json` with SHA-256 provenance. The MAT analyzer result retains the
initial environment warning; the JSON contains the final factory result.
The small example event is also committed as
[JSON](results/saved_perception_matching_20260914/frame_28_measurement.json).
Generated binaries and the recorded inputs remain local.
