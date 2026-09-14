# Diagnosis of saved-perception matching bias

Date: 2026-09-14. Baseline: `8db54b202cdfb06dd19e40e7578ed6119a679309`.

The investigation reproduced the existing pose discrepancies and tested source
resolution, curb covariance geometry, map scale, and multistart association.
None of these changes produced a reliable improvement across the additional
frames. The production registration and map defaults remain unchanged.

## Experimental scope

Inputs are the saved 1170-frame fine-perception observations and published
Gaussian map in `output/mississippi_perception_video_20260912/` and
`output/mississippi_mapping_20260912/`. No perception was rerun. All tests use the
existing XY D2D solver and the previous three starts: [0.5 m, -0.4 m, 2 degrees],
its negative, and zero relative to recorded pose.

The original set is frames 28, 214, 458, 600, 855, 943, 1137 (21 trials).
The additional set was fixed before its candidate-map evaluation as
`setdiff(15:30:1170,[28 214 458 600 855 943 1137])`: 38 frames, 114 trials.
These additional frames were not used to select the tested parameter values,
but they still belong to the mapping sequence. They are not an independent
route or ground-truth localization test. Saved global features are inverted
using recorded poses, as documented in the original experiment.

## Results

Accepted-only position differences from recorded poses are shown below. Full
acceptance sets differ, so `summary.json` also includes common-acceptance
comparisons and their sample counts.

| Change | Original full accepted | Original median/max (cm) | Additional full accepted | Additional median/max (cm) |
| --- | ---: | ---: | ---: | ---: |
| Baseline | 20/21 | 9.53 / 24.22 | 103/114 | 7.61 / 73.07 |
| Source XY cells 0.9 to 0.15 m | 20/21 | 7.90 / 23.66 | 103/114 | 8.17 / 120.49 |
| Map tiles 8 to 4 m, halo 2 to 1 m | 20/21 | 9.61 / 23.53 | 106/114 | 7.81 / 122.92 |
| Seven starts, highest accepted similarity | 21/21 | 7.01 / 24.22 | 110/114 | 8.02 / 120.95 |

The additional baseline includes one accepted directional event and ten
rejections. Its worst full accepted case is frame 1035, negative perturbation
(73.07 cm); frame 195 has a 42.38 cm negative-start discrepancy. The 0.15 m
source and 4 m map variants both introduce an accepted discrepancy over 1.2 m
at frame 1095, negative start. Keeping an unchanged count of accepted trials
therefore does not imply keeping the same acceptance set or tail behavior.

For multistart, the common 103 full-accepted additional cases have essentially
unchanged median discrepancy (7.6093 to 7.6091 cm). Thus the larger acceptance
count is not evidence that the existing geometric bias has been removed.
For the 4 m map, the common 101 full-accepted cases change median from 7.6093
to 7.5631 cm, while newly accepted failures worsen the tail.

The source-resolution sweep also evaluated 0.30, 0.45, and 1.8 m cells.
Original-set medians were respectively 8.62, 9.63, and 9.46 cm; no setting
removed the approximately 24 cm tail. The saved fine-point adapter uses existing
covariance floors and occupancy rules, so reducing cell size also changes the
number and quality distribution of source components. This is a representation
ablation, not a pure floating-point precision experiment.

## What the evidence establishes

1. **Initial conditions affect association and the selected local solution.**
   Frame 458 returns 6.70, 22.59, and 0.72 cm discrepancies from the three starts.
   Expanded evaluation reveals larger accepted alternative solutions. Trying
   more starts and selecting maximum similarity is insufficient: it can favor a
   more displaced accepted solution. Similarity and positive definite model
   information are not independent accuracy checks.
2. **The recorded pose is not generally an optimum of the current geometric
   estimator.** Frame 600 moves 11.70 cm even from zero perturbation, and frame
   1137 moves 17.62 cm. Reaching the same result from several starts rules out
   initial error alone as an explanation for these cases. It does not identify
   a unique physical error source or establish the recorded pose as exact truth.
3. **Classes provide different preferred corrections and incomplete geometry.**
   Starting at recorded pose and retaining only curb gives frame 600 a rank-two
   directional result displaced 32.09 cm with 1.945 degree yaw correction; this
   is not a full-pose accuracy measurement. Other curb-only cases also have
   rank two. Many frames have only one or two pole/sign source components,
   insufficient for standalone class registration under the existing three-
   component gate. Those early-return zero corrections must not be interpreted
   as accurate class measurements. Detailed results are in `class_ablation.json`.
4. **The tested covariance-normal explanation is too small to explain the main
   discrepancy.** For curb, rotating the predictive covariance eigenvectors to
   the within-block covariance eigenvectors while retaining its eigenvalues
   changes original median only from 9.5335 to 9.5299 cm. Substituting the full
   within-block covariance changes median to 9.4194 cm and maximum to 24.5010 cm.
   These are diagnostic projected clouds, not modified canonical maps.

The remaining geometry/model hypotheses include viewpoint-dependent feature
support and centroid motion, fitting a straight Gaussian to locally varying
curb geometry, and ambiguous same-class correspondences. They remain hypotheses;
this experiment does not isolate calibration, detection, map-fit bias, and
recorded-pose error from one another. Shrinking tiles changes both geometry and
publication, and did not establish a consistent gain.

## Decision and next technical direction

Preserve the current default map and matching algorithm. Do not select an
initializer using the recorded answer, apply frame-specific corrections, or
inflate rejection thresholds to hide the observed bias.

A defensible next experiment should first characterize spatial residuals and
association ambiguity on an independently mapped pass, then test a local curb
geometry/landmark representation whose reference location is stable under
changing visible support. Candidate objectives and correspondence gates should
be judged by common-case accuracy, newly accepted outliers, rejection rate, and
information consistency together. Merely maximizing the current similarity or
changing Gaussian resolution is not supported as a reliable correction by these
results. No lower-bias production algorithm is claimed in this task.

## Implemented experimental support and validation

The previously nested saved-feature adapter is now
`scripts/buildSavedFeatureProbabilityCloud.m`, used by
`matchSavedPerception.m`. The entry point accepts an optional source configuration
for reproducible resolution comparisons. Default output clouds match the seven
previously cached clouds exactly, including component ordering, covariance,
quality, weights, and calibration. The online pillar-only perception path is
unchanged.

Existing registrationInformationTest, repeatabilityRegistrationTest, and
geometricRegistrationTest: 45 passed, zero failed/incomplete. Seven exact source
comparisons passed. Factory Code Analyzer reported zero findings for both
changed scripts. All accepted events generated through matchSavedPerception
passed the existing information checks. The alternative full map passed the
canonical schema validator and remains local under a distinct filename.

The first covariance diagnostic was stopped by the validator because changing
XY covariance without updating retained XYZ covariance violates the exact-
marginal contract. It was corrected by explicitly projecting the diagnostic
cloud to XY before ablation. A later map-summary command had invalid MATLAB
concatenation syntax and was corrected before execution. Neither error changed
the production map or contributed a successful trial to the metrics.

## Reproduction and artifacts

Original technical artifacts are under `output/matching_bias_20260914/`:

- `diagnose_classes.m`, `class_ablation.mat/json`.
- `resolution_study.m`, `resolution.mat/csv` (105 solver trials).
- `covariance_study.m`, `covariance.csv` (63 completed solver trials).
- `map_scale_study.m`, `map_tile4.mat`, `map_scale_log.txt`, and
  `map_tile4_summary.json` (all 1170 observation frames, unchanged statistical
  parameters except tile size and halo; includes iteration-limit counts).
- `evaluate_map_scale.m`, `tile4_original/`, `tile4_additional/`.
- `multistart_study.m`, `multistart.csv`: original initial point and six offsets
  [plus/minus 0.75 m X, plus/minus 0.75 m Y, plus/minus 3 degrees yaw]. Each
  candidate uses existing registration; only full accepted candidates within
  the original prediction correction bounds can replace the incumbent, selected
  by higher similarity. The cached baseline supplies the center candidate.
- `additional_baseline/`, `resolution015_additional/`, `tests.mat`,
  `checks.json`, `diagnostic_notes.txt`, and `manifest.json`.

The map was built using MATLAB batch with `map_scale_study.m`; diagnostics and
matching runs used MATLAB MCP, with their scripts executed via `run` and
`matchSavedPerception`. The CSV/JSON files beside this report are small result
exports. Large MAT files, full maps, and original observations remain local.
The archive retains export copies of reproducible diagnostic scripts and metrics.
