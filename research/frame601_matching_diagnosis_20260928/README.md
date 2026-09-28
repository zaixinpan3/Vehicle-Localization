# Mississippi frame 601: maximum map-matching error

The 85.16 cm peak is a **biased geometric registration accepted under weak
support**, not a failure to find a better initial guess. A short, one-sided
curb segment and one pole dominate the solve. The current pole agrees with
original fine labels but disagrees with the multi-frame map center. Curb
precision filtering removes longer-range support, allowing the conflict to
be accommodated by a 4.532 degree yaw error and a large pose translation.
No production algorithm, default, label or map was changed in this diagnosis.

## Reproduction and error decomposition

The source is the full-route experiment `research/mississippi_matching_20260928`.
Frame 601 is the maximum over all 1,170 recursive outputs. Reconstructed fresh
coarse perception and five-scan source windows reproduce frames 585--610 to
less than 9.4e-10 maximum pose-coordinate discrepancy. Frame 601 differs by
only 2.8e-12. Geometry, matching configuration, map crop, stored tilt and
recorded motion are identical to the experiment.

- Position error: **0.851588 m**.
- Vehicle-frame errors: forward **-0.447459 m**, left **-0.724557 m**.
- Heading error: **+4.532018 degrees**.
- Initial guess already has 0.656909 m / +3.568683 degrees error, but starting
  at the exact recorded reference still converges to the same 0.851588 m result.
- Eight curb distributions, one pole distribution and no signs enter matching.
  These are 1.2 m output Gaussian cells pooled across five scans from 0.6 m
  detection pillars, not nine raw points or nine 0.6 m pillars.

## Evidence for the mechanism

### 1. Curb support collapses and loses spatial extent

Curb support falls from 30 components at frame 587 to eight at 601; signs
vanish by frame 590. Frames 597--599 have no confirmed pole and therefore only
directional updates. The pole returns at 600, but curb support remains short.
At 601 all retained curb means are on the right, approximately Y=-2.97 m,
covering forward X=2.94--11.35 m. They match three almost parallel map segments
(global IDs 544, 540 and 539). Curb-only registration is rank two.

Disabling the semantic precision gate for the diagnostic window yields
32 curb components spanning X=2.94--22.05 m, still on the right. Restoring
**only that curb class**, with the original pole and all matching settings,
reduces error to **0.273782 m / 1.4817 degrees**. This demonstrates sensitivity
to the lost support. It does not prove all restored components are true curbs,
or justify disabling precision filtering in production.

### 2. The pole is geometrically inconsistent with its map component

Positions in the reference frame-601 vehicle axes:

| Pole representation | Forward X (m) | Left Y (m) |
|---|---:|---:|
| Coarse five-scan mean | 12.64748 | -3.99439 |
| Coarse current-scan mean | 12.6423 | -3.9892 |
| Original fine points, current scan (72 points) | 12.65209 | -3.96737 |
| Map component 1046 | 12.47174 | -3.67761 |

The coarse-window/map offset is **36.23 cm**; fine-point/map offset is
**34.13 cm**. Replacing only the pole class by original fine-point geometry
leaves **85.31 cm** pose error. Thus this discrepancy is not explained by
coarse pole selection or whole-pillar contamination alone. The frozen fine
center also reproduces the corresponding stored mapping-observation center
within 1e-8 m, ruling out a different diagnostic point transform here.

Map component `pole:60542:622107:1` uses effective support from frames 597--622.
The associated fine-point cluster centers move over **91.76 cm** in fixed map
coordinates across these frames. At 597/601/605 their distances from the map
center are 51.71/34.13/9.84 cm. A single map Gaussian averages this inconsistent
geometry; its XY principal standard deviations are 17.07 and 20.90 cm.

This is evidence of cross-scan source/map disagreement, **not evidence that a
physical pole moved**. Sensor/reference alignment, timing or motion distortion,
view-dependent sampling, and map aggregation are possible contributors. This
study does not identify their individual physical causes. The calibration
file explicitly contains an empirical same-drive translation and an unestimated
identity rotation; no new extrinsic estimate is claimed.

### 3. The objective and acceptance checks permit the biased solution

The one pole contributes **46.37% of final robust correspondence weight**, almost
as much as all eight curbs combined, after semantic class balancing and temporal
stability. With the available short segment, a yaw/translation combination aligns
the pole and reduces curb residuals. A 4.532 degree rotation at roughly 12.65 m
has about a 1 m lateral effect, which is compensated by vehicle translation.
The pole's center mismatch falls from 36.23 cm at reference to **2.98 cm** at
the fitted pose while the vehicle origin becomes 85.16 cm wrong.

The class-summed robust objective decreases from **0.666650** at reference to
**0.055772** at the fitted pose. The optimizer therefore prefers the wrong
reference-relative pose under its current model; exact initialization cannot
fix this. The smallest/largest scaled information-eigenvalue ratio is
**0.02201**, above the full-rank acceptance threshold 0.01. At the solution,
curb/pole information-weighted class corrections are 0.06816/0.01705, both
below the 0.50 rejection limit. Full rank and small final residuals therefore
do not provide independent protection against this systematic geometry error.

## Controlled experiments

| Offline control | Position error | Qualification |
|---|---:|---|
| Reproduced production | 85.16 cm | Full pose accepted |
| Exact reference initialization | 85.16 cm | Initialization is not the remedy |
| Exact reference window motion | 69.98 cm | Motion transport contributes, but is insufficient |
| Original fine pole geometry only | 85.31 cm | No meaningful improvement |
| Original fine curb geometry only | 67.11 cm | Curb geometry contributes |
| Restore pre-precision curb support only | 27.38 cm | Precision tradeoff unvalidated |
| Replace pole center by its map location at reference | 39.67 cm | Oracle; not implementable as evidence |
| Restore curb support plus pole-center oracle | 3.04 cm | Joint offline attribution only |

These nonlinear controls do not provide additive error percentages. Turning off
canonical merging, lifting the refinement trust limit, and uniform map priors
leave approximately 85--88 cm errors. The fine-refinement shift is only 0.057 mm,
so this peak is not caused by the trust-radius retention branch. Removing the
pole leaves rank-two geometry; its candidate error must not be advertised as a
replacement full-pose localization result. Single-scan results are likewise
qualified by acceptance/rank in `controls.csv`.

The practical priorities are to audit cross-scan landmark alignment and map
representation, preserve useful curb spatial extent under the precision budget,
and avoid accepting overconfident full poses when one point landmark supplies
the otherwise missing direction. Global relaxation of curb filters is not
validated by this one-frame diagnostic.

## Reproduction, artifacts and limits

From the repository root, with the previous full-route outputs and original
frozen fine partitions available:

```sh
matlab -batch "addpath('research/frame601_matching_diagnosis_20260928'); diagnoseFrame601; inspectGeometry601; auditMap601;"
uv run --offline --with numpy --with pandas --with matplotlib python research/frame601_matching_diagnosis_20260928/analyze_diagnosis.py
```

- `controls.csv`, `geometry_controls.csv`, `support_controls.csv`: all controls.
- `correspondences.csv`, `class_costs.csv`, `class_diagnostics.csv`: associations,
  objective and acceptance evidence.
- `map_pole_support.csv`, `map_pole_observed_centers.csv`, `map_pole_summary.json`:
  map provenance and cross-scan disagreement.
- `source_neighborhood.csv`, `source_geometry.csv`, `map_targets.csv`: support
  loss and geometry; `neighborhood.csv` is the original experiment neighborhood.
- `diagnosis.png` / `.pdf`: geometry, support and control plots.
- `validation.json`: independent pose calculations, reconstruction, reference
  center agreement, control assertions and unchanged production source hashes.
- `code_analysis.json`: MATLAB factory analysis; Python byte compilation and
  scoped whitespace checks also run. No new full-sequence algorithm test is claimed.
- MAT snapshots and logs stay in `output/frame601_matching_diagnosis_20260928/`.

This is a same-drive map and detector-reference diagnosis, not surveyed physical
accuracy or an independent test. Reference motion, fine masks and the pole-center
oracle are explicitly offline controls. The original map, masks and production
code remain unchanged. The deeper physical origin of the cross-scan displacement
and a fix satisfying the full-sequence precision requirement remain open.
