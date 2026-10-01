# Frame 895 curb geometry and covariance semantics

The main newly isolated error mechanism is a mismatch between the meaning of
map covariance and the geometry used by the matcher. The published map sums
within-acquisition feature scatter, a stable-offset prior and reference-mean
uncertainty. The matcher treats the principal axis of that sum as physical
feature orientation. At the influential curb component 685, the sum rotates
the axis from 3.48356 to 4.13912 degrees in frame-895 reference vehicle axes.
That rotation biases yaw and, through the distant pole, vehicle lateral position.

With production source code at `9219f05ee760b0138157f93e2fcbb15a5e43dbfb`,
supplying the map's already stored within-acquisition shape reduces this
frame's position discrepancy from **15.8718 to 6.5104 cm**. The latter control
uses no reference XY/yaw or fine query labels as matcher inputs. It retains
the original prediction, coarse source, map centers and published position
covariances. Known INS tilt remains the existing perception input. This is
an isolated-frame diagnosis, not a new full-route accuracy result or an
installed production change. All discrepancies use the recorded INSPVA
evaluation trajectory, not surveyed physical truth.

## How uncertainty changes the map direction

The original component is `curb:60550:622077:1`, published as curb 685, with
13 observed blocks. Its covariance decomposition is retained in the original
temporal map and reproduced exactly in `map_covariance.csv`:

| Term | Major-axis angle | Major variance | Minor variance |
| --- | ---: | ---: | ---: |
| Within-acquisition shape C | 3.48356 deg | 4.10884 m2 | 0.00250 m2 |
| Stable-offset prior Q | 6.71172 deg | 1.00000 m2 | 0.02250 m2 |
| Reference-mean uncertainty P | 6.03271 deg | 0.09712 m2 | 0.00180 m2 |
| Published predictive covariance C + Q + P | 4.13912 deg | 5.20335 m2 | 0.02941 m2 |

`initializeGeometry` in `mapping/buildTemporalStabilityGmmMap.m` initializes
the offset priors from within-block scatter. During variational fitting C is
updated, while Q remains the fixed prior for that objective. In this component
their final axes differ by 3.23 degrees. `publishTile` correctly exports their
sum as a predictive position covariance. This experiment does not establish
that the probabilistic map fit itself is invalid, nor justify rotating its
prior on every EM iteration.

The mismatch occurs downstream. `mapping/buildViewConditionedLandmarkMap.m`
copies the published curb covariance without preserving its shape component.
`localization/conditionSemanticMapOnView.m` provides intrinsic covariance for
pole/sign views, leaving curb entries zero. Consequently,
`localization/supportRegistrationGeometry.m` uses the total predictive
covariance to construct the curb's angular and sliding geometry. The component
is long enough to bypass neighborhood orientation estimation. Its uncertainty
axis therefore becomes an object-orientation constraint.

This distinction belongs to the statistical description of the same feature:
physical spatial scatter and uncertainty about its location represent different
random quantities. A distribution-to-distribution matcher can retain both
without creating separate feature identities. Strong elongation alone does
not establish that the major axis of an uncertainty-inflated covariance is an
unbiased estimate of the physical feature direction.

## Controlled matching results

The main controls use `matchLocalProbabilityCloud` with the original prediction
and the view-conditioned map frozen at that prediction. Existing production
acceptance tests accept every row below with rank 3 and 15 correspondences.

| Map shape supplied | Historical transport | Position | Lateral | Yaw |
| --- | --- | ---: | ---: | ---: |
| Published predictive shape | Original motion | 15.8718 cm | -14.7262 cm | +0.69338 deg |
| Within-acquisition shape for 685 only | Original motion | 6.9403 cm | -6.0425 cm | +0.26667 deg |
| Within-acquisition shape for all curbs | Original motion | 6.5104 cm | -5.6118 cm | +0.24505 deg |
| Published predictive shape | Reference motion oracle | 9.6852 cm | -8.1123 cm | +0.48767 deg |
| Within-acquisition shape for all curbs | Reference motion oracle | 2.8007 cm | +1.0864 cm | +0.03491 deg |

The shape controls populate the existing `intrinsicCovariance` interface from
the original map's `withinBlockCovariance`. They leave map means and published
position covariance arrays exactly unchanged, but change derived sliding
covariance, angular scale and potentially associations. The reference-motion
rows are explicitly unavailable online and only diagnose historical transport.
These interventions are not an additive error budget.

A stricter factor experiment freezes the production correspondence set, target
position covariance, source statistics, angular scales and class weights.
Only the target normal of component 685 is replaced by the within-acquisition
direction. Unconstrained minimization of the otherwise identical robust
objective reduces error from **15.8716 to 7.3172 cm**, with yaw +0.26771 degrees.
The unchanged frozen objective reproduces the production pose within 4.2e-6
in the mixed pose-coordinate norm. All eight factor optimizations converge.
This isolates the direction mechanism from changed target assignment or
changed statistical weights. Its outputs are diagnostic minima, not independently
accepted production estimates.

Other frozen-factor controls support the same interpretation: removing only
685's angular term gives 10.0257 cm; removing only its position term gives
13.3361 cm; removing both gives 5.0579 cm. Replacing its direction with PCA
of all nearby accumulated map points worsens error to 17.4260 cm. Thus merely
refitting a direction on the accumulated point set is insufficient.

The pole is approximately 12.31 m ahead. At the original +0.69338 degree yaw
discrepancy, its rotation contributes about +14.9 cm laterally and nearly
cancels the vehicle-origin error of -14.7 cm. After the shape control, yaw
is +0.24505 degrees and lateral-origin error is -5.61 cm. This explains why
a well-aligned pole can coexist with a substantially misplaced vehicle origin.

## Perception and historical support still contribute

Fresh raw-frame perception for frames 891 through 895 reproduces every cached
component exactly. The reconstructed five-frame source also matches exactly.
The shared detection pillars are 0.6 m; the Gaussian output aggregation is the
existing 1.2 m grid. Frame 895 selects 19 curb pillars, contributing to ten
current curb Gaussians. Those pillars cover 56 of 99 frozen fine curb points;
two selected pillars contain no such reference point. These are detector-label
agreement measures, not physical annotations or measures of pose observability.

There is a concrete fallback defect. Pillar 5947 has 63 ground returns and no
frozen fine curb point. Its whole-pillar center is [8.02925, -1.75258] m. The
boundary fit proposes a lateral shift of -0.49609 m, with explained fraction
0.79266 and residual standard deviation 0.02659 m. The proposed boundary fails
the same-pillar gate; the original whole-pillar mean remains in the cloud and
is aggregated with a neighboring boundary estimate. `rejected_boundary_gates.csv`
checks the pillar gate explicitly. The diagnostic window intervention also
bypasses any unrecorded face-count rejection, so it is not a deployable policy.

This visible defect is not the dominant explanation. Moving just this current
pillar to its proposed boundary gives 15.4605 cm, an improvement of only
4.11 mm. Dropping it gives 15.5602 cm. Applying the diagnostic rejected-fit
center substitution across the five scans gives 13.8675 cm. Replacing centers
only in selected pillars that contain frozen fine points gives 12.0495 cm;
empty-reference pillars retain their original centers. Combining that control
with reference transport gives 7.7940 cm. All retain original covariance arrays
for the center substitutions, so they are controlled geometry interventions,
not complete fine-perception pipelines.

Reverting to whole-pillar centers happens to give 9.1260 cm on this frame.
That isolated improvement is not evidence for reverting the existing boundary
estimator globally: it changes neighborhood axes, temporal association and
error cancellation. Earlier full-route boundary experiments remain unchanged.

Historical translation contributes independently of the map representation.
The preceding diagnosis measured 6.74 cm disagreement between the oldest
scan's motion-based and reference-based lateral transport over 0.4009 s.
With the map shape issue removed, reference transport further reduces error
from 6.51 to 2.80 cm. This identifies a motion/reference disagreement but
does not determine which trajectory is physically correct, and does not
reintroduce the deleted auxiliary velocity-bias learner.

The map's raw fine observations also change with acquisition: within the
fixed X interval [5,13] m, the PCA line intercept at X=9 m varies from -2.3023
to -2.1206 m for frames 885 through 897 having enough longitudinal support.
Frame 892 has 13.79 cm transverse scatter; frames 893 through 895 have
2.20 to 2.78 cm. Across 455 local map points from 18 acquisitions, pooled
orientation is 4.3569 degrees, while summed within-acquisition scatter gives
3.6993 degrees. These rectangular-strip moments are descriptive, not the
GMM's responsibility-weighted fit. They support investigating view, annotation,
scan-motion and reference consistency; they do not prove a wrong physical map
or a false fine annotation.

## Reproduction and validation

```matlab
setupVehicleLocalization;
addpath('research/frame895_curb_geometry_20261001');
analyzeCurbGeometry895;
auditFallbackAndMap895;
isolateCurbFactors895;
compareMapShapeSemantics895;
validateDiagnosis895;
```

Then run `python research/frame895_curb_geometry_20261001/plotDiagnosis.py`.
[The diagnostic figure](diagnosis.svg) is rendered without opening a desktop
window. Large MAT files and local point exports remain in the ignored
`output/frame895_curb_geometry_20261001/` directory. CSVs in this folder contain
the measurements and all final controls. Input and source hashes identify the
state used by these experiments; per-file static analysis is recorded with
the executable consistency checks in `validation.json`.

Initial development runs stopped on treating a correspondence table as a
struct and on using `range`, unavailable with the installed toolboxes.
Both harness errors were fixed; the final scripts complete without those
dependencies. MATLAB factory Code Analyzer reports zero findings. No full-route
replay, observer gain change, runtime perception change or production map
replacement was performed in this investigation. The next implementation
candidate is to preserve within-acquisition feature shape through map export
and use it consistently for angular/sliding geometry while retaining predictive
position uncertainty, followed by full-route and independent-data validation.
