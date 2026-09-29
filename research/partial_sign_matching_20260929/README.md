# Partial sign observations in semantic map matching

The adopted change reduces Mississippi frame 178 from **16.224 to 2.776 cm**
without rejecting the confirmed sign. Its displaced partial patch retains a
panel-normal constraint; coherent observations of the same target retain XY
center constraints. The whole-route maximum moves to frame 932, **15.986 cm**.
This is a substantial repair of frame 178, but only a 1.47% reduction of the
route maximum. It does not resolve the new largest curb/pose discrepancy.

The starting implementation is commit
`64d025105e523d2e305570aac5399f4732ac3f9b`, whose matching output is identical
to the preceding implementation commit
`4985af121073f32d659df49a67c751167fbd0c05`. Baseline CSVs are in
`research/matching_objective_20260929/`. All experiments use the same 1,170
Mississippi scans, offline map, independent wheel/gyro motion, initialization,
INS tilt, 0.6 m detector and temporal source confirmation.

## Geometry and decision rule

The previous diagnostic established that source sign 22 contains genuine
fine-labeled sign returns, but its partial center is 42.30 cm from the complete
conditioned map center. Even its fine-only subset center is 31.61 cm away.
Repetition confirms existence without making these different geometric centers
interchangeable. The implemented correction changes the residual geometry.

The view-conditioned map previously retained the total scatter

\[
 S=\sum_i w_i S_i+\sum_i w_i(\mu_i-\mu)(\mu_i-\mu)^T.
\]

It now also exports `intrinsicCovariance = sum(w_i*S_i)` in an aligned component
array. The original total covariance remains the spatial compatibility metric.
The intrinsic array separates within-acquisition shape from between-acquisition
center drift. At the diagnosed sign, intrinsic XY eigenvalues are approximately
0.0033 and 0.0341 m², revealing the panel direction hidden by the total scatter.
This is an empirical shape estimate, not a calibrated physical plane model.

`prepareSemanticRegistrationGeometry` enables partial-panel handling for sign
targets with intrinsic major variance at least 0.01 m² and eigenvalue ratio at
least five. Within each currently associated target, at least three source
components are required. For each component, a neighborhood includes other
centers within 0.15 m along the panel tangent. Its support is the sum of source
quality times temporal stability. A neighborhood must contain at least two
components and carry a strict majority of the group's support. Its members
retain the original center residuals.

An observation outside that neighborhood is treated as a partial patch only
when its normal-coordinate center is within 0.20 m of the support-weighted
neighborhood normal center. That observation keeps a point-to-panel-normal
residual, using the same summed source/target scatter, class balancing,
temporal stability, view support and robust loss. Its tangent center difference
supplies no force or pose information. No observation is removed by this rule.
Groups with too few observations, no majority, incompatible normal position,
or no supported intrinsic shape preserve their original point constraints.

The classification is recomputed with current correspondences; each solver
line-search freezes the selected normal/point residuals and their precision,
consistent with existing frozen-association optimization. The exported
`partialSignSurface` correspondence flag records the decision. The same shared
model is used by registration and pose-graph factors. Information remains
uncalibrated geometry, and unsupported directions remain unsupported.

The production switch and scales are in `distributionRegistrationConfig`.
Cropping and projection preserve the intrinsic array. Legacy maps without view
observations retain the original behavior. Runtime uses stored offline map
statistics and selected coarse source distributions; it does not read fine
query labels, raw point membership, reference XY/yaw or frame-specific rules.
The existing map file can be used without rebuilding it because intrinsic
scatter is derived from its already stored observation rows.

## Full-route controls

Eighteen experimental route replays test mechanism and neighboring settings:

| Intervention | Maximum after initialization | Outcome |
|---|---:|---|
| Original matching | 16.224 cm | Baseline |
| All eligible signs use only surface normals, ratio 2 or 3 | 26.904 cm | Rejected |
| All eligible signs use only surface normals, ratio 5 or 9 | 15.986 cm | Frame 178 still 15.318 cm; rejected |
| Coherent-center neighborhoods of 10/15/20/25 cm | 15.986 cm | Frame 178 2.776 cm in all cases |
| Map-direction scatter scales 0.15/0.25/0.5/1 | 15.483/17.218/23.412/31.869 cm | Broad orientation reweighting not adopted |
| Conditional curb directions, 3/5/8 m kernels | 24.407/18.858/20.619 cm | Rejected |
| Conditional curb centers and directions, 3/5/8 m kernels | 22.580/19.292/17.584 cm | Lower RMSE, higher maximum; rejected |

Unconditional surface matching discards useful center information from the
coherent observations. Simple point merging likewise preserves a displaced
aggregate center, as shown by the preceding diagnostic. The adopted rule
changes only the displaced portions in a supported group. A middle 15 cm
neighborhood is retained rather than treating a single best setting as a
physical constant. The other values are development sensitivity checks.

The map-direction 0.15 candidate lowers the maximum to 15.48 cm, but increases
RMSE/P95 and changes curb orientation influence broadly; it does not repair
the sign measurement definition. It remains a recorded alternative rather
than part of this deployment. Conditional curb centers reduce route RMSE to
5.55--5.70 cm, but their 17.58--22.58 cm maxima exceed the baseline. These
findings do not establish that the new maximum cannot be improved further.

## Adopted result and remaining limitation

| Metric | Baseline | Adopted |
|---|---:|---:|
| Frame 178 horizontal discrepancy | 16.224 cm | 2.776 cm |
| Maximum after initialization | 16.224 cm, frame 178 | 15.986 cm, frame 932 |
| All-frame RMSE | 6.10057 cm | 6.10627 cm |
| P95 | 10.45378 cm | 10.45378 cm |
| Full / directional updates | 1147 / 11 | 1147 / 11 |

RMSE rises by approximately 0.057 mm; P95 is unchanged. The first configured
0.640312424 m displacement is excluded only from the maximum, and included in
RMSE/P95. The literal maximum including initialization remains unchanged.
Reference poses are used after each estimate for scoring; recursive estimates
are never reset to the reference.

Frame 932 contains a historical confirmed pole and 15 matched curb components,
with no matched sign. Its yaw discrepancy is approximately 0.673 degrees.
A reference seed retains about 15.99 cm discrepancy; removing the pole gives
a roughly 20.93 cm directional result and removing line direction gives
19.90 cm. Its remaining map/source curb geometry and acquisition transport
require separate work. The sign correction does not remove this pole or relax
two-acquisition confirmation. The frame-178 old/new pole handover rule is also
unchanged because earlier broad singleton-admission controls did not reliably
improve whole-route error.

## Validation and reproduction

The final raw replay executes current perception and production matching on
all 1,170 original scans. Every current cloud and temporal source cloud is
asserted exactly equal to the original cache. Pose output is checked against
the independent cached-source adopted replay. This verifies unchanged all-class
detection and confirmation, rather than claiming new precision/recall metrics.

The selected regression suites exercise mapping, perception, registration,
observability, height/association, temporal confirmation and graph behavior.
New class-based tests check majority selection, retained normal force,
incompatible normal position, absent consensus, reduced center bias, gradient
finite differences, intrinsic/total-scatter separation, metadata projection
and invalid scatter. The complete selected regression run passes 311 tests.
The final targeted run passes nine tests, including a subsequently added empty
map-crop case: 312 distinct passing tests in total. Factory Code Analyzer
reports zero findings across the five production files and the new test file.
All 1,170 current and temporal source clouds match the original cache exactly;
the maximum raw-versus-cached pose difference is 4.66e-9. An additional 1,170
frame legacy-map replay reproduces its previous poses exactly.

The combined validation process ended with exit code 143 after raw frame 1,100,
without exporting a complete raw replay. The separate raw retry completed all
1,170 frames and passed the checks above; only that complete replay is counted.
An initial test harness lacked the repository root on its MATLAB path, and an
initial analyzer harness used a character array instead of a cell of file
paths. Both were corrected and rerun; their failed commands are not passes.

The paired 1,170-frame runtime check alternates previous and adopted matching
on the same conditioned map. It verifies both output routes while measuring
crop, conditioning and solve together. Median time is 8.8845 versus 9.0350 ms;
the median paired increase is 0.153 ms, and P95 is 12.659 versus 12.934 ms.
The raw replay ran concurrently, so these are local comparative timings rather
than a real-time guarantee. The CSV's `legacy` timing label denotes the previous
matcher on this same map, not the separate legacy-map parity control.

For a sensitivity control, the point-landmark view model is built only from
even scans; 584 odd query scans after initialization are excluded from those
point observations. Their maximum remains 15.361 cm at frame 1,147. RMSE changes
from 5.9120 to 5.9389 cm, and P95 from 10.697 to 10.720 cm. This control does not
show a general error reduction. It still reuses the full recording's curb map
and scaffold, so it is not an independent-drive validation. Percentiles in the
Python export use MATLAB-compatible midpoint plotting positions.

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/partial_sign_matching_20260929');
replayPartialSign('output/root_cause_matching_20260929/finalSurface_sources.mat', ...
    'production',distributionRegistrationConfig(), ...
    'output/mississippi_mapping_calibrated/view_conditioned_cloud.mat');
checkPartialImplementation;
replayRawPartial;
benchmarkPartialMatching;
```

The unconditional-surface and map-direction screens require their respective
`rejected_*.patch` in an isolated checkout; their harnesses reject missing
prototype code. These patches are mutually independent alternatives and are
not deployed. `screenCurbViews` implements its conditioning entirely in the
research harness with a separate local map product; it does not change the
production curb map. `screenConsensusSign` runs on the adopted code.

Large recordings, MAT source caches and research maps stay local under
`data/` and `output/`. Compact CSV/JSON, code, plots and technical input/artifact
hashes preserve the actual experiments. The map and queries share the recording
and existing INSPVA reference. Fine-map labels are offline detector output,
not exhaustive physical annotations. These are development results; no new
independent-drive, surveyed-truth, sensor-latency or accuracy-floor claim is made.
