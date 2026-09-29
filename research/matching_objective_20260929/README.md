# View-conditioned semantic map matching

The adopted implementation reduces Mississippi's maximum horizontal discrepancy
after configured initialization from **29.86 cm (frame 829) to 16.22 cm (frame
178)**, a **45.67%** reduction. All-frame RMSE changes from **12.13 to 6.10 cm**
and P95 from **20.44 to 10.45 cm**. This replaces the previous study's practical
30 cm plateau with a measured improvement; it does not establish an accuracy
floor for the new result.

The baseline is commit `0ac9acc76da7f5565dcbc01313c2de65250a0d37`. Both runs use
all 1,170 scans, the same independent wheel/gyro trajectory, initial displacement,
known INS tilt, calibration, detector, source window and recursive acceptance
policy. The first frame's configured 0.640312424 m displacement is excluded
**only** from the operational maximum, and included in RMSE/P95. The literal
maximum including initialization therefore remains unchanged. Reference XY/yaw
are used after estimation for scoring, never for recursive pose resets.

## Corrected diagnosis

The previous work identified individual geometric biases but did not repair
the dominant map representation mismatch. A landmark's observed XY center
changes with acquisition position. Pooling those observations into a fixed
global Gaussian, or moment-matching adjacent global modes, creates a displaced
anchor. The optimizer can reduce its residual by moving the vehicle away from
the reference pose. Earlier timing diagnostics establish that scan motion
contributes to the observation spread, but visibility, return sampling and
other acquisition effects are not individually identified by this experiment.

The useful intervention is an empirical conditional observation model, rather
than another change to the online pillar classifier. Offline fine pole/sign
observations are assigned to canonical same-class map landmark groups. Each
mapping acquisition contributes one XY mean and spatial covariance per group.
At runtime, the predicted acquisition origin weights these fixed map samples:

\[
 w_i(q) \propto \exp[-\|o_i-q\|^2/(2h^2)],\quad
 \mu(q)=\sum_i w_i\mu_i,\quad
 S(q)=\sum_i w_i[S_i+(\mu_i-\mu(q))(\mu_i-\mu(q))^T].
\]

Only observations with compatible heading are used. The deployed kernel
bandwidth is 5 m, heading tolerance 30 degrees, and XY variance floor
0.0025 m². Offline grouping/assignment radii are 1.5 m, with at least three
fine returns per acquisition/group. Return count does not multiply an
acquisition's mixture weight. The map retains observation statistics; it does
not read query fine labels or a query reference pose at runtime.

Simply normalizing the kernel would give a distant, unsupported view full
influence. The nearest mapping-origin distance therefore supplies a separate
continuous support factor, `exp(-d_min²/(2*5²))`, applied after semantic class
balancing. This is an engineering coverage weight, not a calibrated probability.
It avoids both unsupported sharp anchors and abrupt hard visibility cutoffs.
The conditional map is frozen during each registration solve. Exported
information remains uncalibrated geometry conditional on that map view.

The controlled comparisons separate these mechanisms:

| Complete-route control | Maximum after initialization |
|---|---:|
| Previous production | 29.86 cm |
| Canonical map without conditional refitting | 37.25 cm |
| Globally refit equal-acquisition point distributions | 36.14 cm |
| Conditional scatter only | 42.70 cm |
| Conditional means only | 20.92 cm |
| Conditional means and scatter, static fallback | 20.92 cm |
| Hard view-support cutoff, 5 m kernel | 18.41 cm |
| Continuous support, 5 m kernel, even-frame point observations | 15.88 cm |
| Adopted implementation, all mapping observations | 16.22 cm |

Changing the hard-cutoff kernel from 5 to 4 m creates an 82.12 cm outlier at
frame 830, so that policy is rejected. Continuous support with 4/5/6 m kernels
instead gives 15.31/15.88/16.95 cm maxima on the even-observation control. The
middle setting is retained. A local linear conditional-mean model did not
improve the hard-support result. Broad local curb maps, sign surface matching,
ambiguity-direction projection and prediction-ranked geometric hypotheses
also failed to improve the baseline and are not deployed.

## Frame evidence and remaining peak

| Frame | Previous production | Adopted production |
|---|---:|---:|
| 829 | 29.86 cm | 4.80 cm |
| 806 | 29.62 cm | 5.54 cm |
| 1096 | 29.79 cm | 7.95 cm |
| 1149 | 19.47 cm | 9.90 cm, directional update |
| 178 | 16.94 cm | 16.22 cm, new maximum |

At frame 1149, the retained sign comes from earlier source-window acquisitions.
The nearest fine-map sign observation origin is 16.26 m away. The initial
conditional-map prototype falls back to its global sign mean and reaches
20.92 cm; continuously reducing unsupported sign influence yields a directional
update that preserves the prediction along the weak road direction. It does
not claim a new full-pose measurement.

Frame 178 still has no confirmed pole in its pooled matching source. A
diagnostic single scan gives 4.57 cm, a reference seed gives 10.19 cm, and
removing line direction gives 45.92 cm. Thus its remaining error involves
source pooling and correspondence basins; it is not explained solely by a
missed pole or an optimizer iteration limit. Giving a high-score singleton
pole weak influence improves the full-route peak by only 2.38 mm and weakens
the temporal admission contract, so it is rejected. Current point centers,
current curb centers, and current point geometry peak at 18.77/20.25/18.69 cm.
The original detector and two-acquisition confirmation remain deployed.

## Corrected motion-compensation control

Earlier deskew tests changed both geometry and pillar membership. This study
fixes the original selected pillars and every original member point, recomputes
only their compensated moments, and uses coherently deskewed maps. The complete
0/50/100 ms phase replays peak at 30.43/28.23/38.56 cm. The middle phase improves
the previous baseline but is weaker than the adopted conditional map. It is
not enabled in production, and no absolute sensor latency is claimed.

The first frozen-membership harness attempt exposed the ground raster's
internal/public index transpose; the corrected harness asserts counts for every
pillar. Empty boundary-fit output is handled explicitly. These failed setup
attempts are not counted as completed experiments.

## Validation and limits

`replayRawFinal` reprocesses every original raw scan. All 1,170 current clouds
and all 1,170 temporal source clouds are exactly equal to the previous production
cache. Every pose agrees with the independent cached-source production replay
within 1e-7. Perception code, grid, masks and the source-window algorithm are
unchanged; this study makes no detection-recall improvement claim. Downtown and
legacy maps retain their previous behavior. The legacy-map full-route parity
replay reproduces the original errors after the solver integration changes.

All **303 tests in 21 suites pass**, with no incomplete tests. Fifteen changed
MATLAB implementation/configuration/test files have zero factory Code Analyzer
findings. One obsolete suppression was removed after the first analyzer pass;
the follow-up analyzer check is recorded separately. The first raw harness
compared an added wrapper summary field against a direct `perceiveFrame` cache;
the final harness uses the same public perception entry as its baseline and
checks the complete cloud, without relaxing geometry comparisons.

Alternating-order paired timing includes cropping, conditioning and solving:
median **14.24 to 7.95 ms**, with a paired median difference of **-6.10 ms**.
Perception is unchanged. An unrelated MATLAB workload was present, so these
are local timing observations rather than real-time certification. Earlier
prototype timings exclude map conditioning and are not used for this claim.

The map and queries reuse the same recording and INSPVA reference. Two
additional controls restrict **only the new pole/sign observation model**:

| Mapping observation selection | Scored excluded frames | Maximum | RMSE |
|---|---:|---:|---:|
| Even frames only | 584 odd frames after initialization | 15.36 cm | 5.91 cm |
| Every fourth frame | 877 other frames | 20.19 cm | 6.29 cm |

The original curb map and the grouping scaffold still reuse all mapping
observations, and neighboring scans are correlated. These checks support the
conditional point-model mechanism; they are not a disjoint whole-map test or
an independent-drive accuracy result. Fine labels and INSPVA are comparison
references, not exhaustive physical annotations or surveyed truth. The model
does not resolve absolute timing or extrinsic calibration.

## Production paths and reproduction

`landmarkViewMapConfig`, `buildViewConditionedLandmarkMap` and
`conditionSemanticMapOnView` implement the model. Cropping/projection preserve
its component indices. `registerSemanticProbabilityCloud` conditions once and
skips a second map canonicalization. Registration and the existing pose graph
apply the same target-support factor. Map-building entry points publish a
separate conditional XY map; the original map and fine observations remain
intact. `featureMapBuildConfig().probabilityCloudPath` now selects that product.
This map deliberately supplies no view-conditioned height model.

```matlab
addpath(pwd); setupVehicleLocalization;
buildMississippiViewConditionedMap;
addpath('research/matching_objective_20260929');
checkImplementation;
replayRawFinal;
benchmarkMatching;
```

`replayRawFinal` verifies the committed `productionAll.csv` cached result as
well as local baseline source MAT files. The runtime comparison likewise needs
the preceding baseline replay. To rebuild those controls, call `replayStudy`
with an explicit map path; its default is intentionally frozen to the original
map. `runViewConditionedMap`, `runViewMapControls`, `runViewSourceControls`, and
the diagnostics preserve the executed research interventions.

```bash
uv run --offline --with numpy --with pandas --with matplotlib python \
  research/matching_objective_20260929/analyzeStudy.py
```

`rejected_geometry_prototypes.patch` preserves the rejected projection and
surface branches against the starting commit. Apply it only in an isolated
baseline checkout. The replay harness rejects their options when the prototype
is absent. Research MAT files, raw recordings and timing sidecars remain under
`output/` and `data/`; they are not published. CSVs, diagnostic controls, summary
JSON, test results, source scripts and static plot exports are the committed
technical artifacts. No desktop visualization was opened during this task.
