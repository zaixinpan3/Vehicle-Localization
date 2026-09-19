# Coarse-pillar matching and localization tail-error reduction

Date: 2026-09-19. Baseline source revision:
`549700f89e2dae602d5c75651070ae5915a17939`.

The complete Mississippi replay now contains **zero position discrepancies
above 1 m** in either matching mode and in all seven observer scenarios.
This is an observed result on this recorded drive, not a universal error bound.
No reference-based clipping, trajectory alignment, or frame removal is used.

## Implementation

Online perception still runs once per raw frame and exports statistics of
whole XY pillars. Its parameters, feature decisions, and the frozen map are
unchanged. Localization uses no fine perception, ring structure, vertical
subdivision, or reference position/yaw feedback after recursive initialization.

1. **Stored map mass in correspondence assignment.** Same-class association
   minimizes the existing covariance-normalized geometric distance plus
   `-2 log(pi_j / max(pi))`, where `pi_j` is the stored map `mixtureWeight`.
   Zero-mass targets are excluded. No second stability score is estimated;
   `repeatability` and target quality are not multiplied into residual weights
   again. Source quality remains balanced between semantic classes. Curb and
   facade tangents bound association, while their normals constrain the pose.
   This is hard Gaussian association with a stability prior, not the withdrawn
   all-pair NDT mixture-overlap objective or a claim to implement standard NDT.

2. **Causal three-scan source window.** At most three sets of coarse Gaussian
   statistics, at most 0.25 s old, are transported to the current scan using
   separately integrated wheel/gyro/lateral motion. Matched poses never enter
   this transport. Full XY covariances are rotated. Source class normalization
   prevents exact scan copies from multiplying pose information. No future
   scan is needed; the output timestamp remains the current acquisition time.
   The temporal product is XY only. Relative-motion uncertainty and remaining
   temporal correlation are not calibrated into a pose-error covariance.

3. **Geometry-aware class consistency.** For class normal matrix `H_c` and
   its supported Newton correction `d_c`, the gate uses
   `sqrt(d_c' H_c d_c / lambda_max(H_c))`, with the existing 0.5 threshold.
   Coordinates are the solver's scaled `[dx,dy,10*dpsi]`. The previous norm
   could reject an entire scan because a weak class direction produced a large
   correction. The raw norm remains available as a diagnostic. This engineering
   score is not a statistical confidence or a chi-square test. Tests retain
   rejection of strong conflicting classes and acceptance of shared geometry
   despite a conflict confined to a weak direction.

4. **Unsaturated information-dependent fusion.** The MnCAV observer uses the
   existing `W = I / (I + scale*Identity)` gain law with LiDAR scale 16, matching
   the GNSS scale, instead of 0.001. Nominal position gain remains 4/s. The old
   scale made almost every nonzero LiDAR geometry matrix imply nearly full
   gain. This remains an engineering gain setting, not empirical calibration
   of the exported information matrix. Local information and genuine null
   directions are preserved, and `informationCalibrated` remains false.

5. **Both components of motion bias.** The observer already compared past
   accepted LiDAR displacement with wheel/gyro/lateral integration over 2 s.
   It now retains both the longitudinal and lateral components of the inferred
   constant body-velocity discrepancy. Previously it discarded the longitudinal
   component, leaving wheel-speed bias to accumulate during missing measurements.
   Existing bounds, speed participation and 4 s smoothing remain in use. When
   LiDAR is unavailable, the learned bias is held rather than moved toward a
   stale target. No evaluation-reference velocity is used. The main observer
   remains seven-state; the existing auxiliary bias estimate becomes two-axis.

The unsuccessful weighted-NDT solver, configuration, support class, tests and
obsolete dispatch check were removed from active code. Its historical source
and measured negative results remain identified by Git revision and the prior
research report, without an unused alternative runtime implementation.

## Full matching results

All 1170 raw scans were processed afresh in both modes, against the same map.
Errors are map-frame XY discrepancies from recorded INSPVA at acquisition time.
Rejected scans are included as motion predictions in the all-output population.

| Method / population | Samples | RMSE (m) | P95 (m) | Maximum (m) | Frames above 1 m |
|---|---:|---:|---:|---:|---:|
| Previous geometric, all outputs | 1170 | 0.22263 | 0.47391 | 1.08777 | 1 |
| Withdrawn NDT overlap, all outputs | 1170 | 0.38842 | 0.94104 | 1.98567 | 56 |
| Current recursive, all outputs | 1170 | 0.18253 | 0.32802 | 0.84467 | 0 |
| Current recursive, accepted measurements | 1166 | 0.18235 | 0.32802 | 0.84467 | 0 |
| Current reference-seeded diagnostic, all outputs | 1170 | 0.18480 | 0.34124 | 0.83979 | 0 |

On the same 1069 frames accepted by both geometric versions, RMSE improves
from 0.19723 m to 0.15818 m. The remaining four recursive rejections are
nonconvergence; no error-dependent rejection or post-hoc removal is applied.
Accepted-only statistics alone are insufficient: a stricter intermediate class
gate accepted 1091 frames with 0.16297 m RMSE, but its predicted outputs still
reached 1.04445 m during consecutive rejections. The final gate removes that
failure in this replay while accepting more difficult measurements.

Stronger Cauchy rejection was also tested on the full cached coarse sequence.
Changing the standardized distance from 2.5 to 1 produced 1.616 m maximum
all-output error and was not adopted. Merely raising the original class limit
to 0.75 or 1 also reduced missing outputs, but the implemented consistency
metric directly accounts for the class's constrained directions.

Median coarse perception and matching-window/registration times in the final
recursive run were 58.25 and 9.99 ms respectively; total median was 68.33 ms,
P95 89.30 ms, with 16 frames over 100 ms including startup. MATLAB used eight
computational threads. Other workstation jobs and regression checks ran during
parts of the experiment, so this is not an isolated throughput benchmark or a
hard real-time guarantee.

## Fusion and missing-channel checks

The aligned observer has 1169 frames because of motion-source coverage.
Each scenario uses the same first LiDAR pose and initial motion state. Outages
withdraw the stated channel(s) from receiver time 40 to 60 s. All reported
maxima include every output, including prediction and recovery intervals.

| Scenario | RMSE (m) | P95 (m) | Maximum (m) | Frames above 1 m |
|---|---:|---:|---:|---:|
| GNSS + LiDAR | 0.08103 | 0.15025 | 0.35361 | 0 |
| LiDAR only | 0.17840 | 0.37313 | 0.89459 | 0 |
| GNSS only | 0.08815 | 0.19108 | 0.26083 | 0 |
| GNSS outage | 0.08371 | 0.15776 | 0.35361 | 0 |
| LiDAR outage | 0.08200 | 0.15045 | 0.35361 | 0 |
| Both channels absent for 20 s | 0.19058 | 0.49676 | 0.61189 | 0 |
| Alternating channels | 0.16060 | 0.28654 | 0.51972 | 0 |

Fusion improves mean and P95 error against GNSS-only, but its maximum remains
higher than GNSS-only. With identical final matching inputs, restoring the
near-saturated LiDAR gain worsens fused RMSE to 0.11072 m and maximum to
0.51660 m. On the same inputs and gains, retaining lateral bias alone yields
1.44889 m maximum during the dual-channel outage; retaining both bias components
reduces it to 0.61189 m. See `observer_ablation.csv`.

The map contains observations from this same drive, including query frames.
Recorded roll/pitch supplies known tilt. The evaluation reference and BESTPOS
share a receiver; BESTPOS is INS aided. Reference-seeded matching resets from
reference plus `[0.5 m,-0.4 m,2 degrees]` every frame and is only a diagnostic.
Candidate selection used this drive repeatedly. These are development results,
not an independent-drive accuracy claim or a guarantee for arbitrary outages.

## Validation and reproduction

124 unique MATLAB tests passed, including the unchanged recorded perception
and mapping regression, source-window causality and expiry, covariance transport,
calibration mismatch, information/null-space invariance, strong/weak class
conflicts, missing-channel behavior, and longitudinal-bias drift reduction.
Factory Code Analyzer settings produced zero messages for 18 changed/new MATLAB
files. The first analyzer invocation encountered a stale user settings path;
the factory-settings rerun is the recorded analyzer result.

`analyze_results.py` independently recomputes errors, all tail counts and
percentiles, checks positive accepted information, reconstructs recursive
predictions from preceding outputs and independent motion increments, and
checks that rejected outputs equal predictions. P95 in tables uses MATLAB's
`prctile` / NumPy Hazen convention; individual replay summaries use linear
interpolation and therefore differ slightly.

```matlab
setupVehicleLocalization; maxNumCompThreads(8);
runMississippiMapMatchingExperiment('output/matching_refinement_20260919/validated');
runMncavFullObserverExperiment('output/matching_refinement_20260919/observer_final', ...
    MatchingFolder='output/matching_refinement_20260919/validated/recursive');
addpath('research/matching_refinement_20260919'); exportValidation;
```

```bash
uv run --offline --with numpy --with pandas --with matplotlib python research/matching_refinement_20260919/analyze_results.py
```

Authoritative MAT results and per-frame CSVs remain under the two output folders.
`exportValidation` also references the retained `validated_observer` intermediate
artifact for the lateral-only bias ablation. Full plots are
`output/matching_refinement_20260919/validated_comparison.png` and `.pdf`.
Small summary CSV/JSON files, checks and test records are committed here;
raw scans, generated plots, MAT caches and temporary prototypes are excluded.
