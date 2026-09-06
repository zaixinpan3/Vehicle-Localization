# Facade and traffic-sign restoration in pillar perception

Prepared September 5, 2026. The comparison baseline is commit
`89141407604af210fbdbe930c05b978c5eafb23a`, exported before editing. See the full recorded SHA
in `results/structural_perception_20260905/validation.json`.

## Implemented behavior

The modern path previously published only curb, road marking, and pole, even
when `facadeDetectionEnabled` was true. The legacy path retained both missing
classes. The default cloud and candidate schema now contains all five classes.
`perceptionConfig("Downtown")` enables facades; `perceptionConfig()` and the
Mississippi profile leave the facade layer empty. Omitting a class from
`coarseProbabilityCloud.semanticNames` disables its publication and refinement.

Facade proposals reuse the existing oriented weighted Hough transform of the
pillar run-height and line-shape rasters. The old two-cell refinement halo is
now an **online pillar proposal expansion**, using occupied columns and XY
line ownership. This matters: delaying the expansion until offline refinement
would consider points outside the published candidates. The online branch
never calls the dense facade patch validator. Facade candidate pillars are
excluded from pole proposals, restoring the original class interaction.

Offline facade validation recovers only nonground candidate members, grouped
by their assigned line. A vertical plane seeded by that line is robustly
refitted using three SVD/inlier iterations. Every point receives a distance
acceptance decision, capped at 0.20 m; minimum support is 12 points, 1 m of
height and 1 m of horizontal extent, with a 20-degree verticality/orientation
limit. A median absolute deviation scale supplies the tighter residual cutoff.
A failed group stays empty. These metric engineering limits are explicit in
`finePerceptionConfig`, and have not been statistically calibrated.

Traffic-sign candidates use pillar maximum intensity above the retained raw
sensor threshold of 1600. Offline validation tests each candidate point against
that threshold. Coarse Gaussian moments include **all nonground members** of
selected sign pillars, retaining height and xz/yz coupling even when a bright
sign shares its footprint with nonreflective support. Reducing members by their
pillar membership is not point-level semantic verification. Other structural
moments preserve the previous radiometric preprocessing.

Both classes use the existing regularized XYZ Gaussian output with an exact XY
marginal. Mixture weights renormalize when classes are added. The offline map
configuration includes signs; `buildFeatureMap` automatically includes the
facade class when its facade switch is enabled. The schema-2 repeated-observation
map, cloud export, and queries accept both labels without map-model changes.
Facade alignment uses the surface-normal residual, preventing artificial
along-wall observability. Signs use the XY landmark residual. The estimated
pose remains exactly `[X,Y,psi]`; retaining Z introduces no additional state.

## Recorded comparisons

The deterministic comparison uses frames 100, 200, 300, 400, and 500 from each
of `MissisipiPointClouds.mat` and `downTownPointClouds.mat`. Inputs and native
MEX binaries remain local. Detailed counts and all warm timings are committed
under `results/structural_perception_20260905/`.

- Traffic-sign fine masks match the legacy baseline **exactly on all 10 frames**.
- Mississippi curb, marking, and pole fine masks match the previous modern
  pipeline exactly on all five evaluation frames. Existing broader perception
  regressions also pass.
- Downtown facade candidates contain **all 10,387 legacy facade points** across
  the five frames. The candidate stage recovers 28,267 nonground points; fine
  validation retains 22,633, including 8,769 legacy points (84.4%).
- The new fine facade masks are not copies of the old patch-to-column masks.
  They add plane-consistent returns and reject individual outliers. The legacy
  masks are a software comparison, **not annotated ground truth**; agreement
  cannot establish precision or recall against actual buildings. The frame-200
  comparison shows cyan support along the same nearby planar structures, with
  broader coverage in the new result. Independent labels are still needed to
  assess false detections and the usefulness of additional support.
- Restoring the facade exclusion changes 126 and 63 pole point labels on
  Downtown frames 100 and 200 respectively. The other sampled Downtown pole
  masks and all sampled curb/marking masks are unchanged from the previous
  modern pipeline. This is an intentional class interaction, not an exact
  five-class preservation claim.

Each version receives two warmup calls and 11 timed coarse calls per frame,
excluding loading, plotting and offline point validation. Both use the same
native kernel. The median over 55 calls per dataset is:

| Dataset | Previous three-class path | Restored path | Difference |
| --- | ---: | ---: | ---: |
| Mississippi | 60.764 ms | 64.262 ms | +3.498 ms (+5.8%) |
| Downtown | 53.766 ms | 58.105 ms | +4.339 ms (+8.1%) |

Individual-frame median differences range from -5.88 to +4.48 ms in Mississippi
and +3.41 to +8.66 ms Downtown. These sequential host timings include ordinary
runtime variability, and are not a deployment deadline or a speedup claim.

## Validation and reproduction

There are 131 passing tests across ten affected test classes: perception,
coarse native/fallback equivalence, XY/XYZ probability clouds, both registration
methods, both map representations, and stored pipeline regression. The new
structural class contributes 21 cases. These include point rejection inside
accepted pillars, missing/disabled features, candidate-only sign validation,
Downtown native/MATLAB equivalence, rank-two parallel-wall geometry, three-state
sign registration, and both new labels passing through the mapping entry point.
The latter uses three copies of one Downtown frame and identity poses strictly
as an interface fixture, not evidence of real temporal repeatability or route
localization accuracy. Eighteen changed MATLAB files pass factory Code Analyzer
checks. A profiled Downtown online call enters none of the point-refinement or
dense off-ground construction functions.

The existing working tree contains a separate observer-folder move;
`setupVehicleLocalization` warns about its missing old directory. Those files
and agent instructions are excluded from this task. A concurrent committed
Gaussian-registration cleanup was preserved.

```matlab
setupVehicleLocalization;
setenv('VEHICLE_LOCALIZATION_DATA_ROOT', fullfile(pwd,'data'));
runtests({'tests/structuralSemanticPerceptionTest.m', ...
    'tests/pillarPerceptionTest.m','tests/coarseSemanticProbabilityCloudTest.m', ...
    'tests/coarsePerceptionPerformanceTest.m','tests/geometricRegistrationTest.m', ...
    'tests/distributionRegistrationTest.m','tests/heightProbabilityCloudTest.m', ...
    'tests/temporalStabilityGmmMapTest.m','tests/semanticNdtGridMapTest.m', ...
    'tests/pipelineRegressionTest.m'});
% baselineRoot: immutable export of the recorded baseline, sharing the MEX.
evaluateStructuralPerception(fullfile(pwd,'data'), ...
    fullfile(pwd,'output','structural_perception_reproduction'), baselineRoot);
cfg = perceptionConfig("Downtown");
p = perceiveFrame(loadPointCloudFrame('data/raw/downTownPointClouds.mat',200),cfg);
cfg.executionMode = "offline";
fine = perceiveFrame(loadPointCloudFrame('data/raw/downTownPointClouds.mat',200),cfg);
```

Traffic-sign intensity remains specific to the sensor's units and can include
other reflective objects; this is not sign-type recognition. Facade geometry
alone can also accept other upright planar structures. No Downtown route-level
pose-error experiment or calibrated cross-sensor semantic accuracy is claimed.
