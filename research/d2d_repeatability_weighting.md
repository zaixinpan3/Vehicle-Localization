# Map repeatability in geometric D2D registration

Date: 2026-09-07. Baseline: `c13bfb84d5aec8b7f00a4544a624b45964142af0`.

The default geometric D2D now discounts each correspondence using the matched
map component's temporal stability posterior. The production configuration
continues to use XY distributions and optimize only `[X,Y,psi]`.

## Meaning of the map statistic

`buildTemporalStabilityGmmMap` compares a stable-geometry model with a
variable-geometry model across observed blocks. Its component `repeatability`
is the posterior probability of the stable model. Publication also requires
enough observed blocks, sufficient effective support and the configured
repeatability threshold. Blocks with no assigned observations supply no
geometric evidence; there is no visibility-conditioned detection/miss model.
Consequently this statistic measures repeated geometric stability, not a
calibrated probability of detection on an arbitrary future frame. See the
[map design](repeated_observation_map_design.md).

The map already exports `repeatability`, but the former projection discarded
it. The default matcher consequently gave every published schema-2 target
unit semantic quality. Only the zero/positive mixture-mass gate affected its
availability; continuous stability did not affect geometric weights.

## Weight contract

Let `a_ij` be the product of the existing source and target semantic quality,
`P_c` the current same-class correspondences, `K` the number of shared semantic
classes, and `r_j` the matched map component's repeatability. The new weight is

\[
w_{ij}=\frac{a_{ij}}{K\,\max(\sum_{(u,v)\in P_c}a_{uv},\epsilon)}\,r_j.
\]

The stability discount follows class balancing. Dividing by a sum that
already contains `r_j` would cancel a uniformly low-stability class, contrary
to the intended reliability discount. For otherwise equal correspondences,
repeatability 0.9 versus 0.3 gives three times the geometric weight. If all
components in a class have repeatability 0.25, its nominal total weight is
one quarter of the unit-repeatability class budget.

The same weights enter the robust objective, frozen-correspondence line search,
normal equations, class diagnostics and coverage-adjusted similarity. With
the existing standardized residual `q` and robust scale `s=2.5`, the normal
equation weight is `w/(1+q/s^2)`. This is a posterior reliability discount in
the existing geometric estimator, not a new exact generative mixture
likelihood or calibrated sensor information matrix. Geometric covariance
still controls precision, and residual robustness still limits outliers.

Zero-repeatability targets cannot be associated. Positive repeatability does
not change the geometric association distance: a distant stable feature does
not win solely through its posterior. Existing class-conflict rejection is
retained, so a downweighted but geometrically inconsistent class can still
reject a full-pose measurement. No exponent, threshold or acceptance parameter
was tuned for this change.

Map mass remains `referenceMass * repeatability` for the density/query product.
Geometric D2D does not multiply by that mass: reference-cell support area is
not temporal confidence, and multiplying both mass and repeatability would
count the latter twice. The new discount is map-side only; online single-frame
perception does not estimate temporal repeatability.

## Interface and compatibility

- `mappingSupport.validateSemanticProbabilityCloud` validates an optional
  numeric, real, finite per-component vector in `[0,1]` and stores a column.
  Malformed fields raise `VehicleLocalization:InvalidRepeatability`.
- `registrationSupport.projectSemanticProbabilityCloud` preserves the field in
  XY and XYZ.
- Missing repeatability retains unit weights, with result provenance
  `repeatabilitySource="legacyUnitWeight"`; supplied fields report
  `"mapPosterior"`. Existing legacy support-amplitude quality is retained.
- Returned correspondences expose `mapRepeatability`, `weight` before robust
  discounting and `robustWeight`. `weightSemantics` and `similaritySemantics`
  describe the revised result. Geometry and all-one repeatability reproduce
  the baseline objective, while positive mixture-mass rescaling remains inert.
- Explicit `densityOverlap` retains its historical mass-based objective;
  `scoreSemanticProbabilityCloudAlignment` remains that legacy diagnostic.
  The changed production method is `geometricD2D`.

## Validation method

Behavior tests cover conflicting same-class landmarks, absolute discounts
within and between classes, all-one and absent-field equivalence, positive
mass invariance, zero-weight exclusion, malformed probabilities, XY/XYZ
projection and map-side-only weighting. The conflicting fixture contains two
symmetric groups of four pole distributions whose preferred translations
differ by 0.6 m. The equal-weight compromise is 0.3 m; a ten-to-one stability
ratio must put the solution within 0.08 m of the preferred group, and swapping
the ratio must reverse the preference.

Recorded validation uses the existing cached raw frames and maps in
`output/localization_pipeline_timing30_20260906/inputs.mat`: Mississippi query
frames `[120,260,350,550,700,900,1050]`, each mapped using the following 30 frames
with the query excluded, three initial poses per frame, and the default four
channels excluding facade. These are 21 distinct cases; repetitions measure
execution variability, not independent scenes. Recorded-pose distances measure
consistency with the mapping trajectory, not independent ground-truth accuracy.

The two baseline files `localizeLidarFrame.m` and
`registerSemanticProbabilityCloud.m` are extracted with `git show` from the
baseline SHA and both function identifiers receive the suffix
`RepeatabilityBaseline`. They live in an untracked temporary directory added
to MATLAB's path. Unchanged helpers are shared; the updated projection and
validator also run on the baseline, but its solver ignores repeatability.
This isolates the added weighting and diagnostics; it does not independently
time the old validation path. Unit-repeatability runs additionally check exact
pose, curvature and score equality against that baseline on all 21 cases.

`compareLocalizationPipelineTiming` executes two warmup passes followed by
20 interleaved repetitions, seed 20260906, randomizing case and method order.
Every timed call recomputes coarse perception from the preloaded raw frame and
includes D2D and event output. Disk loading and offline map construction are
outside the scope. Small exports are in
`research/results/d2d_repeatability_20260907/`; raw maps, large MAT results,
native binaries and temporary baseline sources are not committed.

## Measured results

All **100 focused tests passed**, including 17 new stability cases and the
recorded Mississippi/Downtown perception and mapping regression. There were
no failures or incomplete cases in the final run. Factory Code Analyzer found
zero issues in the four changed MATLAB files. Initial test setup attempts
could not resolve the project-root setup script after the runner changed
directory; the new test class now installs a root path fixture. The inherited
missing observer-directory path warning is unrelated to this change and was
not repaired. An inherited unavailable R2025b analyzer-settings file was
bypassed by explicitly selecting factory settings for the final check.

The 21-case unit-repeatability control exactly reproduces baseline pose,
normal matrix and similarity. Both methods accept all 420 measured calls;
coarse probability clouds remain exactly equal. New weights change some final
correspondences and move the pose by at most **0.0281085 m** and **0.120203 deg**.
Matched repeatability reaches as low as 0.5520, but its per-case mean is at
least 0.9461: most published targets are already geometrically stable.

Recorded-trajectory position discrepancy has median 0.23946 m before versus
0.23960 m after, and maximum 0.37204 m versus 0.35797 m. Median heading
discrepancy is 0.17431 deg versus 0.14221 deg; maximum is 0.73956 deg versus
0.69086 deg. Some cases improve and others slightly worsen. These observations
support compatibility with the current dataset, not a claim of independent
localization accuracy improvement.

MATLAB R2026a Update 3, eight computational threads, the existing native CPU
perception kernels, and a shared desktop were used. The CSV method label
`optimized` denotes the new repeatability-weighted implementation because the
existing comparison harness uses that label; this task does not claim a speedup.

| Warmed stage | Baseline median | Weighted median | Weighted P99 | Weighted maximum |
|---|---:|---:|---:|---:|
| Coarse perception | 59.829 ms | 60.056 ms | 73.894 ms | 75.683 ms |
| D2D | 6.222 ms | 6.347 ms | 8.266 ms | 10.737 ms |
| Complete call | 66.227 ms | 66.359 ms | 80.176 ms | 82.695 ms |

Both methods have zero calls above 100 ms among their 420 measured calls.
Quantiles interpolate sorted observations at `1+(n-1)*p`. Their median
complete-call difference is 0.132 ms, about 0.20%; stage medians need not sum
to the complete-call median. The preliminary one-repetition pilot had a
103.157 ms baseline maximum and 80.452 ms weighted maximum.

**Warmup is excluded by the pre-existing benchmark protocol, not by a timing
threshold.** The retained 42-call warmup per method includes severe stalls:
baseline maximum 16,199.044 ms and weighted maximum 40,508.308 ms, both during
the second pass. All warmup calls are accepted. No causal attribution for
those stalls is established by this experiment. The 82.695 ms warmed maximum
is therefore not an all-call latency bound, a cold-start guarantee or a
validated observer delay. No observer delay is changed here.

The main reproducible MATLAB commands, after extracting and adding the
renamed baseline sources to the path, are:

```matlab
addpath(pwd); setupVehicleLocalization;
report = compareLocalizationPipelineTiming( ...
    'output/localization_pipeline_timing30_20260906', ...
    'output/d2d_repeatability_20260907', ...
    @localizeLidarFrameRepeatabilityBaseline,20);
setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(pwd,'data'));
results = runtests({'tests/repeatabilityRegistrationTest.m', ...
    'tests/geometricRegistrationTest.m','tests/distributionRegistrationTest.m', ...
    'tests/heightProbabilityCloudTest.m','tests/temporalStabilityGmmMapTest.m', ...
    'tests/semanticNdtGridMapTest.m','tests/coarseSemanticProbabilityCloudTest.m', ...
    'tests/pipelineRegressionTest.m'});
assertSuccess(results);
```

The versioned CSVs contain individual measured and warmup calls, the 21 paired
pose/stability diagnostics, all final test results and analyzer counts.
`summary.json` records the timing scope, baseline, quantiles, pose comparisons
and platform. Only this coherent implementation and validation are archived;
ordinary preceding explanations are not separate research activities.
