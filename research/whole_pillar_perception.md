# Whole-pillar coarse perception

Date: 2026-09-08. Reference revision:
`9dcd1aeab1a029ce374a27be6d5db37fb8acf066`.

## Boundary and statistical requirements

The online spatial unit is the complete XY pillar. `pillarizePointCloud`
now directly filters returns and assigns two-dimensional membership; it never
calls the voxelizer or quantizes Z. The default online geometry has two
spacing values. A supplied historical third spacing does not affect the result.
Ground preprocessing uses the same XY lattice, including when spacing changes.
Ground and nonground return subsets support their respective statistics without
constructing finer spatial cells.

The statistics follow detector requirements:

| Statistics | Purpose |
| --- | --- |
| Point count and original pillar ID | Support reliability, XY density/contrast, footprint support, distribution weight |
| XYZ mean | Empirical spatial location; projected Gaussian mean |
| Full XYZ population covariance | Vertical spread, tilt and transverse scatter; all correlations for the projected Gaussian |
| XYZ minima/maxima | Metric height/extent and complete support bounds |
| Finite intensity/reflectivity count and maximum | Whole-pillar high-reflectivity candidates; missing attributes remain unavailable |
| Ground-branch robust low height, ground height spread and road-conditioned reflectivity | Existing terrain/curb energy, road adjacency and marking thresholds |

`statistics` has one row per occupied pillar. `covarianceXYZ` stores
`[xx xy yy xz yz zz]`; these are the six independent entries of the full 3D
population covariance, not only the XY marginal. All retained returns enter
the geometric statistics, including high-intensity returns. No per-pillar
range mean or unused radiometric moments are computed. The original aligned
return attributes remain available for terrain preprocessing and fine analysis.

For pole geometry, let `C` be the whole-pillar XYZ covariance. The slope of
least-squares XY-on-Z regression is `[C_xz, C_yz]/C_zz`. Its squared transverse
spread is `C_xx+C_yy-(C_xz^2+C_yz^2)/C_zz`, clipped at zero for rounding.
The detector checks these statistics, metric height, Z spread and point count,
then selects compact singleton/edge-pair footprints using neighboring pillar
support and XY point/line shape. Empty-height or horizontal distributions
cannot seed a pole. Facade Hough voting and orientation scores use whole-pillar
point support over the XY neighborhood. These are approximate online candidates;
detailed physical validation remains offline.

The modern structural configuration contains no height-bin occupancy, run,
slice or split-layer parameters. Historical 3D branch construction is isolated
in `deriveLegacyOffGroundGrid`; detailed structural height processing is in
`analyzeFineStructuralCandidates`, reachable only from fine perception.
Shared XY footprint and shape routines accept scalar evidence per pillar and
do not create subpillar cells.

## Fine baseline preservation

Fine perception independently rebuilds detailed structural candidates after the
online result is complete. This retains the established detailed geometric
filters and per-point tests without making coarse candidate recall a gate on
fine classification. The outputs explicitly distinguish:

- `candidates`: online whole-pillar candidates, identical between online and
  offline invocations of the same configuration;
- `fineCandidates`: independent detailed search support used offline;
- `refinement`: evaluated original point indices and per-point decisions.

This preserves the existing candidate-point classifier; it does not establish
exhaustive semantic classification of every raw frame point into a mutually
exclusive taxonomy. Invalid/out-of-ROI returns and unselected features retain
the established mask behavior.

The default `fine.poleRecoveryEnabled=false` preserves the requested fine
baseline. An experimental opt-in combines strong whole-pillar geometry with
relaxed detailed seeding, then applies the original per-point validation.
It recovers the reported frame-260 point 63010, but earlier probes also added
many unreviewed points (frame 800: 33 to 154 with a development setting).
It is therefore not enabled silently. In the default configuration, point
63010 is a coarse pole candidate but remains absent from the fine pole mask.
No broader manual labeling of additional poles was performed.

## Executed comparisons

Before implementation, froze frames, default offline masks/candidates and
five warmed coarse timings from the reference checkout. The first 24 cases
were Mississippi 75, 150, 225, 260, 300, 326, 370, 450, 475, 550, 600, 700,
775, 850, 900, 1000, 1100, 1125, 1150 and Downtown 100, 200, 300, 400, 500.
Added Mississippi 200, 500, 800 and 1050 from the same unchanged checkout to
cover all existing point-regression frames. These are development/regression
cases, not a new untouched accuracy test set.

Across all **28 frames**, default fine masks match the frozen revision exactly
for every requested channel: **zero added and zero removed points**. This
includes 4,312 Mississippi pole points, 1,612 Downtown pole points and 22,633
Downtown facade points. It preserves detector output, not independently measured
accuracy. Ground labels, historical stored-reference channels and mapping
regression tests also pass.

Coarse curb, marking and traffic-sign candidate IDs match exactly. The new
coarse structural classifier is **not equivalent** to the old layer-dependent
classifier:

| Coarse candidate comparison | Shared | Added | Removed |
| --- | ---: | ---: | ---: |
| Mississippi pole | 85 | 156 | 69 |
| Downtown pole | 33 | 90 | 44 |
| Downtown facade | 588 | 498 | 405 |

Consequently, preserved fine results must not be presented as evidence that
online localization accuracy or coarse structural fidelity has been preserved.
The recorded additions/removals require further localization and labeled-feature
assessment. No recorded trajectory matching or observer replay was rerun here.
The retired tests requiring exact coarse height-count maps and legacy pole
candidate identity were replaced by explicit whole-pillar statistics/boundary
and geometric tests; ground fidelity and the original fine fidelity thresholds
remain unchanged. `tests/reference/pipelineReference.mat` was not regenerated.

Final sequential loaded-frame timing on the existing MATLAB R2026a Update 3
session used the native kernels, five warmed calls per frame, no GPU or parallel
pool. Median values below are medians of per-frame medians; maxima cover every
new timing call in the final 140-call batch.

| Dataset | Reference median | Whole-pillar median | Whole-pillar maximum |
| --- | ---: | ---: | ---: |
| Mississippi, 23 frames | 63.845 ms | 68.288 ms | 80.340 ms |
| Downtown, 5 frames | 54.381 ms | 60.722 ms | 75.034 ms |

All final measured perception calls were below 100 ms. This change is not a
speedup over the frozen reference. A development timing batch included a
111.016 ms outlier. The limited warm measurements exclude loading, plotting,
registration and observer work, and are not a hard real-time guarantee.

Full-suite validation: **295 passed, zero failed, two filtered by assumptions**.
A final configuration-inheritance fix passed all 22 structural tests, including
one new custom-intensity regression; merged distinct validation is **296 passed,
zero failed, two filtered**. The default masks were rechecked exactly on all
28 frozen frames after that fix.
The two unexecuted tests require YALMIP and SDP synthesis dependencies that are
not on this MATLAB session's path. All 24 changed/new MATLAB files have zero
factory Code Analyzer findings. New tests cover full XYZ moments, Z-spacing
independence, empty inputs, radiometric membership, a pole straddling a pillar
boundary, horizontal rejection, reported-point recovery and forbidden online
calls. A recorded frame-260 runtime profile contains zero calls to voxelization,
detailed height detection or point refinement.

Exports: [fine fidelity](results/whole_pillar_20260908/fine_fidelity.csv),
[coarse fidelity](results/whole_pillar_20260908/coarse_fidelity.csv),
[timing](results/whole_pillar_20260908/timing.csv),
[tests](results/whole_pillar_20260908/test_results.csv),
[Code Analyzer](results/whole_pillar_20260908/code_analyzer.json), and
[online profile](results/whole_pillar_20260908/online_profile.json).

## Reproduction

Raw MAT snapshots and the exported reference checkout are ignored local
artifacts in `output/pillar_only_20260908/`. Each snapshot contains `frame`,
`cfg`, `baseline` (the reference offline result), `seconds` (five warmed coarse
calls) and `frameIndex`. Capture them by running the listed source frames
through `perceiveFrame` from a `git archive` export of the reference revision,
with `perceptionConfig(dataset)`, first in offline mode, then in coarse mode.
Do not overwrite existing frozen snapshots.

```matlab
setupVehicleLocalization;
setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(pwd,'data'));
report=evaluateWholePillarPerception('output/pillar_only_20260908');
results=runtests('tests');
```

Source data, reference exports, MAT snapshots, local native binaries and unrelated
observer work are excluded from the project commit.
