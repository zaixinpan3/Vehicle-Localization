# Reduce coarse perception to D2D pose latency without changing feature decisions

Date: 2026-09-06. Baseline commit:
`883c2e4cb58ea3e90ffbe4e67a15a062b8b43560`.

The final implementation meets the requested **measured 100 ms target for the
warmed default 30-frame-map workload**. Its complete call has median
**70.292 ms**, P99 **84.071 ms** and maximum **97.356 ms** over 420 calls.
The same-process, interleaved baseline has median **87.114 ms**, so the
median reduction is **19.31%**. All 420 calls are accepted, the probability
clouds are exactly equal, correspondence indices are identical, and the
maximum pose-component difference is **2.220446049250313e-16**.

Default paced processing also stays below 100 ms: maximum availability-to-pose
latency is 95.586 ms at the recorded median rate, and 96.284 ms at 10 Hz.
An additional six-frame-map run has one 108.114 ms outlier. These are measured
results on a shared desktop, not a universal execution-time bound.

## Algorithm and implementation findings

Profiling identified ground-feature raster calculations and repeated D2D
component operations as useful targets. The change preserves all semantic
channels, pillar membership, curb refinement stages, thresholds, retained XYZ
statistics, objective terms, robust weights, acceptance gates and `[X,Y,psi]`
state dimension. No approximate correspondence search, point subsampling or
iteration-budget reduction is introduced.

1. **Prepare class membership once per registration.** Eligible target
   components, positive-quality source components and line/point semantics
   do not change with pose. Their repeated string comparisons and lookups
   move outside the nonlinear iterations.
2. **Batch distribution comparisons and planar algebra.** Within each class,
   covariance rotation and candidate distances are computed in arrays.
   Source blocks target at most 65,536 pair entries per temporary matrix;
   a single source with more targets still requires one target-length vector.
   The first target in a distance tie is preserved. Cholesky whitening,
   Jacobians, robust normal equations and frozen-correspondence line-search
   costs use array operations. The public correspondence table is constructed
   at final output, rather than at each iteration. The conditional height gate
   remains active in XYZ mode and never adds a Z pose state.
3. **Fuse exact perception statistics in native CPU kernels.** Neighbor
   height differences visit valid neighbors directly. Directional line
   support computes the same four orientations and side counts only where
   the final center/component gates can be positive; convolution boundary
   validity is preserved. Per-cell XY/XYZ means and centered covariance use
   two accumulation passes, retaining small scatter at large coordinates.
   No raw-moment subtraction approximation replaces centered scatter.
4. **Keep the MATLAB fallback usable.** The native interface now reports
   version 2. An old binary falls back to MATLAB in `auto` mode; explicit
   `native` mode requests a rebuild. The fresh binary is compiled, version
   checked and smoke tested by `buildPerceptionKernels`. No binary is tracked.

The coarse implementation continues to analyze pillars. Its native reductions
accumulate member-point statistics; they do not perform fine point-level
semantic validation. Offline mapping retains its fine refinement.

## Paired comparison

The raw frames, 30-frame maps and starts are the same as the preceding
[latency study](localization_pipeline_latency.md): Mississippi query frames
`[120,260,350,550,700,900,1050]`, maps built from the following 30 frames with
the query excluded, and three starts (recorded pose and opposing
`[0.5,-0.4,2 degrees]` offsets). Each method/case is warmed twice. Twenty
repetitions use mt19937ar seed 20260906, random scene/start order and random
method order within each pair. Thus each method executes 420 measured calls
on 21 distinct cases, not 420 independent scenes.

The baseline helper extracts the eight affected call-chain functions from
Git and only renames their function identifiers, allowing interleaved calls
without changing the MATLAB path between measurements. Private helpers come
from that commit. Unchanged public helpers and the unchanged legacy native
commands are shared. The extracted baseline was checked against the saved
pre-edit cloud and pose before timing. No source probability cloud is reused
between timed calls.

Platform: AMD Ryzen 7 7800X3D, MATLAB 26.1.0.3276743 (R2026a) Update 3,
eight MATLAB computational threads, current compiled CPU kernels. Desktop
load is not isolated. Thread-count exploration did not justify a production
change; the original thread setting was restored and retained.

| Stage | Baseline median | Optimized median | Optimized P95 | Optimized P99 | Optimized max |
|---|---:|---:|---:|---:|---:|
| Coarse perception and probability cloud | 70.244 ms | 63.433 ms | 72.045 ms | 77.087 ms | 84.355 ms |
| D2D | 15.513 ms | 6.814 ms | 8.492 ms | 9.593 ms | 12.971 ms |
| Complete call | 87.114 ms | 70.292 ms | 79.807 ms | 84.071 ms | 97.356 ms |

Baseline complete-call P99/max are 105.867/113.711 ms; 11/420 baseline calls
exceed 100 ms, versus 0/420 optimized calls. Marginal stage quantiles do not
necessarily sum to total quantiles. Quantiles interpolate sorted samples at
`1+(n-1)*p`, as in the previous study.

An earlier prototype paired run is retained separately: baseline median/max
88.709/136.564 ms, optimized 71.774/98.501 ms. That prototype predates the
final pair-workspace bound and native-version check. Exploratory thread,
backend and initial interleaved trials are also retained, including slower
and first-execution results. The historical 80.381 ms baseline from the
previous task was collected at a different time; the 19.31% improvement uses
the interleaved baseline above, not that earlier measurement.

## Paced processing

The existing wall-clock serial harness supplies regular frame availability,
then measures waiting plus complete computation. Raw inputs/maps are already
in memory. It does not replay live ROS transport or the original variable
frame-to-frame arrival schedule.

| Map / period | Calls | Accepted | Median availability-to-pose | P99 | Maximum | Above 100 ms |
|---|---:|---:|---:|---:|---:|---:|
| Default 30 frames / 91.693 ms | 420 | 420 | 68.878 ms | 83.444 ms | 95.586 ms | 0 |
| Default 30 frames / 100 ms | 210 | 210 | 70.212 ms | 80.990 ms | 96.284 ms | 0 |
| Pilot 6 frames / 91.693 ms | 210 | 180 | 72.371 ms | 90.132 ms | 108.114 ms | 1 |

There is no sustained queue growth in these runs. Default recorded-rate
waiting peaks at 3.914 ms. The pilot's longest computation is 108.085 ms on
frame 550/start 2; the next frame waits 16.451 ms and the queue then clears.
Its 30 rejections are frame 700 `inconsistentClasses`, the same pilot-map
acceptance limitation previously reported. No rejected pose becomes an
accepted event for the timing target.

Together, the final default batch and paced studies have **1,050 optimized
calls, all accepted, none above 100 ms** in their respective measured scopes.
The two paired studies plus three paced studies contain 2,520 formal timed
calls including baselines and the pilot. Warmup and exploratory trials are
separate. Larger/unseen maps, other loads and first execution are not bounded
by these observed maxima.

## Validation and remaining scope

- **151 distinct tests passed**, zero failures/incomplete cases. Two focused
  final suites execute 165 tests including 14 repeated geometric tests; the
  exported table retains the latest result per name. Coverage includes XY/XYZ
  distribution matching, height compatibility, degenerate geometry, class
  conflict, 1,200-component duplicate/tie handling across blocks, native edge
  behavior, empty/singleton moments and small scatter at large coordinates.
- Recorded Mississippi and Downtown perception tests pass, including fine
  masks, facade, traffic sign, all requested feature subsets, and the stored
  pipeline reference. No reference artifact or threshold was changed.
- Both 420-pair comparisons preserve the entire probability-cloud struct and
  correspondence indices exactly, with pose differences at roundoff level.
  This establishes regression equivalence on these cases, not independent
  ground-truth accuracy or a general perception-quality theorem.
- Eleven changed MATLAB files have **zero factory Code Analyzer findings**.
  The Python baseline helper compiles. The native C++ build and smoke tests
  pass. An actual saved old binary was loaded temporarily: automatic fallback
  was used, explicit native selection raised `perception:NativeUnavailable`,
  and the current version-2 binary was restored and verified.

Timing starts with an in-memory raw frame and a cached local probability map,
and ends with a pose event or rejection. It excludes disk reads, offline map
construction, global map lookup, live sensor acquisition/transport and
visualization. Warmed maxima are not cold-start bounds. No production observer
or fixed-delay parameter changed. The earlier 150 ms simulation candidate is
not replaced by a certified 100 ms timestamp-to-output bound.

## Reproduction

With the cached maps from the preceding study:

```bash
python scripts/createLocalizationTimingBaseline.py output/latency_baseline
```

```matlab
setupVehicleLocalization();
buildPerceptionKernels();
addpath(genpath('output/latency_baseline'));
report = compareLocalizationPipelineTiming( ...
    'output/localization_pipeline_timing30_20260906', ...
    'output/latency_comparison',@localizeLidarFrameLatencyBaseline,20);
% Keep paced exports separate from the earlier campaign.
mkdir('output/latency_paced');
copyfile('output/localization_pipeline_timing30_20260906/inputs.mat', ...
    'output/latency_paced/inputs.mat');
calls = measurePacedLocalizationPipeline( ...
    'output/latency_paced',0.09169316291809082,20);
```

Small CSV/JSON exports, profiler entries, test results and source/native
provenance are in
[the result folder](results/localization_pipeline_optimization_20260906/).
Local MAT checkpoints, extracted baseline sources, raw/map caches and native
binaries remain under ignored `output/` or the normal ignored binary location.
