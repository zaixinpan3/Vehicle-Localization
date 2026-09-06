# Coarse perception to planar D2D pose: latency measurements

Date: 2026-09-06. Baseline: `2692e9e5c12d7419b6b4224b5d4ccba2fbe49860`.

The complete default path runs successfully on seven Mississippi scenes with
current 30-frame local maps. Over 420 warmed calls, median raw-frame-to-result
time is **80.381 ms**, P95 **90.475 ms**, P99 **97.685 ms**, and maximum
**106.036 ms**. All 420 calls emit a finite `[X,Y,psi]` event. A separate paced
test at the recorded median frame interval has maximum availability-to-output
latency **101.008 ms**, with no accumulating queue in that run.

These results support **150 ms as a candidate fixed-delay simulation value for
this warmed default workload**. They do not establish a worst-case execution
time or constant physical delay. Earlier six-frame-map runs demonstrate that
ordinary runtime variation can cause substantial queue growth near capacity.
The production observer and its existing delay configuration remain unchanged
while the delay choice is evaluated.

## Measurement scope

Platform: AMD Ryzen 7 7800X3D, 8 cores / 16 logical CPUs; MATLAB
26.1.0.3276743 (R2026a) Update 3; the existing native perception kernel is
enabled. This is the ordinary shared desktop, without real-time isolation.

Timed entry point: `localizeLidarFrame`, beginning with an in-memory raw
organized frame and cached exported local probability map. Every call executes
coarse pillar perception, probability-cloud construction, geometric D2D,
acceptance checks and event construction. Source clouds are never reused
between timed calls. Registration estimates only `[X,Y,psi]`; the default XY
objective is used and source XYZ statistics remain available.

Offline file reads, fine perception for mapping, map fitting/export,
visualization, live sensor acquisition/transport and external ROS scheduling
are excluded. Single-frame MAT reads in the pilot were about 0.5 s and are
reported separately in `preparation.csv`. The test does not include a search
through an entire route map: the intended cached local-map input is supplied.

Dataset: `data/raw/MissisipiPointClouds.mat`, queries
`[120,260,350,550,700,900,1050]`, using the matching 1:1170 pose CSV. The current
offline pipeline builds each map from the following N frames, excluding its
query. N=30 is the default-size study; N=6 is the initial pilot. Both use
schema-2 repeated-observation maps and the configured Mississippi channels:
curb, roadMarking, pole and trafficSign. Recorded attitude supplies known tilt.

Three starts are the recorded pose and offsets `[0.5,-0.4,2 degrees]` and
`[-0.5,0.4,-2 degrees]`. Each scene/start is warmed twice, then measured 20
times in randomized order with mt19937ar seed 20260906. Thus 420 invocations
represent 21 distinct scene/start cases, not 420 independent localization
scenes. Each study rebuilds its maps with current code. Raw inputs and fitted
cloud caches remain in ignored `output/` folders.

## Default 30-frame local maps

| Stage | Median (ms) | P95 (ms) | P99 (ms) | Maximum (ms) |
|---|---:|---:|---:|---:|
| Coarse perception and probability cloud | 65.392 | 75.578 | 82.242 | 90.893 |
| D2D registration | 14.457 | 19.927 | 21.794 | 22.180 |
| Complete call | 80.381 | 90.475 | 97.685 | 106.036 |

Marginal stage quantiles need not sum to the complete-call quantile.
The actual call is timed directly. Three of 420 calls exceed 100 ms; none
exceeds 150 ms. Quantiles linearly interpolate sorted samples at
`1+(n-1)*p`; no Statistics Toolbox is required.

| Query | Map Gaussians | Source Gaussians | Accepted calls | Maximum XY difference from recorded pose (m) |
|---|---:|---:|---:|---:|
| 120 | 81 | 104 | 60/60 | 0.2734 |
| 260 | 57 | 82 | 60/60 | 0.2108 |
| 350 | 85 | 92 | 60/60 | 0.2768 |
| 550 | 72 | 97 | 60/60 | 0.0992 |
| 700 | 57 | 86 | 60/60 | 0.1442 |
| 900 | 64 | 94 | 60/60 | 0.3720 |
| 1050 | 60 | 88 | 60/60 | 0.3152 |

The maximum absolute yaw difference is 0.7396 degrees. These are consistency
differences against the same route poses used for offline mapping, not
independent ground-truth accuracy. Passing the acceptance gate is not proof
that a pose is correct under arbitrary scene ambiguity.

## Paced serial processing and queue sensitivity

The original exported front-LiDAR timestamp table contains 1,170 frames.
Its median frame interval is 91.693 ms (about 10.906 Hz); the observed
minimum and maximum are 84.443 and 94.353 ms. The difference between rosbag
record time and message header time has median 3.088 ms, P99 6.601 ms and
maximum 12.581 ms. This records timestamp relationships; it does not establish
which point within a scan defines the sensor timestamp or isolate transport.

`measurePacedLocalizationPipeline` executes the same complete pipeline under
regular wall-clock frame availability and serial service. It measures actual
waiting and computation in MATLAB. Each run contains 210 calls over the same
21 scene/start cases in deterministic shuffled order. It is a paced
in-memory experiment, not live ROS replay or a full-route deployment.

| Map / paced run | Period (ms) | Mean compute (ms) | Median availability-to-output (ms) | P99 (ms) | Maximum (ms) |
|---|---:|---:|---:|---:|---:|
| 30-frame, recorded median rate | 91.693 | 80.716 | 80.572 | 96.579 | 101.008 |
| 30-frame, 10 Hz | 100.000 | 80.901 | 80.889 | 100.312 | 103.402 |
| 6-frame pilot, recorded rate, first | 91.693 | 93.926 | 348.471 | 604.584 | 622.741 |
| 6-frame pilot, recorded rate, repeat | 91.693 | 89.158 | 98.862 | 187.268 | 196.837 |
| 6-frame pilot, 10 Hz | 100.000 | 89.527 | 91.235 | 157.801 | 164.424 |

In the first pilot paced run, mean service time exceeded the arrival period
and waiting time grew to 519.882 ms. Merely increasing a constant delay cannot
bound a queue under sustained service slower than arrival. The later runs show
that this is not an invariant property of the default pipeline: the 30-frame
case maintained the recorded median rate in its measured run.

The map-size studies and paced runs were sequential, not randomized against
each other; CPU/runtime conditions were not controlled. Differences in their
timings cannot be attributed solely to map size. The source perception code
and raw queries are identical, yet its measured time varies between studies.
The pilot must therefore remain visible when discussing timing robustness.

The pilot batch study has median 87.466 ms, P99 124.815 ms and maximum
138.261 ms. It accepts 360/420 calls. The 60 rejections are all frame 700,
with `inconsistentClasses`, not rank deficiency. The 30-frame maps resolve
that acceptance failure in these tests. Bigger maps change both the geometric
constraints and the number of candidate associations.

Adding the historical per-frame header-to-bag interval to the default paced
latencies gives reconstructed maxima of 103.963 ms at recorded median rate
and 106.513 ms at 10 Hz. These sums combine recorded timestamp differences
with current measured execution and are explicitly not a live end-to-end
measurement. `latency_model.json` in the local pilot folder also contains an
offline FCFS calculation from batch timings. Its optimistic prediction did
not capture the slow paced run, reinforcing the need to measure serial load.

A separate initial sanity call on the pre-existing 58-component map took
392.044 ms (337.630 ms perception, 53.259 ms registration). It includes first
execution effects and was not a controlled cold-start benchmark. Warmed
statistics must not be represented as startup bounds.

## Delay decision and observer work

The current default workload supports retaining the original input rate for
the next experiment. An 80 ms fixed delay is too small even for the warmed
default median. A 150 ms value leaves margin above the measured default tails;
200 ms is a useful additional sensitivity case. Neither value is a certified
bound across startup, unseen routes, CPU contention or unstable queues.

Actual delay is timestamp-to-availability plus queue waiting plus computation.
A constant-delay proof requires a corresponding measurement-time assumption;
assigning a constant in configuration does not make physical latency constant.
This benchmark does not alter timestamps or implement delayed release.

Before the timing priority was clarified, an exploratory delayed-epoch
observer with a current-state model predictor was examined. A pure-feasibility
SeDuMi solve yielded a discrete posterior-error candidate at sample period
0.1 s, scaling 3.5 and contraction factor 0.98. All 32,768 tested output/model
vertices had negative residual, with worst value -0.0018977159. The earlier
quadratic-objective solve encountered numerical problems; an SDPT3 attempt
encountered an incompatible `mexschurfun`. The successful candidate is saved
with explicit preliminary status in `preliminary_observer_candidate.json`.

This is not a completed delayed-system certificate or implemented observer.
The draft runtime edits were restored to their pre-task contents, preserving
the user's ongoing relocation. No new production delay is selected here.
The delayed-state/predictor direction is consistent with the alternative
discussed in Remark 7 of
[Sanz, Garcia and Krstic (2019)](https://flyingv.ucsd.edu/papers/PDF/319.pdf),
but that paper does not certify this project's candidate. Further work must
match the actual sample schedule, GNSS dropout and valid LiDAR observation
conditions before changing the production observer.

## Reproduction and validation

```matlab
setupVehicleLocalization();
pilot = benchmarkLocalizationPipeline("output/pipeline_pilot",20,6);
default = benchmarkLocalizationPipeline("output/pipeline_default",20,30);
paced = measurePacedLocalizationPipeline("output/pipeline_default", ...
    0.09169316291809082,10);
paced10Hz = measurePacedLocalizationPipeline("output/pipeline_default",0.1,10);
```

The repeated pilot paced run used a separate folder with the same cached
inputs to preserve the first result. In total, two 420-call batch studies and
five 210-call paced studies produce **1,890 measured invocations**. Every
invocation checks that acceptance agrees with event presence; batch calls also
check the accepted event has exactly three finite pose components. Both new
MATLAB scripts have zero factory Code Analyzer findings. No perception,
mapping, registration or observer algorithm was changed by this benchmark.

CSV calls, warmup, preparation, summary, paced results, JSON validation and
source/native-kernel provenance are exported under
`results/localization_pipeline_latency_20260906/`. Cached point clouds, maps
and MAT results remain under `output/localization_pipeline_timing*_20260906/`
and are excluded from the public commit. Only the benchmark scripts, research
report and small result exports are committed.
