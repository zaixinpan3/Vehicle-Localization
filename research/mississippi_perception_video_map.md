# Mississippi perception video and offline map

Date: 2026-09-09. Fine-perception baseline:
`5acbdfd3e4f2b54fc3f7470dcc73000a99f42271`.

## Execution contract

The user selected the camera interactively on Mississippi frame 687 and asked
for a continuous video over every frame, with the same visualization, while
also constructing the offline semantic map. A subsequent request adds the
current frame number to the visualization.

`runPerceptionVideoMap` captures the existing tagged pcshow figure. Camera
position, target, up vector, view angle, projection, axis limits and aspect
ratios remain fixed. Source/feature colors and marker settings come from the
existing figure. The original display is retained while only XYZ data and
feature counts change. The current entry point adds a fixed frame counter.

`collectFeatureObservations` now accepts an optional frame consumer after each
frame's fine features have been registered. Video recording consumes that same
perception result, so perception executes once per frame and the map does not
receive independently recomputed masks. Existing callers without a consumer
retain their behavior. Detector and mapping thresholds are unchanged. An empty
curb-proposal guard was added after a sequence-specific runtime failure, as
described below.

The mapping uses the existing LiDAR calibration and matched GNSS/INS poses,
then the current schema-2 canonical repeated-observation Gaussian field.
Acquisition windows contain 30 frames with stride 20; overlapping windows do
not ingest an observation twice. Raw observations are retained separately from
the fitted map. This is an offline map registered using the recorded reference
poses; no new pose-estimation or localization experiment is implied.

## Video capture and timing

The saved camera has perspective projection, view angle approximately
0.731229 degrees, position `[-1481.213970, 469.204308, 1012.115742]`, target
`[-2.548633, 2.604143, 5.186812]`, and up vector
`[0.522711, -0.164447, 0.836499]`. Exact camera/axis values are saved as JSON
and MAT artifacts. The unusual large camera distance and narrow angle reflect
the user's actual zoom; they were not replaced with a preset viewpoint.

Native capture from the visible niri MATLAB session has 1910 by 2086 pixels.
The recording's initial capture exactly matches the saved desktop capture
(mean absolute RGB difference zero). The background, perspective, palette and
marker sizes are retained. A headless pilot was used only to check the data
contract; its software graphics and different capture dimensions were not
used for the full video.

One video image corresponds to one input frame. Playback frame rate is
`(N-1)/(last_timestamp-first_timestamp)`, approximately 10.90903931 Hz. The
recorded first-to-last timestamp span is 107.15884018 seconds. Constant-cadence
playback preserves that mean timing; it does not reproduce the small individual
interval variation (84.44--94.36 ms). Offline processing time does not control
playback speed.

The frame-number request arrived after full-sequence recording had started.
For this run, the final MP4 receives a uniform `Frame n / 1170` overlay during
encoding, including the already-recorded frames. The AVI master retains the
unannotated capture. The current MATLAB entry point also displays the counter
for subsequent invocations. Neither change reruns or modifies perception.

## Interrupted run and recovery

The initial native recording stopped at frame 1012: no anchor pair satisfied
curb proposal-length constraints, and normalization of the resulting empty
array raised an incompatible-size error. The fix returns an empty boundary
proposal before normalization when no eligible pair exists. It does not change
thresholds, accepted nonempty proposals, or the unorganized-XYZ interface.

The closed AVI retained frames 1--1011. The remaining frames 1012--1170 were
recorded in the same desktop figure with the saved camera and the original
full-sequence frame rate. The two unannotated AVI parts were joined without
re-encoding; the continuous counter was then added to the final MP4. The
initial failure log and each part remain available as diagnostic evidence.

Because collection returned observations only after completing its requested
range, the initial interruption lost its in-memory mapping observations.
Frames 1--1011 were therefore recomputed in saved batches of at most 100 frames,
then merged with the tail's 159 observations before fitting the full map.
This recovery required a second perception pass for those 1011 frames; it did
not replace the video images already recorded. Source hashes, initial failure,
recovery method, and final checks distinguish the actual run from the normal
single-pass entry point.

## Implemented checks

A three-frame pilot produced exactly three video frames. Its complete
registered `featureData` was exactly equal to a separate invocation of the
original collection interface without a callback. Both existing
`pipelineRegressionTest` cases passed. Factory Code Analyzer reported zero
findings in the two changed/new MATLAB files.

A second three-frame pilot verified that the counter advances to
`Frame 3 / 1170` and that its complete registered observations remain exactly
equal to the first pilot. The full-run output checks and measured result counts
are recorded with the completed output artifacts. After the empty-proposal
fix, all 13 curb geometry tests and all 31 current perception reference tests
passed (30 recorded mask comparisons and one execution-mode check). All four
changed/new MATLAB files pass factory Code Analyzer with zero findings. The
previously failing frame 1012 completed with 105 curb points. No test fixture
was updated. A final three-frame pilot reused an existing frame-counter handle,
verified that exactly one counter remains and advances to `Frame 3 / 1170`,
and preserved the original pilot observations exactly.

## Full-sequence result

All 1170 source frames are present in both the AVI master and H.264
MP4. The MP4 is 1910 by 2086 pixels, contains a continuous `Frame n / 1170`
overlay, and lasts approximately 107.2505 seconds at the mean LiDAR
cadence. It was opened for playback in the current niri session.

The complete map ingests each frame once across 58 acquisition
windows and exports 1902 positive-mass Gaussian components.
Registered observation counts (repeated views retained before map compaction):

| Feature | Point observations |
| --- | ---: |
| curb | 123225 |
| roadMarking | 97901 |
| pole | 235379 |
| trafficSign | 75530 |

All exported XY covariance matrices are positive definite.
Of 1902 component-center queries, 1902
are within valid coverage; their probabilities range from
4.0100215e-284 to 0.99206097.
These checks establish artifact completeness and numerical consistency, not
independent semantic accuracy. The map overview shows published components
and the reference-pose trajectory.

Compact measurements and artifact hashes are in
[`results/mississippi_video_map_20260909/`](results/mississippi_video_map_20260909/).
Original video, observations and map files are in
`output/mississippi_video_map_20260909/full_sequence/`.

## Reproduction

With a preview figure containing source and semantic scatter layers tagged by
`PointLayerName`, adjust its camera and run:

```matlab
setupVehicleLocalization;
result = runPerceptionVideoMap("output/new_sequence_run", figureHandle);
```

Use a new output folder and keep the figure size fixed during native capture.
The optional third argument is `featureMapBuildConfig()`, including a contiguous
`frameIndices` subset for a short pilot. The saved observations can be used to
repeat map fitting without repeating perception or recording:

```matlab
loaded = load("output/new_sequence_run/feature_observations.mat");
cfg = featureMapBuildConfig();
map = buildSlidingWindowMap(loaded.featureData,cfg);
```

Generated video, point observations, figures and map binaries remain outside
Git. The committed research results contain compact validation records and
configuration references, not the raw recorded dataset.
