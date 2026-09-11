# Mississippi full-sequence fine-perception video

## Scope and execution

All 1170 frames of `data/raw/MissisipiPointClouds.mat` were processed with the
current offline fine perception configuration. The perception implementation
is unchanged from commit `1c2930eeb059b1981ac98f1fa5a31c7eb15e7820`.
The user-selected native pcshow view was captured at frame 615 and held fixed
throughout the video. Road markings remain disabled; the displayed classes
are curb, pole and traffic sign, colored directly on original points.

`scripts/runPerceptionVideoMap.m` now accepts `cfg.buildMap=false` to record
perception and reusable observations without fitting a map. The combined
video-and-map behavior remains the default. The title updates to the current
frame instead of retaining the preview frame number. Every rendered frame
asserts the captured camera and axis geometry, the number of finite original
points, and constant image dimensions. The frame counter shows the source
frame index out of 1170.

The figure was copied to the isolated MATLAB rendering display so native
window interaction could not alter the recording. Its 1910 x 2086 captured
pixel dimensions and camera were preserved. Native MATLAB used 96 DPI with
approximately 2.25 operating-system display scale; the isolated display used
72 DPI. Point-unit fonts and line widths were scaled by three and marker
areas by nine to preserve displayed sizes. This applies uniformly to source
and feature points. The source marker area remains 8 in the captured native
figure; its rendered physical equivalent is 72 in the isolated display.

An initial two-frame recording verified the video-only path. Visual inspection
found a scaling mismatch in that preliminary recording, which was corrected
before the complete run. The read-only ScreenPixelsPerInch setting was not
changed; explicit graphics scaling was used. Software-rendering warnings were
checked against the complete-point assertions rather than assuming the
renderer retained every point.

The initiating MCP request timed out after 300 seconds while MATLAB continued
processing. Completion was established from the atomic progress file, final
MATLAB result and independent full-video decoding, not the tool-call status.

## Result and validation

- Processed and recorded frames: 1170 (1 through 1170, inclusive).
- Dimensions: 1910 x 2086 pixels.
- Playback cadence: 10.909039311 frames/s, computed from the full
  matched LiDAR timestamp range.
- Duration: 107.251 s.
- Processing/rendering elapsed time: 3536.218 s. This is
  machine elapsed time, not user work hours or a real-time throughput claim.
- Aggregate feature observations: {'curb': 138089, 'pole': 195297, 'trafficSign': 70950}.
- Code Analyzer findings in the modified runner: 0.

Both the Motion JPEG AVI master and H.264 MP4 contain exactly 1170 decoded
frames at the expected dimensions and decode fully without errors. The MP4
uses libx264, CRF 14, medium preset, yuv420p, faststart and passthrough cadence.
The CSV frame indices form the exact continuous sequence 1:1170, and their
feature totals equal the MATLAB run result. This validates video integrity;
it does not establish ground-truth semantic accuracy for every frame.

## Artifacts

All original generated artifacts are under
`output/mississippi_perception_video_20260911/`:

- `perception.mp4`: portable playback export.
- `perception.avi`: Motion JPEG master, one image per source frame.
- `captured_view.fig`, `captured_view.mat`, `captured_view.png`: native view.
- `initial_view.fig`, `run_configuration.mat`: calibrated recording view and
  exact camera, configuration, dimensions, frame list and cadence.
- `feature_observations.mat`, `frame_feature_counts.csv`: identical perception
  outputs registered to matched poses, retained as reusable observations.
- `frame_timestamps.csv`: source frame indices and LiDAR timestamps.
- `validation.json`, `video_integrity.json`, `checks.json`: run assertions,
  independent decoding/count checks and static checks.
- `frame_0001.png`, every 100th-frame PNG, and `frame_1170.png`: review samples.

No map was fitted during this video task. Large generated data and videos are
excluded from the source-code commit. The artifact integrity manifest records
SHA-256 values; its existence does not imply semantic correctness.

MP4 SHA-256: `7a4487b294e0bf6484a76d1d516078908258c6b6e19d3c0720fdf117b9ef55f7`.

AVI SHA-256: `1b17c45f4bac74e1437fb82ceb4e5ce787d0a97d6e4be6d33e3a86537fdc83aa`.
