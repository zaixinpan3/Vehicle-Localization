# Mississippi full-sequence fine-perception video

## Scope and execution

All 1170 frames of `data/raw/MissisipiPointClouds.mat` were processed with the
current offline fine perception configuration. The perception implementation
is unchanged from commit `9b82a46f897d5c5cfe388b5b027d22ae5a5551d8`.
The user-selected native pcshow view was captured at frame 380 and held fixed
throughout the video. Road markings remain disabled; the displayed classes
are curb, pole and traffic sign, colored directly on original points.

The existing `scripts/runPerceptionVideoMap.m` ran with `cfg.buildMap=false`.
Neither perception nor the runner was changed for this regeneration. Every
rendered frame checks captured camera/axis geometry, the finite original-point
count and constant image dimensions. The frame counter shows the source index
out of 1170. The earlier export remains in its separate output directory.

The figure was copied to the isolated MATLAB rendering display so native
window interaction could not alter the recording. Its 1910 x 2086 captured
pixel dimensions and camera were preserved. Native MATLAB used 96 DPI with
approximately 2.25 operating-system display scale; the isolated display used
72 DPI. Point-unit fonts and line widths were scaled by three and marker
areas by nine to preserve displayed sizes. This applies uniformly to source
and feature points. The source marker area remains 8 in the captured native
figure; its rendered physical equivalent is 72 in the isolated display.

A two-frame smoke recording passed after display calibration. Software
perspective box clipping initially omitted central returns from the raster,
even though the scatter retained all source coordinates. Disabling clipping
on both axes and scatter restored the dense cloud, confirmed by inspecting
the rendered preview and first smoke frame. Camera geometry and point data
were unchanged. Legend position was calibrated for the isolated display.
Point-count assertions alone do not prove raster completeness.

The initiating MCP request timed out after 300 seconds while MATLAB continued
processing. Completion was established from the atomic progress file, final
MATLAB result and independent full-video decoding, not the tool-call status.

## Result and validation

- Processed and recorded frames: 1170 (1 through 1170, inclusive).
- Dimensions: 1910 x 2086 pixels.
- Playback cadence: 10.909039311 frames/s, computed from the full
  matched LiDAR timestamp range.
- Duration: 107.251 s.
- Processing/rendering elapsed time: 4032.212 s. This is
  machine elapsed time, not user work hours or a real-time throughput claim.
- Aggregate feature observations: {'curb': 139407, 'pole': 194816, 'trafficSign': 70950}.
- Code Analyzer findings in the unchanged runner: 0.

Both the Motion JPEG AVI master and H.264 MP4 contain exactly 1170 decoded
frames at the expected dimensions and decode fully without errors. The MP4
uses libx264, CRF 14, medium preset, yuv420p, faststart and passthrough cadence.
The CSV frame indices form the exact continuous sequence 1:1170, and their
feature totals equal the MATLAB run result. This validates video integrity;
it does not establish ground-truth semantic accuracy for every frame.

## Artifacts

All original generated artifacts are under
`output/mississippi_perception_video_20260911_v2/`:

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

MP4 SHA-256: `88d0fb3582828fa85493735fa81d1d7d1d516fd31d8939a52168ba834d65959d`.

AVI SHA-256: `834336939d0cf85dfb1d7a04a7340747bba2678d593471e03f1f4beeaee43d55`.
