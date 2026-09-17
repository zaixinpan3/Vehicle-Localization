# Updated MnCAV localization video: aligned BESTPOS and 10 Hz observer

The updated video is `output/localization_video_20260917/mncav_localization.mp4`.
It displays the frozen both-channel result from
`output/mncav_bestpos_alignment_20260917/experiment.mat`. The previous September
16 video and its inputs remain intact. No observer, map matching or perception
inference was rerun for this visualization.

## What changed

The geographic view retains the existing USGS/USDA aerial orthophoto, current
INSPVA reference and estimated positions/headings, accumulated trajectories,
route overview and true metric scale. The right panel retains the actual
current-frame curb, pole and traffic-sign points with gray LiDAR context. The
metric offset inset and error timeline remain, without separating or amplifying
the two geographic trajectories.

All poses, motion and metrics now come from the aligned BESTPOS experiment,
replacing the older 100 Hz, 10.14 cm result. The accuracy panel displays:

- Full-run horizontal position RMSE **7.85 cm** and median **4.62 cm**.
- Same 1,083 accepted frames: fusion **6.82 cm**, raw LiDAR **14.86 cm** RMSE.
- **1,169 actual localization outputs at approximately 10 Hz**.

The header reports the current real sample's position and heading error,
wheel-derived speed, LiDAR frame and matching status. The full-run heading
RMSE remains 0.540172 degrees, recorded in metadata; the updated accuracy card
uses the space for the requested position comparison.

The MP4 is 1920 by 1080, 30 fps, H.264/yuv420p, faststart and normal speed:
3,505 encoded frames, 116.833333 seconds, 49,738,332 bytes.
The source lasts 116.7986895341 seconds. The animation **holds the latest actual
synchronized observer output and corresponding perception frame** until the
next acquisition, showing frame age. It does not interpolate or extrapolate
new localization states for the 30 fps animation. The current sample/perception
selection never uses a future frame. Static full-run statistics, the route
locator and gap shading summarize this offline experiment.

The experiment ends at source LiDAR frame 1,169. The original source has one
additional frame outside observer coverage; that frame is excluded rather than
extending the trajectory. No covered localization frame is removed. All 86
frames without accepted full-pose matching remain visible with their original
perception and error. Shared receiver reference and reference-assisted
map/matching still limit independent absolute-accuracy claims. The orthophoto
is geographic context, not a validation reference or live satellite image.

## Validation

`video_validation.json` records final codec, dimensions, duration, frame count,
complete FFmpeg decode and the MP4 SHA-256. `checks.json` independently audits
all original HDF5 pose, clock and wheel-speed samples, original full/paired
metrics, actual display sampling at every video frame, and perception timing.
The exported pose difference is below 1e-8 m/rad. Input time round-trip difference
is at most 4.98e-13 s; the exporter permits 1e-9 s solely for CSV precision.

The reused point cache and orthophoto files match their prior committed manifest
hashes. A fresh coordinate-only HDF5 export was also executed: all 1,169 covered
display clouds and raw point counts match the reused cache bit-for-bit. This
retains 133,735 curb, 193,847 pole and 70,945 traffic-sign points before display
clipping. Gray context alone retains the existing ROI and 10,000-point limit.
No semantic classification changed. The fresh SE(3) round-trip residual is zero.

Six preview times (0, 20, 50, 82, 100 and final seconds), the full-size 50-second
preview, and the decoded 50-second video frame were visually inspected. Their
encoding PSNR is 38.9543 dB, reported separately from localization accuracy
in `checks.json`.
The MATLAB exporter has zero factory Code Analyzer findings. All three changed
Python paths compile. Observer regression is not rerun or claimed for this
visualization-only change; the underlying experiment retains its existing 152
passing tests.

Development checks exposed and resolved CSV timestamp exact-equality and
row/column shape assumptions in the exporter. After successful encoding, a
validation-only variable shadowing error prevented the metadata report from
being written. Renaming the hash stream resolved it; `--validate-only` then
checks the already encoded video. The video content did not require re-encoding.

## Reproduction

```matlab
setupVehicleLocalization;
exportLocalizationVideoData;
```

```sh
uv run --offline --with numpy --with scipy --with h5py python scripts/prepareLocalizationVideoClouds.py --reuse-from output/localization_video_20260916
cp output/localization_video_20260916/orthoimagery.jpg output/localization_video_20260916/imagery_metadata.json output/localization_video_20260917/
uv run --offline --with numpy --with pillow python scripts/renderLocalizationVideo.py --preview-only
uv run --offline --with numpy --with pillow python scripts/renderLocalizationVideo.py
ffmpeg -v error -y -ss 50 -i output/localization_video_20260917/mncav_localization.mp4 -frames:v 1 -update 1 output/localization_video_20260917/decoded_050s.png
mkdir -p output/localization_video_20260917/cache_rebuild_check
cp output/localization_video_20260917/{trajectory.csv,frames.csv,data_metadata.json} output/localization_video_20260917/cache_rebuild_check/
uv run --offline --with numpy --with scipy --with h5py python scripts/prepareLocalizationVideoClouds.py --output output/localization_video_20260917/cache_rebuild_check
uv run --offline --with numpy --with h5py --with pillow python research/localization_video_20260917/verify_video.py
```

Omit `--reuse-from` to rebuild the display cache directly from the existing raw
coordinate and saved semantic datasets. The optional fresh-cache cross-check
was made in `output/localization_video_20260917/cache_rebuild_check`, with its
comparison outcome recorded in `cloud_crosscheck.json`. No algorithm inference
is needed. Run the renderer with `--validate-only` to check an existing MP4.

The new video, previews, decoded sample, frame/state clocks, point cache,
orthophoto, raw display counts and logs remain in the output directory. Compact
metadata, independent checks and artifact hashes are versioned beside this
record; generated videos, images, point caches and datasets are excluded from
the source commit. The previous video's fixed metrics and validation remain
historical and are not overwritten.
