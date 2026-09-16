# Synchronized MnCAV localization video

Created September 16, 2026 from the final frozen
`steering_and_lidar_bias / per_frame_zero` replay. The visualization does not
re-run perception, map matching or either observer, and does not change their
outputs. Original experiments remain in place.

## Deliverable and viewing design

`output/localization_video_20260916/mncav_localization.mp4`

- 1920 x 1080, 30 frames/s, H.264, yuv420p, faststart, no audio.
- 3,508 encoded frames; 116.933333 seconds at normal playback speed.
- Source receiver-time duration: 116.899543614589 seconds.
- File size: 54,691,628 bytes.
- SHA-256: `8cdd60663b6e9af2cfe06f733e1cf49782f93ffb4ff8e361e86d265c3bfed366`.

The large geographic panel follows the vehicle on high-resolution USGS/USDA
aerial orthoimagery. Turquoise is INSPVA ground truth; amber is the final
localization estimate. Both past trajectories and current heading icons use
their original coordinates and true map scale. The geographic view is 185 m
wide and includes a 20 m scale and a north-up route overview. The complete
route in the overview is a static locator, not a prediction. Vehicle icons
are illustrative shapes anchored at the recorded output point.

Centimeter-scale discrepancies are naturally almost invisible on a road-scale
map. A separate, explicitly metric offset plot shows the current estimate
relative to INSPVA with 20 cm grid spacing and the last five seconds of error.
The geographic trajectories are not artificially separated or amplified.

The right-hand perspective panel shows the actual saved current-frame curb,
pole and traffic-sign points, with a vehicle-frame 10 m grid and gray raw
LiDAR context. Class colors are orange-red, blue and magenta. Counts are all
saved points in each class for that frame, before viewport clipping. The
gray context alone is restricted to x in [-15,55] m, y in [-32,32] m and
z in [-4,10] m and deterministically thinned to at most 10,000 points.
No semantic feature is reclassified or discarded from the display cache.
Projection/clipping affects visibility, not the saved semantic selection.

The header shows current position/heading discrepancy, speed, frame index,
measurement status and the age of the most recently acquired LiDAR frame.
The bottom chart reveals the original position error through the current
time. Amber bands mark frames without a new full-pose LiDAR measurement;
interpolated poses are not mislabeled as new measurements. Full-replay
statistics and the known same-drive/reference-seeded limitations remain
visible throughout.

## Coordinates, imagery and synchronization

Trajectory and requested image coordinates are EPSG:32615. The renderer uses
the returned image extent, rather than assuming that the image service
preserves the exact requested bounds or pixel aspect. Imagery was retrieved
from the [USGS NAIP Plus ImageServer](https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPPlus/ImageServer),
which describes public-domain aerial orthoimagery from USGS/USDA. The service
copyright/credit text and returned extent are saved in `imagery_metadata.json`.
The image is an aerial orthophoto, not a live satellite image. Its service
refresh date is not claimed to be its acquisition date. It supplies geographic
context only and is not a centimeter-accurate validation reference.

The previously recorded perception video used ROS elapsed time (about 107 s).
This video instead uses the localization experiment's receiver-time clock
(about 117 s). It reads cached points directly and selects the latest actual
LiDAR timestamp at each 30 Hz video frame. No future perception frame is
displayed. The measured maximum source-frame gap is 0.1628547463 s; maximum
displayed frame age is 0.1479332770 s. Frame age is shown explicitly.
All 1,170 source frames appear at least once.

The 30 Hz animation linearly interpolates the existing state export; those
display samples are not new independent localization estimates. Exported
states retain the original combined integration grid of 12,859 timestamps.
The fixed accuracy panel uses the established 11,690-point uniform 100 Hz
population: horizontal RMSE 0.1014023024 m and heading RMSE 0.3477814706 deg.

Cached semantic points are converted back to the calibrated vehicle frame by
the full inverse of their saved INSPVA SE(3) pose. The visualizer does not
infer a new sensor/CG lever arm. All 133,836 curb, 194,300 pole and 70,950
traffic-sign points are retained across the sequence.

## Validation performed

- Full FFmpeg decode completes without errors; ffprobe confirms 3,508 frames,
  1920 x 1080, 30 fps and yuv420p. Start and final source frames are present.
- Independently read the frozen HDF5 experiment, confirming all 12,859
  exported reference/estimate samples. Maximum CSV export difference is
  4.66e-9 m or rad. Original uniform-position/heading metrics reproduce.
- Independently reconstruct video-to-perception frame selection and confirm
  all 1,170 frames are represented, with no future-frame selection.
- The fast Python point export matches an independently completed MATLAB
  export bit-for-bit for all 1,170 displayed clouds, including labels and
  point counts. Original semantic round-trip checks pass.
- Six preview images at 0, 20, 50, 82, 100 and 116.90 s were inspected.
  An actual decoded video frame at 50 s was also inspected; its compression
  PSNR relative to the intended frame is 38.68 dB. This is an encoding check,
  not a localization metric.
- MATLAB's final exporter has zero factory Code Analyzer findings; all three
  Python scripts compile. No estimator regression test is claimed for this
  visualization-only change.

Development failures were resolved and are retained honestly: the initial
MATLAB bulk point export exceeded the MCP response timeout but completed in
the background and supplied the independent point comparison. The final
workflow reads only x/y/z datasets directly from HDF5; full cache preparation
took about 14.25 s in this run. An initial empty-MATLAB-array reader check was
corrected. An initial post-encode check incorrectly assumed a 120 ms maximum
source interval. The measured source interval exceeds that; validation now
checks the actual recorded intervals and exact latest-past-frame selection.
The already correctly encoded video was unchanged and then fully validated.
These timings describe machine processing, not personal work hours.

## Reproduction

From the repository root, export the frozen pose/motion metadata:

```matlab
setupVehicleLocalization;
exportLocalizationVideoData;
```

Then prepare and render the actual point-cloud cache:

```sh
uv run --offline --with numpy --with scipy --with h5py python scripts/prepareLocalizationVideoClouds.py
uv run --offline --with numpy --with pillow python scripts/renderLocalizationVideo.py --preview-only
uv run --offline --with numpy --with pillow python scripts/renderLocalizationVideo.py
uv run --offline --with numpy --with h5py python research/localization_video_20260916/verify_video.py
```

FFmpeg/ffprobe and an Inter or DejaVu Sans font installation are required.
The basemap is downloaded once if no local image/metadata cache exists;
subsequent rendering uses the recorded cache. `--validate-only` checks an
already encoded video. The original optional bulk MATLAB export is retained
as `video_inputs.mat` in the operational/archive outputs for the completed
cross-check, but is not required by the final rendering path.

Operational outputs include the MP4, six full-resolution previews,
`storyboard.jpg`, decoded-frame sample, pose/frame clocks, gray and semantic
display cache, orthophoto/extent metadata, decoder diagnostics and hashes.
Compact validation/provenance files are exported beside this document.
Large generated videos, imagery, point caches and original datasets are
excluded from the public source-code commit. This video illustrates the
existing offline, same-drive, reference-seeded experiment; it does not add
independent-map, online-causal or physical-calibration evidence.
