# INSPVA-only Mississippi map rebuild

Date: September 15, 2026. Outcome: the mapping default now uses INSPVA-only
poses, the saved perception was reprojected and the map rebuilt, and all
1,170 frames were rematched in the four existing diagnostic modes.

## Behavior and coordinate design

The previous map registered local points using nearest ODOM poses. That
stream contains previously verified position-source substitutions. The new
`prepareInspvaMappingPoses.py` consumes only LiDAR timestamps and INSPVA,
projects position to EPSG:32615, and linearly interpolates XYZ at LiDAR times.
ROS headers are bridged to receiver time using timestamps alone. There is no
spatial alignment fit, timing-offset fit, extrapolation, or quality-based
frame deletion. All 41 frames whose interpolation brackets include non-good
INS status remain present and retain their bracketing status codes.

Attitude uses shortest-arc quaternion interpolation. Before interpolation,
the body-to-map rotation is

\[
R = R_z(90^\circ-\mathrm{azimuth}+\gamma) R_y(-\mathrm{pitch}) R_x(\mathrm{roll}),
\]

where gamma is UTM meridian convergence. This follows the body-axis conversion
in the [NovAtel driver](https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_driver/src/bestpos_handler.cpp)
with the additional true-north to grid-north correction required by projected
XY. Gamma is -0.1415653 to -0.1369590 degrees on this route. Its sign was
independently checked against a numerically projected northward displacement.
Z consistently uses the [INSPVA ellipsoidal height](https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm).
The deployed driver revision is not inferred from the current source.

New `pose_*` fields explicitly identify pose source, CRS, height datum and
recorded INS output point. They take precedence over historical ODOM fields;
incomplete or invalid explicit poses fail instead of silently using ODOM.
`poseRowToRigidTransform` now supplies both map projection and planar/tilt
factorization. Historical pose tables remain readable for existing artifacts.
`featureMapBuildConfig` selects the new INSPVA table and a distinct output
name; `buildMississippiFeatureMap` can prepare the table through cached
uv/NumPy/pyproj dependencies.

Existing saved perception is already global. For each feature point, the
rebuild applies

\[
p_{\mathrm{local}}=R_{\mathrm{old}}^T(p_{\mathrm{old}}-t_{\mathrm{old}}),\qquad
p_{\mathrm{new}}=R_{\mathrm{INSPVA}}p_{\mathrm{local}}+t_{\mathrm{INSPVA}}.
\]

The original feature selection and LiDAR calibration are preserved. All
399,086 points remain: 133,836 curb, 194,300 pole and 70,950 traffic-sign
observations. The original transform round trip has maximum error zero in
this execution; recovering local coordinates after reprojection differs by
at most 4.6589e-10 m. Replacing only the pose table would not have updated the
stored global features or the map.

The retained calibration does not establish the physical translation between
the stored LiDAR origin and the recorded INS output point. No new lever arm
is invented. The experiment isolates a coherent mapping-pose pipeline; it
is not a new sensor calibration or a centimetre-level accuracy certification.

## Executed experiment

The source cache is
`output/mississippi_perception_video_20260912/feature_observations.mat`.
The historical baseline is `output/saved_perception_baseline_20260914/`.
New artifacts are in `output/mississippi_mapping_inspva_20260915/` and
`output/saved_perception_inspva_20260915/`. Original inputs/maps are retained.
Map inference parameters and registration settings are unchanged. The map
contains 1,320 published components, compared with 1,331 previously. Build
time was 192.607 s; schema, mass and query/export error-bound checks passed.

The same three per-frame diagnostic seeds are used: zero offset and opposite
[0.5 m, -0.4 m, 2 degree] offsets from each map's recorded pose. The fourth
mode initializes once and propagates constant map-frame velocity/yaw rate
from the latest two full LiDAR matches. Directional events do not update
velocity. It has no motion-sensor input, observer or later reference reset.
There are 4,680 registration calls, taking 103.235 s excluding input loading,
artifact persistence, map construction and perception.

The MATLAB MCP wrapper timed out after 300 s for the combined map/matching
call. MATLAB continued to completion; the complete MAT, CSV, JSON and PNG/PDF
outputs were subsequently verified. This was not a terminated or repeated
experiment.

## Full accepted measurements against INSPVA

All discrepancies are planar Euclidean distances. Each population includes
every full accepted measurement in its mode. Rejections retain NaN measurement
coordinates; seed/prediction outputs are reported separately in the full CSV.

| Per-frame seed | Old accepted | New accepted | Old RMSE (cm) | New RMSE (cm) | Old median (cm) | New median (cm) |
|---|---:|---:|---:|---:|---:|---:|
| Zero | 1,092 | 1,084 | 22.7011 | 14.8634 | 7.6768 | 7.8867 |
| Positive | 1,074 | 1,066 | 23.5297 | 17.7102 | 9.7129 | 10.4577 |
| Negative | 1,088 | 1,069 | 25.9859 | 20.7377 | 10.0843 | 10.3202 |

Zero-seed RMSE decreases by about 34.5%, and the maximum decreases from
135.9934 cm to 57.5065 cm. Errors at least 1 m decrease from 22 to zero.
However, the median and full acceptance count do not improve. The new zero
mode has 1,084 full, four directional and 82 rejected frames. Across the
1,066 frames fully accepted by both versions, RMSE is 22.4822 cm before and
14.6871 cm after; the reduction therefore also holds on an unchanged subset.
This common subset is supplemental and does not replace full-population scores.

At frame 1086, the prior match was 130.3360 cm from INSPVA but only 2.8115 cm
from its ODOM mapping pose. The new match is 3.5594 cm from the INSPVA mapping
pose. This resolves the large source-consistency discrepancy at that frame.
It does not prove physical position accuracy of either map. A negative-seed
outlier remains at frame 840 (135.1770 cm), showing that source consistency
alone does not remove all local registration failures.

The new constant-velocity LiDAR-only recursive run still fails: 157 full,
215 directional and 798 rejected frames; all-frame RMSE 302.3995 m and final
error 841.5898 m. The first 0.5 m crossing is frame 88, and the first 10 m
crossing is frame 201. The previous all-frame recursive RMSE was 301.8153 m.
A small median of its early accepted matches is not evidence of successful
whole-drive localization. No improvement in this recursive mode is claimed.

Both maps and query features come from the same drive, and the three seeded
modes use reference-relative initialization. These are map/matching consistency
diagnostics, not independent ground-truth accuracy or full observer results.
The change combines INSPVA source selection, interpolation, grid orientation
and height consistency; their individual causal contributions are not isolated.

## Verification and reproduction

- Four Python unit tests pass: angular wrap, equivalent quaternion signs,
  grid-north direction, and retaining free-navigation brackets while
  interpolating height.
- Seventy MATLAB tests pass across `inspvaMappingPoseTest`,
  `heightProbabilityCloudTest`, `geometricRegistrationTest`,
  `temporalStabilityGmmMapTest` and the data-enabled `pipelineRegressionTest`.
- Factory Code Analyzer reports zero findings across all 13 new/modified
  MATLAB files. The first check reported an unavailable user analyzer
  settings file and automatically used defaults; explicit factory checks
  remove that environment warning without changing user settings.
- Independent CSV analysis verifies all 4,680 ordered calls, reproduces all
  full/all-output RMSE and median values within 1e-9 m, and checks rejection
  NaNs. Source and output hashes are retained in `artifact_manifest.json`.
- Saved configuration comparisons confirm identical temporal-map parameters,
  window length/stride, feature names, calibration, registration parameters
  and moving-cloud parameters (`configuration_checks.json`).
- The new native-CSV clock differs from the previous serialized reference by
  at most 35.117 microseconds, with maximum planar reference difference
  0.2921 mm. The previous ROS timestamp CSV differs by up to 5.007 microseconds.
  An initially overstrict equality assertion failed; these measured
  differences are retained in `pose_checks.json`, not fitted away.
- Full generated pose CSVs, recorded inputs, large MAT outputs and generated
  plots remain outside the public project commit. The research folder retains
  executable analysis, manifests, diagnostic metrics, frame errors and tests.

```bash
uv run --offline --with numpy --with pyproj python scripts/prepareInspvaMappingPoses.py
uv run --offline --with numpy --with pyproj python -m unittest discover -s tests -p test_inspva_pose_preparation.py
```

```matlab
setupVehicleLocalization;
rebuildInspvaSavedFeatureMap('output/mississippi_mapping_inspva_20260915');
runSavedPerceptionMatchingBaseline('output/saved_perception_inspva_20260915', ...
    ObservationFile='output/mississippi_mapping_inspva_20260915/feature_observations.mat', ...
    MapFile='output/mississippi_mapping_inspva_20260915/probability_cloud.mat', ...
    MapRebuilt=true);
```

Run the two MATLAB commands separately when using a tool with a 300-second
request timeout. Afterward, `python research/inspva_mapping_rebuild_20260915/analyze_results.py`
reproduces the full and common-population comparisons from the retained outputs.
