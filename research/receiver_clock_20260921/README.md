# Shared receiver clock implementation and validation

Date: 2026-09-21. Implements the publication-jitter repair from
[the timing design](../frame307_matching_diagnosis_20260921/TIMING_REPAIR_DESIGN.md).
Absolute sensor latency and scan motion compensation remain uncalibrated.

## Implementation

`scripts/receiverClock.py` fits
one robust affine relation, using only paired ROS-header and native GPS
week/seconds timestamps:

```text
receiver_seconds = scale * (header_seconds - source_origin) + offset
```

A deterministic median of long-baseline slopes initializes the fit. Iterative
MAD rejection removes delayed publications before least-squares refinement.
At least 20 increasing pairs, 60% inliers, 90% time coverage and a maximum
2 ms inlier absolute-residual P95 are required. A failed affine-segment check
raises an error; it does not silently stretch time. Clock resets must be
segmented explicitly. Missing samples are allowed. Full GPS weeks are
subtracted separately before seconds-of-week differences, including rollover.
No position, matching error or vehicle trajectory is fitted.

The source-hashed `.clock.json` beside the native INSPVA CSV is the shared
artifact. Python regenerates it when its source or fitting configuration
changes. MATLAB loads the same coefficients through `loadReceiverClock` and
`receiverClockTime`, checks the source SHA-256 and a cross-language binary
coefficient digest, and never fits a second clock. Both implementations reject
nonfinite timestamps and extrapolation beyond a declared 50 ms **header-clock**
edge allowance. Pose interpolation additionally requires native INS coverage.

Mapping pose preparation, native reference and BESTPOS export, motion-input
preparation, calibration input timing, coarse localization replay and current
observer runners use this convention. Pose tables, maps, motion inputs and
matching products carry the model identity. Replay rejects stale or undeclared
identities. The source map converter preserves the identity. Old-time paired
trajectories were removed from the current observer experiment's comparison;
frozen historical experiments remain diagnostic artifacts.

The localization path still uses only whole-pillar coarse perception. No
feature selectors, fine labels, registration objective, observer gains or
active wheel-calibration parameters were retuned. Corrected map poses and
runtime inputs have separate paths, preserving the earlier experiment files.
The independent-drive input calibration exporter was also exercised into a
separate review file; its proposed coefficients were **not** activated.

## Recorded clock evidence

The 5,847 INS pairs give a scale of `1.0909036655077327` and an offset of
`0.00012640098290446608` receiver seconds at the first recorded INS header.
The observed rate relation is not interpreted as physical oscillator drift.
There are 4,395 inliers; their absolute residual P95 is 0.992 ms. Holding out
each of ten contiguous time blocks changes predictions by at most 0.0324 ms
relative to the full fit. These are clock-fit consistency statistics, not a
calibrated bound on absolute measurement time.

All 1,170 frame epochs were regenerated. Frame 307 moves forward by
38.397 ms relative to the old piecewise bridge. The largest correction is
62.098 ms. MATLAB and Python agree within `4.98e-13` seconds when checking
all frame conversions against the exported Python table.

The raw-bag audit independently associates cloud frame 307 with Ouster
hardware scan 10397: 63 nonzero range/ring pairs from column 100 match, versus
1--3 for neighboring scans. Recorded sensor timestamps are approximately
4,882 seconds, while native GPS seconds-of-week are approximately 490,114;
no absolute clock-mode/configuration is inferred from these values. Per-point
`t` spans 0--99.87217 ms. This establishes scan identity and an intra-scan
span, **not** a GPS anchor or the header's start/end offset. The production
repair preserves the declared point-cloud-header epoch instead of inventing
a scan offset. See `raw_packet_epoch.json` and its reproducible audit script.

## Validation results

The saved local feature sets were reprojected with corrected full SE(3) poses
and all 1,170 frames contributed to a rebuilt stability-weighted map. Counts
remain 133,836 curb, 194,300 pole and 70,950 traffic-sign points. The map
publishes 1,320 components. Local-coordinate preservation, map schema and
probability-mass reconstruction checks pass; `map_validation.json` records
the numerical bounds. Mapping algorithms/configuration agree with the
historical solver snapshot apart from provenance and an assertion identifier.

| Experiment | Before | After |
|---|---:|---:|
| Frame 307, historical fine-input D2D, actually rematched on rebuilt map | 57.506 cm | 8.978 cm |
| Same 1,076 accepted frames, historical fine-input D2D XY RMSE | 14.769 cm | 10.130 cm |
| All accepted frames, historical fine-input D2D XY RMSE | 14.863 cm / 1,084 frames | 10.356 cm / 1,108 frames |

Historical controls use the frozen `d6502080` solver and saved configuration,
unchanged fine labels and per-frame reference seeds. They are not production
coarse-only localization or independent ground-truth evaluation. The new
worst accepted fine-input discrepancy is still 60.197 cm at frame 177. Clock
repair does not eliminate correspondence, geometry or initialization errors.

The earlier **rescore-only** control holds old matching poses and the old map
fixed: frame 307 becomes 7.736 cm and overall accepted RMSE becomes 12.439 cm.
That control diagnoses the reference-time effect; its metrics are not the
coherent rebuilt-map result in the table above.

A complete current production-path run also succeeded:

- 1,170/1,170 coarse-only scans processed and accepted; the call trace confirms
  no fine-refinement or saved-feature adapter calls in localization.
- Raw coarse matching: XY RMSE 18.037 cm, P95 40.615 cm, maximum 92.799 cm.
- 1,169 fusion samples: GNSS/BESTPOS + LiDAR XY RMSE 8.768 cm, maximum
  37.566 cm. BESTPOS-only position-channel RMSE is 8.784 cm. The very small
  RMSE difference is not evidence of a substantial fusion improvement.
- All seven declared observer channel-availability scenarios completed.
- Stale maps, stale motion models and undeclared LiDAR-call clocks were
  rejected before matching/replay. The original historical map remains intact.

This is an offline, same-drive map experiment. BESTPOS is INS-aided; INSPVA
is a reference rather than independent surveyed ground truth. The model uses
future timestamp pairs. Constant publication latency, per-sensor latency,
scan-epoch offset, scan deskew and empirical information-matrix calibration
remain outside this repair. Inlier residuals are not added to the estimator
as if they were calibrated absolute timestamp uncertainty.

## Reproduction

From the repository root, generate corrected preparation products:

```bash
uv run --offline --with numpy --with pyproj python scripts/prepareInspvaMappingPoses.py
uv run --offline --with numpy --with pyproj python scripts/prepareInspvaObserverReference.py
uv run --offline --with numpy --with pandas --with pyproj python scripts/prepareMncavBestpos.py
python research/receiver_clock_20260921/validate_clock.py
uv run --offline --with numpy --with rosbags python research/receiver_clock_20260921/audit_raw_packet_epoch.py
```

Then, in MATLAB:

```matlab
setupVehicleLocalization();
rebuildInspvaSavedFeatureMap('output/mississippi_mapping_synchronized');
runMncavCoarseLocalizationExperiment('output/receiver_clock_20260921/coarse_pipeline');
addpath('research/receiver_clock_20260921');
rematch_saved_features(pwd); % Requires the documented existing solver snapshot.
validate_pipeline();
```

Run `validate_clock.py` once more after rematching to refresh the paired-frame
comparison.

Validation: 13 Python tests and 62 MATLAB tests passed. Tests cover delivery
bursts, dropped samples, GPS rollover, clock resets/nonlinear drift, invalid
coefficients, bounded coverage, stale sources, modified coefficients,
MATLAB/Python agreement, pose interpolation, wheel input behavior, planar
motion, frame-aligned replay and temporal-map semantics. Factory Code Analyzer
checks report zero issues in all 18 changed/new MATLAB files examined.
See `python_tests.txt`, `matlab_tests.json` and `code_analysis.json`.

Large corrected datasets, MAT outputs and plots are retained under `data/raw`
and `output`; compact results and reproducible diagnostics are in this folder.
Historical recordings, unrelated `AGENTS.md` changes and the untracked
`reference/` tree are excluded from the project commit.
