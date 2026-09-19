# Raw coarse-perception localization replay

Date: 2026-09-18. Perception source: `6719eb87967d0f93d9dd8276a941b6eaff2f88af`.
This experiment uses the optimized whole-pillar perception implementation,
the existing INSPVA-aligned semantic map, and the current synchronous observer.
No detector threshold, map, observer gain or receiver calibration was retuned.

## Results

All 1170 Mississippi raw scans were processed. D2D accepted 1072 full poses
(91.62%), rejected 96 for class inconsistency and two for nonconvergence, and
accepted no directional-only poses. The observer has 1169 outputs and 1071
accepted LiDAR measurements: the final scan lies 9.54 ms beyond the prepared
motion interval. Matching permits a bounded terminal motion hold; observer
synchronization requires covered motion samples and excludes that last scan.

| Observer inputs | Position RMSE | Position median | Position P95 | Heading RMSE |
| --- | ---: | ---: | ---: | ---: |
| BESTPOS + coarse LiDAR | 10.3445 cm | 6.5768 cm | 21.0695 cm | 0.7062 degrees |
| Coarse LiDAR only | 20.1626 cm | 9.9146 cm | 45.7500 cm | 0.5895 degrees |
| BESTPOS only | 8.8252 cm | 5.7051 cm | 19.1080 cm | 1.4038 degrees |

The fused maximum position error is 44.28 cm, and 70.74% of outputs are within
10 cm of the shared INSPVA reference. Fusion improves heading relative to
BESTPOS-only operation but does **not** improve its position RMSE in this run.
Both ablations include the same wheel/IMU/lateral dynamics and common first
LiDAR initialization; BESTPOS-only is not an independent cold-start test.

The accepted raw D2D measurements have position RMSE **19.71 cm**. Including
motion predictions on rejection gives a recursive matching trajectory RMSE of
22.26 cm. These are different populations from fused observer outputs. The
matching-only all-frame heading RMSE is 0.6910 degrees. Every accepted full
measurement has a positive-definite information matrix; the smallest eigenvalue
across all accepted matrices is 0.544315 in physical `[X, Y, yaw]` coordinates.
The matrix is the registration model's information, not a calibrated inverse
empirical error covariance.

Seven observer scenarios were evaluated: both sources, each source alone,
each source withdrawn during 40--60 s, both withdrawn, and alternating source
availability. All 21 full/outage/recovery metric rows are in `metrics.csv`.
With both sources withdrawn, full-run position RMSE rises to 55.11 cm. These
withdrawals operate on the recorded observer inputs; they are not reruns of a
closed-loop sensor/matcher system under each withdrawal.

## Online computation

| Stage or statistic | Milliseconds |
| --- | ---: |
| Coarse perception median | 55.5235 |
| Registration median | 6.3605 |
| Map selection + coarse perception + registration + event median | 62.1060 |
| Combined P95 | 72.2666 |
| Combined P99 | 80.8992 |
| Combined maximum | 325.4730 |

Five frames exceed 100 ms. No warmup frames were discarded. Timing excludes
block disk reads, upstream sensor preparation, observer integration and
visualization. MATLAB R2026a Update 3 used eight computational threads and the
version-3 native perception kernels; no other experiment ran during matching.
These measurements do not establish a hard end-to-end 10 Hz guarantee.
The observer still assumes zero LiDAR processing delay and uses offline GNSS
bracket synchronization, whose future-endpoint wait is reported separately.

## Executed pipeline and evaluation boundary

1. Read raw `data/raw/MissisipiPointClouds.mat`, frames 1--1170.
2. Run `localizeLidarFrame -> perceiveCoarseProbabilityCloud -> perceiveFrame`.
   The wrapper forces `coarseProbabilityCloud` mode. Whole XY pillars retain
   their point-distribution statistics. No height subdivision, ring-based
   semantic detector or point-level fine refinement is used.
3. Match curb, pole and traffic-sign Gaussian distributions against the fixed
   1320-component map at
   `output/mississippi_mapping_inspva_20260915/probability_cloud.mat`.
   This map was previously built offline from fine perception and is not
   rebuilt by this experiment.
4. Initialize the first pose with reference plus `[0.5 m, -0.4 m, 2 degrees]`.
   Subsequently predict from recorded four-wheel speed, corrected gyro and
   lateral-observer velocity, then correct using accepted D2D measurements.
   No per-frame reference position/yaw reset occurs. Recorded roll/pitch
   supplies known tilt, while receiver time supplies the acquisition clock.
5. Read full-precision matching MAT results into the current observer. Invalid
   full-pose measurements are NaN and marked unavailable. The complete physical
   3-by-3 information matrix accompanies each accepted pose.
6. Fuse with recorded BESTPOS XY using the existing separate-drive output-point
   calibration and unchanged gains; score at native LiDAR timestamps.

The earlier 7.8474 cm fused result used saved fine features **and per-frame
reference matching seeds**. The new 10.3445 cm result changes both the moving
representation and prediction policy, so their difference cannot be attributed
solely to coarse perception or its efficiency optimization. That optimization
separately preserved all 1170 coarse outputs exactly against its own baseline.
Both localization evaluations are same-drive map consistency checks, not
independent absolute-accuracy tests. The map includes query observations and
the BESTPOS solutions are INS aided and share the receiver with INSPVA.

## Validation and implementation

- An independent profiled frame-1 call confirmed the coarse entry executed and
  no `refinePerceptionCandidates` call occurred. Profiling was disabled before
  sequence timing. The same forced-mode entry processes every scan.
- Independent Python recomputation verified all 21 observer metric rows,
  with maximum numerical discrepancy below 5e-15. It also checked all accepted
  information matrices, rejected payloads and recursive predictions from
  saved wheel/gyro motion, rather than evaluation poses.
- The observer has one update per covered frame, no virtual pose updates,
  no integration substeps and no state resets. Repeating the fused observer
  gives exactly the same state history.
- 48 existing MATLAB tests passed across full-observer source availability,
  synchronization and registration-information suites. Factory Code Analyzer
  reported no findings in the four changed/new MATLAB scripts.
- The observer entry now accepts only fresh coarse-replay results. The former
  default saved-fine measurement path was removed. `MatchingFolder` selects
  the result directory, and the full experiment entry regenerates measurements
  from raw scans before observer evaluation.
- During input integration, CSV epoch timestamps lost up to 5.01 microseconds
  through decimal formatting. MAT is now authoritative for matching values;
  old exported clocks are compared with a strict 1e-10 s tolerance. The old
  and native relative clocks differ by at most 4.98e-13 s. Other fixed input
  differences are limited to interpolation roundoff, reported in
  `independent_checks.json`. Historical files were preserved.

The MATLAB MCP test transport returned EOF, so the same existing suites were
executed successfully with `matlab -batch`. Initial input-adapter assertions
stopped only observer postprocessing; the completed raw matching run was
retained and consumed after the precision fixes.

## Reproduction and artifacts

```matlab
setupVehicleLocalization;
maxNumCompThreads(8);
report = runMncavCoarseLocalizationExperiment;
% Observer-only replay of those fresh coarse measurements:
observer = runMncavFullObserverExperiment;
```

The full entry uses existing local wheel/IMU/steering exports, calibrated vehicle
parameters, lateral-observer design, BESTPOS export and semantic map. Paths are
declared in its source; none of these artifacts is regenerated from fine
features during localization.

```bash
uv run --offline --with numpy --with pandas --with h5py python research/mncav_coarse_localization_20260918/verify_results.py
```

The main output is `output/mncav_coarse_localization_20260918/`:

- `matching/calls.csv` and `matching/report.mat`: every raw scan, prediction,
  accepted/rejected pose, information entries, errors and stage latency.
  CSV is an inspection export; use MAT for full numerical precision.
- `observer/experiment.mat`, `observer/metrics.csv`, `trajectory.csv`: aligned
  sources, all scenario states, error metrics and the fused trajectory.
- `localization_errors.png/pdf`: position and heading comparison, rendered by
  `plot_results.py` from the recorded states without rerunning estimation.
  `matching/replay.png` contains the recursive trajectory and timing plots.
- `call_path_audit.mat/json`, `tests.mat/csv`, `validation.json`,
  `code_analyzer.csv`: execution and validation records.

This directory preserves compact numerical exports and the independent audit
script. Raw recordings, maps, generated MAT files and compiled binaries remain
outside the source commit. The previous localization video is unchanged.
