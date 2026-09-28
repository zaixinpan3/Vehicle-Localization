# Full Mississippi coarse map matching, 2026-09-28

All **1,170 raw scans** were processed in each of two modes against the existing
complete calibrated map (**1320 components**). Current 0.6 m whole-pillar
perception was rerun for curb, pole and traffic signs on every query. This study
changes no production algorithm or parameter. Downtown was not run.

| Measure | Recursive (primary) | Reference-seeded diagnostic |
|---|---:|---:|
| Full-pose matches | 1150/1170 | 1150/1170 |
| Directional-only updates | 19 | 12 |
| Prediction-only outputs | 1 | 8 |
| All-output horizontal RMSE | 16.609 cm | 17.828 cm |
| Full-pose-only horizontal RMSE | 16.297 cm | 16.282 cm |
| All-output horizontal median | 10.858 cm | 10.964 cm |
| All-output horizontal P95 (linear) | 33.733 cm | 38.455 cm |
| Maximum horizontal error | 85.159 cm | 85.159 cm |
| All-output heading RMSE | 0.6241 deg | 0.6366 deg |
| Median computation | 143.668 ms | 143.898 ms |
| P95 computation | 179.943 ms | 179.934 ms |
| Calls exceeding 100 ms | 1144/1170 | 1149/1170 |

The reference trajectory spans 116.900 receiver seconds
and approximately 1276.2 m of accumulated XY travel. Errors use synchronized
recorded INSPVA at the LiDAR acquisition epoch and recorded INS output point.
No trajectory alignment, fitted time shift, clipping or warmup exclusion is used.
Directional-only outputs are separately scored in `independent_metrics.csv`;
they are neither complete pose measurements nor pure prediction on rejection.
In the recursive run, the only prediction-only frame is frame 1, before the source window has two
observations confirming features. This is expected startup behavior.

The largest recursive position error is frame 601:
0.851587991 m, versus 0.851587981 m
with reference-assisted initialization. This peak is accepted by the matcher.
The nearly identical peak demonstrates that cumulative recursive drift alone
does not explain it. Local feature/map consistency and association need further
investigation; this experiment does not identify the physical cause or retune it.
Accepted matching is an algorithmic gate, not a guarantee of physical correctness.

## Protocol and practical limits

- Main mode starts at reference + [0.5 m, -0.4 m, 2 degrees] once. Later predictions
  compose previous outputs with independent four-wheel speed, corrected gyro and
  current lateral-observer motion. No position/yaw reference resets or global
  GNSS/LiDAR fusion are used. Recorded INS tilt is supplied throughout.
- Diagnostic mode resets the same reference offset every scan. It diagnoses local
  convergence; it is not autonomous localization performance.
- Existing calibrated map: `output/mississippi_mapping_calibrated/probability_cloud.mat`.
  Full-map coverage is used via a 100 m local crop around the predicted pose;
  matching is local SE(2), not a global lost-position search. The map was not rebuilt.
- Source support uses at most five scans/0.45 s, requiring two observations.
  Current geometric D2D uses the canonical pyramid and 0.15 m refinement trust radius.
  Complete configurations are retained in the local per-mode `report.mat` files.
- Current MnCAV gains were synthesized fresh. The working tree already contained
  observer/configuration changes; these were executed as found, hashed before and
  after, and preserved without including them in this study's commit. Therefore
  this is not an isolated before/after test of perception alone. The local
  `working_tree_before.patch` records those source changes; its hash is in the input
  manifest. Reproduction from the base commit alone does not reconstruct that state.
- The map includes these query observations. Perception models and empirical
  calibration were also developed with this drive. These are **same-drive consistency
  results**, not independent physical ground truth or new-route generalization.
- Recursive perception median is 131.635 ms; source-window plus
  registration median is 11.431 ms. The median total is
  143.668 ms (roughly 6.96 Hz reciprocal,
  not sustained throughput). Current execution does not meet the 10 Hz deadline.
  All calls, including a 2409.342 ms cold maximum, are retained.
  Timings exclude disk read, map preparation, gain synthesis and motion preparation;
  include current coarse perception, source window, registration, map crop and event.
  Timing results come from sequential runs on this workstation, not a paired
  speed comparison or an isolated real-time scheduler benchmark.

## Reproduction and validation

From the repository root:

```sh
matlab -batch "addpath('research/mississippi_matching_20260928'); runStudy;"
uv run --offline --with numpy --with pandas --with matplotlib python research/mississippi_matching_20260928/analyze_results.py
```

The runner refuses to overwrite a completed `experiment.mat`. It loads installed
YALMIP and SeDuMi from the sibling `RobustVehicleLocalization/external` directory.
The initial missing-solver-path failure is retained as `initial_missing_solver.log`
in the output directory; the corrected complete run is `run.log`. No solver binary
or dependency is copied into this repository. Input and executed-source hashes
are in `input_manifest.json`, `source_manifest.json` and `artifact_hashes.json`.

Independent analysis checks all 1,170 ordered frames in both modes, monotone
receiver epochs, fresh-perception metadata, per-frame XY/wrapped-yaw residuals,
all four MATLAB metric rows, positive-definite information for full-pose matches,
recursive seed propagation from previous outputs and motion, and prediction-only
outputs after rejection. Maximum aggregate metric discrepancy is
3.3e-09. Recorded production-source hashes are unchanged
throughout replay. Metric CSV P95 follows MATLAB/Hazen; per-mode summary P95 and
timing CSV use linear interpolation and are labeled accordingly.

`code_analysis.json` records factory MATLAB analysis of the new runner; Python
byte compilation and scoped Git whitespace checks also passed. No production code
was changed, so no additional unit-test pass is claimed by this study.

## Outputs

- `recursive/` and `referenceSeed/`: complete calls, source-window diagnostics,
  dead-reckoning traces, metadata and summaries.
- `independent_metrics.csv`, `independent_checks.json`, `timing.csv`: independently
  recomputed error populations, checks and complete timing distributions.
- `recursive_replay.png` / `.pdf`: trajectory, errors, partial updates and runtime.
- `matching_errors.png` / `.pdf`: complete-mode error comparison and accepted CDF.
- Full MAT results, logs, candidate poses, disk blocks and the excluded source patch
  stay under `output/mississippi_matching_20260928/`; raw data and binaries are not
  added to Git. Original MATLAB plots remain local; the research plots distinguish
  directional updates from rejected measurements and use readable light styling.
