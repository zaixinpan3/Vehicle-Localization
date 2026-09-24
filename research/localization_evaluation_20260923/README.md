# Localization accuracy check and ground-truth leakage audit

Date: 2026-09-23. Code revision `7fc6959fba5c325cf2b40c6652d22a8538bf31ec`
(only `AGENTS.md` was modified in the working tree). No algorithm, configuration
or map was changed. This record reruns the current production localization chain
from raw scans, then audits where the INSPVA evaluation reference enters.

## What was run

1. The 12 localization test suites used by the previous height-association
   record (`runEvalTests.m`): **182/182 passed**, 0 failed, 0 incomplete
   (`tests.csv`).
2. A fresh replay (`runEvalReplay.m`): all 1,170 Mississippi scans were
   reprocessed through whole-pillar coarse perception and the five-scan source
   window (83.9 s). Then `runMncavFullObserverExperiment` was run with closed-loop
   GNSS-aided rematching in all seven sensor scenarios.
3. `analyzeEval.m`: reproducibility, matching-level and fused-level metrics.
4. `leakAudit.m`: an isolated rerun plus perturbations of every runtime input
   derived from the reference.

## Accuracy of the current production chain

The fused poses of all seven scenarios equal the recorded production replay
`output/gnss_aided_matching_20260922/production_fresh` **exactly** (maximum
pose difference 0). All 1,170 regenerated source clouds have XY components
identical to the previous cache.

Errors are against INSPVA at the 1,169 synchronized native LiDAR frame times.
The initial error at frame 1 is the declared 64.03 cm initialization offset.

| Scenario | RMSE (cm) | Median | P95 | Max | RMSE after 2 s | Max after 2 s | Heading RMSE (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|
| GNSS + LiDAR | 8.22 | 5.06 | 16.73 | 64.03 | 7.88 | 23.58 | 0.392 |
| LiDAR only | 24.30 | 15.37 | 52.22 | 75.93 | 24.09 | 75.93 | 0.483 |
| GNSS only | 9.19 | 6.17 | 19.27 | 64.03 | 8.87 | 26.02 | 1.460 |
| GNSS outage 40–60 s | 8.72 | 6.03 | 16.93 | 64.03 | 8.41 | 23.58 | 0.392 |
| LiDAR outage 40–60 s | 8.26 | 5.12 | 16.73 | 64.03 | 7.93 | 23.58 | 0.392 |
| Both outage 40–60 s | 39.52 | 6.69 | 112.65 | 137.96 | 39.80 | 137.96 | 0.383 |
| Alternating 1 s | 19.26 | 14.55 | 35.07 | 64.03 | 19.25 | 57.54 | 0.487 |

Raw LiDAR matching in the fused run accepts 1,167 of 1,169 frames. Frames 1
and 169 are rejected. Accepted-frame position RMSE is 11.51 cm, with median
8.52, P95 21.15, P99 24.88 and a 33.35 cm maximum at frame 840. One frame
exceeds 30 cm. Heading RMSE is 0.402 deg. These matching errors are
conditioned on GNSS hypothesis selection and are not a GNSS-independent
LiDAR trajectory. Observed matching time per call is 12.0 ms median, 15.5 ms
P95 and 112.6 ms maximum. This covers registration only, not perception, and
was measured in a desktop run, so it is not a latency benchmark.

Fusion improves position by 0.98 cm RMSE over GNSS only (8.22 vs 9.19 cm) and
heading by a factor of 3.7 (0.39 vs 1.46 deg). LiDAR alone is 24.3 cm. During a
20 s outage of both sources, dead reckoning drifts to 1.38 m. In the following
10 s recovery window the RMSE is 14.8 cm (median 4.7 cm).

## Where the reference enters

### Online runtime: no reference input

`runSynchronousLocalizationObserver`, `matchLocalProbabilityCloud` and
`updateLocalizationSourceWindow` read no reference field. `leakAudit.m`
reconstructs the fused run in a function workspace that contains only these
inputs:

- `highRate`: time, steering, longitudinal/lateral acceleration, yaw rate and
  four-wheel speed.
- `lateral`: lateral-observer velocity and side-slip rate.
- `gnss`: BESTPOS XY with its information matrix.
- The fixed map and the regenerated source clouds.
- The observer configuration, including its initial state.

The rerun reproduces production exactly (maximum difference 0). The evaluation
columns `referenceX/Y/Psi` in the matching table and the INSPVA trajectory are
not reachable from the matcher or observer. The source window uses only
**relative** wheel/gyro/lateral odometry. The dead-reckoning anchor
`reference(1)+offset` cancels in its relative transforms.

### Declared reference-derived inputs, measured by perturbation

| Input | Source | Perturbation | Fused RMSE | After 2 s | Match RMSE |
|---|---|---|---:|---:|---:|
| Production | ref(1) + [0.5 m, −0.4 m, 2°] initial pose, INS roll/pitch | — | 8.22 | 7.88 | 11.51 |
| Initial XY | reference | first BESTPOS XY (1.81 m off), yaw = ref ±0/3/8° | 10.70–10.73 | 7.88 | 11.49–11.51 |
| Initial XY | reference | ref + (2, −2) m | 13.71 | 7.88 | 11.51 |
| Initial XY | reference | ref + (−3, 3) m | 18.71 | 7.88 | 11.51 |
| Scan tilt | INS roll/pitch per scan | identity (no tilt compensation) | 8.18 | 7.83 | 11.75 |

The initialization only changes the first two seconds. After 2 s, the fused RMSE of
every initialization variant matches production within 0.001 mm, and the
post-start maximum is identical. The full-run RMSE
increases only because the start-up transient is included. Removing the INS
roll/pitch from perception changes matching RMSE by +0.24 cm and fused RMSE by
−0.04 cm. Neither runtime use materially helps the reported accuracy. Yaw was
never taken from GNSS: the vehicle starts stationary, and BESTPOS gives no
heading there. A reference-free initial heading therefore remains untested.

### Offline uses that the runtime cannot remove

These uses are declared in earlier records. Together they make the result a
**same-drive consistency test, not independent absolute accuracy**:

1. **The map was built from the same 1,170 scans, placed at their INSPVA poses**
   (`output/mississippi_mapping_calibrated/publication.json`: `poseSource`
   INSPVA, `frames` 1170). Each query scan's own features helped form the map it
   is matched against. This is the dominant limitation. Map-matching errors
   therefore measure the ability to recover the INSPVA-registered trajectory.
   Performance on a different drive or on a map built from a separate pass has
   not been measured.
2. **The LiDAR origin translation `[2.268, 0.154]` m** was fitted on this
   evaluation drive against the INS reference. It was selected after two
   complete 1,170-frame comparisons for lower RMSE on this sequence
   (`config/mississippiLidarFrameCalibration.json`, `evaluationDriveUsed:
   true`). The independent-drive fit differs: `[3.29, −0.21]` m.
3. **The observer gains `[4, 4, 12, 4]`** were selected on t ≤ 60 s of the
   recorded drive against the reference (`config/motionAidedObserverConfig.m`).
4. **BESTPOS is INS-aided output from the same NovAtel receiver** that produces
   the INSPVA reference. It is not reference data, but its errors are not
   independent of the reference.
5. INSPVA timestamps define the shared receiver clock. This affects timing only.

Wheel radius and lag, CAN sign/offset and the BESTPOS output point were
calibrated only on the separate 12-11-24 drive
(`evaluationDriveUsed: false`).

## Conclusion

No online path reads the ground truth. The two runtime quantities derived from
it, the initial pose and the scan tilt, were perturbed or removed. After the
first two seconds they leave the result unchanged or change it by at most a
quarter centimeter. The 7.9–8.2 cm fusion accuracy is nonetheless optimistic as
an absolute figure. The map comes from the same scans at their INSPVA poses,
and the LiDAR origin and observer gains were tuned on this drive. The next
informative test would build the map from one drive (for example 12-11-24) and
localize on the other.

## Reproduction

From the repository root, with `VEHICLE_LOCALIZATION_DATA_ROOT=<repo>/data`:

```matlab
run('research/localization_evaluation_20260923/runEvalTests.m');
run('research/localization_evaluation_20260923/runEvalReplay.m');   % writes output/localization_evaluation_20260923
run('research/localization_evaluation_20260923/analyzeEval.m');
addpath('research/localization_evaluation_20260923'); leakAudit;
```

`analyzeEval.m` and `leakAudit.m` write their CSVs to
`output/localization_evaluation_20260923`. Copies are kept here as
`scenario_summary.csv`, `frame_errors.csv` and `leak_audit.csv`.
`observer_metrics.csv` and `comparison.png` are the experiment's own outputs.
Large caches (`sources.mat`, `observer/experiment.mat`) remain under `output/`.
MATLAB R2026a.
