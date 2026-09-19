# Fresh coarse map-matching-only evaluation

Date: 2026-09-19. Production algorithm baseline:
`b26aad4421e0ba11e6d260c1f177ff884c595691`.

All 1170 Mississippi raw scans were processed twice with the current coarse
perception and geometric distribution-to-distribution matcher. No global
GNSS/LiDAR fusion observer was run. No detector threshold, registration
parameter, map, extrinsic calibration or observer gain was changed.

## Results

The primary result scores the **accepted raw matching measurements** from the
normal recursive pipeline. A rejection produces no pose measurement; its
propagated initial guess is excluded from that primary metric.

| Population | Frames | Horizontal position RMSE | Heading RMSE | Position median | Position P95 | Maximum position error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Recursive, accepted matching poses | 1072 | **19.7132 cm** | **0.589264 deg** | 11.8719 cm | 39.8874 cm | 108.7768 cm |
| Recursive, all outputs including predictions after rejection | 1170 | 22.2628 cm | 0.691024 deg | 12.6557 cm | 47.3914 cm | 108.7768 cm |
| Reference-assisted diagnostic, accepted matching poses | 1071 | 20.5773 cm | 0.627252 deg | 11.9167 cm | 39.4580 cm | 220.8285 cm |

The recursive pipeline accepts 91.62% of scans. It rejects 96 for
`inconsistentClasses` and two for `notConverged`; neither mode emits a
directional-only pose. Among accepted recursive measurements, 40.49% have
position error at most 10 cm, 77.15% at most 20 cm, and 88.90% at most 30 cm.
In reference vehicle axes, forward and lateral RMSE are 14.59 and 13.26 cm;
their mean signed errors are +0.35 and -3.04 cm.

The two modes have 1062 jointly accepted scans. On this identical population,
recursive and reference-assisted position RMSE are **19.5483 and 20.3634 cm**,
respectively. Thus repeatedly resetting the initial guess near the reference
does not remove the roughly 20 cm discrepancy. This supports investigating the
matching measurements themselves, beyond observer fusion weights or accumulated
prediction drift. It does not isolate a particular detector, association,
distribution representation, calibration or optimization defect.

The reference-assisted seed has a deliberate offset; this is not an experiment
starting exactly at the reference or a proof of global optimality. Frame 966
is accepted with a 1.088 m error in recursive mode and 2.208 m in the control.
Frame 840 has approximately 0.902 m accepted error in both modes. Acceptance
and a positive information matrix therefore do not establish small true error.
No high-error accepted scans were discarded from the reported metrics.

## Protocol and interpretation

- `scripts/runMississippiMapMatchingExperiment.m` calls the existing
  `replayMississippiLocalization` for both modes, reading the raw point-cloud
  MAT in blocks and recomputing coarse perception for every scan in each mode.
  `localizeLidarFrame -> perceiveCoarseProbabilityCloud -> perceiveFrame`
  forces whole-pillar coarse mode; no fine point classification is invoked.
- The frozen 1320-component semantic map is
  `output/mississippi_mapping_inspva_20260915/probability_cloud.mat`.
  The matcher uses XY marginals and estimates `[map X, map Y, yaw]`.
- Recursive mode initializes once at reference plus `[0.5 m, -0.4 m, 2 deg]`.
  Later guesses use the previous accepted pose or rejected-frame prediction,
  integrated four-wheel speed, corrected gyro and lateral-observer velocity.
  No subsequent reference XY/yaw enters those recursive guesses. The lateral
  velocity observer supplies motion only; it is not a fused global pose output.
- The diagnostic mode starts every scan at reference plus the same offset.
  It is explicitly reference assisted and is not an autonomous localization
  result. Its rejected outputs are the artificial seeds, not measurements.
- Both modes retain recorded roll/pitch for tilt compensation and use the
  established receiver-time bridge. Reference poses are INSPVA interpolated
  to the LiDAR acquisition time, in the existing recorded INS output-point
  convention. The experiment does not estimate a new lever arm or height.
- All 1170 scans are included, spanning 116.899544 receiver seconds. The final
  scan uses the existing bounded 9.5436 ms terminal motion hold. There is no
  trajectory alignment, fitted time shift, error clipping or warmup exclusion.
- Horizontal RMSE is `sqrt(mean((X-Xref)^2 + (Y-Yref)^2))`; heading RMSE uses
  wrapped yaw differences. Position and heading are separate quantities with
  different units, not a combined three-component Euclidean pose score.
  Table percentiles use MATLAB `prctile`, equivalent to NumPy's Hazen method.
- INSPVA is a recorded reference, not independent absolute ground truth. The
  map was built from this drive and includes query observations. These are
  same-drive map-consistency discrepancies, not held-out-drive accuracy.

## Validation

Both fresh MATLAB batch passes completed successfully. Factory Code Analyzer
reported no findings in the new entry point. The independent Python analysis
reconstructs all four summary metric rows directly from exported poses and
references; maximum discrepancy is 3.52e-9, within the 1e-8 m CSV roundoff
tolerance. The MAT result retains full MATLAB precision.

The audit checks scan coverage, increasing times, accepted information-matrix
positive definiteness, exclusion of rejected guesses from measurement metrics,
and both initial-guess policies. All recursive non-timing CSV fields reproduce
the September 18 raw matching run exactly. That equality is a result of this
fresh execution, not reuse of old poses. Numerical checks and the comparison
on jointly accepted scans are preserved in `independent_checks.json` and
`independent_metrics.csv`. No new unit suite was needed or run for unchanged
production algorithms.

## Reproduction and artifacts

```matlab
setupVehicleLocalization;
maxNumCompThreads(8);
set(groot,'defaultFigureVisible','off');
report = runMississippiMapMatchingExperiment;
```

The driver uses the existing local wheel/IMU/steering exports, vehicle parameter
JSON and saved lateral-observer design named in its source. It invokes neither
map building nor global observer fusion. MATLAB R2026a Update 3 used eight
computational threads. No randomness is introduced.

```bash
uv run --offline --with numpy --with pandas --with matplotlib python research/mississippi_matching_only_20260919/analyze_results.py
```

Full local artifacts are under `output/mississippi_matching_only_20260919/`:

- `recursive/` and `referenceSeed/`: per-frame `calls.csv`, full-precision
  `report.mat`, motion predictions, run metadata and matching summaries.
- `experiment.mat`, `metrics.csv`, `summary.json`, `run.log`: combined result
  and the completed fresh-execution record.
- `matching_errors.png/pdf`: error histories and accepted-measurement empirical
  CDFs; gray markers denote rejected-frame recursive predictions.
- `recursive_largest_accepted_errors.csv` and
  `referenceSeed_largest_accepted_errors.csv`: diagnostic frame lists.
- `code_analyzer.json/log`: validation of the new MATLAB entry point.

This research directory stores the report, numerical summaries, artifact hashes
and independent analysis driver. Recorded datasets, generated MAT files, plots,
native binaries, and unrelated `AGENTS.md`/`reference/` changes are excluded
from the project commit. Existing production configuration remains unchanged.
