# Canonical MnCAV defaults: implementation and validation

Prepared September 25, 2026. Current simulation and recorded-experiment entrypoints
use the adopted MnCAV vehicle and steering configuration. No new physical
parameters were identified in this operation. See `config/MNCAV_CONFIGURATION.md`.

## Implementation

- Default `lateralObserverConfig()` selects MnCAV; archived sedan regression
  tests explicitly select `"reference"`.
- MATLAB `mncavReplayConfig` and Python `mncavParameters.replay_parameters`
  resolve current vehicle/steering from versioned configuration. Optional old
  calibration exports can supply IMU corrections only. Fixed raw DBW corrections
  were promoted unchanged from the September 16 interface audit.
- Experiment defaults synthesize matching gains. Vehicle mismatch checks reject
  incompatible saved designs. Cache-dependent comparisons reject obsolete
  vehicle/steering metadata and require upstream regeneration.
- Wheel geometry derives from the same physical configuration. Synthetic truth,
  its derivatives, sideslip and stored lateral truth use the configured output
  point rather than comparing its estimate against the bicycle-origin velocity.
- Historical research outputs remain evidence of their original assumptions.
  They are not current parameter sources. Explicit parameter sweeps remain possible.

## Validation and limits

`tests.csv`, `validation.json`, `synthetic_cascade.json` and
`integration_checks.json` describe the working-tree checks. The working tree also
contains independent, uncommitted observer-synthesis/runtime changes and modified
reference MAT fixtures. Those changes are excluded from this commit.

To separate that context, the staged project tree was exported with `git archive`
to `/tmp/mncav-default-index-check`. The final isolated snapshot is identified in
`isolated_validation.json`; all **90 tests passed**, including the 46 global
observer tests. The six suites were `mncavDefaultsTest`, `mncavVehicleConfigTest`,
`wheelMotionInputTest`, `receiverClockTest`, `lateralObserverTest` and
`improvedObserverTest`. The staged/working SHA-256 comparison in `source_hashes.json`
identifies differing source files; no archive report is hashed. Administrative
agent instructions are omitted from that technical manifest.

The isolated default gain synthesis is certified on its configured design grid
(maximum certificate margin -0.5730526382). Its 24-second nominal lateral
simulation has settled lateral-velocity RMSE 0.0053317561 m/s after 6 seconds.
The earlier working-tree synthesis uses another implementation, giving margin
-0.0519861767 and essentially the same nominal settled RMSE; these margins must
not be treated as results from the same implementation.

The working-tree synthetic cascade ran four deterministic 40-second cases
(GNSS/LiDAR, clean/bounded sinusoidal errors), plus integration refinement checks.
All four practical acceptance checks passed. Noisy position RMSE was 0.050743 m
for GNSS and 0.013568 m for LiDAR. LiDAR runs report rate-envelope violations;
empirical acceptance is not an unconditional physical cascade certificate.
The plant and estimator share nominal dynamics: this is not a mismatch or real
vehicle validation. No RNG was used for these deterministic sinusoidal scenarios.

Default recorded-input preparation used
`output/mncav_wheel_only_20260916/sensors`, no odometry, and produced 11,690 samples.
The default and explicit historical-interface-calibration calls produced exactly
identical high-rate inputs while retaining current Cf and steering zero. A Python
temporary-file check also confirmed that poisoned vehicle/steering fields cannot
override canonical values while an explicit IMU bias override is retained.
No full real-data localization experiment was rerun.

Python compilation passed for the three configuration/calibration scripts. Six
additional cache-consumer scripts had zero factory Code Analyzer findings. Scoped
staged whitespace checks passed. An initial MATLAB run lacked the solver path;
YALMIP and SeDuMi were then explicitly added and all required tests passed. An
isolated summary-export attempt referenced a nonexistent `validation` field;
the export was corrected to use actual `certified` and `maxCertificateMargin`
fields, without changing or rerunning the successful simulation.

## Reproduction

From the repository root, add YALMIP and an SDP solver to the MATLAB path, then:

```matlab
setupVehicleLocalization;
results = runtests({'tests/mncavDefaultsTest.m', ...
    'tests/mncavVehicleConfigTest.m','tests/wheelMotionInputTest.m', ...
    'tests/receiverClockTest.m','tests/lateralObserverTest.m', ...
    'tests/improvedObserverTest.m'});
assert(all([results.Passed]));
design = designLateralObserverGains();
scenario = simulateLateralObserverScenario(design);
report = demoSyntheticVehicleObserver('output/mncav_default_config/synthetic');
```

The final cache-consumer checks and stored synthetic truth-field correction were
reviewed and statically checked after the working-tree cascade run. They do not
alter the numerical cascade metrics. Recorded caches, generated gains, figures,
external solvers, unrelated edits and datasets are excluded from this commit.
