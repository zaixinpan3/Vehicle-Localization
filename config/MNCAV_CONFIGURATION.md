# Default vehicle for simulations and experiments

All current simulations, vehicle experiments and replay preparation use
`config/mncavVehicleParameters.json` as the nominal vehicle source. Steering
conversion uses `config/mncavReplayInterface.json`. Do not copy these constants
into a script, use an old `output/*/vehicle_parameters.json` as the nominal
vehicle, or reuse another vehicle's saved gains.

- `mncavVehicleConfig()` loads the current physical parameters.
- `lateralObserverConfig()` and `lateralObserverConfig("mncav")` select the same
  current vehicle and configured lateral-output point.
- `mncavReplayConfig()` adds canonical steering conversion and the fixed DBW
  sensor corrections in `mncavInputCorrections.json`.
- `mncavReplayConfig(calibrationFile)` optionally uses that file's
  `input_correction` only. Vehicle and steering always come from current config.
  Python scripts use `mncavParameters.replay_parameters` with the same rule.
- `wheelSpeedObserverConfig()` reads geometry from the vehicle configuration;
  radii and equivalent lag corrections stay in `mncavWheelSpeedCalibration.json`.

A simulation's raw steering command is already a road-wheel angle; do not
subtract the recorded steering-wheel zero twice. Replay preparation converts
`(rawSteeringWheelAngle-steeringWheelOffsetRad)/steeringRatio` once.

The default lateral LPV speed domain is `[1,30] m/s`, using exponential
midpoint integration. The previous `[5,30] m/s` default is retired. Below
1 m/s or above 30 m/s, lateral velocity aiding remains unavailable; this
change does not extend the LPV certificate to standstill.

Generate gains with `designLateralObserverGains()` or an explicitly supplied
configuration. Lateral simulation/execution rejects a design whose stored
vehicle differs from the requested configuration. Default experiment scripts
synthesize current gains instead of loading reference MAT gains. YALMIP and the
configured SDP solver must be available. No old gains are silently relabeled
as certificates for the new parameters.

Experiments that depend on frozen input/matching caches must rebuild the
upstream replay after vehicle or steering changes. A cache is evidence of the
configuration used to create it, not an alternative default. Cache-consuming
motion comparison entrypoints validate its vehicle and steering metadata before
execution. Do not overwrite old experimental outputs merely to make checks pass.

MnCAV is the only supported vehicle profile. The old `"reference"` profile
and both obsolete saved lateral-design MAT files have been removed. Tests
synthesize the current design, so YALMIP and an SDP solver are required for
lateral runtime tests as well as synthesis tests. Parameter sweeps may explicitly
modify a configuration derived from the default; record those overrides and
synthesize matching gains. Versioned historical research artifacts retain their
originally recorded assumptions and results; reproduce those at their recorded
Git revision. They are not live configuration sources.

Synthetic truth must be expressed at the configured output point:
`vy_output=vy_origin-forwardOffsetM*yawRate`, with the corresponding derivative
and sideslip transformation. Plant states and IMU inputs remain at their
physical model origin. Otherwise nominal simulation can report a false lateral
error merely because it compares velocities at different points.

The current global observer factory is `mncavFullObserverConfig()`. It returns
parameters without numerical gains. After configuring all experiment inputs,
call `[design,cfg]=synthesizeMncavInvariantDesign(cfg)` and use that temporary
configuration for the intrinsic eight-state observer. Every new test/experiment
solves again; the script neither loads nor saves gains, and the former active
gain JSON has been removed. Runtime can verify an in-memory design without a
solver, but rejects changes to its synthesis inputs until gains are re-solved.
Tests share fresh designs only within their fixture lifetime. The former main-observer
factories and saved measured-course gains were
removed on 2026-10-13; retrieve their historical code from Git if needed.
The lateral vehicle parameters and their synthesis workflow are unchanged.

The global ISS speed domain is the true course-speed interval
`[invariantObserver.redundancy.minimumCourseSpeed, iss.maximumSpeed]`, with a
strictly positive lower bound, for every L/G/M sensor family. The ordinary
configuration uses [10,15] m/s. This is separate from the lateral LPV
longitudinal-speed interval. The main runtime has no speed denominator and can
execute at rest, but reports no ISS certification there. Its speed-squared
output is active in matched feedback; the projected-speed replacement and
scalar speed-gain synthesis have been removed. This architecture change
invalidates earlier gains and requires a fresh solve.

## Temporary small-offset global design

`mncavSmallOffsetObserverConfig()` selects an explicit design scenario with
a 0.01 m GNSS measurement-point offset and global course-speed bounds
`[1,30] m/s`. The assumed offset keeps the measured forward/left direction.
The measured calibration in `mncavBestposOutputPoint.json` and the ordinary
`mncavFullObserverConfig()` remain unchanged. This scenario changes geometry;
it does not reduce GNSS noise or the declared alignment uncertainty.

The scenario uses a 1 m heading length instead of 10 m, including the
corresponding LiDAR pose-information normalization. Its normalized information
floors and coupling bounds therefore refer to that new scale. The L-position
feedback cap is 16 instead of 8; the G-position, velocity-direction and heading
caps retain their ordinary values. The nonlinear overlap mismatch bounds,
vehicle parameters and observer equations are unchanged.

```matlab
cfg = mncavSmallOffsetObserverConfig();
[design, cfg] = synthesizeMncavInvariantDesign(cfg);
```

The result must pass all L, G and coupling-preserving Mixed constraints and
the conditional regional ISS verification. Gains are returned in memory and
must be solved again for the next experiment. Changing the offset or domain
invalidates the temporary design. `GnssOffsetNormM` is an explicit geometry
override, not a promise of feasibility at every requested value.

Synthetic sensor outputs must use the assumed measurement point. Applying
this scenario directly to native BESTPOS recordings with the measured
approximately 1.75 m offset would use the wrong geometry. It is not a new
calibration of those recordings. The associated sampled tests use ideal
motion inputs and ideal pose measurements at the assumed point; they do not
qualify a real overlap map or establish a discrete-time ISS theorem.
