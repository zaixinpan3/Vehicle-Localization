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
