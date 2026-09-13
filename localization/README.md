# Continuous localization observer

The global observer integrates the seven states
`[X,Vx,Ax,Y,Vy,Ay,psi]`. It implements the two continuous measurement contracts
in [the ISS derivation](../improved_observer_derivation.md): position-only
GNSS, or uniformly informative LiDAR pose with a known fixed delay. The mode is
fixed for a run. The independent lateral observer supplies `vy`, `beta`, and
`betaDot`; its output can also be supplied explicitly for isolated testing.

The runtime has no measurement-arrival queue, correction pulses, metric
resets, timer-dependent matrices, source fusion, input-flow transport, or
state replay. The vehicle and observer remain continuous systems; the
integration grid is a numerical approximation of their equations.

## Continuous input contract

Call `setupVehicleLocalization` from the repository root. `sensorData.highRate`
contains aligned finite column vectors `time`, `steeringAngle`,
`longitudinalSpeed`, `longitudinalAcceleration`, `lateralAcceleration`, and
`yawRate`. Time increases strictly. Body acceleration is inertial acceleration
after gravity/lever-arm compensation. These arrays define linear numerical
input reconstructions; they are not measurement-arrival events.

The selected source is exactly one of:

* `sensorData.gnss.evaluate(t)`: a pure function returning the current physical
  position `[X;Y]`. No continuous heading output is used.
* `sensorData.lidar.evaluate(t)`: a pure function returning a struct with
  `pose=[X;Y;psi]` and a symmetric physical `information` matrix. At evaluation
  time `t`, the pose describes time `t-d`, where `d=source.delay` equals
  `cfg.measurement.fixedLidarDelay`. Declare `headingConvention="unwrapped"`.
  The runner subtracts its own estimate at **that same past time** and does
  not delay the measurement again.

Providers must meet the continuous-signal contract over the whole interval.
They are queried only at current integration/evaluation times; missing or
nonfinite measurements raise an error. Information bounds are checked at every
evaluated LiDAR point. These finite evaluations cannot establish a bound
between evaluations for an arbitrary function provider.

For an explicitly reconstructed numerical signal, replace `evaluate` with
`representation="piecewiseLinear"`, `time` equal to the high-rate grid, and
`position` (N-by-2) or `pose` (N-by-3). LiDAR also needs a 3-by-3-by-N
`information` array, its fixed `delay`, and the lifted-heading declaration.
Finite aligned arrays have no missing-sample fallback. Interpolation of positive
information matrices preserves their common lower matrix bound. Linear
reconstruction from stored samples can use future endpoints and is an offline
input model, not a claim of physical continuous delivery or online causality.

`reconstructContinuousObserverSignals` is the separate recorded-data adapter.
It takes physical pose timestamps, applies the fixed LiDAR offset once, trims
to covered input times, and explicitly constructs aligned continuous arrays.
It rejects original gaps exceeding its declared interpolation limit (default
0.12 s) and rejects inadequate LiDAR directions. That limit is a reconstruction
quality setting; it is not a timer or a measurement-switching theorem.
Registration records from `localizeLidarFrame` are data products and are not
direct continuous-observer inputs.

## GNSS position mode

```matlab
cfg = improvedObserverConfig("gnss");
design = improvedObserverReferenceDesign(cfg);
data.highRate = highRateInputs;
data.gnss = struct('evaluate', @(t) [8*t;0]);
cfg.observer.initialState = [0;8;0;0;0;0;0];
estimate = runImprovedVehicleObserver(data, lateralDesign, design, cfg);
```

The first six states use the position and three rotational-invariant outputs.
The yaw row uses raw estimated velocity in
`psiHatDot = r_m + kPsi*(VyHat*cos(psiHat+betaHat)-VxHat*sin(psiHat+betaHat))`.
It does not feed heading errors back into the first six states. State and
prediction are never clipped; only the first three auxiliary maps are
extended outside their velocity/acceleration box.

The theorem requires sustained motion and an invariant local heading-error
sector. Standstill runs remain numerically possible, but their diagnostics
report that the motion condition is unmet; the observer cannot infer heading
at rest from position alone. A measured speed check does not prove the true
speed bound or the initial heading-error bound. `initialHeading` is an initial
estimate used if `initialState` is omitted, not another measurement channel.

## LiDAR fixed-delay mode

```matlab
cfg = improvedObserverConfig("lidar");
design = improvedObserverReferenceDesign(cfg);
d = cfg.measurement.fixedLidarDelay;
data = struct('highRate', highRateInputs);
data.lidar = struct('delay', d, 'headingConvention', "unwrapped", ...
    'evaluate', @(t) struct('pose', [8*(t-d);0;0], ...
                           'information', 1e6*eye(3)));
cfg.observer.initialState = [0;8;0;0;0;0;0];
history = @(t) [8*t;8;0;0;0;0;0];
estimate = runImprovedVehicleObserver(data, lateralDesign, design, cfg, ...
    InitialHistory=history);
```

For positive delay, `InitialHistory(t)` must return seven finite states on
`[t0-d,t0]` and match the initial current estimate at `t0`. An old LiDAR pose
cannot silently initialize the current state. Arbitrary continuous estimated
histories are permitted by the theorem; their true error is an initial
condition whose magnitude is not known from sensor data alone.

RK4 uses the method of steps: a numerical step is no longer than the delay,
so every stage can read an already available past estimate. Cubic Hermite
interpolation uses accepted endpoint states and their actual ODE/DDE
slopes. Numerical cuts also respect propagated initial-history derivative
breaks at multiples of the delay. Only a delay-length history bracket is kept;
old outputs are never changed. A zero delay is supported as the current-output
ODE limit, with its matrices reverified for that delay.

Information is normalized with the fixed pose scales `S`:
`J=S'*information*S`, `W=J/(gainInformationScale*I+J)`. The injection is
`T*K*W*(S\measuredPose-S\pastEstimatedPose)`. It retains all translation/yaw
cross terms. No inverse information, artificial eigenvalue floor, or partial
pose substitution is used. Insufficient information raises an explicit error.
All four extended auxiliary outputs remain continuously active in this mode.

## Design and certification

`improvedObserverReferenceDesign(cfg)` loads
[the constant-matrix artifact](../config/continuousObserverCertificate.json)
and recomputes its numerical certificate. `verifyImprovedObserverDesign`
checks actual matrices and gains; it ignores saved `certified` flags.
`designImprovedObserverGains` constructs the GNSS chain gains or synthesizes
constant LiDAR `P,Q,R` for fixed gains using a norm enclosure of the whole
uncertainty family. The older continuous-design names are aliases to these
same entries. There is one runtime and one current certificate implementation.

| Reference mode | Scaling | Course-rate bound | Additional requirement |
|---|---:|---:|---|
| GNSS position | 20 | 0.6 rad/s | Positive speed and admitted local yaw sector |
| LiDAR pose | 1 | 0.00215032 rad/s | 150 ms delay, `0.99852005 I <= W <= I` |

Both use true velocity/acceleration component bounds of 16 m/s and 5 m/s².
The LiDAR artifact is a conservative existence example, not a newly certified
0.6 rad/s vehicle design. Increasing delay or widening its sector requires
successful reverification or synthesis. Actual rate excursions are evaluated
without clamping and are reported as outside the certificate envelope.

`observer.certificateVerified` means the continuous matrix inequalities passed.
`observer.certified` remains false because true-state/noise/heading assumptions
and numerical integration error are not established by the runner.
`diagnostics.certificateConditions` separates coefficient observations from
these unverified hypotheses. A successful simulation is not a full ISS
certificate for a physical sensor implementation.

## Outputs and validation

`z` and `onlineZ` contain the same immutable states with **lifted yaw**.
`pose` and `heading` wrap yaw to `[-pi,pi]`; `headingUnwrapped` retains the lift.
Other outputs include position, velocity, acceleration, speed, lateral inputs,
pose/auxiliary innovations, normalized information weights, and the delayed
states actually used. No residual is wrapped across angle branches internally.

`LateralInputs` optionally supplies aligned `time`, `lateralVelocity`,
`sideSlipAngle`, and `sideSlipAngleRate`; pass `struct()` for the unused
`lateralDesign` when isolating the global stage. If omitted, the real lateral
observer runs once and supplies its interface as before.

```matlab
results = runtests('tests/improvedObserverTest.m');
report = validateStandaloneObserver;
```

The tests compare GNSS yaw with its nonlinear analytic solution and delayed
yaw with the independent MATLAB `dde23` solver, as well as checking the delay
history, information directions, missing measurements, source clocks, unit
normalization, bounded memory, and immutable prefix outputs. The standalone
validation exercises all seven states under analytic motion and continuous
bounded noise. See [the runtime validation](../research/continuous_observer_runtime_20260913/validation.md).

Historical pulse/timer synthesis, transport and outage-diagnostic executables
were removed; their source remains in commit `f14e9a5cab326798d6c247ce3f887ba4a0dff1ae`
and earlier commits. Dated research reports and original experimental results
remain historical evidence. The recorded-data entry now offers only separate
`gnss` and `lidar` reconstructions and retains contract failures as failures.

## Registration helpers

The unchanged registration entry points are `localizeLidarFrame`,
`registerSemanticProbabilityCloud`, and `scoreSemanticProbabilityCloudAlignment`.
Shared methods remain in `registrationSupport`: `projectSemanticProbabilityCloud`,
`prepareSemanticRegistration`, `semanticGaussianOverlap`, and
`registrationPoseMeasurement`. Perception and registration geometry were not
changed by this observer refactor.
