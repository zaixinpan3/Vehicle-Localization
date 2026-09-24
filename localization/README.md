# Continuous localization observer

## Current synchronous runtime

`runFullLocalizationObserver` defaults to one backward-Euler update per common
localization frame. `synchronizeLocalizationInputs` aligns motion/lateral
estimates and BESTPOS positions on native LiDAR times, approximately 10 Hz.
No GNSS/LiDAR pose is propagated between frames and no 100 Hz localization
trajectory is generated. `highRate` remains the compatibility field name, but
contains frame-rate samples after synchronization. The wheel and lateral
estimators retain their separate upstream preprocessing; this is not a claim
that raw sensor acquisition and every estimator run at 10 Hz.

```bash
uv run --offline --with numpy --with pandas --with pyproj python scripts/prepareMncavBestpos.py
uv run --offline --with rosbags --with numpy --with pandas --with pyproj python scripts/calibrateMncavBestposOutputPoint.py
```

```matlab
setupVehicleLocalization;
report = runMncavCoarseLocalizationExperiment;
validation = validateFullLocalizationObserver;
```

The complete replay runs raw scans through `localizeLidarFrame` and its
whole-pillar coarse perception entry only. The existing offline map stays
fixed. Recursive D2D prediction uses four-wheel speed, gyro and lateral
velocity plus accepted matches after a single initial pose. No per-frame
reference pose resets the matcher. `runMncavFullObserverExperiment` can then
rerun observer scenarios from those coarse matching results; it reads their
MAT file to preserve timestamp, pose and information-matrix precision.

The default experiment reads `/novatel/oem7/bestpos` in EPSG:32615 instead of
ODOM XY. The recorded BESTPOS types 54, 55 and 56 are INS-assisted solutions;
this channel must not be called pure GNSS. No ODOM position is read by the
current entry point. INSPVA timestamps locate the common epoch; INSPVA state
is used for evaluation and the existing reference-assisted map, not the
BESTPOS export. `mncavFullObserverConfig` loads a relative output-point offset
calibrated using seconds 1--40 of the separate 12-11-24 drive. Runtime subtracts
the rotated offset using its own estimated heading and adds calibration/heading
uncertainty to the receiver covariance. This empirical planar correction is
not a surveyed physical installation transform.

GNSS alignment uses bounded linear brackets of actual samples, not motion
extrapolation. It records both endpoint timestamps and the required future
sample wait. No interpolation crosses an invalid endpoint or a bracket longer
than 0.15 s. LiDAR poses are unchanged at their native frame times. Motion and
lateral values use the existing 100 Hz preparation, interpolated at the common
frame time. Thus this experiment is offline synchronization; zero LiDAR
processing delay does not mean zero alignment latency.

Each available source must have exactly the same time array as `highRate`.
Invalid or missing source packets withdraw only that frame's channel; older
poses are not retained as substitute measurements. Both sources missing leaves
only state dynamics. Synchronized lateral estimates are required explicitly.
One implicit solve uses each frame interval; `maximumIntegrationStep` is unused
in this mode. MnCAV GNSS position gain is now 4/s, matching the LiDAR gain;
other gains are unchanged. The continuous gain certificate is
reported as context, with `sampledSystemCertified=false` for the complete new
sampled nonlinear implementation. See the
[current calibration, gain and accuracy report](../research/mncav_bestpos_alignment_20260917/README.md)
and the [discrete equations](../research/mncav_synchronous_bestpos_20260917/README.md).

Longitudinal speed remains exclusively wheel derived. The lateral observer
exports its velocity and side slip at the INSPVA output point that the pose
state, map and GNSS correction share: its master state belongs to the IMU
location, and `lateralObserverConfig("mncav").outputPoint.forwardOffsetM`
(2.36 m, `config/mncavMotionOutputPoint.json`, fitted on the separate
12-11-24 drive) transports it by `vyOutput = vyObserver - d*r`. The same
transported velocity drives the source-window odometry. The IMU accelerations
still enter the global observer at the IMU location; their lever-arm terms
(`rDot*d`, `r^2*d`) are not compensated.
`prepareWheelMotionInputs` reads wheel rates, steering and IMU with no alternate
speed fallback. Missing/expired wheel aiding is an error, except the declared
short stationary startup. The latest output directory is
`output/mncav_coarse_localization_20260924b`. On 1170 raw scans, the 1169
observer outputs have fused position RMSE 5.96 cm (5.42 cm after the 2 s
initialization transient) and heading RMSE 0.39 degrees; GNSS-only is 6.33 cm
and LiDAR-only 14.83 cm (14.24 cm after the transient). See
[the canonical map pyramid record](../research/canonical_pyramid_20260924/README.md),
[the output-point transport record](../research/output_point_transport_20260924/README.md)
and, for the earlier 10.3445 cm result without GNSS aiding or the transport,
the [coarse-only replay report](../research/mncav_coarse_localization_20260918/README.md).
The earlier 7.8474 cm experiment used stored fine features and per-frame
reference matching seeds. It is a historical comparison, not an isolated
coarse-versus-fine test. Both experiments use a same-drive map and shared
INSPVA reference, rather than independent absolute ground truth.

## Historical transported full runtime

Saved configurations without `timing`, or explicit
`cfg.timing="historical_transport"`, retain the old 100 Hz/asynchronous replay
for reproducibility. That path propagates the last GNSS/LiDAR anchor between
packets and expires it after 0.2 s. It is no longer the current default.
Its [conditional continuous analysis](../research/full_observer_20260916/design.md)
and [recorded validation](../research/full_observer_20260916/validation.md)
remain historical and do not certify the new frame-rate discretization.

## Historical LiDAR-only motion-aided baseline

For the historical MnCAV experiment with precomputed, zero-delay LiDAR, use
`runMncavMotionAidedExperiment`. Its seven-state motion-aided observer directly
corrects velocity and acceleration with body-frame motion measurements.
LiDAR position residuals correct position without feeding the derivative
states. The nominal physical gains `[kp,kv,ka,kpsi]=[4,4,12,4]` were selected
on the first 60 s, subject to a new common quadratic certificate and four
accuracy constraints. The same frozen gains improve position RMSE, peak,
P95 and heading RMSE against INSPVA over the full and reserved intervals.
The all-reference/all-metric gate remains false because mixed-ODOM full-run
P95 rises by 5.18 mm; this exception is retained in the report.

```matlab
cfg = motionAidedObserverConfig;
design = designMotionAidedObserverGains(cfg);
estimate = runMotionAidedVehicleObserver(data, lateralInputs, cfg);
% Recorded comparison using all existing precomputed frames:
report = runMncavMotionAidedExperiment;
```

This path accepts aligned piecewise-linear LiDAR pose with `delay=0` and
`headingConvention="unwrapped"`. Motion inputs and lateral outputs use the
same grid as the prior zero-delay experiment. Initialize from the first
measurement and measured motion, or supply `cfg.initialState` on that yaw
lift. The runner never receives evaluation references. Geometric weights
come from the full information matrix; its XY block and yaw diagonal are
applied in separate cascade stages, without XY-yaw feedback cross terms.
Qualified knot weights are linearly reconstructed between knots.

See the [derivation and gain design](../research/mncav_motion_aided_20260914/design.md)
and [recorded/synthetic validation](../research/mncav_motion_aided_20260914/validation.md).
This design has its own conditional ISS certificate. The old delayed-HGO
certificate is not reused for it. Positive-delay and GNSS behavior below
continue to use `runImprovedVehicleObserver`.

The global observer integrates the seven states
`[X,Vx,Ax,Y,Vy,Ay,psi]`. It implements the two continuous measurement contracts
in [the ISS derivation](../improved_observer_derivation.md): position-only
GNSS, or uniformly informative LiDAR pose with a known fixed delay. The mode is
fixed for a run. The independent lateral observer supplies `vy`, `beta`, and
`betaDot`; its output can also be supplied explicitly for isolated testing.

The vehicle and observer are continuous systems with constant matrices within
each measurement mode. The integration grid is a numerical approximation of
their equations.

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

For reduced GNSS startup peaking, select the optional constant-gain preset:

```matlab
cfg = improvedObserverConfig("gnss", "lowPeaking");
design = improvedObserverReferenceDesign(cfg);
```

This uses `theta=8`, chain coefficients `[6;8;3]` and `yawGain=0.1`.
The metric is recomputed and verified at the original operating bounds.
The reference profile remains the default. In the same synthetic noisy
sedan trial, the acceleration-error peak falls from 134.17 to 34.60 m/s²,
while sampled settling increases from 0.89 to 3.51 s. Peaks remain substantial;
this preset has a smaller certificate margin and is not an optimum or a
general physical validation. Run `tuneSyntheticObserverPeaking` for the
[parameter comparison](../research/observer_peaking_tuning_20260913/validation.md).

`improvedObserverReferenceDesign(cfg)` loads
[the constant-matrix artifact](../config/continuousObserverCertificate.json)
and recomputes its numerical certificate. `verifyImprovedObserverDesign`
checks actual matrices and gains; it ignores saved `certified` flags.
`designImprovedObserverGains` constructs the GNSS chain gains or synthesizes
constant LiDAR `P,Q,R` for fixed gains using either a norm enclosure or
a four-vertex course-rate enclosure with residual norm bounds. The older continuous-design names are aliases to these
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
demo = demoSyntheticVehicleObserver;
```

`demoSyntheticVehicleObserver` provides a self-contained sedan experiment
using the actual lateral/global cascade, analytic steady-turn truth, incorrect
initial states, and clean/noisy GNSS and delayed LiDAR signals. It displays
all state traces and saves figures and metrics. See the
[synthetic demonstration results](../research/synthetic_vehicle_observer_20260913/validation.md),
including GNSS startup peaking and the LiDAR startup certificate excursion.

The tests compare GNSS yaw with its nonlinear analytic solution and delayed
yaw with the independent MATLAB `dde23` solver, as well as checking the delay
history, information directions, missing measurements, source clocks, unit
normalization, bounded memory, and immutable prefix outputs. The standalone
validation exercises all seven states under analytic motion and continuous
bounded noise. See [the runtime validation](../research/continuous_observer_runtime_20260913/validation.md).

The recorded-data entry offers separate `gnss` and `lidar` reconstructions
and retains contract failures as failures.

## Registration helpers

The unchanged registration entry points are `localizeLidarFrame`,
`registerSemanticProbabilityCloud`, and `scoreSemanticProbabilityCloudAlignment`.
Shared methods remain in `registrationSupport`: `projectSemanticProbabilityCloud`,
`prepareSemanticRegistration`, `semanticGaussianOverlap`, and
`registrationPoseMeasurement`. Perception and registration geometry were not
changed by this observer refactor.

## LiDAR tracking preset

`improvedObserverConfig("lidar","tracking")` selects a verified constant-gain
alternative for the current continuous delayed LiDAR observer. Each Cartesian
chain changes from `[3;3;1]` to `[3;3;1.5]`; theta, N, yaw gain, delay and
operating/information bounds are unchanged. Load it through
`improvedObserverReferenceDesign(cfg)`; stored matrices are reverified.
The default stays `reference`. The preset reduces settled position, velocity
and acceleration error by about 20% on a synthetic variable-speed gentle turn,
with a 36% larger acceleration-error peak. Its narrow course-rate certificate
is unchanged. See the [experiment and tradeoffs](../research/lidar_tracking_tuning_20260914/validation.md)
and `tuneContinuousLidarTracking` for reproduction.

## Target vehicle: UMN MnCAV

Use `mncavVehicleConfig` and `lateralObserverConfig("mncav")` for new
vehicle-dependent work. The central source identifies the 2021 Pacifica Hybrid,
stock mass/geometry/steering ratio, and explicitly unmeasured inertia/stiffness
priors. Re-synthesize lateral gains for this vehicle; the archived reference
sedan gains are not its identified design. `validateMncavObserverParameters`
checks the global LiDAR profiles with motion generated by the nominal MnCAV
bicycle plant and separate dynamic-parameter sensitivities. The global
kinematic equations themselves contain no mass or stiffness parameter.
See [MnCAV parameter provenance and limits](../research/mncav_parameter_validation_20260914/validation.md).
The current `tracking` profile remains provisional: recorded MnCAV course rates
extend far outside its narrow LiDAR certificate. Stock parameter matching
alone does not establish deployment suitability.


## MnCAV LiDAR operating-range preset

Select `improvedObserverConfig("lidar","mncav")` for the new data-informed
course-rate design. It uses theta=2 and a common four-vertex delay certificate
covering |yawRate+sideSlipAngleRate| <= 0.4 rad/s at 150 ms delay. This covers
the existing recorded audit maximum of 0.354975 rad/s, with unchanged stated
information and true-state bounds. The 0.4 bound is a design envelope, not a
physical vehicle limit or a guarantee for every future drive.

The verifier retains q and q^2 explicitly instead of replacing both by one
unstructured disturbance bound. It recomputes all four matrix inequalities;
no runtime turn-rate clamping is introduced. The reference/tracking profiles
remain historical alternatives. Fourteen wider-turn nominal MnCAV simulations
show improved position/velocity/acceleration tracking with larger initial
peaks and slightly worse heading noise response. Use
`validateMncavObserverParameters(folder,SteeringScale=220,Profiles=["tracking","mncav"])`
to reproduce. See the [proof, results and limitations](../research/mncav_lidar_envelope_20260914/validation.md).

## Full recorded MnCAV cascade

`runMncavFullLocalizationExperiment` freshly synthesizes MnCAV lateral and
global gains, runs the actual lateral observer, uses its velocity with
corrected gyro/speed for recursive raw-scan map matching, and executes the
global observer. `auditMncavFullLocalizationExperiment` compares both outputs
at identical scan timestamps and exports actual gains and error traces.

The current full experiment is explicitly offline: accepted pose gaps up to
one second are linearly reconstructed, and information gain-shaping lambda
is set to .001 with fresh verification. Original matching information is
preserved. Recorded gaps require future scans for some reconstructed inputs;
the 150 ms DDE setting is not demonstrated real-time latency. Same-drive map
results show improved heading and worse position RMSE after global fusion.
See the [complete experiment and limitations](../research/mncav_full_localization_20260914/validation.md).

## Precomputed LiDAR with zero processing delay

For the current offline experiment, run `runMncavZeroDelayExperiment`.
It computes and saves all 1,170 raw-frame matching results before starting
the global observer. A separate cache records frame IDs, capture times,
acceptance, original poses and information. Rejected frames have no valid
cached measurement pose; their prediction output is not relabeled as LiDAR.

`reconstructFrameAlignedLidarSignals` merges the original LiDAR timestamps
into the numerical input grid. With `fixedLidarDelay=0`, each accepted frame
supplies its unchanged pose/information at exactly its own capture time.
The existing continuous observer integrates between frames using the declared
offline linear reconstruction of adjacent accepted poses. There is no
instantaneous state reset or 150 ms source shift. All frame times, including
the final fractional motion-grid interval, have explicit output rows.

The entry retains the MnCAV feedback gains and independently verifies the
existing certificate matrices at zero delay. It freshly designs and runs
the lateral observer. Whole-sequence metrics use the original 100 Hz grid
and native scan grid separately, avoiding extra weight for inserted knots.
Both the mixed ODOM reference and the same-receiver INSPVA diagnostic
reference are reported. See the [zero-delay experiment](../research/mncav_zero_delay_20260914/validation.md).
