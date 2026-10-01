# Lateral observer performance: simulation and recorded drives

Evaluation date: 2026-10-01. MATLAB R2026a, YALMIP + SeDuMi. The evaluated
code is the **working tree** on top of commit `a9d1642`: the lateral observer
sources, `config/lateralObserverConfig.m` and `tests/lateralObserverTest.m`
carry uncommitted ISS-synthesis changes that this task did not author and did
not modify. `input_hashes.csv` pins every evaluated source, configuration and
sensor input by SHA-256. No gain, parameter or calibration was changed.

## Summary

1. **The synthesized LPV gain is numerically zero.** With the configured
   decay rate of 2 1/s the minimum-gain synthesis returns vertex gains of norm
   5.5e-10. The hidden LPV branch is therefore the open-loop bicycle model
   driven by steering and speed; the IMU enters only through the master
   kinematic state and the output-point transport.
2. On matched-model simulation the estimate is accurate: 0.0053 m/s settled
   RMSE under nominal noise (1.6 % of the signal RMS), and a 1 m/s initial
   error falls below 0.05 m/s in 0.37–0.68 s.
3. Robustness to model error is that of an open-loop model: rear cornering
   stiffness at 70 % gives 0.23 m/s RMSE (36 % of the signal).
4. On the recorded drives the observer is indistinguishable from the
   open-loop model: 0.046 m/s RMSE on 12-11-24 and 0.251 m/s on 12-09-31,
   where always predicting zero scores 0.252 m/s.
5. Raising the decay rate to obtain nonzero gains changes the recorded RMSE
   by less than 0.002 m/s, so the 12-09-31 discrepancy is not a gain problem.

## Reproduction

```matlab
addpath('~/MATLAB/toolboxes/sedumi'); addpath(genpath('~/MATLAB/toolboxes/YALMIP'));
setupVehicleLocalization;
addpath('research/lateral_observer_performance_20261001');
report = evaluateLateralObserverPerformance();   % about 2.5 minutes
results = runtests('tests/lateralObserverTest.m');
```

Full traces are saved to the ignored
`output/lateral_observer_performance_20261001/experiment.mat`, together with
the run log, the starting Git status and `tested_source.patch` (the working-tree
diff of the evaluated sources against `a9d1642`). The recorded replay needs the
local sensor exports listed in `input_hashes.csv`; a clean checkout alone is
not sufficient. The Code Analyzer reports zero findings for the evaluator.

## Design and certificate

| Quantity | Value |
|---|---:|
| Selected tau / Finsler slack scale | 0.1 / 0.003 |
| Certified decay rate | 2 1/s |
| Minimized gain bound | 9.46e-11 |
| Largest vertex-gain norm | 5.48e-10 |
| ISS gain from measurement noise | 3.43e-9 |
| Worst Psi eigenvalue, 3,003-point dense grid | -0.0520 (no positive point) |
| Minimum eigenvalue of P on the dense grid | 1.424 |
| Slowest frozen error pole (30 m/s) | -4.3295 1/s |

The certificate holds on 1,001 speeds from 5 to 30 m/s at accelerations -3, 0
and +3 m/s², evaluated independently of the synthesis. A dense grid is not a
proof between samples, and the certificate concerns the nominal LPV branch
only, not the hybrid master, discretization, or model error.

The gain is zero because the requirement is already met without feedback. The
open-loop bicycle poles have real parts from -22.7 1/s at 5 m/s to -4.33 1/s
at 30 m/s (`gain_schedule.csv`), which exceeds the requested 2 1/s including
the Lipschitz margin, so the smallest certifying gain is none. Closed-loop and
open-loop frozen poles agree to nine digits. The fifteen candidate pairs with
tau <= 0.1 all return bounds of order 1e-9 or smaller, so the selected pair is
decided by solver noise.

Unit tests: **17 of 18 passed** (`unit_tests.csv`).
`currentDesignSatisfiesTheOriginalCertificate` fails at
`tests/lateralObserverTest.m:129`: the largest vertex-gain norm 5.48e-10
exceeds the minimized bound 9.46e-11. Both numbers are solver noise around
zero; the failure is a symptom of the zero-gain optimum, not of a violated
certificate.

## Simulation method

Truth comes from an evaluator-local bicycle plant integrated with ten RK4
substeps per 10 ms sample, not from the observer's model code. With linear
tires it reproduces `simulateLateralObserverScenario` to 1.4e-15 in truth,
measurements and estimate (`harness_check.csv`). The plant can differ from
the observer model in mass, yaw inertia, axle stiffness and CG location, can
saturate each axle force as `Fmax*tanh(C*alpha/Fmax)` with the static axle
load, and switches to the no-slip kinematic state below 0.3 m/s. Truth is
scored at the configured output point (2.36 m behind the observer point).

Measurements at 100 Hz use white noise of sigma 0.05 m/s² (lateral
acceleration) and 0.002 rad/s (yaw rate), the configured priors. The initial
error is [0.5 m/s, 0.05 rad/s] unless stated. Settled metrics use t >= 6 s.
The campaign has 105 runs over 44 scenarios; every run stayed finite.
Stress magnitudes follow the sensitivity entries of
`mncavVehicleParameters.json` and `mncavSensorParameters.json`.

Speed and steering profiles:

- default: the configured scenario, 15 to 22.6 m/s, steering within ±2 degrees;
- urban: 8 to 12 m/s, steering up to 5 degrees, 40 s;
- highway: 25.5 to 28.5 m/s, lane-change steering of 0.6 degrees, 30 s;
- sine: 20 m/s, 0.25 Hz sinusoidal steering;
- launch: stationary, 0-8-0 m/s with 6 and -4 degree turns, stationary;
- straight: 32 m/s, above the 30 m/s certificate.

## Simulation results

Relative error is settled RMSE divided by the settled RMS of the true lateral
velocity. Multi-seed rows are means of per-run RMSE.

### Matched model

| Scenario | Runs | RMSE (m/s) | Relative (%) | Worst error (m/s) |
|---|---:|---:|---:|---:|
| Noiseless | 1 | 0.0008 | 0.2 | 0.001 |
| Nominal noise, seeds 2026–2045 | 20 | 0.0053 | 1.6 | 0.022 |
| Twofold noise, seeds 2026–2035 | 10 | 0.0105 | 3.1 | 0.045 |
| Fivefold noise, seeds 2026–2035 | 10 | 0.0261 | 7.7 | 0.112 |
| Initial error [3 m/s, 0.3 rad/s] | 1 | 0.0072 | 2.1 | 0.021 |

Error scales linearly with the noise level. Per-seed nominal RMSE ranges from
0.00515 to 0.00547 m/s. Side-slip RMSE is 0.16 degrees.

### Convergence

Noise-free straight driving from a lateral-velocity initial error
(`convergence.csv`):

| Speed (m/s) | 1 m/s error: time to 0.05 m/s (s) | 3 m/s error: time to 0.05 m/s (s) | Overshoot for 1 m/s (m/s) |
|---:|---:|---:|---:|
| 6 | 0.52 | 0.78 | 0.141 |
| 10 | 0.54 | 0.80 | 0.107 |
| 15 | 0.53 | 0.68 | 0.067 |
| 20 | 0.37 | 0.72 | 0.048 |
| 28 | 0.68 | 0.98 | 0.056 |

Every case is faster than the certified 1.50 s bound for a 95 % reduction.
Convergence here is the open-loop model decay plus the master correction, not
output injection. A residual of 0.22 % of the initial error persists beyond
8 s; it decays with the 60 s accelerometer-bias time constant.

### Operating envelope (nominal noise, five seeds)

| Scenario | RMSE (m/s) | Bias (m/s) | Worst error (m/s) | Mean dynamic participation |
|---|---:|---:|---:|---:|
| Urban, 8–12 m/s | 0.0051 | -0.0006 | 0.022 | 1.000 |
| Highway, 25.5–28.5 m/s | 0.0054 | -0.0006 | 0.021 | 1.000 |
| Constant 5.1 m/s | 0.0115 | -0.0045 | 0.041 | 0.009 |
| Constant 29.9 m/s | 0.1158 | +0.0682 | 0.302 | 0.008 |
| Launch, turn and stop | 0.0063 | -0.0012 | 0.036 | 0.442 |
| Straight 32 m/s, ay bias +0.1 m/s² | 2.0769 | +2.0107 | 2.968 | 0.000 |

Accuracy is uniform inside 6–29 m/s. In the 29–30 m/s fade-out band the
dynamic correction is almost withdrawn and the error rises to 0.12 m/s. Above
30 m/s it is fully withdrawn and a constant accelerometer bias integrates
into drift, reaching about 3 m/s after 24 s. These two rows are operating-limit
probes of the intended design, not regressions. The stationary, crawl and
dynamic modes hand over without a visible step in the launch scenario.

### Vehicle-parameter mismatch (truth changed, observer nominal)

| True vehicle | RMSE (m/s) | Relative (%) | Worst error (m/s) |
|---|---:|---:|---:|
| Both axle stiffnesses x0.7 | 0.0958 | 23.4 | 0.213 |
| Both axle stiffnesses x1.3 | 0.0621 | 21.3 | 0.141 |
| Front stiffness x0.7 / x1.3 | 0.0409 / 0.0420 | 17.1 / 9.6 | 0.120 / 0.132 |
| Rear stiffness x0.7 | 0.2297 | 35.6 | 0.575 |
| Rear stiffness x1.3 | 0.0751 | 31.8 | 0.187 |
| Mass 2473 / 2673 / 2858 kg | 0.0213 / 0.0412 / 0.0590 | 6.0 / 11.2 / 15.5 | 0.058 / 0.100 / 0.136 |
| Yaw inertia x0.7 / x1.3 | 0.0107 / 0.0113 | 3.2 / 3.3 | 0.032 / 0.034 |
| CG 0.15 m rearward / forward | 0.0633 / 0.0457 | 14.4 / 17.2 | 0.175 / 0.122 |
| Urban, both stiffnesses x0.7 / x1.3 | 0.0597 / 0.0347 | 21.3 / 16.5 | 0.137 / 0.086 |

Tire stiffness dominates, the rear axle most of all; yaw inertia is nearly
irrelevant. The errors are proportional to the maneuver and have near-zero
mean, as expected when a wrong model is run open loop. This is a sensitivity
result, not a parameter-robust certificate.

### Sensor errors (default scenario)

| Sensor error | RMSE (m/s) | Bias (m/s) |
|---|---:|---:|
| Lateral-acceleration bias +0.2 / -0.2 m/s² | 0.0106 / 0.0122 | +0.0091 / -0.0109 |
| Yaw-rate bias +0.01 / -0.01 rad/s | 0.0343 / 0.0326 | -0.0338 / +0.0320 |
| Speed scale 0.98 / 1.02 | 0.0155 / 0.0169 | -0.0001 / -0.0018 |
| Road-wheel steering offset +0.15 / -0.15 degrees | 0.0165 / 0.0156 | -0.0126 / +0.0108 |
| IMU delay 20 / 50 / 100 ms | 0.0070 / 0.0126 / 0.0236 | below 0.002 |
| Dynamic validity withdrawn during [8, 16) s | 0.0406 | -0.0245 |

A 0.01 rad/s gyro bias is the most damaging sensor error tested; about
0.024 m/s of its 0.034 m/s bias is the output-point transport
(2.36 m x 0.01 rad/s). A lateral-acceleration bias of 0.2 m/s², which is also
what a 1.2 degree road bank produces, costs 0.01 m/s.

### Saturating tires (20 m/s, 0.25 Hz steering)

| Steering amplitude, friction | Peak true ay (m/s²) | RMSE (m/s) | Relative (%) | Mean participation |
|---|---:|---:|---:|---:|
| 1.8 degrees, linear tires | 2.84 | 0.0053 | 1.6 | 1.000 |
| 1.8 degrees, mu 0.9 | 2.80 | 0.0089 | 2.6 | 1.000 |
| 3.6 degrees, mu 0.9 | 5.34 | 0.0615 | 8.4 | 0.914 |
| 5.0 degrees, mu 0.9 | 6.92 | 0.1029 | 9.3 | 0.589 |
| 1.8 degrees, mu 0.4 | 2.62 | 0.0419 | 11.2 | 1.000 |
| 3.6 degrees, mu 0.4 | 3.92 | 2.0241 | 72.0 | 0.643 |

On a dry surface the error stays below 10 % up to 6.9 m/s². On a low-friction
surface at the limit the true vehicle slides at several m/s while the linear
model holds the estimate near its own prediction, and the estimate is wrong
by up to 5 m/s. The lateral-acceleration gate does not withdraw the model
there because the acceleration is capped below its 4 m/s² threshold.

### Integration step

One noiseless 1 ms truth replayed at 5, 10, 20, 50, 100 and 200 ms gives
settled RMSE 0.00080, 0.00080, 0.00080, 0.00081, 0.00083 and 0.00117 m/s.
The RK4 runtime is insensitive to the sample time on smooth inputs.

## Decay-rate sensitivity

`decay_rate_sensitivity.csv` re-synthesizes the gains at other decay rates and
reruns a fixed subset (seed 2026). All eight designs are certified. This is an
evaluation-only diagnostic; the configuration is unchanged.

| Decay rate (1/s) | Gain norm | ISS gain | Nominal noise | Tires x0.7 | Rear x0.7 | ay bias +0.2 | Delay 50 ms | Recorded 12-11-24 | Recorded 12-09-31 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 | 5.5e-10 | 3.4e-9 | 0.0053 | 0.0958 | 0.2297 | 0.0106 | 0.0126 | 0.0460 | 0.2510 |
| 3 | 0.28 | 6.1 | 0.0053 | 0.1014 | 0.2047 | 0.0065 | 0.0150 | 0.0457 | 0.2508 |
| 5 | 1.10 | 5.1 | 0.0053 | 0.1121 | 0.1568 | 0.0089 | 0.0197 | 0.0451 | 0.2503 |
| 8 | 2.42 | 107.5 | 0.0053 | 0.1191 | 0.1253 | 0.0148 | 0.0230 | 0.0448 | 0.2501 |
| 10 | 3.26 | 14.7 | 0.0053 | 0.1217 | 0.1132 | 0.0169 | 0.0243 | 0.0447 | 0.2500 |
| 15 | 5.44 | 18.9 | 0.0054 | 0.1230 | 0.1069 | 0.0179 | 0.0248 | 0.0447 | 0.2500 |

Values are settled RMSE in m/s. Nonzero gains first appear between 2 and
3 1/s. Feedback halves the rear-stiffness error but worsens uniform stiffness
error, accelerometer-bias and delay sensitivity, and leaves noise unchanged.
The reported ISS gain is not monotonic in the decay rate because the minimized
quantity is the gain bound, not the ISS gain. No decay rate in this range is
better on every case, and none changes the recorded result.

## Recorded drives

Both June 7, 2024 MnCAV recordings are replayed from four-wheel speed, the
corrected DBW IMU and steering, with the current configuration
(`mncavReplayConfig`). The reference is the INSPVA body-frame lateral
velocity, interpolated to the 100 Hz replay grid; it never enters the
observer. The first and last 0.5 s of INSPVA coverage are excluded. The 17 May
2024 bags contain no INSPVA topic and cannot score lateral velocity.

| Drive, population | Samples | RMSE (m/s) | Bias (m/s) | Demeaned RMSE | P95 | Zero-vy baseline | Open-loop model | Correlation | Side slip RMSE (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12-11-24, all | 7,693 | 0.0460 | -0.0146 | 0.0436 | 0.0900 | 0.1098 | 0.0423 | 0.918 | 0.226 |
| 12-11-24, straight, vx >= 5 | 2,484 | 0.0323 | -0.0101 | 0.0307 | 0.0568 | 0.0480 | 0.0277 | 0.794 | 0.162 |
| 12-11-24, turning, vx >= 5 | 4,292 | 0.0476 | -0.0160 | 0.0449 | 0.0853 | 0.1136 | 0.0426 | 0.920 | 0.256 |
| 12-11-24, 1–40 s (calibration window) | 3,901 | 0.0478 | +0.0011 | 0.0478 | 0.0994 | 0.1270 | 0.0455 | 0.926 | 0.246 |
| 12-11-24, after 40 s (held out) | 3,692 | 0.0420 | -0.0337 | 0.0250 | 0.0775 | 0.0818 | 0.0361 | 0.957 | 0.208 |
| 12-09-31, all | 11,592 | 0.2510 | -0.2289 | 0.1032 | 0.3965 | 0.2516 | 0.2500 | 0.666 | 1.238 |
| 12-09-31, straight, vx >= 5 | 6,242 | 0.2796 | -0.2670 | 0.0830 | 0.3944 | 0.2802 | 0.2786 | -0.108 | 1.278 |
| 12-09-31, turning, vx >= 5 | 4,490 | 0.2299 | -0.2110 | 0.0914 | 0.4097 | 0.2318 | 0.2304 | 0.810 | 1.178 |

Straight and turning split at |measured yaw rate| 0.03 rad/s. "Open-loop
model" is the nominal bicycle driven by the same steering and speed with no
IMU. Durations are 76.9 s and 115.9 s; speeds reach 12.9 and 14.9 m/s and
lateral acceleration 1.8 and 2.0 m/s², so **no recorded sample exercises the
15–30 m/s half of the design range**. The replay costs about 129 microseconds
per sample as a batch average, which is not a real-time latency bound.

On 12-11-24 the observer tracks the reference (correlation 0.92, RMSE 42 % of
the zero baseline). That drive supplied the output-point, steering-zero, IMU
and wheel-radius calibrations over seconds 1–40; only the segment after 40 s
is held out from those fits, and it scores 0.042 m/s with a -0.034 m/s bias.

On 12-09-31 the observer is no better than predicting zero. Most of the
squared error is an offset: the demeaned RMSE is 0.103 m/s. The observer and
the open-loop model coincide (0.2510 against 0.2500 m/s), which follows
directly from the zero gain.

`recorded_windows.csv` locates the offset. On the straight final 20 s of
12-09-31 (mean |yaw rate| 0.004 rad/s, 11.7 m/s) the reference reports
+0.25 to +0.29 m/s, a crab angle of 1.2 to 1.4 degrees, while the measured
lateral acceleration averages 0.02 to 0.16 m/s² and the observer reports
about zero. Reference crab stays at about +1.0 to +1.5 degrees throughout
10–70 s. On 12-11-24 the reference crab stays within ±0.5 degrees above
10 m/s and follows the turns. The INSPVAX log of 12-09-31 reports an azimuth
standard deviation of at most 0.113 degrees. A regression of the error on
speed and yaw rate explains 48 % of its variance on 12-09-31 with a slope of
-0.041 m/s per m/s, and 3.5 % on 12-11-24 (`recorded_diagnostics.csv`).

This evaluation does not identify the cause of that offset. It is consistent
with the earlier diagnosis in `research/mncav_lateral_diagnosis_20260916`,
which found the reference lateral velocity incompatible with the bicycle
output equation on this drive. It could lie in the reference frame or heading
convention, in unmodeled road or vehicle effects, or in the model; the data
here cannot separate them.

For context, the 2026-09-25 evaluation reported 0.0552 and 0.2502 m/s on the
same two drives with the vehicle parameters and gains of that date, so the
two sets are not a controlled comparison.

## Limitations

- The synthetic plant uses small-angle slip, a tanh force saturation with
  static axle loads, and no roll, load transfer, bank or relaxation length.
- Synthetic noise is white at 100 Hz; the recorded DBW IMU is native 50 Hz,
  linearly reconstructed offline with future samples.
- Parameter, sensor and tire cases use one seed each.
- INSPVA is the stipulated reference, not independent truth; its
  inconsistency on 12-09-31 is unresolved.
- Only two recorded drives exist with a lateral reference. 12-11-24 is partly
  a calibration drive and 12-09-31 is a reused evaluation drive.
- Full localization, LiDAR matching and the global observer were not rerun.

## Files

| File | Content |
|---|---|
| `evaluateLateralObserverPerformance.m` | Evaluator |
| `design_summary.csv`, `gain_schedule.csv` | Synthesis result, dense certificate, gains and poles by speed |
| `harness_check.csv` | Evaluator plant against the production scenario |
| `synthetic_metrics.csv`, `synthetic_summary.csv` | Per-run and per-scenario synthetic metrics |
| `convergence.csv`, `integration_metrics.csv` | Initial-error and sample-time studies |
| `decay_rate_sensitivity.csv` | Re-synthesis at other decay rates |
| `recorded_metrics.csv`, `recorded_diagnostics.csv`, `recorded_windows.csv` | Recorded-drive results |
| `unit_tests.csv` | `lateralObserverTest` outcomes |
| `input_hashes.csv` | SHA-256 of evaluated sources and inputs |
| `simulation_performance.png`, `recorded_performance.png` | Figures |
