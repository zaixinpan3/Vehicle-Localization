# Lateral observer performance evaluation

Evaluation date: 2026-09-25. MATLAB R2026a Update 3. The current working-tree
observer converges accurately on matched-model simulations, but does not
deliver comparably accurate lateral velocity on the held-out recorded drive.
Tire-model mismatch and the upper-speed fade-out materially reduce accuracy.
No observer gains, production parameters, or calibration were changed here.

## Version and reproduction

The starting commit was `669b765498c431c57b30fbbcd4dc68c3bf94a737`, with
pre-existing modifications to the lateral observer, synthesis, configuration,
unit tests and both saved gain files. This evaluation uses those working-tree
versions, **not the unmodified starting commit**. `input_hashes.csv` identifies
37 source, design, calibration and sensor inputs. The evaluated source snapshot,
starting Git status and source diff are retained locally under
`output/lateral_performance_20260925/`. Unrelated changes were preserved.
This task's commit contains the evaluator and compact result exports only;
the pre-existing production edits and binary gain files remain outside it.

```matlab
setupVehicleLocalization;
addpath('research/lateral_performance_20260925');
report = evaluateLateralPerformance();
results = runtests('tests/lateralObserverTest.m');
```

For the synthesis regression, add an existing YALMIP and SeDuMi installation
to the path first. This run used the local installation at
`../RobustVehicleLocalization/external/{YALMIP,sedumi}`. No dependencies were
installed or vendored. Reproduction requires the input versions in the hash
manifest and the local raw sensor exports; a clean checkout alone is insufficient.

`experiment.mat` in the ignored output directory retains full traces, configs,
metric tables and the MATLAB version. `performance.png` is a compact plot.
The source snapshot is local reproducibility material, not a second production
implementation. No recorded datasets or generated MAT files are committed.

## Existing regressions and certificate

All **18/18** existing lateral-observer tests passed. The first run passed 17
and filtered the synthesis test because YALMIP was not on the MATLAB path.
After adding the installed dependencies, that test passed in a separate run;
`unit_tests.csv` combines the 17 initial passes and this completed rerun.
Coverage includes algebra/scheduling, independent stored-certificate checks,
gain resynthesis, initial-error convergence, bounded copied nonlinearity,
output-point transport, stationary behavior and continuous mode transitions.

Both profiles also passed an independently evaluated dense grid: 1,001 speeds
from 5 to 30 m/s, each with longitudinal acceleration -3, 0 and +3 m/s².

| Profile | Worst Psi eigenvalue | Minimum P eigenvalue | Largest real frozen pole |
|---|---:|---:|---:|
| Reference sedan | -0.032879 | 0.84689 | -4.5430 |
| MnCAV | -0.043157 | 0.53531 | -4.5460 |

All 6,006 Psi checks were negative. The noise-augmented certificate maximum
was within 8e-15 of zero, below the 1e-9 numerical tolerance. The designed
decay rate is 2/s. A dense numerical grid is not a proof between samples;
this certificate concerns the nominal continuous-time LPV branch, not the
hybrid master, discretization, arbitrary vehicle parameter errors or real data.

## Synthetic accuracy, convergence and robustness

84 runs cover the reference and MnCAV profiles. Each uses 24 seconds at
100 Hz and the configured raised-cosine steering trajectory. Default speed
ranges from 15 to approximately 22.64 m/s with acceleration amplitude 2 m/s².
Initial hidden-state error is [0.5 m/s, 0.05 rad/s]; the master inherits the
lateral-velocity error. White noise defaults to sigma(ay)=0.05 m/s² and
sigma(r)=0.002 rad/s. Nominal cases use seeds 2026–2045 (20 per profile),
fivefold noise uses 2026–2035 (10 per profile), other cases use seed 2026.

The following RMSE values use t >= 6 s. Monte Carlo entries are means of
per-run RMSE, not pooled samples or confidence bounds. The MnCAV truth is
transported to the declared output point using the **true** yaw rate, while
the observer necessarily uses the measured yaw rate. This exposes gyro noise
in the output-point transport instead of incorrectly comparing different points.

| Scenario | Reference RMSE (m/s) | MnCAV RMSE (m/s) |
|---|---:|---:|
| Noiseless matched model | 0.001027 | 0.000994 |
| Nominal noise, 20 seeds | 0.002309 | 0.005461 |
| Fivefold noise, 10 seeds | 0.010195 | 0.026720 |
| Initial error [3 m/s, 0.3 rad/s] | 0.006553 | 0.008232 |
| True front/rear tire stiffness -30% | 0.288738 | 0.292970 |
| True front/rear tire stiffness +30% | 0.164618 | 0.166808 |
| True mass +20% | 0.139344 | 0.144161 |
| Additional ay bias +0.2 m/s² | 0.020099 | 0.023424 |
| Additional gyro bias +0.01 rad/s | 0.013976 | 0.036796 |
| Dynamic validity withdrawn during [8,16) s | 0.042022 | 0.042556 |
| Constant speed 5.1 m/s | 0.013012 | 0.012908 |
| Constant speed 29.9 m/s | 0.205614 | 0.208801 |
| Straight stop/go with ay bias +0.1 m/s² | 0.005397 | 0.007160 |
| Straight 32 m/s, ay bias +0.1 m/s² | 2.058364 | 2.058405 |

All runs remained finite. Finiteness is not an accuracy pass criterion.
In the noiseless cases, lateral error enters and stays within 0.05 m/s after
0.48/0.49 s (reference/MnCAV). With large initialization error and seed 2026,
this takes 0.66/0.68 s. The settling definition requires the bound for the
entire remaining record; late re-entry after a maneuver is not evidence of
fast convergence. Raw settling values and yaw/side-slip errors are in
`synthetic_metrics.csv`.

Plant-parameter cases change the truth model only; the scored observer retains
the nominal model and gains. The preliminary simulation's estimate is discarded.
The largest tire-mismatch error reaches 0.659 m/s. This experiment demonstrates
sensitivity, not a parameter-robust certificate. The copied-nonlinearity unit
test does not establish robustness to unknown tire parameters.

At 29.9 m/s the dynamic correction is almost withdrawn (mean participation
about 0.004) by the designed 29–30 m/s fade-out. At 32 m/s it is completely
withdrawn; persistent acceleration bias produces drift, reaching about 2.92 m/s
error by 24 s. These are intentional operating-limit probes, not evidence
that the LPV design covers speeds above 30 m/s. Stop/go is a straight synthetic
trajectory with cosine ramps 0–8–0 m/s and biased noisy acceleration, not a
nonlinear tire simulation of parking turns.

## Recorded sensor replay

Both June 7, 2024 drives were recomputed from four-wheel speed, corrected IMU
and steering, using the current MnCAV gain file and configuration. The existing
clock model aligns samples; the first and final 0.5 s of INSPVA coverage are
excluded. No INSPVA velocity enters the observer. Body lateral reference is
computed directly from INSPVA east/north velocity and azimuth, then interpolated
onto the 100 Hz observer timestamps. No tuning or refitting was done.

| Drive/population | Samples | RMSE (m/s) | Bias (m/s) | Absolute P95 (m/s) | Maximum (m/s) |
|---|---:|---:|---:|---:|---:|
| 12:11:24, all | 7,693 | 0.05517 | -0.01311 | 0.10873 | 0.16593 |
| 12:11:24, after 40 s | 3,692 | 0.05582 | -0.03495 | 0.09640 | 0.15182 |
| 12:09:31, all | 11,592 | 0.25017 | -0.22781 | 0.38566 | 0.49916 |
| 12:09:31, straight, measured vx >= 5 m/s | 6,242 | 0.27426 | -0.26250 | 0.37937 | 0.48447 |
| 12:09:31, turning, measured vx >= 5 m/s | 4,490 | 0.23642 | -0.21474 | 0.44189 | 0.49916 |

Straight/turning uses |measured yaw rate| < / >= 0.03 rad/s. The all-sample
durations are 76.92 and 115.91 s. The 12:11:24 drive was previously used for
output-point calibration over seconds 1–40; only its later segment is held out
from that fit. The 12:09:31 drive was not used for the output-point fit, but is
a reused project evaluation drive, not a newly acquired independent dataset.

On 12:09:31, always predicting zero lateral velocity gives RMSE 0.25165 m/s;
the current observer's 0.25017 m/s is only marginally better. Removing the
mean error **after scoring** gives 0.10339 m/s residual RMS, establishing that
a constant offset explains much of the squared error. This is a diagnostic,
not an achievable reference-free correction. The lateral observer alone has
not eliminated this offset. Its cause cannot be uniquely identified here
among reference/frame alignment, sensor error and vehicle-model mismatch.

Side-slip RMSE at reference longitudinal speed >= 6 m/s is 0.2803 degrees on
12:11:24 and 1.2488 degrees on 12:09:31. Saved tables also report populations,
zero-velocity baselines and untransported observer-point errors.

Recorded replays took 1.048 and 1.563 s, about 136 and 135 microseconds per
sample including batch output bookkeeping. These are single observed average
costs, not per-call latency quantiles or a worst-case real-time guarantee.
Offline clock fitting and IMU/steering interpolation use future samples.

## Integration and result checks

An independent 1 ms truth trajectory was subsampled to 5, 10, 20, 50, 100 and
200 ms for noiseless reference-profile replay. Post-6 s RMSE was respectively
0.001026, 0.001028, 0.001033, 0.001075, 0.001292 and 0.003051 m/s. At 200 ms,
transient peak grew to 0.739 m/s. This limited smooth-input check does not
establish behavior for arbitrary packet gaps, discontinuities or slower rates.

Factory Code Analyzer: zero findings in the new evaluator. Saved traces were
reloaded and all retained synthetic seed-2026 RMSE rows and both recorded
all-sample rows were recomputed to within 1e-12; the squared-RMSE bias/variance
identity also passed. The plot was rendered and visually inspected. The initial
evaluator attempt stopped at empty-struct concatenation before any campaign;
that harness issue was corrected before the completed run reported here.

## Engineering decision

Retain the current tuning during this evaluation. Prioritize diagnosis of the
recorded lateral-velocity offset, independently identified vehicle dynamics,
and the high-speed fallback behavior before claiming broad real-vehicle
accuracy. Treat the LPV certificate, synthetic accuracy, output-point
calibration and recorded accuracy as separate results. Full localization,
LiDAR matching and global-observer position error were not reevaluated here.
