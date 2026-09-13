# Constant GNSS gain tuning to reduce startup peaking

Executed on 2026-09-13 in MATLAB R2026a Update 3. The selected optional
`lowPeaking` preset reduces the noisy synthetic acceleration-error peak by
74.21%, from 134.1655 to 34.5986 m/s². Heading-error peak decreases from
44.9351 to 16.2411 degrees. The peak is reduced, not eliminated; sampled
settling slows from 0.89 to 3.51 s. The same full operating coefficient
bounds pass the current constant-matrix verifier.

## Reproduction and selection

```matlab
setupVehicleLocalization;
report = tuneSyntheticObserverPeaking;
cfg = improvedObserverConfig("gnss", "lowPeaking");
design = improvedObserverReferenceDesign(cfg);
% Supply current sensor data and initial estimate to the existing runner.
```

The sweep loads `output/synthetic_vehicle_observer/traces.mat`, generating
it using `demoSyntheticVehicleObserver` if absent. It saves all executed
states, configurations, metrics and chosen design in
`output/observer_peaking_tuning/`, with a PNG/PDF startup comparison.
Small summary, metrics, test and analyzer CSV/JSON exports are copied here.

The optional preset is exposed through the existing configuration/design
interfaces. The default remains `reference`; `lowPeaking` applies only to
GNSS, and requesting it for LiDAR is rejected. A custom GNSS chain obtains
a newly solved Lyapunov matrix and independent verification before use.
No output clipping, gain scheduling, initial-state adjustment, changed input
noise, or operating-bound reduction is used in this experiment.

## Controlled comparison

Reuse both clean/noisy 40 s trials from the
[synthetic sedan demonstration](../synthetic_vehicle_observer_20260913/validation.md):
8 m/s, 0.04 rad/s true yaw rate, 100 Hz input/output, 5 ms maximum global
integration step. Nominal sedan is 1575 kg with yaw inertia 2875 kg m²,
axle distances 1.2/1.6 m and cornering stiffness 75000/56000 N/rad.
Initial global error is `[1,.3,.1,-1,-.2,-.1,10 degrees]` in state order
`[X,Vx,Ax,Y,Vy,Ay,psi]`. Initial position error norm is 1.414 m.

All candidates use exactly the same saved output of the actual lateral
observer, including its startup error. A separate public-preset run executes
the real lateral/global cascade again and reproduces the selected sweep
states within 1e-12. The sweep's reference trajectory reproduces the earlier
reference states exactly (maximum absolute difference zero).

Noise consists of the original deterministic bounded sinusoids: .005 degree
steering, .02 m/s longitudinal speed, .02 m/s² each body acceleration,
.0002 rad/s yaw rate, and .05 m GNSS position per axis. There is no RNG.
All RMSE values below cover 20–40 s. Peaks and settling are evaluated on
the 100 Hz output grid. Settling means all subsequent reported errors
through 40 s stay below .15 m position, .15 m/s velocity, .15 m/s²
acceleration and 1 degree yaw; it is not an infinite-horizon guarantee.

Nine settings are screened for each noise condition. Eight certify and are
simulated (16 completed trials); the reference-shaped `theta=10` setting
fails the matrix certificate and is not integrated in either condition.
Its CSV NaN values/JSON nulls indicate a rejected candidate, not divergence.

| Noisy setting | theta | Chain K per axis | Yaw gain | Accel. peak (m/s²) | Velocity peak (m/s) | Heading peak (deg) | Settling (s) | Matrix margin |
|---|---:|---|---:|---:|---:|---:|---:|---:|
| Reference | 20 | [3,3,1] | .5 | 134.166 | 22.724 | 44.935 | .89 | 8.20763 |
| theta16 | 16 | [3,3,1] | .5 | 85.587 | 18.228 | 42.269 | .86 | 4.20697 |
| theta14 | 14 | [3,3,1] | .5 | 65.711 | 15.995 | 40.509 | .83 | 2.20641 |
| theta12 | 12 | [3,3,1] | .5 | 48.368 | 13.672 | 38.401 | .94 | .20555 |
| theta10 rejected | 10 | [3,3,1] | .5 | — | — | — | — | -1.79588 |
| shaped8 | 8 | [6,8,3] | .5 | 34.599 | 12.419 | 34.516 | 1.24 | .14878 |
| **lowPeaking** | **8** | **[6,8,3]** | **.1** | **34.599** | **12.419** | **16.241** | **3.51** | **.14878** |
| shaped8 yaw .2 | 8 | [6,8,3] | .2 | 34.599 | 12.419 | 21.671 | 1.86 | .14878 |
| shaped8b | 8 | [8,12,4] | .2 | 35.359 | 13.979 | 21.564 | 1.85 | .33809 |

The selected preset's noisy settled RMSE is .050759 m position,
.057286 m/s velocity, .064461 m/s² acceleration, and .18204 degrees heading.
Reference values are .050743 m, .055673 m/s, .066679 m/s² and .31366 degrees.
The clean preset has acceleration-error peak 33.778 m/s², heading-error peak
15.997 degrees and sampled settling 3.57 s. All 16 executed trials pass the
original global steady-state limits; upstream lateral behavior is unchanged.

## Parameter mechanism and verification

The physical position-residual injection into each Cartesian chain is
`[theta*K1; theta^2*K2; theta^3*K3]`. It changes from
`[60;1200;8000]` to `[48;512;1536]` with units appropriate to its rows.
The smaller velocity/acceleration injections reduce the startup response to
the deliberately displaced position estimate. The independent yaw gain
changes from .5 to .1, reducing the angular transient with slower settling.
The first six rows do not depend on this yaw gain, as reflected by identical
position/velocity/acceleration results across the shaped8 yaw sweep.

The nominal chain metric solves `F'*P+P*F=-I` for the changed `K`.
`verifyImprovedObserverDesign` recomputes a positive uniform margin
.14878395 using the original velocity/acceleration component bounds
16 m/s and 5 m/s², and supplied course-rate bound .6 rad/s. This is the
conditional matrix certificate; motion, heading-sector and disturbance
assumptions are not established for arbitrary physical use. The new margin
is much smaller than the reference margin; an unexamined further reduction
is not endorsed. The search is a finite candidate comparison, not an optimum.

Validation performed:

- 49/49 MATLAB tests pass: three new preset/behavior tests and all 46
  existing global-observer tests. Tests verify preserved operating bounds,
  the recomputed certificate, reduced startup peaks with settled accuracy,
  and rejection of a GNSS preset in LiDAR mode.
- Factory Code Analyzer reports zero findings in all five changed MATLAB
  files. The first analyzer attempt encountered an unavailable user settings
  file; explicitly selecting factory settings resolved this environment issue.
- The selected noisy trial repeated with a 2.5 ms global step differs by
  at most 4.75e-6 m position, 6.45e-5 m/s velocity, 2.19e-4 m/s² acceleration,
  and 2.32e-7 rad yaw per coordinate. The lateral grid remains unchanged.
- The startup comparison plot was rendered and visually inspected.
- An exploratory Python chain screen used NumPy after SciPy was found
  unavailable; only the actual MATLAB runtime results above support the
  selected preset's reported performance.

Additional gains, gain scheduling, observer redesign, changed initialization,
parameter mismatch and real-data validation were not explored here. The
original LiDAR configuration and its earlier findings remain separate.
