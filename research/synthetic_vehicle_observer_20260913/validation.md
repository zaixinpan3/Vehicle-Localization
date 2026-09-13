# Synthetic vehicle observer demonstration

Prepared and executed on 2026-09-13 using MATLAB R2026a Update 3.

The actual lateral observer followed by the actual seven-state observer works
in these four nominal synthetic experiments: all meet the stated settled-error
limits. GNSS exhibits severe startup peaking with deliberately incorrect
initial position. The LiDAR cascade converges numerically but temporarily
leaves the reference certificate's course-rate envelope during lateral startup.
These findings must accompany the successful steady-state result.

## Reproduce and inspect

From the repository root:

```matlab
setupVehicleLocalization;
report = demoSyntheticVehicleObserver;
assert(report.allPassed);
```

The entry point is [demoSyntheticVehicleObserver.m](../../scripts/demoSyntheticVehicleObserver.m).
It displays four figures containing trajectory, all seven states, and body
lateral velocity. Black is truth; dashed red is the estimate. PNG/PDF figures,
`traces.mat`, `metrics.csv`, and `summary.json` are written to
`output/synthetic_vehicle_observer/`. Each MAT result retains truth, sensor
functions and high-rate samples, both observer outputs, initial history,
configurations, parameters, limits, and metrics. The small CSV/JSON outputs
are copied into this research directory for versioned results.

No perception, map, registration, bag data, solver synthesis, or observer
algorithm change is involved. The stored lateral design and current hybrid
runtime configuration are used, together with the existing constant global
gains. Exact lateral velocity is **not** supplied to the global observer.

## Vehicle, truth, and measurements

The assumed nominal sedan uses the stored lateral-design parameters:
mass 1575 kg, yaw inertia 2875 kg m², front/rear axle distances 1.2/1.6 m,
front/rear axle cornering stiffness 75,000/56,000 N/rad. These are synthetic
assumptions, not identified parameters. Plant and observer share them.

Each experiment lasts 40 s with a 100 Hz input/output grid. Longitudinal
body speed is 8 m/s. GNSS uses yaw rate 0.04 rad/s, steering 0.8011587 degrees;
LiDAR uses 0.001 rad/s, steering 0.02002897 degrees and a 150 ms fixed delay.
The different turn rates respect the intended nominal operating sectors;
these runs do not establish a like-for-like sensor ranking.

Solve `A*[vy;r]+B*delta=0` for steady lateral speed and steering. The true
body sideslip is `beta=atan2(vy,8)`, speed is `hypot(8,vy)`, and yaw is
`psi=0.2+r*t`. Integrate the constant-speed course `psi+beta` analytically
to obtain X/Y; velocity and acceleration are analytic derivatives. Body
inertial acceleration is `ax=-vy*r`, `ay=8*r`. This includes the centripetal
term in longitudinal acceleration even though body longitudinal speed is
constant. Prehistory uses the same analytic trajectory at negative time.

In noisy runs, deterministic sine/cosine errors are added as follows. The
numbers are amplitudes, not standard deviations. There is no random seed.

| Measurement | Error amplitude | Angular frequency |
|---|---:|---:|
| Steering | 0.005 degrees | 0.8 rad/s |
| Longitudinal speed | 0.02 m/s | 1.1 rad/s |
| Longitudinal acceleration | 0.02 m/s² | 1.7 rad/s |
| Lateral acceleration | 0.02 m/s² | 1.4 rad/s |
| Yaw rate | 0.0002 rad/s | 0.6 rad/s |
| GNSS X/Y | 0.05 m per axis | 1.3/0.9 rad/s |
| LiDAR X/Y | 0.01 m per axis | 1.3/0.9 rad/s |
| LiDAR yaw | 0.1 degrees | 0.7 rad/s |

Pose providers are continuous functions. High-rate inputs are linearly
reconstructed by the runtime. These are synthetic continuous sensor inputs,
not a discrete GNSS/LiDAR delivery or dropout experiment. LiDAR information
is the declared `1e6*eye(3)` in normalized pose units; it is chosen to meet
the reference contract, not inferred as inverse noise covariance.

Initial global state error, ordered `[X,Vx,Ax,Y,Vy,Ay,psi]`, is
`[1,0.3,0.1,-1,-0.2,-0.1,10 degrees]`: position error norm 1.414 m.
The estimated history carries the same constant error. Lateral master
velocity starts 0.3 m/s high; hidden yaw rate starts 0.01 rad/s high.
The sideslip interface starts at its true steady value with zero rate;
it subsequently responds to the incorrect estimated lateral velocity.
Truth is used for initialization and scoring, not as a corrective channel.

## Executed results

RMSE is scored on 20–40 s. Limits fixed before execution are 0.15 m position,
0.15 m/s velocity, 0.15 m/s² acceleration, 1 degree heading, and 0.05 m/s body
lateral velocity. `passed` means finite seven-state output and these RMSE
limits; it does not mean no transient overshoot or a full physical certificate.

| Mode | Noise | Position (m) | Velocity (m/s) | Acceleration (m/s²) | Heading (deg) | Lateral velocity (m/s) |
|---|---|---:|---:|---:|---:|---:|
| GNSS | None | 5.44e-9 | 1.47e-7 | 1.11e-6 | 0.001491 | 0.0002032 |
| GNSS | Bounded | 0.050743 | 0.055673 | 0.066679 | 0.313661 | 0.0007996 |
| LiDAR | None | 1.44e-7 | 3.93e-7 | 2.63e-7 | 8.72e-7 | 0.0002032 |
| LiDAR | Bounded | 0.013568 | 0.011997 | 0.003803 | 0.057245 | 0.0007972 |

The first sampled time after which all instantaneous errors remain below
the corresponding limits through 40 s is 0.95/0.89 s for clean/noisy GNSS,
and 2.02/2.01 s for clean/noisy LiDAR. These are finite-horizon sampled
settling times, not guaranteed bounds between samples or beyond the run.

Noisy GNSS startup peaks are 22.724 m/s velocity error, 134.166 m/s²
acceleration error, and 44.935 degrees heading error. Noisy LiDAR peaks are
1.107 m/s, 0.302 m/s², and 10 degrees. Global initialization far from the
position measurement is therefore consequential for high-gain startup.

Both LiDAR runs report 81 output samples outside the 0.00215032 rad/s
reference course-rate envelope. The supplied `r+betaDot` reaches about
0.15854 rad/s in the noisy run because of lateral startup, despite the true
steady yaw rate being only 0.001 rad/s. GNSS stays inside its 0.6 rad/s
coefficient envelope. The actual global matrices reverify in every run;
this does not certify the whole LiDAR trajectory or all physical hypotheses.

## Validation and limits

- All four scenario acceptance checks pass via MATLAB MCP.
- Analytic clean body-acceleration identities and actual upstream observer
  selection were explicitly asserted in both modes and passed.
- Repeating both noisy runs with a 2.5 ms global step instead of 5 ms gives
  maximum GNSS coordinate differences below 5.22e-6 m position,
  1.70e-4 m/s velocity, 0.00150 m/s² acceleration, and 2.48e-6 rad yaw.
  LiDAR differences are below 8.06e-11 in each coordinate's units. GNSS
  startup peaking persists under refinement; it is not removed by that change.
  The lateral 100 Hz integration grid is unchanged by this check.
- Factory MATLAB Code Analyzer reports zero findings for the new entry point.
- MATLAB reports software graphics/vector-export performance warnings;
  the exported plots exist and were visually inspected.

This is a nominal, steady-turn, continuous-signal functionality demonstration.
It does not test parameter mismatch, changing curvature/speed, standstill,
outages, outliers, poor LiDAR information, or an independent vehicle model.
