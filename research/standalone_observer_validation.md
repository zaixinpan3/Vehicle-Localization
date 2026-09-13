# Standalone seven-state observer validation

Prepared and executed on 2026-09-13 in MATLAB R2026a Update 3.

## Finding

The existing global observer works for synthetic position/yaw tracking with
regular full-pose measurements, independently of the rest of the localization
system. Five noisy, delayed maneuver runs give position RMSE 0.0872--0.1012 m
and yaw RMSE 0.2960--0.3312 degrees over 10--30 s. This is an empirical result
for the specified inputs, not a stability theorem or validation of the original
paper's implementation. The complete all-state screening is **6/12 passing**:
four nominal cases miss a strict acceleration transient criterion, and two
measurement stress cases miss the velocity criterion. No thresholds or gains
were relaxed after observing these failures.

## Relation to Bessafa et al.

Source examined: Hichem Bessafa, Cedric Delattre, Zehor Belkhatir, Ali Zemouche,
and Rajesh Rajamani, *Generalized multi-output high-gain observer with
application to ego vehicle trajectory and orientation estimation*, Automatica
188 (2026), 112915; local PDF in `reference/`. Sections 3.1--3.2, equations
(26)--(38), the correction structure (41)--(42), and simulation sections
5.1--5.2 were inspected. The original transformed observer has six states,
recovers yaw from velocity direction and sideslip, and uses constant-speed,
constant-steering/sideslip assumptions in its nominal transformation.

This experiment evaluates the repository's **improved seven-state** observer:
independent yaw, four intrinsic measurement channels, measured course rate,
full pose-information weighting, and fixed-delay transport. It uses the
unchanged stored gains (`theta=sigma=3.5`), not the original paper's numerical
six-state gains. The stored historical certificate is checked for gain
provenance; the transported runtime remains explicitly uncertified.

## Isolation and implementation

`runImprovedVehicleObserver(..., LateralInputs=series)` bypasses the lateral
observer while retaining the exact production propagation, nonlinear channels,
pose-event handling, weighting, and correction code. `series` provides `time`,
`lateralVelocity`, `sideSlipAngle`, and `sideSlipAngleRate` on exactly the
high-rate clock. Invalid or nonfinite series are rejected. The usual four-input
call still runs the lateral cascade. A diagnostic states which source was used.

The driver supplies analytic body motion, inertial acceleration, sideslip and
synthetic absolute pose measurements. It runs no perception, mapping,
registration, lateral estimator, GPS, or recorded-data processing. The initial
estimate uses deliberately biased truth, only once at initialization; truth is
not supplied as the evolving estimate. Synthetic pose inputs include heading,
so stationary yaw recovery is aided by a direct heading measurement.

## Reproducible experiment

- Duration 30 s; high-rate inputs 100 Hz; pose acquisition 10 Hz; correction
  pulse 30 ms; event-split RK4 with maximum step 10 ms. Delay is either zero or
  150 ms. Events arriving after the horizon are omitted.
- Initial truth position `(2,-1)` m and heading 0.3 rad. Initial state error in
  `[X,Vx,Ax,Y,Vy,Ay,psi]` is `[2,-1,0.5,-1.5,0.8,-0.3,15 deg]`, so initial
  position error is 2.5 m. No gain synthesis or gain tuning is performed.
- Straight: speed 8 m/s. Circle: speed 8 m/s, yaw rate 0.15 rad/s, sideslip
  0.03 rad. Stationary: zero speed and yaw rate with absolute pose observations.
- Maneuver: speed `8+1.2*sin(0.35*t)` m/s, yaw
  `0.3+0.12*t+0.25*sin(0.4*t)` rad, sideslip `0.03*sin(0.5*t)` rad.
  Velocity and acceleration are analytic derivatives of this physical motion;
  position is independently integrated with `ode45`, relative tolerance 1e-11
  and absolute tolerance 1e-12. The observer dynamics do not generate truth.
- This maneuver intentionally has omitted nominal jerk terms
  `v''*e_course + v*q'*J*e_course`, maximum norm 0.454351 m/s^3. Consequently,
  exact zero-error acceleration tracking is not expected under this model.
- All noise is independent unit Gaussian draws clipped to +/-3 before scaling.
  Noise scales: pose XY 0.05 m, pose yaw 0.5 degrees, longitudinal/lateral
  velocity 0.02 m/s, acceleration 0.03 m/s^2 per axis, gyro 0.002 rad/s,
  sideslip 0.1 degrees, and sideslip rate 0.001 rad/s. These are simulation
  settings, not identified sensor precisions. The independently perturbed
  sideslip/rate inputs are imperfect cascade surrogates.
- Pose information is `diag([400,400,1/deg2rad(0.5)^2])`; gain information
  scale is 5. Weak-direction case rotates XY information by 0.7 rad and
  changes its eigenvalues to `[0.2,400]` over acquisition times `[12,18)` s.
  Its weak gain weight is 0.2/5.2, without an artificial positive floor.
- Outage removes all absolute pose acquisitions in `[12,15)` s; there is no
  GPS fallback. Delayed corrections persist briefly from the last acquisition.
- Seed 20260913 for the main cases, plus 20260914--20260917 for four additional
  noisy delayed maneuvers. MATLAB RNG state is restored on exit.

## Results and criteria

RMSE is computed over inclusive samples 10--30 s. XY, velocity, and
acceleration use Euclidean vector error; yaw uses wrapped angular error.
For exact nominal cases, all four RMSE limits are 0.01 in their respective
units (m, degrees, m/s, m/s^2). For maneuver/noise/stress cases, the limits are
0.5 m, 2 degrees, 0.5 m/s, and 1.5 m/s^2. These are screening criteria chosen
before execution, not established application requirements. CSV retains full
transient peaks and late-window metrics, even when the main criterion fails.

| Case | XY RMSE (m) | Yaw RMSE (deg) | Velocity RMSE (m/s) | Acceleration RMSE (m/s^2) | All-state criterion |
|---|---:|---:|---:|---:|---|
| Ideal straight | 0.000964 | 0.0000250 | 0.00501 | 0.0342 | Fail: acceleration |
| Ideal circle | 0.000767 | 0.0000151 | 0.00391 | 0.0267 | Fail: acceleration |
| Ideal stationary | 0.000786 | 0.0000182 | 0.00406 | 0.0278 | Fail: acceleration |
| Ideal variable maneuver | 0.01351 | 0.000356 | 0.0701 | 0.4749 | Pass |
| Ideal circle, 150 ms delay | 0.001713 | 0.0000310 | 0.00820 | 0.0278 | Fail: acceleration |
| Noisy maneuver, 150 ms delay | 0.10117 | 0.31444 | 0.3246 | 0.4967 | Pass |
| Noisy maneuver, 3 s outage | 0.21708 | 0.32364 | 0.5697 | 0.5142 | Fail: velocity |
| Noisy maneuver, weak direction | 0.37596 | 0.31446 | 0.5983 | 0.5126 | Fail: velocity |

All four extra noisy seeds pass the maneuver criteria. All twelve trajectories
remain finite, with no estimated velocity/acceleration envelope clipping and
no rejected pose events. Full-run speed-error peaking is substantial:
approximately 11.0--11.5 m/s, despite only 1.28 m/s initial velocity error.
The current high gains prioritize rapid position correction; downstream uses
of estimated velocity must account for this startup transient.

For the three zero-delay exact nominal cases, position/yaw enter and remain
within 0.1 m / 1 degree at 1.13 s; the delayed circle does so at 1.57 s.
Acceleration converges more slowly: its 25--30 s RMSE is 0.00121--0.00204
m/s^2 across the four exact nominal cases. These late results explain the
strict 10--30 s failures; they do not retroactively change the scoring window.
The corresponding final acceleration errors are 0.000573--0.001043 m/s^2.
Settling timestamps in noisy cases mean only 'stays within bounds through
30 s'; a late crossing near the horizon does not prove sustained convergence.

The outage and weak-direction position peaks after 10 s are 1.3078 m and
1.1560 m. Their 25--30 s position RMSE recovers to 0.1028 m and 0.0884 m.
Complete removal of pose observations is a negative control: position RMSE
480.515 m over 10--30 s and final/peak position error 1046.2 m. It remains
finite but leaves the operating envelope for 2191 samples. This is failure
of this tested configuration without absolute pose feedback, not a claim
about every possible observer design.

Holding the noisy sensor samples fixed and halving only the maximum integration
step to 5 ms changes position by at most 4.184e-6 m, yaw by 0.0003692 degrees,
velocity by 1.144e-5 m/s, and acceleration by 2.812e-6 m/s^2 over the full run.
This supports numerical consistency of this trial, not an integration-error
bound for all operating conditions.

## Software checks and artifacts

All **60/60** global-observer tests pass, including four added checks for the
new direct-input interface: exact moving-truth preservation, mismatched clock
rejection, nonfinite-input rejection, and equality with the normal cascade
when its output is supplied directly. The synthesis test is deliberately
excluded because gains were reused and their stored certificate checked.
The three changed/new MATLAB files have zero Code Analyzer findings.

The first driver attempts contained a plotting delimiter error and a fieldless
structure-accumulation error. Both were corrected in the harness; two complete
campaigns then ran, the second adding late-window metrics. No observer-gain or
runtime propagation defect was inferred from those harness failures.

```matlab
setupVehicleLocalization;
report = validateStandaloneObserver('output/standalone_observer_20260913');
suite = testsuite('tests/improvedObserverTest.m');
results = run(suite(~contains({suite.Name},'synthesis')));
assertSuccess(results);
```

Versioned summaries: `research/standalone_observer_20260913/{metrics.csv,
summary.json,tests.csv,validation.json}`. Full local trajectories, sensor inputs,
configuration, gain matrices and test objects: `output/standalone_observer_20260913/`.
The same directory contains `errors.png/.pdf` and `states.png/.pdf`; these
were visually inspected. Generated MAT and image/PDF files are excluded from
Git and retained in the local output and an identified archive export.
