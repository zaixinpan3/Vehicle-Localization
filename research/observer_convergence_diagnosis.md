# Causes of standalone observer convergence limitations

Executed on 2026-09-13, MATLAB R2026a Update 3. This diagnosis follows
`standalone_observer_validation.md` and reuses its saved synthetic measurements.

## Conclusion

The observed behavior has distinct causes: initialization error combined with
strong position-to-velocity correction causes startup peaking; the present
acceleration gains produce a slow nominal mode; persistent maneuvering error
reflects model mismatch filtered through the available correction bandwidth;
and loss/weakening of absolute measurements removes correction authority.
The observations do not show that the generalized high-gain observer framework
is intrinsically unable to converge. They also do not support fixing all the
issues by increasing one global gain.

The earlier 6/12 screening count combines different criteria: four nominal
cases failed a 0.01 m/s^2 acceleration RMSE limit over 10--30 s even though
their late acceleration errors decayed to roughly 0.001--0.002 m/s^2. This
is a slow transient, not evidence of a nonconvergent noiseless nominal system.

## Nine production-runtime ablations

Only initial conditions and supplied measurements change. Production gains,
nonlinear correction, pulse timing, transport and operating bounds are unchanged.
Maneuver cases use 150 ms delay, 10 Hz virtual LiDAR, 30 ms pulses and 100 Hz
motion inputs. Straight cases retain the original zero-delay setup.
All cases are 30 s. RMSE uses 10--30 s; peak error uses the entire run.
Noise realizations are taken from the original seed-20260913 traces without
resampling. The biased noisy baseline reproduces its saved states exactly.

| Change | Velocity error peak (m/s) | XY RMSE (m) | Acceleration RMSE (m/s^2) |
|---|---:|---:|---:|
| Straight, original biased initialization | 10.996 | 0.000964 | 0.03420 |
| Straight, exact initialization | 1.09e-12 | 3.62e-14 | 6.35e-14 |
| Maneuver, no noise, biased initialization | 11.286 | 0.02935 | 0.49225 |
| Maneuver, no noise, exact initialization | 0.398 | 0.02939 | 0.49309 |
| Maneuver, all noise, biased initialization | 11.324 | 0.10117 | 0.49666 |
| Maneuver, all noise, exact initialization | 0.932 | 0.10121 | 0.49750 |
| Maneuver, all noise, correct initial XY only | 1.281 | 0.10110 | 0.49368 |
| Maneuver, pose noise only, biased initialization | 11.324 | 0.10117 | 0.49663 |
| Maneuver, motion-input noise only, biased initialization | 11.286 | 0.02935 | 0.49229 |

Exact initialization is a diagnostic use of truth, not a deployable
initialization method. In the initial-XY-only case the original velocity,
acceleration and 15-degree yaw errors remain unchanged. Its reduced peaking
therefore specifically implicates the initial position residual in this trial.
A practical implementation would use a valid initial pose/prior and a tested
startup correction policy, not access simulation truth.

Position/yaw noise dominates the tested steady position error: removing only
motion-input noise has essentially no effect (0.10117 m); removing pose noise
reduces it to 0.02935 m. Removing both noise and initial error barely changes
the maneuver acceleration RMSE. Thus the roughly 0.49 m/s^2 maneuver error
cannot be attributed mainly to bad initialization or these chosen noise levels.

## Why the present gain allocation is slow

The physical pose gain is `L=diag(theta.^[1,2,3,1,2,3,1])*K`, with theta 3.5.
Its dominant same-axis X-chain entries are approximately
`[16.887;127.889;31.022]` for position, velocity and acceleration. These rows
have different physical units and should not be compared as dimensionless
numbers. Nevertheless, a position residual directly creates a large velocity
derivative through the second row. A 2 m residual can create a roughly
256 m/s^2 correction contribution during an active pulse before weighting,
transport, coupling and other terms. This explains why rapid position
recovery can coexist with substantial velocity peaking.

The gain artifact explicitly records
`gainDesign.accelerationRowScaleFromResearchCandidate=0.1`, with the reason
of reducing acceleration correction peaking during sparse pose updates.
The present acceleration allocation therefore reflects an earlier design
tradeoff, not performance optimization for this new standalone maneuver.

A **local linear diagnostic**, not a new certified gain design, linearizes the
intrinsic output map at the true straight-motion operating point. For the
original zero-delay 10 Hz schedule, the on-pulse error Jacobian is

```text
A - Nphysical*H - expm(A*t)*Lcandidate*W*C/expm(A*t),  0 <= t < 0.03
```

and the off-pulse Jacobian is `A-Nphysical*H` for 0.07 s. The on-pulse matrix
transition is integrated using ode45 (RelTol 1e-10, AbsTol 1e-12), followed by
the exact off-pulse matrix exponential. Only acceleration rows of the candidate
pose gain are scaled. This includes within-pulse transport; the initial
exploratory constant-current-pose approximation was replaced before saving
the final diagnostic results.

| Acceleration row multiplier | Period-map spectral radius | Dominant decay time constant (s) |
|---|---:|---:|
| 0.5 | 0.98995 | 9.899 |
| 1 (current) | 0.97739 | 4.373 |
| 2 | 0.95158 | 2.015 |
| 5 | 0.87052 | 0.721 |
| 10 | 0.90618 | 1.015 |

The current local dominant mode is stable but slow. Multiplying the
acceleration rows by five speeds up this nominal local decay markedly;
ten is already worse than five. This is evidence that gain allocation matters,
not permission to deploy the factor-five candidate. The sweep does not cover
150 ms delay, nonlinear transients, noise amplification, weak geometry or
outages. No certificate snapshot was altered to admit changed runtime gains.

## Why the maneuver retains acceleration error

Let `v` be speed, `q` course rate, `e_c` the course-direction unit vector, and
`J` a 90-degree planar rotation. The current reduced model predicts global jerk
as `q^2*V + 2*q*J*A`. Differentiating the physical maneuver leaves the omitted
terms `v''*e_c + v*q'*J*e_c`. They are nonzero in the maneuver (maximum norm
0.454351 m/s^3) and zero in the exact straight/circle nominal fixtures.
This is a structural approximation of the present motion model. Its sustained
forcing can leave nonzero tracking error even when initial error and measurement
noise vanish. The nonzero-error exact-initialization maneuver and machine-level
straight case are consistent with that explanation. The existing step-refinement
result also makes the 10 ms integration step an implausible dominant explanation.

More correction bandwidth can reduce this forced response, generally with
noise/transient tradeoffs to be measured. Improving the nominal model or adding
a justified disturbance/acceleration measurement treatment is another route.
Any such change requires a new derivation and validation; this diagnosis does
not implement or validate those alternatives.

## Measurement-loss limits and next engineering decision

The earlier outage and weak-direction results remain evidence of reduced
measurement authority. With no absolute pose measurements, the intrinsic
channels have no X/Y dependence, so absolute translation cannot be corrected
from them. Changing gains cannot create missing absolute-position information.
The extreme no-pose drift in the earlier run also includes imperfect velocity/
acceleration dynamics; it is not solely a translation-observability statement.

The justified next step is to improve initialization/startup handling and tune
position/velocity/acceleration gain allocation jointly, explicitly balancing
startup velocity peak, late acceleration error and noise sensitivity. Retain
separate outage tests. If maneuver acceleration accuracy remains insufficient,
revisit the omitted dynamics rather than keep raising a single high-gain scale.
The diagnostic factor-five candidate is a starting hypothesis, not a selected
production parameter. No production algorithm or configuration was changed.

## Reproduction and checks

```matlab
setupVehicleLocalization;
% Run validateStandaloneObserver first if its local traces do not exist.
report = diagnoseStandaloneObserverConvergence;
```

Inputs: `output/standalone_observer_20260913/traces.mat` from the previous
campaign. Full diagnostic states and matrices: `output/observer_convergence_diagnosis_20260913/`.
Versioned CSV/JSON summaries: `research/observer_convergence_diagnosis_20260913/`.
Nine nonlinear runtime ablations and five local linear gain candidates were
executed. Baseline state reproduction is checked to 1e-12 and is exact.
Code Analyzer reports zero findings for the new diagnostic driver.
The production runtime's SHA-256 remains the previously tested
`aa12015cbd63515911f5403640b08ad1ca99e8482bc5dccc2fe2997a319c46b2`.
No new unit suite or robust gain certificate is claimed for this research-only
addition.
