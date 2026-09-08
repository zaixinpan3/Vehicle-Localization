# Pose-pulse observer implementation and recorded validation

Date: 2026-09-07. Baseline: `0879a1a8dc340830ce1c598ccdc2d97074e4cdd4`.

The [information and timing study](lidar_information_stability_conditions.md)
identified an unstable GNSS-free feedback structure and a feasible replacement
using the base pose channel. This implementation installs that structure,
reduces acceleration peaking, and aligns physical-time pulses and causal
outputs with the stated model. On the complete Mississippi
`2024-06-07-12-09-31` replay, the 20-second GNSS-position outage now completes
with position RMSE **0.32694 m** and maximum **0.91476 m** during the outage.
Removing GNSS position for the entire run after initialization gives full-run
RMSE **0.46960 m**, maximum **3.65918 m**. These are empirical results; the
recorded measurement gaps exceed the verified timer certificate.

## Implemented observer

The output remains `[X,Y,psi]`; the internal global state is
`[X,Vx,Ax,Y,Vy,Ay,psi]`. The existing lateral observer supplies lateral velocity,
sideslip and its derivative. During a qualified LiDAR pulse, the correction is

```text
T*K*diag(1,1,wPsi)*(measuredPose - predictedPose)
  + T*N/theta^3*(measuredInvariants - predictedInvariants).
```

The yaw residual uses the shortest arc. The model and invariant correction
continue between pose pulses. The former auxiliary `T*(P\Cl')*W` XY feedback
is removed. After information admission, XY has unit base-channel weight;
heading weight lies in `[0.15,1]`. Relative to the original design, `N` and
the acceleration rows (3 and 6) of `K` are each multiplied by 0.1. Other rows
of `K` are retained. Actual runtime scaling is `sigma=theta=3.5`.

`computeLidarInformationWeights` checks the complete 3-by-3 pose information
matrix, including XY/yaw cross terms, against a configurable matrix lower
bound. Missing, nonfinite or rank-deficient information disables the whole
LiDAR pose, including heading. Heading weighting uses marginalized yaw
information. The default normalized eigenvalue floor is `1e-8` with pose
scales `[1 m,1 m,1 rad]`. This is a numerical full-rank admission rule, not a
calibrated bound on matching error. `errorBoundValidated` remains false.

Each admitted pose acts over a half-open 30 ms interval at its physical
timestamp. GPS may substitute XY within the same full-pose pulse, preserving
the homogeneous gain covered by the certificate. It does not add an extra
position gain between recent LiDAR pulses. If a full LiDAR pose is no longer
recent (110 ms), GPS-only position pulses continue. This continuation does
not have the full-pose/heading stability certificate; GPS position availability
alone is not a proof of heading observability.

The nonlinear invariant map evaluates velocity and acceleration arguments
through componentwise clipping to 16 m/s and 5 m/s². This is a Lipschitz
extension of the invariant map, not a state clamp: the estimated state and
linear model prediction are never clipped or reset. Almost-everywhere clip
derivatives lie in `[0,1]`, so the extended map's generalized Jacobians remain
inside the same interval box used for verification. It removes the need to
assume the *estimated* velocity and acceleration stay inside that box.
True-state bounds, bounded disturbances and a consistent heading chart remain
necessary assumptions.

RK4 steps split at actual pulse start/end and mode-transition boundaries.
Terminal stages use the left-limit mode, avoiding a quadrature-dependent pulse
duration. The default maximum substep is 10 ms and future-timestamp tolerance
is zero. Delayed measurements replay physical-time history with a 1-second
buffer; the default fixed LiDAR delay is 150 ms. Public `pose`, `position`,
`heading`, `velocity`, `acceleration`, `speed` and `onlineZ` are causal.
`revisedZ` and the legacy `z` field explicitly contain retrospectively revised
history. Recorded accuracy is calculated from causal outputs.

## Certificate, assumptions and unsuccessful extensions

The shipped JSON stores the gains and a piecewise-affine timer metric for
qualified-pose intervals **50–110 ms**, pulse duration 30 ms, track-angle rate
bounded by 0.6 rad/s, and the operating box above. The design loader checks
every matrix inequality independently; runtime rejects changed gains, metric,
timing or operating assumptions against its verification snapshot.

| Independent numerical check | Result |
| --- | ---: |
| Distinct robust flow inequalities | 1,441,792 |
| Largest flow eigenvalue, including decay term | -0.04975026729 |
| Minimum / maximum metric eigenvalue | 0.08742975135 / 2.71717676777 |
| Largest reset eigenvalue | -0.00009999981307 |
| Symmetry discrepancy | 0 |
| Decay rate for the Lyapunov error norm | 0.01 /s |

SeDuMi reported numerical-difficulty status 4. The final recovered point
passed exhaustive strict residual checks after adding violating vertices;
this establishes numerical feasibility, not solver optimality or an
interval-arithmetic proof. The homogeneous timer result yields a conditional
bounded-disturbance error inequality. Matching, hold, model, lateral-observer
and numerical errors still need a justified common budget. Thus
`certificateVerified` reports the design check, while `observer.certified`
remains false for an uncalibrated complete runtime.

For a consistent causal replay tail of length `tau`, a union of full-pose,
prediction and GPS-only modes gives normalized Euclidean logarithmic norm
`L=10.89497450 /s`. Its general finite-tail bound is

```text
||epsilon(t)|| <= exp(L*tau)*||epsilon(t-tau)||
               + dEbar*(exp(L*tau)-1)/L.
```

The homogeneous factors are 5.12559 at 150 ms and 26.27170 at 300 ms. This
bound explicitly includes GPS-only continuation and arbitrary mode ordering
inside the finite tail. A smaller LiDAR-only single-pulse bound, 1.88316 at
150 ms, is retained as a specialized calculation; it is not substituted for
the general bound. Consistent replay, bounded relevant delays, sufficient
history, valid model bounds and bounded input errors are prerequisites.

The first installed research candidate (reduced `N`, original acceleration
rows of `K`) still reached 46.19 m maximum error without GNSS. Extending the
invariant map alone reached 47.47 m. These were failed accuracy outcomes,
despite finite completion. Reducing the acceleration rows and resynthesizing
the metric produced the reported 3.659 m maximum.

An isolated position triple-integrator calculation explains the tradeoff.
For a 30 ms pulse and 500 ms update interval, the cycle spectral radius falls
from 1.2907 to 0.8829 after this gain reduction. At 100 ms it rises from
0.8977 to 0.9753: less aggressive acceleration feedback sacrifices nominal
convergence speed. At 1.3 seconds it remains above one (2.1902), so this
calculation cannot establish stability for repeatedly occurring long gaps.
It is not a substitute for the seven-state robust certificate.

Attempts to extend the chosen timer-metric ansatz to 50–300 ms and 50–1500 ms
were infeasible. This does not prove those ranges are impossible for another
observer or certificate. The final range remains 50–110 ms. The synthesis
script also rounds timer knots before deduplication to avoid near-zero-width
segments from floating-point duplicates.

## Complete recorded experiment

All 1,170 raw scans were processed again by coarse perception and D2D against
the existing frozen map. The observer comparisons then reused those freshly
computed calls. The same nominal/sourced MnCAV vehicle configuration and
lateral design were retained. Each run has 11,690 samples, from 0 to 116.89 s.
The GNSS-position outage is `[40,60)` seconds. Initialization is the same
reference-based offset (`X+0.5 m`, `Y-0.4 m`, `psi+2 degrees`) with zero initial
global velocity and acceleration. No subsequent reference position/yaw is fed
as a localization estimate in the no-GNSS case.

| Scenario | Delay (ms) | Full-run position RMSE / max (m) | Outage-window RMSE / max (m) | Yaw RMSE (deg) |
| --- | ---: | ---: | ---: | ---: |
| GNSS + LiDAR | 150 | 0.28264 / 2.22285 | 0.28688 / 0.66866 | 0.5821 |
| GNSS absent for 20 s, LiDAR available | 150 | 0.29017 / 2.22285 | **0.32694 / 0.91476** | 0.5821 |
| No GNSS position after initialization | 150 | **0.46960 / 3.65918** | 0.32766 / 0.91478 | 0.5811 |
| GPS position only | — | 0.21617 / 1.58037 | 0.20389 / 0.22833 | 1.9173 |
| GNSS + LiDAR, longer fixed delay | 300 | 0.22122 / 1.57641 | 0.21707 / 0.47289 | 0.5887 |
| GNSS absent for 20 s, no LiDAR | — | 9.44099 / 42.74984 | **22.47695 / 42.50246** | 1.4855 |

The previous observer failed at 49.57 s in the 20-second GNSS outage; the new
observer completes that case. Normal fusion RMSE changes from the prior
0.23473 m to 0.28264 m: this is not an improvement in every accuracy metric.
The longer-delay case is a sensitivity check, not evidence that more delay is
better. GPS continuation changes the mixture of corrections in that tail.
The no-LiDAR outage control remains inaccurate despite finite completion.

There are 963 D2D accepted matches; 962 arrive before the 150 ms replay ends
(961 with 300 ms). The full no-GNSS case accepts zero GNSS-position events.
The maximum qualified-pose gap is **1.298701 s**, and 184 intervals exceed
110 ms in the 150 ms cases. No interval is shorter than 50 ms. Therefore the
recorded trajectory does **not** satisfy the certificate's timing hypothesis.
The finite errors above validate these particular runs empirically; they do
not certify the recorded schedule or arbitrary future outages.

The map was built from this same drive and includes query observations.
Recorded INS roll/pitch is retained for D2D; vehicle dynamic parameters are
nominal rather than fully identified. This is neither independent-map
generalization nor a complete loss of the GNSS/INS device. Matching-error
calibration and a certificate covering the actual accepted-event schedule
remain open technical requirements for a stronger deployment claim.

## Timing, checks and reproduction

With eight MATLAB computational threads, fresh coarse-perception + D2D total
time has median **71.671 ms**, p95 **81.904 ms**, p99 **89.684 ms**, and maximum
**738.184 ms**. Seven calls exceed 100 ms; four exceed 150 ms, all within the
first five frames. The first frame is the maximum. No hard 150 ms processing
deadline is established by this cold-start run. The fixed 150 ms replay delay
is an experiment setting, not a proven worst-case runtime. Perception and
registration algorithms were unchanged in this task.

Observer replay costs 0.849 ms per output sample for fusion, 0.809 ms for the
20-second outage and 0.648 ms for full no-GNSS operation, amortized over the
whole run including replay. These are measured CPU-work averages, not
worst-case event response times.

**50 distinct tests passed**: 26 global-observer, 15 lateral-observer, seven
registration-information and two recorded-data regression tests. The two
data tests initially skipped without a data-root environment variable; both
were rerun with the available data root and passed. Tests exercise synthesis
and exhaustive verification, information cross terms and rank rejection,
LiDAR-only synthetic timing, GPS substitution, event splitting, causal
publication, fixed-delay replay, invariant extension and stale-design
rejection. Factory MATLAB Code Analyzer reports zero findings for the checked
implementation files. Recorded trajectory/error figures were visually
inspected. MATLAB version: R2026a Update 3.

```matlab
setupVehicleLocalization;
validatePoseObserverImplementation( ...
    'output/mississippi_20240607_120931_20260907', ...
    'output/observer_implementation_20260907');
```

The default reruns perception/D2D. The final six-case invocation used
`RerunPerception=false` because the complete calls had already been freshly
generated earlier in this task. YALMIP/SeDuMi are required for synthesis;
reference-matrix verification does not require optimization.

Compact numerical exports, source/input hashes, comparisons, 10 Hz error
traces and test outcomes are committed under
[results/observer_pulse_implementation_20260907](results/observer_pulse_implementation_20260907/).
Full-rate local artifacts remain under `output/observer_implementation_20260907`.
The existing flat `localization/` reorganization is completed in this change;
old continuous-design functions are retained under explicit research names.
Unrelated agent instructions, user draft/reference files, raw data, solver
dependencies and generated binary outputs are excluded from the commit.
