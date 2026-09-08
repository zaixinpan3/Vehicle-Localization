# Vehicle observer

The localization output is `[X,Y,psi]`. The independent lateral observer feeds
lateral velocity, sideslip and its derivative to the seven-state global model
`[X,Vx,Ax,Y,Vy,Ay,psi]`. Use `setupVehicleLocalization` from the repository root.

## Production pulse observer

`improvedObserverConfig` and `improvedObserverReferenceDesign` define the shipped
`aperiodic-pose-v1` design. The reference function exhaustively verifies its
stored matrices. `designImprovedObserverGains` can re-synthesize the timer metric
using YALMIP/SeDuMi; it accepts a recovered point only after independent checks.

A qualified LiDAR pose acts for 30 ms at its physical timestamp:

```
zHatDot = model(zHat,input)
        + T*K*diag(1,1,wPsi)*(pose - [XHat;YHat;psiHat])
        + T*N/theta^3*(measuredInvariants - predictedInvariants)
```

Yaw residual uses the shortest arc. XY has unit base-channel weight. There is
no additional `T*(P\Cl')*W` position injection. Invariant gain is one tenth of
the earlier design; acceleration rows of the base pose gain are also reduced
by a factor of ten to limit peaking under sparse updates. Theta is 3.5. GPS may substitute XY during the same pulse.
It does not inject a second gain in the gap between recent LiDAR pulses. If
there is no recent full LiDAR pose, GPS-only position pulses continue, but that
mode is explicitly outside the full-pose/heading certificate.

`computeLidarInformationWeights` uses the complete 3x3 information matrix,
including XY/yaw cross terms and a configurable matrix lower bound. Missing,
nonfinite or rank-deficient information disables the entire LiDAR pose channel.
Qualified heading weight uses marginalized yaw information and lies in
`[0.15,1]`. The default admission tests numerical full rank; it does **not**
assert that D2D curvature has been calibrated as an inverse covariance.

## Certificate and runtime conditions

The timer certificate covers qualified pose intervals from 50 to 110 ms,
30 ms pulses, velocity components bounded by 16 m/s, acceleration components
by 5 m/s² and track-angle rate by 0.6 rad/s. The invariant map uses clipped
velocity/acceleration arguments outside this box, a Lipschitz extension whose
Jacobians remain in the same certified interval box. This removes an
estimated-velocity/acceleration domain assumption; true-state, disturbance and
heading-chart conditions still matter. The stored metric is piecewise
affine in time since a qualified pose. All 1,441,792 distinct flow inequalities
and all permitted timer resets are checked. `sigma=theta` means verification
at the actual gain scaling, not reuse of a continuous-output design condition.

The runtime checks that gains, metric, envelope and pulse duration match the
verified model. It exposes `observer.certificateVerified` separately from
`diagnostics.certificateConditions`. A verified matrix is not an unconditional
certificate for the measured trajectory: missing/rapid poses, excessive gaps,
delay mismatch, initial peaking, model/input error and information calibration
must be assessed separately. `observer.certified` remains false because the
complete runtime's disturbance, domain and numerical-error budgets have not
been established. GPS availability alone does not prove yaw observability.

RK4 steps split exactly at arrived pulse boundaries and use the left limit at
terminal stages. A pulse cannot acquire a different duration from numerical
quadrature or a timestamp tolerance. No future timestamp allowance is used.
Delayed poses replay their physical-time history. `pose`, `position`, `heading`,
`velocity`, `acceleration` and `onlineZ` are causal outputs. `revisedZ` and the
legacy `z` field contain the subsequently revised history. Fixed LiDAR delay defaults to
150 ms and is explicitly checked in diagnostics. Buffer duration must cover it.
The integrator does not reset/clamp a diverging state or silently extend a pulse.

## Reproduction

```matlab
setupVehicleLocalization;
cfg = improvedObserverConfig;
design = improvedObserverReferenceDesign(cfg);
estimate = runImprovedVehicleObserver(sensorData,lateralDesign,design,cfg);
results = runtests('tests/improvedObserverTest.m');
```

The former GNSS-present continuous design is retained as
`designContinuousObserverGains` / `verifyContinuousObserverDesign` for research
and legacy artifact audits; it is not accepted by the production runtime.

See `research/lidar_information_stability_conditions.md` for the derivation
and `research/observer_pulse_implementation.md` for recorded validation and
its limits. The flat `localization/` directory preserves the existing workspace
reorganization; lateral-observer functions remain under `lateralObserver/`.
