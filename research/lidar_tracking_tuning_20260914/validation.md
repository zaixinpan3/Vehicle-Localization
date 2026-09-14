# Fixed-delay LiDAR tracking gain tuning

Executed 2026-09-14 on the current continuous delayed observer (MATLAB R2026a
Update 3). This comparison uses the current implementation, not the historical
pulse/transport observer from the first standalone experiment.

## Result and use

The optional `tracking` LiDAR preset changes each position-driven state-chain
coefficient from `[3;3;1]` to `[3;3;1.5]`, at unchanged theta=1. Thus only the
acceleration rows of K increase by 50%. The yaw gain and all auxiliary gains
N remain unchanged. The new constant P/Q/R certificate is saved in
`config/lidarTrackingCertificate.json` and independently checked whenever
loaded. The default `reference` configuration and the GNSS `lowPeaking`
profile remain available and unchanged.

```matlab
setupVehicleLocalization;
cfg = improvedObserverConfig("lidar","tracking");
design = improvedObserverReferenceDesign(cfg);
% Supply the normal continuous delayed pose source and initial history:
% estimate = runImprovedVehicleObserver(data,lateralDesign,design,cfg, ...
%     InitialHistory=history);

% Reproduce the search (YALMIP/SeDuMi must be on the path):
report = tuneContinuousLidarTracking;
```

The preset loads stored constant matrices without a solver. Calling
`designImprovedObserverGains(cfg)` also uses the selected K/N when synthesizing
a fresh certificate; that path was executed and independently verified.

## Controlled independent experiment

- Global observer only, with directly supplied analytic lateral inputs; no
  perception, map, registration, GNSS or lateral-estimator execution.
- Duration 40 s, high-rate grid 100 Hz, maximum RK4 step 5 ms. Continuous
  virtual LiDAR is supplied through `evaluate(t)`. Its pose is truth at
  `t-0.15`, plus continuous bounded noise. This is not a 10 Hz pulse experiment.
- Speed `8+1.2*sin(0.35*t)` m/s; course angle
  `0.2+0.001*t+0.00125*(1-cos(0.4*t))` rad; course rate
  `0.001+0.0005*sin(0.4*t)` rad/s; sideslip `0.0002*sin(0.3*t)` rad.
  Yaw is course minus sideslip. Velocity and acceleration are exact analytic
  derivatives. Position uses independent 40-node Gauss-Legendre quadrature,
  checked at 21 times against MATLAB adaptive `integral` to 2.874e-13 m.
- Initial error in `[X,Vx,Ax,Y,Vy,Ay,psi]` is
  `[1,.3,.1,-1,-.2,-.1,10 degrees]`. The initial delayed history is analytic
  truth plus that same fixed bias on [-0.15,0]; it is an imperfect prior,
  not an ongoing state correction. Every candidate uses the identical history.
- Noisy high-rate inputs: longitudinal speed .02*sin(1.1*t+phase) m/s;
  longitudinal acceleration .02*sin(1.7*t+phase) m/s^2; lateral acceleration
  .02*cos(1.4*t+phase) m/s^2; yaw rate .0002*sin(.6*t+phase) rad/s.
  The supplied lateral velocity/sideslip/rate remain exact to isolate the
  global observer. There is no random number generator.
- Pose noise: [.01*sin(1.3*t+phase), .01*cos(.9*t+phase)] m and
  .1*sin(.7*t+phase) degrees. Information is 1e6*I for all candidates, giving
  W=1e6/(1e6+5)*I. Information is a declared design weight, not a calibrated
  inverse sensor covariance. Noise, information, pose scales and delay are
  never improved as part of tuning.
- Original operating bounds are retained: component speed <=16 m/s,
  acceleration <=5 m/s^2, course rate <=0.002150319819 rad/s, and
  W >=0.998520048402*I. The simulated true course rate is <=0.0015 rad/s;
  including the bounded gyro noise its supplied magnitude is <=0.0017 rad/s.
  This is a variable-speed **gentle turn**, not a certificate for ordinary
  turns near 0.1--0.6 rad/s. The reference certificate remains restrictive.

## Search and selection rule

Twenty explicitly listed K-chain candidates are in `certificate_screen.csv`.
Only K position/velocity/acceleration entries change; theta, delay, N, yaw K,
all bounds and the desired certificate rate .05/s stay fixed. The same
constant-matrix synthesis constraints as the production design routine are
solved for each non-reference candidate using YALMIP and SeDuMi. Sixteen
candidates pass independent verification; four are infeasible under this
certificate and are not integrated. Infeasibility is not evidence of actual
dynamical instability. All 32 clean/noisy accepted-candidate runs complete.

The predeclared selection rule minimizes noisy acceleration RMSE subject to
position RMSE <=1.05 times baseline, velocity RMSE <=baseline, both full-run
velocity/acceleration peaks <=1.5 times baseline, and no supplied course-rate
excursion. RMSE is evaluated over inclusive 20--40 s samples; peaks use the
entire 0--40 s record. This is a finite candidate comparison with an explicit
startup tradeoff, not a global optimum or an application-level safety limit.

| Noisy metric | Reference [3,3,1] | Selected [3,3,1.5] | Change |
|---|---:|---:|---:|
| Position RMSE (m) | 0.083774 | 0.067376 | -19.57% |
| Velocity RMSE (m/s) | 0.250024 | 0.199979 | -20.02% |
| Acceleration RMSE (m/s^2) | 0.258457 | 0.208083 | -19.49% |
| Heading RMSE (deg) | 0.0572425 | 0.0572419 | Essentially unchanged |
| Full-run velocity-error peak (m/s) | 1.11144 | 1.20159 | +8.11% |
| Full-run acceleration-error peak (m/s^2) | 0.398128 | 0.539833 | +35.59% |
| Independently checked matrix margin | 0.118590 | 0.142528 | Positive for both |

A more aggressive certified candidate [6,10,8] reduces noisy acceleration
RMSE to 0.131713 m/s^2, but its velocity peak rises to 3.37248 m/s and its
acceleration peak to 2.46415 m/s^2. Its margin is only 0.00653556. It is not
selected because it violates the peak criterion. This demonstrates the cost
of optimizing settled error without controlling the transient.

Three additional noise phases 1, 2 and 3 radians, unused for selection, give
selected position RMSE 0.06776--0.06801 m, velocity RMSE 0.19999--0.20039 m/s,
and acceleration RMSE 0.20786--0.20816 m/s^2. Corresponding reference ranges
are 0.08425--0.08461 m, 0.25015--0.25053 m/s and 0.25837--0.25855 m/s^2.
These six independent-phase runs confirm the improvement on the same motion;
they are not independent vehicle/trajectory validation.

## Verification and limitations

- 53/53 tests pass: all 46 continuous observer tests, 3 existing GNSS
  low-peaking tests, and 4 new LiDAR profile/integrity checks. The latter
  verify unchanged bounds/delay/N, reject a GNSS mode request for this profile,
  and ensure altered delays/gains cannot reuse a stored success flag.
- Public `tracking` profile run reproduces the selected sweep states exactly.
- Fresh selected-profile certificate synthesis succeeds with margin 0.142528.
- Repeating the selected noisy trajectory with 2.5 ms steps changes any
  position coordinate by at most 2.734e-11 m, velocity by 3.014e-11 m/s,
  acceleration by 1.698e-11 m/s^2, and yaw by 6.354e-14 rad. This checks the
  smooth trial's numerical consistency, not an all-input error bound.
- Factory MATLAB Code Analyzer reports zero findings in all five changed/new
  MATLAB files. The final diff passes `git diff --check`.
- `comparison.png` was rendered and visually inspected. Full states, inputs,
  all accepted certificates, public-profile validation, fresh synthesis and
  figures are retained under `output/lidar_tracking_tuning_20260914/`.
  CSV/JSON summaries, tests and analyzer counts are versioned here.

This reduces sustained errors rather than forcing zero error under nonzero
model disturbance. It does not change or add terms to the dynamic model,
change initialization, clip the estimate, inflate information, reduce delay,
or narrow the existing certificate range. The continuous LiDAR assumptions,
initial history, physical disturbance bounds and yaw representation still
need to hold. The matrix certificate is conditional; `observer.certified`
is not set to true by this result. No real-data, dropout, sharp-turn,
parameter-mismatched vehicle, or complete cascade accuracy claim is made.

## Target-vehicle qualification

The target is UMN MnCAV, based on the 2021 Pacifica Hybrid. This original
mass-free kinematic sweep is a generic independent-observer experiment.
The [subsequent MnCAV parameter check](../mncav_parameter_validation_20260914/validation.md)
uses source-qualified stock geometry and explicitly unidentified dynamic priors
to generate bicycle motion. Its gentle-turn comparisons preserve the gain
improvement, but recorded MnCAV course rates exceed the present certificate.
The profile must not be described as validated for the full MnCAV operating
range or its actual lateral/sensor uncertainty.
