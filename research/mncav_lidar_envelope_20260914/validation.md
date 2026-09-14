# MnCAV continuous delayed-observer operating envelope

Date: 2026-09-14. This supersedes the narrow-rate limitation for the new
`mncav` profile only; historical `reference` and `tracking` results keep their
original configuration and validity scope.

## Design requirement and meaning of a certificate

A certificate here means positive matrices and strictly negative matrix
inequalities supporting the project's conditional continuous-time
input-to-state stability theorem. It is not a LiDAR hardware certificate or
a vehicle turn-rate limit. An operating-bound excursion removes coverage of
that sufficient proof; it does not prove instability. Actual motion must drive
the design envelope rather than be reduced to fit a convenient proof.

The existing saved MnCAV replay audit
`output/continuous_observer_runtime_20260913/recorded_conditions.json` reports
maximum supplied course rate 0.354975175657248 rad/s. This is the prior audit,
not a new real LiDAR replay. The coefficient is q = measured yaw rate + supplied
sideslip-angle derivative, not yaw rate alone. Select |q| <= 0.4 rad/s, about
12.7% above this observed maximum, as a data-informed design requirement.
It is not an independently established bound for every future drive. Retain
the synthetic 150 ms delay, velocity/acceleration component bounds 16 m/s and
5 m/s^2, and information sector 0.9985200484020392 I <= W <= I.
Sensor delay, information and disturbance limits remain to be established for
a physical MnCAV LiDAR implementation.

## Less conservative proof with the same functional

Let b=0.4 and use the state scaling T of the existing derivation. The scaled
model has the form

    T^-1 M(q) T = theta*A + (q^2/theta)*B2 + 2*q*B1,

where B2(3,2)=B2(6,5)=1, B1(3,6)=-1, B1(6,3)=1, with all other entries zero.
The old construction replaced this entire turn structure by a norm ball
centered at q=0. Its illustrative certified neighborhood was only
0.002150319818947585 rad/s. Widening that number without verifying new matrix
inequalities would not constitute a new proof.

Instead enclose (q,q^2) in [-b,b] x [0,b^2]. The four combinations of the
interval endpoints define four drift matrices. Although some corners are
physically impossible, their convex hull contains every physical q in the
interval. With a=(q+b)/(2*b) and c=q^2/b^2, the weights
[(1-a)*(1-c),(1-a)*c,a*(1-c),a*c] are nonnegative, sum to one, and reproduce
the actual scaled drift. The Schur-form delayed LMI is affine in that drift.
Common P,Q,R,g therefore prove the inequality over the entire interval when
all four vertices pass. This is continuum coverage, not a rate-grid test, and
allows time-varying q without needing a derivative of the constant metric.
Course angular acceleration still contributes to the plant-model disturbance.

Auxiliary-output and information uncertainty retain the prior norm enclosure:

    u = ||N||*sqrt(16*vmax^2+4*amax^2+2)
        + theta*||K||*||C||*(1-wmin).

The symmetric LMI perturbation has norm at most
2*(||P||+delay*||R||)*u. The verified uniform margin is the minimum of the
four recovered block margins minus this bound. No change is made to the
underlying observer equations, delayed innovation, full information matrix,
auxiliary outputs, or constant-metric Lyapunov-Krasovskii functional.

Exploratory structured synthesis at theta=[1,1.5,2,2.5,3,4], using tracking
K/N, rate .05, 150 ms delay and the new rate range, succeeds for 1.5 and 2.
The other four solver problems are reported infeasible; this does not prove
physical instability or infeasibility under every possible functional/gain.
Theta=2 has the larger recovered margin among these two candidates and is
selected here, not claimed as the optimum for noise or initial peaks.
Its independent verifier gives minimum vertex margin about 0.1731 and
remaining uniform margin about **0.0861 > 0**. The solver also recovers
positive P,Q,R and g. The stored numerical artifact is
`config/mncavLidarCertificate.json` and all values are rechecked at load/run.

## Implementation and reproduction

```matlab
setupVehicleLocalization;
cfg = improvedObserverConfig("lidar","mncav");
design = improvedObserverReferenceDesign(cfg);
% With YALMIP and SeDuMi on the path, synthesize independently:
freshDesign = designImprovedObserverGains(cfg);
report = validateMncavObserverParameters( ...
    "output/mncav_lidar_envelope_20260914", ...
    SteeringScale=220, Profiles=["tracking","mncav"]);
results = runtests({'tests/improvedObserverTest.m', ...
    'tests/lidarTrackingPresetTest.m','tests/mncavLidarCertificateTest.m'});
```

`continuousLidarCertificateVertices` supplies the structured enclosure to
both synthesis and verification. The original norm-ball method remains
available for historical profiles. The new profile uses theta=2 and tracking
chain coefficients [3;3;1.5] with unchanged N and yaw K. Since theta changes,
the physical injection gains also change. Config selection and stored
`certified` flags cannot bypass matrix reverification.

## Wider-turn isolated simulations

The same source-qualified nominal MnCAV bicycle plant from the preceding
parameter study is used, with exact lateral velocity/sideslip/rate supplied
to isolate the global observer. Seven plants vary inertia/front stiffness/
rear stiffness separately by 0.7 and 1.3, with other parameters fixed. These
remain unmeasured nominal priors; this is not full-cascade uncertainty
validation. No perception, mapping, registration or lateral estimation runs.

Steering-wheel command is 220*(.006+.002*sin(.4*t)) rad, divided by the stock
ratio 16.2. Its maximum road-wheel angle is about 0.10864 rad, below the
previously recorded maximum 0.191555 rad. Longitudinal speed remains
8+1.2*sin(.35*t) m/s. Both profiles run on identical full inputs, without
clamping q. The 7 plant maximum course rates span 0.28938--0.38714 rad/s;
new-profile stage diagnostics remain within the 0.4 bound. Old-profile
matrices still pass their narrow-domain check but every old-profile trajectory
violates that domain; its numerical results are explicitly out-of-certificate
comparisons, not certified performance at these rates.

Each of the fourteen runs is 40 seconds with 100 Hz input and <=5 ms RK4
steps. Plant integration uses ode45 (relative 1e-10, absolute 1e-12).
Virtual continuous LiDAR has 150 ms delay, information 1e6*I, sinusoidal
noise .01 m per axis/.1 degree yaw; high-rate sinusoidal errors are .02 m/s,
.02 m/s^2 and .0002 rad/s. Initial state/history error is
[1,.3,.1,-1,-.2,-.1,10 degrees]. No RNG is used. RMSE covers 20--40 seconds;
peaks cover 0--40 seconds. These are synthetic assumptions, not calibrated
physical sensor specifications. All simulated true velocity and acceleration
components remain below the declared bounds (maxima 9.2023 and 3.5533).

| Nominal plant metric | Old tracking, outside proof range | New mncav |
|---|---:|---:|
| Position RMSE (m) | 0.16812 | 0.01975 |
| Velocity RMSE (m/s) | 0.50821 | 0.10427 |
| Acceleration RMSE (m/s^2) | 0.54046 | 0.21374 |
| Heading RMSE (deg) | 0.05690 | 0.06765 |
| Velocity-error peak (m/s) | 1.1592 | 4.2436 |
| Acceleration-error peak (m/s^2) | 1.1345 | 3.7070 |

Position/velocity/acceleration RMSE improves in all seven plants. Heading
noise response and startup derivative peaks worsen; the larger bandwidth
trades smaller tracking error for more peaking. The certificate guarantees
conditional ISS, not a desired precision or peak bound. Variable-speed jerk,
course angular acceleration, input errors, and pose noise can leave residual
error even with a valid certificate.

## Validation and scope

- 57/57 MATLAB tests pass: 46 continuous runtime, 4 historical tracking, and
  7 new structured certificate tests. New tests independently reconstruct
  the actual scaled model from the four-vertex interpolation and reject
  reused matrices at 0.8 rad/s, 0.5 s delay or information weight 0.5.
  The same new matrices do not pass the old norm-ball proof at 0.4 rad/s.
- Seven changed/new MATLAB files have zero factory Code Analyzer findings.
- An additional nominal new-profile run with 2.5 ms steps differs from the
  5 ms run by at most 5.0007e-9 over all state samples. This is a refinement
  check, not a formal numerical integration error bound.
- Full fourteen-run histories are in the ignored local output folder;
  CSV/JSON summaries and this report are versioned. The fifteen executed
  simulation runs include the refinement run.
- The new conditional certificate covers the data-informed rate range;
  a physical LiDAR stream, stops, calibrated vehicle dynamics, full cascade
  and all future operating conditions have not been validated here.
