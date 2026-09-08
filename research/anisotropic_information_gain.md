# Information-dependent anisotropic observer gains

Date: 2026-09-08. Baseline: `2bec42742a96ef20b959d997d08d66eee23c7b7b`.

The previous implementation used the information matrix mainly to admit a
full pose and then fixed XY weights to one. That did not implement the intended
direction-dependent LiDAR gain. The current implementation uses the complete
information matrix in every LiDAR correction, preserves XY/yaw cross terms,
and retains observed directions when an incoming measurement is rank deficient.
It does not raise weak weights to satisfy the stability certificate.

## Gain and information fusion

Let the LiDAR information be `I_L`, the pose normalization be `D`, and the
normalized information be `J=D*I_L*D`. For `s>0`, define

```text
W = (s*I + J)^(-1)*J
L_L = T*K*D*W*D^(-1).
```

If `J=U*diag(lambda)*U'`, then
`W=U*diag(lambda/(s+lambda))*U'`. Thus the normalized weight has the same
principal directions as the information. Information eigenvalue `s` gives half
weight; small information gives small correction, and a null direction gives
zero correction. No scalar summary, diagonal approximation or positive floor
replaces this matrix. `D` makes the pose-coordinate units explicit; the physical
weight `D*W/D` need not be symmetric when its coordinate scales differ.

The default is `D=diag([1 m,1 m,1 rad])` and `s=5`. The scale is a configurable
gain-design parameter, not a statistical confidence threshold. It was set
before the complete recorded comparison and was not swept to optimize its
errors. The existing base gain, invariant gain, acceleration-row reduction,
state equations and lateral observer are retained.

With GPS, use `G=D*diag([I_Gx,I_Gy,0])*D` and `A=s*I+J+G`:

```text
correction = T*K*D*A^(-1)*( J*D^(-1)*r_L + G*D^(-1)*r_G )
r_L = [X_L-Xhat; Y_L-Yhat; wrap(psi_L-psihat)]
r_G = [X_G-Xhat; Y_G-Yhat; 0].
```

Both residuals contribute through separate matrices. GPS no longer overwrites
the LiDAR XY residual, and no GPS yaw observation is invented. The total
normalized homogeneous weight is `A^(-1)*(J+G)`, symmetric and between zero
and identity. Adding GPS information cannot reduce it in the Loewner order.
Its default XY information `[25,25] m^-2` is an explicit design reference;
neither sensor's information is claimed to be a calibrated inverse covariance.

Only unusable numerical inputs (missing, nonfinite, materially indefinite or
all-zero information) produce no LiDAR correction. Small negative eigenvalues
within relative roundoff tolerance are removed; positive directions are not
inflated. The observer accepts partial-rank input and exposes its full gain.
The upstream D2D full-pose acceptance policy is unchanged in this task: it
currently emits only accepted full-rank poses. Partial-rank observer behavior
is validated through constructed inputs, not claimed as a new D2D output mode.

At recorded timestamp 95.577984 s, the actual LiDAR-only normalized weight is

```text
[ 0.493176  -0.089944   0.010937
 -0.089944   0.138703   0.075003
  0.010937   0.075003   0.988866 ]
```

Its eigenvalues are `[0.110697,0.514597,0.995451]`: approximately nine times
more weight in the strongest direction than the weakest. Both XY and
XY/yaw cross terms enter the runtime's 7-by-3 LiDAR gain. Across the 962
received poses, the median smallest eigenvalue is 0.598515. The committed
`information_weights.csv` and `example_gain.json` expose these matrices.

## Certificate for arbitrary information directions

The old fixed-XY certificate is not reused. The new design certifies the
normalized sector `alpha*I <= W <= I` at arbitrary orientations, with
`alpha=0.8`, 30 ms pulses and inter-pulse intervals 50–110 ms.

Write `W=c*I+r*Delta`, with `c=(1+alpha)/2`, `r=(1-alpha)/2` and
`||Delta||_2<=1`. In normalized observer coordinates, during a pulse,

```text
A = A0 + B*Delta*E
A0 = theta*(A_model + F_vertex + N*H_vertex - c*K*C_pose)
B  = -theta*r*K*D
E  = D^(-1)*C_pose.
```

For timer metric `P(tau)`, decay rate `rho` and multiplier `mu>0`, require

```text
[ A0'*P + P*A0 + Pdot + 2*rho*P + mu*E'*E,  P*B
  B'*P,                                                   -mu*I ] < 0.
```

The Schur complement and the norm bound imply the flow inequality for every
admissible `Delta`. This norm-ball representation conservatively includes
nonsymmetric perturbations too. It covers all symmetric sector weights,
including rotations and cross terms; checking only diagonal endpoints would
not suffice. The construction uses the norm-bounded uncertainty/S-procedure
form in Boyd, El Ghaoui, Feron and Balakrishnan, *Linear Matrix Inequalities in
System and Control Theory*, §2.6.3, pp. 23–24. The timer and observer-specific
specialization above is the project derivation. [Author-hosted book](https://web.stanford.edu/~boyd/lmibook/lmibook.pdf).

Metric and multiplier interpolation is affine within each timer segment.
Endpoint checks therefore cover the continuous segments; reset checks cover
all permitted arrivals. The existing invariant-extension/model box remains
velocity components within 16 m/s, acceleration within 5 m/s² and track-angle
rate within 0.6 rad/s. True-state, heading-chart and disturbance assumptions
remain necessary.

| Independent verification | Result |
| --- | ---: |
| Robust flow block inequalities | 720,896 |
| Largest flow eigenvalue | -0.007437670568 |
| Minimum / maximum metric eigenvalue | 0.03969819301 / 2.76499452534 |
| Largest reset eigenvalue | -0.00009999978859 |
| Symmetry discrepancy | 0 |
| Error-norm decay rate | 0.01 /s |

The first `alpha=0.5` norm-ball ansatz was infeasible. At `alpha=0.8`, six
constraint-generation iterations produced a recovered point passing every
inequality. SeDuMi status 4 (numerical difficulty) is disclosed; strict
independent residuals establish numerical feasibility, not optimality.
A test changes the sector to 0.2 with the stored metric and confirms failure.

**The 0.8 lower bound is only a proof hypothesis.** It is never applied as a
gain floor or an input rejection rule. In the LiDAR-only case it requires
`J >= 4*s*I`, or `J >= 20*I` at the default scale. All 962 received recorded
LiDAR-only weights violate that sufficient sector bound, and 184 timestamp
intervals exceed 110 ms (maximum 1.298701 s). Consequently these recorded
results are empirical; this certificate does not cover their actual weights
or timing. GPS can strengthen the sector, but the current information audit
conservatively checks LiDAR alone. No unconditional runtime flag is set.

The fixed-delay replay and causal outputs remain in place. A finite-tail
bound uses `0<=W<=I` even outside the positive lower sector, giving the
conservative normalized logarithmic-norm upper bound 44.1873 /s. Its
homogeneous factors are 756.04 at 150 ms and about 571,603 at 300 ms. These
large bounds are not accuracy predictions and supply no useful tight error
budget here. They require consistent replay, bounded input/model errors and
the stated model assumptions; a stronger quantitative delay result remains
open. The previous smaller fixed-XY bound is not silently reused.

## Recorded comparison

Six complete observer replays use the **same stored 1,170-frame coarse
perception/D2D results** computed on 2026-09-07, frozen same-drive map,
recorded inputs, lateral design and biased initialization as the baseline.
Perception and matching are unchanged and were not rerun for this comparison.
Each case has 11,690 causal samples over 116.89 s. Delay is 150 ms except
the named 300 ms case. GNSS-position outage is `[40,60)` seconds.

| Scenario | Previous / current position RMSE (m) | Previous / current max (m) |
| --- | ---: | ---: |
| Normal fusion | 0.28264 / **0.26797** | 2.22285 / **2.02914** |
| GNSS position absent for 20 s, full run | 0.29017 / **0.27098** | 2.22285 / **2.02914** |
| No GNSS position after initialization | 0.46960 / **0.39057** | 3.65918 / **1.89310** |
| GPS only | 0.21617 / 0.21970 | 1.58037 / 1.57736 |
| Fusion, 300 ms delay | 0.22122 / 0.22608 | 1.57641 / 1.62575 |
| No LiDAR, GNSS position absent for 20 s | 9.44099 / 9.59243 | 42.74984 / 43.53499 |

During the 20-second outage, the proposed fusion has RMSE **0.28542 m** and
maximum **0.73309 m**, versus 0.32694/0.91476 m previously. The no-LiDAR
control has outage RMSE/max 22.7910/43.2781 m and remains inaccurate despite
finite completion. No-GNSS yaw RMSE changes from 0.58111 to 0.58672 degrees;
improvement is not uniform across all metrics.

The full no-GNSS comparison isolates the LiDAR weighting change because the
state model, base gains, invariant extension and lateral inputs are unchanged.
Normal fusion also changes GPS/LiDAR blending, so its difference cannot be
attributed solely to anisotropy. The no-GNSS run remains within the configured
estimated velocity/acceleration box throughout; no state clamp is used.

Observer computation averages 0.911 ms/sample for fusion, 0.851 for the outage,
and 0.671 for full no-GNSS operation, including replay, compared with
0.849/0.809/0.648 previously. These are amortized run measurements, not
worst-case deadlines. Reusing identical D2D inputs provides a controlled
observer comparison and does not constitute a new perception timing test.

The map includes query-drive observations, recorded INS tilt is retained,
vehicle dynamics are nominal, and initialization uses one biased reference
pose. This is not a complete GNSS/INS device-loss or independent-map test.
Matching-error calibration and a less conservative certificate covering actual
weight/timing sequences remain unresolved.

## Validation and reproduction

**58 distinct tests passed**: 34 global-observer, 15 lateral-observer, seven
registration-information and two recorded-data regression tests. Tests cover
principal-direction rotation, XY/yaw cross terms reaching the actual gain,
rank-one input, zero/indefinite input, coordinate normalization, separate GPS
and LiDAR gains, full-sector verification and a failing wider-sector control,
plus the existing replay, pulse, causal-output and synthesis checks. All 13
changed MATLAB files have zero factory Code Analyzer findings. The recorded
no-GNSS trajectory/error figure was visually inspected.

```matlab
setupVehicleLocalization;
cfg=improvedObserverConfig;
design=improvedObserverReferenceDesign(cfg); % independent matrix verification
estimate=runImprovedVehicleObserver(sensorData,lateralDesign,design,cfg);
% Existing complete matching calls must be available under the output folder.
validatePoseObserverImplementation( ...
    'output/mississippi_20240607_120931_20260907', ...
    'output/anisotropic_observer_20260908',RerunPerception=false);
```

For this run, `recursive_8threads` in the new output folder is a local symlink
to `../observer_implementation_20260907/recursive_8threads`. The driver default
without `RerunPerception=false` instead regenerates matching inputs.
`designImprovedObserverGains` reproduces synthesis with YALMIP/SeDuMi. MATLAB
R2026a Update 3, eight computational threads. Compact exports and hashes are
under [results/anisotropic_observer_20260908](results/anisotropic_observer_20260908/);
full-rate MAT/CSV/figure artifacts remain in the local output folder.
