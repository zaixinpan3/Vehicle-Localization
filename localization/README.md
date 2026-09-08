# Localization and vehicle observer

The module contains 19 MATLAB files, excluding the independent
`lateralObserver/` directory. `localizeLidarFrame` runs online perception,
registration and pose-event export. `registerSemanticProbabilityCloud` and
`scoreSemanticProbabilityCloudAlignment` remain the registration and scoring
entry points; observer design, verification and simulation keep their existing
entry points.

Shared registration helpers are static methods in `registrationSupport.m`.
Call them with the `registrationSupport.` prefix:

| Method | Responsibility |
| --- | --- |
| `projectSemanticProbabilityCloud` | Select XYZ or its exact XY marginal |
| `prepareSemanticRegistration` | Validate calibration and resolve height/tilt uncertainty |
| `balanceSemanticDistributions` | Balance shared semantic-class overlap energies |
| `semanticGaussianOverlap` | Evaluate Gaussian overlap and planar-pose gradients |
| `registrationPoseMeasurement` | Validate and export accepted pose information |

The former standalone helper names are replaced by these qualified calls,
for example `registrationSupport.projectSemanticProbabilityCloud(cloud,3)`.
Certificate-condition diagnostics are local to `runImprovedVehicleObserver`;
their results remain available in `estimate.diagnostics.certificateConditions`.

The public localization output is `[X,Y,psi]`. The independent lateral
observer feeds the seven-state global model `[X,Vx,Ax,Y,Vy,Ay,psi]`.
Use `setupVehicleLocalization` from the repository root.

## Information-dependent anisotropic gain

The production design is `aperiodic-anisotropic-pose-v2`. Every LiDAR pose
carries its full 3x3 information matrix, including XY/yaw cross terms. With
pose normalization `D=diag(cfg.lidar.poseScales)` and `J=D*information*D`,

```text
W = (gainInformationScale*I + J) \ J
L_lidar = T*K*D*W/D
correction = L_lidar * [X_lidar-Xhat; Y_lidar-Yhat; wrap(psi_lidar-psihat)]
```

In normalized coordinates, the gain weight retains the information
principal directions and maps eigenvalues `lambda` to `lambda/(scale+lambda)`.
The default scale is 5; it is the information giving half gain, not an
acceptance threshold. A weak direction is attenuated, and a zero-information
direction receives zero gain. Partial-rank information retains its observed
directions. Nonfinite, materially indefinite or completely zero information
cannot supply a usable correction. No positive eigenvalue floor is imposed.
Information curvature alone does not establish correct association or a
calibrated pose-error distribution.

When GPS is present within a LiDAR pulse, normalized GPS information is
`G=D*diag([cfg.gps.positionInformation;0])*D`. Separate residuals are fused as

```text
A = scale*I + J + G
correction = T*K*D*( A\J/D * r_lidar + A\G/D * r_gps )
r_gps = [X_gps-Xhat; Y_gps-Yhat; 0]
```

GPS does not overwrite the LiDAR position residual. The total homogeneous
weight `A\(J+G)` remains symmetric between zero and identity. GPS has no
invented heading information. Its default XY information `[25;25] m^-2` is an
explicit design reference, not a claim of identified sensor precision.

`diagnostics.lidarPoseWeight`, `gpsPoseWeight`, `totalNormalizedPoseWeight`
and the complete 7x3 `lidarPoseGain` expose the actual matrices. These
histories refer to revised measurement time; public pose/state outputs remain
causal. The first two outputs of `computeLidarInformationWeights` are retained
as compatibility summaries. The fourth output is the complete pose weight,
which the runtime uses without discarding cross terms.

Automatic initialization applies the bounded directional weight to a
partial-rank LiDAR residual, preserving the fallback prior in unobserved
directions. Full-rank initialization and GPS position precedence are retained;
initial events are selected by physical timestamp among already arrived data.
The information diagnostics also expose `marginalizedHeadingInformation`
after eliminating unknown translation. The raw yaw diagonal alone does not
establish independent geometric heading information.

## Directional registration events

`registerSemanticProbabilityCloud` keeps `accepted=true` for a complete pose.
Rank-one or rank-two results can instead set `directionalAccepted=true` after
passing overlap, correspondence, class-consistency, search-boundary and
supported-convergence checks. `partialPoseAvailable` alone never authorizes
an event. `localizeLidarFrame` exports both accepted types; consumers should
check whether the event is empty, rather than equating event availability
with the full-pose flag.

Directional events carry `measurementType="directionalPose"`, nonzero PSD
`information`, `observableRank`, and an explicitly labelled physical
`observableProjector`. The projector is generally oblique: the orthogonal
solver projector must be transformed through its yaw scaling. Information
is projected into the solver's supported subspace before the physical
congruence, suppressing weak directions the solver did not estimate.
`result.information` retains the raw normal matrix for diagnostics;
`result.directionalInformation` is the directional event matrix.

The weighted measurement contract is that supported pose errors are bounded
in a valid local registration/yaw chart. Unsupported coordinates remain a
pose representative, not an absolute measurement. GNSS may complement partial
geometry; the actual combined pulse weight still must meet the certificate
sector. The recorded replay scripts retain the full-pose `accepted` column
and add a separate `directionalAccepted` column; old full-pose CSVs remain
readable.

## Pulse and delay model

The invariant gain and acceleration rows of `K` retain the previous factor
of 0.1 reduction, at `theta=sigma=3.5`. Only nonlinear invariant arguments
are clipped to extend that map; estimated states and linear prediction are
not clipped or reset. The source model, invariant extension and lateral stage
are unchanged by the anisotropic gain correction.

LiDAR corrections act for 30 ms at physical timestamps. GPS joins the same
pulse; GPS-only position continuation is allowed once no recent LiDAR pulse
exists, and remains outside the full-pose certificate. Event-split RK4 uses
left-limit terminal modes, zero future timestamp tolerance and at most 10 ms
substeps. Physical-time replay handles the configured fixed 150 ms LiDAR delay
with a 1-second buffer. `pose`, `position`, `heading`, `velocity`, `acceleration`
and `onlineZ` are causal; `revisedZ` and the legacy `z` are revised history.

Each accepted event now records `incorporationTime`: the output sample clock
after its replay completes. Certificate diagnostics separate acquisition to
delivery, delivery to processing, and total assimilation age. The configured
causal lag includes the largest high-rate sample interval, not merely the RK4
substep. The settled-history audit also accounts for observed longer delays
and known qualified events still awaiting incorporation. This measures
sample-clock causality; wall-clock computation and deadlines need separate
measurements. `registrationPoseMeasurement(result,timestamp,arrivalTime)`
can record an explicit delivery time. Omission produces a marked placeholder;
the runtime reports missing delivery metadata without inventing a delay.

Replay does not remove raw-pose aging during a pulse: even with exact
tracking, the held position residual is `p(t_k)-p(t)`. This deterministic
forcing is included in the conditional ISS disturbance model. Replacing
the hold with an output predictor or a frozen acquisition innovation requires
an augmented error model and a new certificate. The present feedback law and
stored matrices are retained.

## Certificate scope

The new certificate covers **every orientation** of normalized pose weight
`0.8*I <= W <= I`, including arbitrary XY/yaw cross terms. A norm-bounded
uncertainty LMI, checked at all nonlinear model vertices and timer endpoints,
replaces the former diagonal-weight check. There are 720,896 flow checks plus
reset checks for pulse intervals 50--110 ms. The model box remains velocity
components at most 16 m/s, acceleration at most 5 m/s² and track-angle rate at
most 0.6 rad/s. `designAnisotropicPoseCertificate` synthesizes the timer metric;
`verifyAnisotropicPoseCertificate` checks recovered matrices independently.

**The 0.8 bound is a proof hypothesis, never a runtime floor or rejection
rule.** The runtime accepts useful weaker/partial information and reports
`informationWithinCertificate=false` when the actual fused pulse weights
violate the sector. The audit splits pulses at GPS starts and expirations,
including boundaries between high-rate samples. The separate
`lidarInformationWithinCertificate` field preserves the LiDAR-only check;
`posePulseInformationIntervals` and `minimumCombinedWeightEigenvalues`
identify the combined-weight segments on the settled measurement-time horizon.
Prediction intervals are handled by the timer certificate, not required to
have positive pose weight. The actual recorded drive also has
pose gaps longer than 110 ms, so its empirical results are not covered by the
uniform information/timing certificate. The old fixed-XY certificate is not
reused. Gains, metric, multipliers, normalization and timing are checked
against the stored verification snapshot.

`certificateVerified` means the conditional model inequalities pass;
`observer.certified` remains false because full-run disturbance, information
calibration, heading-chart and discretization budgets are not established.
The finite delay-tail bound includes all contraction weights and GPS-only
continuation, but is conservative and is not an asymptotic stability result.

`diagnostics.motionHeadingSensitivity` measures the fourth auxiliary
output's local yaw sensitivity at the extended revised estimate. It vanishes
at zero velocity; absolute stationary yaw correction requires geometry.
See [the proposal assimilation and ISS derivation](../research/observer_proposal_assimilation.md)
for all 13 exact incremental coefficient bounds, the four nominal-drift
vertices, the straight-curb proposition, weighted LiDAR errors, explicit
jerk/cascade/hold disturbances, and the conditional timer/replay ISS bound.
These arguments require no new speed-jerk or angular-acceleration inputs.
The [directional integration and timing follow-up](../research/directional_geometry_and_assimilation_timing.md)
documents the frontend contract, physical subspace transformation, actual
assimilation audit, exact-tracking counterexample and validation limits.

## Reproduction

```matlab
setupVehicleLocalization;
cfg = improvedObserverConfig;
design = improvedObserverReferenceDesign(cfg);
estimate = runImprovedVehicleObserver(sensorData,lateralDesign,design,cfg);
results = runtests('tests/improvedObserverTest.m');
```

YALMIP/SeDuMi are needed for synthesis, not stored-matrix verification.
Historical continuous and fixed-XY designs remain auditable, but are rejected
by the current runtime. See `research/anisotropic_information_gain.md` for the
full derivation and recorded comparison, and
`research/observer_pulse_implementation.md` for the preceding implementation.
