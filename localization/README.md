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

The runtime is `fixed-delay-transport-v1`, using the recorded gains from
`aperiodic-anisotropic-pose-v2`. Every LiDAR pose
carries its full 3x3 information matrix, including XY/yaw cross terms. With
pose normalization `D=diag(cfg.lidar.poseScales)` and `J=D*information*D`,

```text
W = (gainInformationScale*I + J) \ J
L_lidar(t) = F_lidar(t)*T*K*D*W/D
correction = L_lidar(t) * acquisition_coordinate_residual
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
W_lidar = D*(A\J)/D
W_gps = D*(A\G)/D
correction = F_lidar*T*K*W_lidar*r_lidar + F_gps*T*K*W_gps*r_gps
```

GPS does not overwrite the LiDAR position residual. The total homogeneous
source weight `A\(J+G)` remains symmetric between zero and identity. With
different acquisition times, the transported error Jacobians cannot be
replaced by that sum in a stability proof. GPS has no
invented heading information. Its default XY information `[25;25] m^-2` is an
explicit design reference, not a claim of identified sensor precision.

`diagnostics.lidarPoseWeight`, `gpsPoseWeight`, `totalNormalizedPoseWeight`
and the complete 7x3 `lidarPoseGain` expose the actual matrices. These
histories refer to causal delivery time. The first two outputs of
`computeLidarInformationWeights` are retained
as compatibility summaries. The fourth output is the complete pose weight,
which the runtime uses without discarding cross terms.

Automatic initialization applies the bounded directional weight to a
partial-rank LiDAR residual, preserving the fallback prior in unobserved
directions. Full-rank initialization and GPS position precedence are retained;
initial events must have both acquisition and delivery at initialization.
An older pose without input prehistory cannot initialize the current state.
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
geometry; this alone does not certify the transported feedback. Recorded-data
playback scripts retain the full-pose `accepted` column
and add a separate `directionalAccepted` column; old full-pose CSVs remain
readable.

## Fixed delay without observer replay

LiDAR acquisition at `t_k` has delivery `a_k=t_k+tau`, where
`cfg.measurement.fixedLidarDelay` defaults to 0.15 seconds. Explicit delivery
metadata must satisfy that contract. Missing or marked placeholder delivery
metadata is filled from the declared delay and identified as assumed in
diagnostics. GPS may retain its separate acquisition and delivery times.

The nominal dynamics are affine in the seven states:
`zdot=A_m(q)*z+b_m(r)`, with `q=r_m+betaDot_m`. The runtime integrates the
input-derived affine map `z(t)=F(t,t_k)*z(t_k)+g(t,t_k)` alongside the estimate.
At a delayed event, it forms

```text
predicted_acquisition_state = F \ (zhat(t) - g)
r_lidar = y_lidar - C*predicted_acquisition_state   % wrap the yaw residual
L_lidar(t) = F*T*K*W_lidar
```

Each source uses its own acquisition-time map. This transports both the
residual and its injection to the current state. A perfect nominal trajectory
has zero innovation throughout the pulse, including while moving. No
speed-jerk or course-angular-acceleration input is added. The four intrinsic
channels, their bounded output extension, the independent yaw state, and the
lateral stage remain unchanged.

Corrections act for 30 ms starting at delivery. GPS joins active LiDAR pulses;
GPS-only continuation remains available after the recent-LiDAR interval.
Event-split RK4 advances once in time with at most 10 ms substeps. Delivery
and pulse-expiry boundaries are integrated explicitly, including between
output samples. The internal yaw lift is continuous; public yaw is wrapped.

`cfg.measurement.inputHistoryDuration` defaults to one second. Its bounded
buffer holds only nominal affine transition maps derived from motion inputs.
It contains no historical observer states or corrections to rerun. A pose
outside available input history is rejected with an explicit reason. The
former `replayBufferDuration` setting is removed.

`z`, `onlineZ`, `pose`, `position`, `heading`, `velocity`, and `acceleration`
all describe immutable causal outputs. There is no `revisedZ` output or replay
mode. `stateHistoryRecomputed=false`, `integrationStepCount`, and
`maximumInputHistorySegments` expose the forward-only execution. The recorded
entry point `runMncavObserverReplay` retains its historical filename, where
"Replay" now means playback of a recorded sensor sequence only.

`incorporationTime` is the event-driven sensor clock at delivery, not a later
polling sample. The delay audit reports acquisition-to-incorporation age,
assumed delivery metadata, and any unavailable input history. It does not
measure wall-clock deadlines or registration throughput. In recorded checks,
150 ms is an imposed signal delay rather than a measured registration budget.

## Certificate scope

The stored **reference** certificate checks 720,896 flow inequalities plus
metric resets for the preceding current-pose pulse observer. It covers
arbitrary source-weight orientations in `0.8*I <= W <= I`, pulses of 30 ms,
and intervals of 50--110 ms. Its box is velocity components at most 16 m/s,
acceleration at most 5 m/s², and course rate at most 0.6 rad/s. The gains and
stored verification snapshot are still checked for provenance and integrity.

That certificate does not cover the new transported matrices
`F*T*K*W*C/F`. Runtime `observer.certificateVerified`, `observer.certified`,
and `certificateConditions.certificateApplicable` are therefore false.
`observer.referenceCertificateVerified` and `referenceVerification` expose
only the checked historical inequalities. Source-weight and schedule audits
use explicit `WithinReferenceSector` / `WithinReferenceSchedule` fields.
Passing those audits cannot authorize a stability claim for this observer.
The reference sector is never enforced as an information floor.

A new robust certificate must include the input-dependent transition maps,
asynchronous source ages, nonlinear output uncertainty, and transport of model
and measurement errors. Registration-chart, cascade-error, numerical-error,
and actual information/timing assumptions also need validation. The recorded
drive contains weaker geometry and longer gaps than the reference hypotheses.

See [the fixed-delay derivation and validation](../research/fixed_delay_transport_observer.md)
for the actual error equation, design comparison and measured limitations.
[The earlier assimilation note](../research/observer_proposal_assimilation.md)
provides the unchanged 13 exact incremental output bounds, four nominal-drift
vertices, curb observability example, and reduced-model disturbance identity.
Its timer/replay ISS result is historical. The
[directional integration note](../research/directional_geometry_and_assimilation_timing.md)
retains the frontend contract and physical-subspace derivation.

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
