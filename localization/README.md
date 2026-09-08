# Vehicle observer

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
`informationWithinCertificate=false`. The LiDAR-only information audit is
conservative when GPS adds information. The actual recorded drive also has
pose gaps longer than 110 ms, so its empirical results are not covered by the
uniform information/timing certificate. The old fixed-XY certificate is not
reused. Gains, metric, multipliers, normalization and timing are checked
against the stored verification snapshot.

`certificateVerified` means the conditional model inequalities pass;
`observer.certified` remains false because full-run disturbance, information
calibration, heading-chart and discretization budgets are not established.
The finite delay-tail bound includes all contraction weights and GPS-only
continuation, but is conservative and is not an asymptotic stability result.

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
