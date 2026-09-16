# MnCAV interface audit and LiDAR motion correction

## Material Passport

Date: September 16, 2026. Status: VERIFIED for executed extraction, signal
audits, replay metrics and software checks. Physical installation geometry
and loaded-vehicle tire/inertia parameters remain UNIDENTIFIED.

The user confirms that the recordings come from UMN MnCAV. This is an input
fact, not an identification hypothesis. INSPVA remains the stipulated
experimental reference. Its state is not supplied to the new correction.
This study follows the [lateral model diagnosis](../mncav_lateral_diagnosis_20260916/validation.md).

Inputs are the read-only Mississippi bags `raw_data_2024-06-07-12-09-31_0`
and `raw_data_2024-06-07-12-11-24_0`, existing sensor CSVs, and the frozen
INSPVA-map matching and observer experiments from September 15. Operational
input/output identities, including both original bags, are in
`artifact_manifest.json`. The separate 12:11:24 drive supplies the steering
calibration; 12:09:31 supplies the localization comparison. All three existing
matching initialization modes are retained; no rematching is performed.

## Result and decision

Two concrete issues were found: a missing recorded steering-wheel zero
correction, and an architecture that cannot use LiDAR motion to correct its
persistent lateral-velocity discrepancy. Correcting the first alone slightly
worsens localization, demonstrating that it is not the main error source.
Adding a slow LiDAR-displacement correction improves both RMSE and median
position discrepancy in every tested matching mode, with unchanged gains.

At the identical accepted LiDAR frames:

| Matching input | Samples | Raw LiDAR RMSE / median (cm) | Previous observer (cm) | Steering only (cm) | Steering + LiDAR motion correction (cm) |
|---|---:|---:|---:|---:|---:|
| Zero initial perturbation | 1,084 | 14.863 / 7.887 | 11.737 / 9.635 | 11.792 / 9.738 | **9.340 / 6.558** |
| Positive initial perturbation | 1,066 | 17.710 / 10.458 | 13.879 / 10.998 | 13.923 / 11.065 | **12.069 / 8.639** |
| Negative initial perturbation | 1,069 | 20.738 / 10.320 | 15.773 / 11.017 | 15.825 / 11.110 | **14.210 / 8.138** |

These are three input variants of the same drive, not three independent
experiments. The zero-mode position RMSE improves by 20.4% versus the previous
observer and 37.2% versus raw LiDAR. Its accepted-frame P95 drops from
20.670 to 18.797 cm; raw LiDAR P95 is 33.045 cm. The fraction within 5 cm is
35.70%, versus previous 12.36% and raw LiDAR 25.28%. These results do not imply
improvement at every frame or centimeter-level ground-truth independence.

The effective moving lateral-velocity RMSE drops from 0.274292 to
0.143015 m/s. Straight-subset bias drops from -0.263054 to -0.033239 m/s.
This is the velocity supplied to the global observer in the mapped pose
convention; the hidden bicycle state has not thereby become an independently
calibrated CG state. Fast turning discrepancies and startup remain visible.

## 1. Steering conversion: a reproducible omission

The recorded `/vehicle/steering_curvature` agrees with

\[
\kappa=\tan((\delta_{wheel}-1.5^\circ)/16.2)/3.089.
\]

The old replay divided the raw wheel angle by 16.2 without subtracting its
zero. Fitting only the zero on receiver seconds 1--40 of the separate drive,
with wheelbase and ratio fixed, gives 1.503989 degrees. The selected value is
the rounded 1.5 degrees. Headerless curvature is paired using bag timestamps;
paired curvature/steering publication time RMS is 0.225 / 0.192 ms in the two
drives. Full-drive curvature consistency RMS falls:

| Drive | Before (1/m) | After (1/m) | After, t >= 40 s (1/m) |
|---|---:|---:|---:|
| 12:09:31 | 5.23552e-4 | 2.89170e-6 | 1.34728e-6 |
| 12:11:24 | 5.25173e-4 | 2.54828e-6 | 1.07159e-6 |

This identifies the recorded controller's curvature convention, not an
independently measured tire angle, Ackermann geometry or loaded steering
compliance. The three topics named `offset_angle_error` and its predecessors
contain disabled, zero commands throughout both bags; their names are not
evidence of a measured angular error. Steering calibration-fault flags are
zero in both exports.

`config/mncavReplayInterface.json` records this provenance.
`calibrateMncavReplayInputs.py` includes the correction in newly generated
parameter files. `prepareMncavObserverReplay.m` consumes that explicit field;
historical files without it retain their original zero-offset behavior.
Historical parameters and frozen replay outputs are not overwritten.

The equivalent road-wheel change is only 0.09259 degrees. Steering-only
moving lateral RMSE changes from 0.274292 to 0.277096 m/s. Consequently this
corrected convention must not be advertised as the principal localization
improvement.

## 2. Axes, timing, reference point and physical parameters

The native INS audit uses 0.02 s receiver-time samples, a 51-sample order-3
Savitzky--Golay derivative, and excludes 0.52 s at each edge for motion
comparisons. It uses the original separate-drive CAN sign/offset calibration.
The localization and lateral benchmark retain their existing reference CSV
and masks; this new derivative does not silently redefine those scores.

| Moving-sample CAN minus INSPVA-derived residual | 12:09:31 RMS | 12:11:24 RMS |
|---|---:|---:|
| Forward speed (m/s) | 0.05675 | 0.04804 |
| Yaw rate (rad/s) | 0.001979 | 0.001855 |
| Lateral acceleration (m/s²) | 0.18502 | 0.13671 |
| Longitudinal acceleration (m/s²) | 0.26541 | 0.22307 |

ULC speed versus the existing twist input has RMS 0.01279 / 0.01377 m/s.
There is no indication of a gross speed-unit or corrected yaw-sign reversal.
Longitudinal acceleration remains imperfect; this task does not relabel it
as exact or calibrate it on evaluation truth.

The first drive spans 116.92 receiver seconds but 107.177 ROS seconds; the
second spans 77.92 / 71.433 seconds. The existing replay already bridges ROS
headers to receiver time. Replacing that with raw ROS elapsed time would
introduce a separate scale error. A diagnostic yaw alignment sweep from
-0.20 to +0.20 s in 0.01 s increments prefers -0.02 s on both drives, but only
reduces RMS from 0.001982 to 0.001874 and 0.001860 to 0.001642 rad/s on its
fixed interior masks. No physical latency is identified or applied from this
derivative-dependent diagnostic. A small time shift cannot explain a large
nearly straight constant lateral-velocity discrepancy.

Full 3-D rotation using the existing roll/pitch convention changes mean
straight reference lateral velocity from 0.26933 to 0.27742 m/s on the first
drive; it does not remove the discrepancy. The second changes from 0.00200
to 0.03392 m/s. Even when restricting reference yaw rate to less than
0.005 rad/s, the first drive's mean is 0.27282 m/s (1,238 samples), versus
0.00139 m/s (451 samples) in the second. A modest longitudinal lever arm alone
does not explain that near-zero-rate offset.

The bicycle state is at the CG, whereas current map preparation explicitly
uses the recorded INS output point. NovAtel documents the default INSPVA
position/velocity point as the IMU navigation center, with configurable
translation and attitude frames. See the primary
[SPAN reference-point/frame documentation](https://docs.novatel.com/OEM7/Content/SPAN_Operation/SPAN_Translations_Rotations.htm).
Current point-cloud extraction rotates points but does not apply a measured
LiDAR-to-CG translation. The replay contains no measured INS-to-CG transform.

There are no `/tf` or `/tf_static` topics in either bag. One decoded
INSCONFIG snapshot near the first bag's end has zero translation/rotation
entries; the second bag has no snapshot. This establishes missing recorded
calibration evidence, **not** physical coincidence, an unconfigured receiver,
or configuration at every earlier instant. No installation angle or lever arm
is invented from these empty fields. The official
[INSCONFIG definition](https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSCONFIG.htm)
describes what these fields mean, not this deployment's unrecorded geometry.

HEADING2 is a base-to-rover vector, not automatically the vehicle heading.
No sample with computed solution and positive reported heading standard
deviation has standard deviation below one degree in either bag. Medians
among those samples are 88.17 / 127.16 degrees. A length value of -1 alone
is not an invalidity criterion. This stream cannot establish a rigid vehicle
heading calibration here. See the primary
[HEADING2 definition](https://docs.novatel.com/OEM7/Content/Logs/HEADING2.htm).

Descriptive fits `referenceVy = delta * measuredVx + pointX * yawRate` give
(1.2145 degrees, -1.0287 m) on the evaluation drive and
(0.02187 degrees, -1.3643 m) on the separate drive. Transferring the latter
to the former leaves 0.25911 m/s RMS. This equation confounds sideslip,
attitude discrepancy and reference-point translation; neither fit is an
identified mounting geometry and neither is deployed.

Mass/geometry remain the source-qualified stock MnCAV/Pacifica values:
2273 kg, lf=1.374605 m, lr=1.714395 m. Inertia 5874.60733 kg m² and axle
stiffnesses 108238.095 / 80817.778 N/rad remain declared priors, not measured
loaded-vehicle parameters. A diagnostic joint force/yaw fit over moving,
turning samples before 40 s constrains all three positive multipliers to
[0.3, 3]. It reaches the lower bound for every parameter on both drives.
Later force residuals remain 0.4953 / 0.2665 m/s². The unconstrained separate
drive calibration continues to produce negative rear stiffness and inertia.
These are rejected identification results, not new vehicle parameters.

Together, the evidence supports a persistent model/state-interface
inconsistency. It does not determine its unique physical cause or demonstrate
that the bicycle model cannot describe MnCAV motion.

## 3. Implemented LiDAR motion consistency correction

The existing global observer corrects position with LiDAR, but corrects
velocity only toward rotated measured/estimated body velocity. Its position
innovation cannot repair an upstream constant lateral-velocity bias.

The new `correctLateralVelocityFromLidar.m` adapter retains the original
physical observer and estimates a separate effective lateral discrepancy.
For an accepted endpoint k and an accepted endpoint j about two seconds
earlier, define

\[
D=p^L_k-p^L_j-\int_{t_j}^{t_k}R(\psi^L)[v_x,\hat v_y]^Tdt,
\quad B=\int_{t_j}^{t_k}R(\psi^L)dt,
\quad \tilde b=B^{-1}D.
\]

Solving for both body components avoids attributing a forward-speed error
to lateral motion during a turn. Only the lateral component is applied.
Two-second windows require speed at least 5 m/s, no accepted-frame gap over
0.25 s, adequate orientation-integral conditioning, and both candidate bias
components within 0.8 m/s. Inconsistent windows are rejected, not clipped.
The latest accepted lateral candidate is the target of

\[
\dot b_y=(\tilde b_y-b_y)/4,\qquad
v_{y,global}=\hat v_y+b_y.
\]

The filter starts at zero and uses an exact exponential step. Its bounded
target keeps the added state bounded. A smooth speed participation fades its
application between 1 and 5 m/s. The sideslip interface is adjusted by the
corresponding atan2 difference and its backward time difference. No state of
the physical bicycle observer is reset or overwritten.

Constants (2 s window, 4 s time constant, 0.25 s gap, 0.8 m/s gate) were fixed
before this correction replay, with no search for a better evaluation score.
All old lateral gains and global gains `[4,4,12,4]` are retained. The maximum
observed track-angle rate is 0.354965 rad/s, below the existing 0.4 envelope.
The existing certificates keep their original conditional scope; no joint
physical-calibration, full hybrid or zero-error certificate is claimed.

Zero/positive/negative modes admit 813 / 788 / 767 windows, with first
updates at 14.50 / 15.70 / 19.01 s. Maximum applied corrections are
0.3397 / 0.2825 / 0.3530 m/s. There are 12 evaluated configurations: three
matching modes times previous, steering only, LiDAR correction with legacy
steering, and combined correction. The legacy-steering ablation gives
9.339 / 6.556 cm in the primary accepted-frame comparison, confirming that
the performance improvement comes from LiDAR motion consistency.

The adapter uses current/past accepted endpoints. The full replay still uses
offline interpolated motion/poses and future endpoints for long-gap
reconstruction, as previously declared. Processing delay is zero; this is
not a demonstration of a fully causal online system. The adapter does not
read INSPVA, reference errors, GNSS position or previously scored output.
Upstream map construction and matching seeds retain their reference use.

## 4. Validation, remaining limits and reproduction

All 1,170 frame evaluation times and 11,690 uniform samples are retained.
All original accepted poses are preserved exactly, and all information
matrices are unchanged. On the zero mode's full uniform grid, position RMSE
falls from 12.2595 to 10.1402 cm; the already examined later-than-60-s segment
improves from 12.8926 to 11.0326 cm. The later segments also improve in the
other two modes. These temporal strata are not a fresh held-out study.

Uniform-grid maximum error slightly increases, 33.4049 to 34.0951 cm, despite
the better RMS/median. Startup and fast motion discrepancies remain; a
four-second bias filter cannot track arbitrary fast mismatch. The result
does not establish pointwise domination, true physical CG velocity, accurate
unknown extrinsics, independently calibrated noise covariance, or accuracy on
an independent map/drive. LiDAR-derived velocity and position are correlated;
this is a deterministic correction with no independent-sensor covariance claim.

Validation completed:

- 41/41 MATLAB tests: seven new bias recovery/rotation/turning/no-future-input/
  gap/outlier/unbiased-motion tests, 15 lateral, 14 global, five gap tests.
  The seven new tests pass again after the final input-domain guard.
- Four changed/new MATLAB files have zero factory Code Analyzer findings.
- Actual steering conversion differs from the independent subtraction formula
  by at most 2.78e-17 rad; all other motion fields are unchanged to 1e-12.
- Primary combined observer repeats bit-for-bit; halving integration step
  changes position by at most 9.32e-9 m.
- Python independently reproduces 27 native position metric rows to
  5.55e-16 and 16 lateral metric rows to 6.11e-16, and checks improvements in
  both primary metrics for all three modes.
- Standalone PNG/PDF generated and inspected. An initial Python run stopped
  on NumPy's removed `row_stack` alias (changed to `vstack`). An initial
  MATLAB harness rejected its 9.5 ms endpoint NaN; the harness now uses the
  already declared <=20 ms motion edge hold. The full corrected run completes.

```sh
uv run --offline --with rosbags --with numpy python scripts/extractMncavInterfaceEvidence.py
uv run --offline --with numpy --with scipy --with pandas python scripts/auditMncavReplayInterfaces.py
uv run --offline --with numpy --with scipy --with pandas python scripts/calibrateMncavReplayInputs.py --output output/mncav_interface_audit_20260916/vehicle_parameters.json
```

```matlab
setupVehicleLocalization;
runMncavInterfaceCorrectionComparison;
results=runtests({'tests/lidarVelocityConsistencyTest.m', ...
    'tests/lateralObserverTest.m','tests/motionAidedObserverTest.m', ...
    'tests/motionAidedGapReconstructionTest.m'});
assertSuccess(results);
```

```sh
uv run --offline --with numpy python research/mncav_interface_audit_20260916/verify_results.py
```

Operational output is `output/mncav_interface_audit_20260916/`, including
the full correction MAT, original-topic exports, aligned audit CSVs, tests,
and `correction_comparison.png/.pdf`. Compact evidence is exported beside
this report. Raw datasets, generated MAT/figure binaries and unrelated
repository changes are excluded from the public commit.

## Statistical interpretation audit

| Check | Disposition |
|---|---|
| Simpson reversal | All three modes and full/accepted/uniform/early/later populations retained. |
| Ecological inference | RMS, median, tails and mean bias are distinct; no every-frame claim. |
| Berkson selection | Same accepted-frame masks; all frames and uniform results also reported. |
| Collider conditioning | Bias-window acceptance does not select evaluation samples. |
| Base rates | Processed/accepted counts, startup and rejected correction windows disclosed. |
| Regression to mean | Frozen paired measurements and gains; steering-only and adapter-only controls. |
| Survivorship | Negative steering-only result and increased uniform maximum retained. |
| Look-elsewhere | Four fixed variants across three modes; no hidden gain/adapter sweep. |
| Forking paths | Exploratory follow-up on an already examined drive; not a fresh confirmatory test. |
| Correlation/causation | Software controls support the correction mechanism; physical cause is unresolved. |
| Reverse causality | Runtime adapter cannot read scored errors or reference velocity; upstream reference map/seeds disclosed. |

Temporal samples and the three seed variants are dependent. No p-values,
independent-sample confidence intervals or cross-drive localization
generalization claims are made.
