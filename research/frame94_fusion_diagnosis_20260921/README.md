# Corrected-clock fusion peak: Mississippi frame 94

Investigation date: 2026-09-21. Production source revision:
`8990d6c9c507f25fe9f8050f736afa702db00432`.
No production perception, mapping, registration, timing or observer setting
was changed in this investigation.

## Identification and reproduction

The maximum 37.5665 cm fused XY discrepancy occurs at **frame 94**, at
9.2995590333 receiver seconds relative to the first scan. It is not frame 307,
which was the earlier clock-diagnosis case. The matching call and observer
sample have the same epoch.

| Quantity at frame 94 | Position error |
|---|---:|
| GNSS + LiDAR observer | 37.5665 cm |
| Raw coarse LiDAR matching | 92.7991 cm |
| GNSS-only observer | 22.5138 cm |
| LiDAR-only observer | 93.7765 cm |
| GNSS measurement at the fusion observer's output point | 5.9620 cm |

The GNSS measurement row includes the existing estimated-heading output-point
alignment and is distinct from the GNSS-only observer trajectory. All errors
are against the recorded INSPVA reference, not independent surveyed truth.

Replaying all 1,169 baseline observer samples reproduces every saved state
exactly. Reconstructing the frame-94 three-scan coarse source and 100 m map
crop reproduces the stored match to within `9.32e-10` in the pose coordinates.
Both checks are assertions in `diagnose_peak.m`.

## Main causes established by the replay

### 1. Biased, initialization-sensitive matching passes all current gates

Raw matching error rises from 22.56 cm at frame 80 to 77.47 cm at frame 90,
and then 92.80 cm at frame 94. Frame 94 starts from a prediction already
94.49 cm wrong. Its result is mostly displaced to the vehicle's left:
92.27 cm left and 9.92 cm forward. Frame 95 switches to a different solution
on the other side of the reference. This is a sequence of biased accepted
poses, rather than one isolated numerical spike.

The frame-94 match has 138 pairs, full rank, convergence, and similarity
0.31081, above the acceptance threshold 0.15. Scaled geometry eigenvalues
are approximately 7.055, 7.106 and 15.432. Class-consistency scores are
0.2396 for curbs, 0.0622 for poles and 0.0848 for signs, all below 0.5.
These checks describe the selected local solution. They do not establish
that its associations represent the correct physical location.

Changing only the initial pose to the reference produces another accepted
solution, still **42.28 cm** wrong, with almost the same similarity, 0.30679.
A seed using the aligned GNSS XY and the original predicted yaw reaches the
same biased solution. Thus recursive initialization contributes to the large
error, but a better seed alone does not eliminate the frame's matching bias.

Accepted associations include 78 curb components assigned to only five map
components. Their median absolute point-to-line residual is 87.59 cm.
Pole and sign median point-to-point residuals are 26.78 and 38.74 cm.
The selected point correspondences fit worse at the reference pose, with
median residuals around 1.12 m. This supports a mismatch between the current
distributions and selected map associations. It does **not** establish the
physical identity of each pair or isolate map compression, false detections,
extrinsics, and scan distortion from one another.

The objective normalizes evidence per class and robustly downweights residuals.
Its final global compatibility and curvature can therefore remain acceptable
despite poor curb alignment. The information matrix measures local constraint
strength, and is explicitly uncalibrated as a pose-error covariance. The
synchronous observer saturates this information into gains, but applies no
additional GNSS-versus-LiDAR position-innovation rejection. The wrong accepted
position consequently continues to exert correction.

### 2. Turn-time motion error pushes in the same direction

At the peak the wheel speed is 5.153 m/s and yaw rate is 0.299 rad/s.
The lateral observer supplies **+0.394 m/s**, while finite differencing the
reference positions and rotating with reference heading gives approximately
**-0.193 m/s** at the map/reference output point. The realized observer world
X velocity is +0.365 m/s versus reference-derived -0.077 m/s.

This is an observed motion-input/output discrepancy. The bicycle model's CG
convention, reference-point transport, nominal dynamics, inertial corrections
and filtering should be separated in a follow-up; a sign bug or surveyed
lever arm has not been established. The vehicle-parameter provenance already
identifies the unknown INS-to-CG offset and unmeasured dynamic parameters.

The existing LiDAR-displacement velocity-bias correction is zero at this
frame. Its eligibility requires a full two-second history at or above 5 m/s;
the minimum speed in that history is only 2.862 m/s. It has not corrected the
motion discrepancy during this accelerating turn.

### 3. Exact signed contribution accounting

For the realized synchronous update, let `e` be position error,
`Kg` and `Kl` the effective position gain matrices, `eg` and `el` the source
position errors, and `v` the realized observer velocity:

```text
e_k = (I + dt*(Kg+Kl))^-1 *
      (e_(k-1) + dt*v_k - (p_ref_k-p_ref_(k-1))
       + dt*Kg*eg_k + dt*Kl*el_k).
```

Propagating these four signed terms separately gives:

| Accumulated contribution at frame 94 | Map X | Map Y |
|---|---:|---:|
| LiDAR position error | +21.9918 cm | -1.5547 cm |
| GNSS position error | +5.2388 cm | +0.4001 cm |
| Realized motion/discretization discrepancy | +10.3001 cm | -0.4840 cm |
| Common initial state | Negligible | Negligible |
| Sum | +37.5307 cm | -1.6386 cm |

The vector norm of the sum is 37.5665 cm. The decomposition reconstructs the
complete trajectory's position errors within `3.12e-9` m. These are signed
terms with observer memory, not independent variances or a causal partition
of all sensor effects: heading and body inputs also affect the motion term.

## Controlled substitutions and matching ablations

The observer controls replace one input over frames 60:95, retaining saved
information, gains, initial state and the remaining inputs:

| Offline reference substitution | Frame-94 fused error |
|---|---:|
| None | 37.57 cm |
| LiDAR XY only | 15.54 cm |
| LiDAR yaw only | 37.24 cm |
| Lateral-velocity input only | 27.09 cm |

The lateral control retains the original sideslip-rate input deliberately
to isolate the velocity input; it is not a complete reference motion model.
Reference substitutions are **diagnostics**, not implementable online fixes
or new localization accuracy claims.

`matching_controls.csv` records all 16 matching controls, including rejected
candidates. Relevant negative results are:

- Current single-scan coarse input from the recorded prediction still gives
  70.91 cm error; removing the source window is not sufficient.
- Replacing only the three-scan transport motion with reference motion gives
  84.10 cm error from the recorded seed. Source transport contributes, but
  does not explain the full matching discrepancy.
- Saved fine points with the current map and solver give a rejected 78.08 cm
  candidate from the recorded seed, and an accepted 53.27 cm result from the
  reference seed. The corresponding single-scan coarse/reference result is
  28.62 cm but is rejected for nonconvergence. Fine perception alone therefore
  does not resolve this case; these diagnostic fine labels are not used online.
- Uniform map priors give 95.55 cm from the recorded seed and 42.81 cm from
  the reference seed. Existing stability weights are not the sole cause.
- Tightening the distance gate to 1 m or 0.5 m still accepts errors of 97.88
  and 104.39 cm, respectively. Tightening a gate around an already incorrect
  prediction is not a demonstrated remedy.

These results prioritize registration ambiguity/association validation and
the motion reference-point/input mismatch over simply tuning a perception
threshold. Information-based gain scaling needs calibration or additional
cross-sensor consistency evidence to prevent confident biased matches from
worsening GNSS position updates.

## Timing and evaluation limits

This run uses the repaired shared clock consistently for map, query, motion,
GNSS and reference. There is no mismatch between the saved matching and
observer frame epochs. Its peak is overwhelmingly lateral, and the controlled
replays above establish large non-clock contributions. The earlier clock
repair does not guarantee correct feature associations or motion estimates.
Absolute LiDAR latency, scan start/end convention, scan deskew and extrinsic
calibration remain unresolved; this investigation does not exclude their
contribution to the distribution mismatch. At the measured yaw rate, a
0.1 s scan spans about 1.7 degrees, so scan distortion merits a separate audit.

The map uses the same drive, includes query observations, and the reference
is recorded INSPVA. BESTPOS is INS-aided. These results are diagnostic
discrepancies under the existing experiment assumptions, not independent
ground-truth accuracy certification.

## Reproduce and inspect

Run `diagnose_peak.m` with the current project setup and these existing inputs:

- `output/receiver_clock_20260921/coarse_pipeline/observer/experiment.mat`
- `output/receiver_clock_20260921/coarse_pipeline/matching/report.mat`
- `output/mississippi_mapping_synchronized/probability_cloud.mat`
- `output/mississippi_mapping_synchronized/feature_observations.mat`
- `output/saved_perception_inspva_20260915/experiment.mat` (fine adapter config)
- `data/raw/MissisipiPointClouds.mat` and the synchronized 1:1170 pose CSV.

The script exports compact CSV/JSON results here and a detailed MAT file plus
`diagnosis.png` under `output/frame94_fusion_diagnosis_20260921/`. The plot
shows neighboring errors, exact signed contributions, lateral motion and
accepted associations. No interactive perception window is modified.

MATLAB Code Analyzer reports no issues. The complete diagnostic script and
all numerical reproduction/decomposition assertions pass. Production tests
are not rerun because production code is unchanged. Binary outputs, recorded
data and unrelated `AGENTS.md` / `reference/` changes are excluded from the
research commit.
