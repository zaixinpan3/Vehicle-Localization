# LiDAR-only localization with GNSS unavailable for the whole drive

Date: 2026-09-23. Code revision `d123ff204fa7ca39bf571352430db077e4e1b42d`.
The production code is unchanged. Every remedy below is an experiment-level
input or configuration substitution, evaluated in the closed loop. Each run
rematches all 1,170 scans with its own fused seed and receives no GNSS at any
frame. Metrics use the 1,149 frames after the first 2 s. The same-drive INSPVA
map is accepted by the project. Reference poses are used only for evaluation
and for the diagnostics that are explicitly marked as such.

## Starting point

The current observer without GNSS has **24.09 cm** position RMSE, 51.89 cm
P95 and 211 frames above 30 cm. Its own raw LiDAR matches are better
(16.84 cm RMSE). Seeding every scan at the reference pose gives an
evaluation-only per-frame matching ceiling of **11.48 cm** RMSE, with only 5
frames above 30 cm (`diagnoseLidarOnly.m`, `diagnosis_frames.csv`).

This separates two problems:

1. **The observer degrades the LiDAR measurements.** At frames 80–110 and
   790–830 the matches have 10–17 cm median error, while the fused error
   reaches 58–61 cm. The LiDAR weight is about 0.3, which gives an effective
   position gain of about 1.2/s. The observer velocity is 0.5–0.7 m/s wrong,
   so the fused position lags by roughly 0.6/1.2 ≈ 50 cm.
2. **Seed errors select displaced matching modes.** At 13–21 s and 83–89 s,
   closed-loop matches have 20–37 cm median error, while reference-seeded
   matches have 10–16 cm.

## Root cause of the velocity error

Four-wheel speed matches the reference body speed to 0.080 m/s RMSE. The
lateral velocity does not: its error is 0.295 m/s RMSE. A regression on the
evaluation drive explains almost all of it:

\[ v_{y,\mathrm{observer}} - v_{y,\mathrm{ref}} = -0.248 + 2.734\,r
\quad(\text{residual } 0.094\ \mathrm{m/s}). \]

The lateral observer estimates velocity at its CG-based point. The observer
position and the map use the INSPVA output point. Velocity at a point
\(d\) metres ahead of the CG is \(v_y + r d\), so an output point behind the
CG sees \(v_y - r|d|\). The observer applied the CG value directly. Its online
LiDAR velocity-bias learner cannot represent this yaw-rate-dependent term, and
it is inactive below 5 m/s.

`scripts/calibrateMncavMotionOutputPoint.m` (originally
`calibrateLateralLeverArm.m` in this folder) fits the lever arm **on the
separate 12-11-24 drive only** (seconds 1–40, INSPVA velocity), using the production lateral
observer design:

| Population | Raw vy RMSE | Lever arm removed | Locally fitted lever arm |
|---|---:|---:|---:|
| 12-11-24 fit (1–40 s) | 0.247 m/s | 0.054 m/s | 2.36 m |
| 12-11-24 held out (>40 s) | 0.095 m/s | 0.059 m/s | 1.66 m (little turning) |
| 12-09-31 evaluation drive | 0.275 m/s | 0.254 m/s | 2.32 m |

The output point lies **2.36 m behind** the lateral observer point, and the
evaluation drive agrees independently (2.32 m). The evaluation drive also
carries a constant lateral velocity offset of about −0.25 m/s that 12-11-24
does not have (+0.004 m/s). This drive-specific offset is left to the online
bias learner rather than calibrated.

## Remedies measured without GNSS

`runLidarOnlyVariants.m`, `variants.csv`:

| Variant | RMSE (cm) | P95 | Max | >30 cm frames | Seed RMSE | Match RMSE | Heading (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Current observer | 24.09 | 51.89 | 75.93 | 211 | 25.28 | 16.84 | 0.479 |
| LiDAR gain scale 16 → 4 | 19.38 | 39.25 | 71.91 | 131 | 20.46 | 16.85 | 0.483 |
| Lever-arm transport | 18.87 | 40.43 | 72.57 | 100 | 19.32 | 16.83 | 0.502 |
| Lever arm + bias learner above 2 m/s | 18.66 | 39.93 | 72.57 | 94 | 19.08 | 16.83 | 0.502 |
| Lever arm + gain | 17.42 | 37.61 | 71.41 | 78 | 17.82 | 17.02 | 0.503 |
| Lever arm + bias 2 m/s + gain | 17.30 | 37.35 | 71.41 | 76 | 17.68 | 17.00 | 0.502 |
| + relative height (marginal pole/sign) | **16.46** | 37.05 | 71.54 | 73 | 16.87 | 16.03 | 0.452 |
| Reference vy (diagnostic) | 16.65 | 32.27 | 69.26 | 64 | 16.84 | 16.61 | 0.505 |
| Reference vy + gain (diagnostic) | 16.28 | 33.42 | 70.41 | 67 | 16.44 | 16.94 | 0.494 |

Lever-arm transport replaces the lateral velocity with \(v_y - 2.36\,r\) and
adds the implied change of `atan2(vy, vx)` to the side-slip-angle rate.
With perfect reference lateral velocity, fused RMSE is still about 16.3–16.7 cm.
**The observer-side remedies have therefore removed essentially all of the
velocity-lag error.** The fused error now equals the matching error.

The switchable sliding-window pose graph (`runLidarOnlyGraph.m`,
`graph_variants.csv`) is worse without GNSS. It gives 32.17 cm with the recorded
odometry and 26.73 cm with lever-arm-transported odometry. Its 1 s relative
odometry error is 30.1 and 26.6 cm RMSE, mostly from the −0.25 m/s lateral
offset. The graph's 4 cm per-frame motion noise treats this biased odometry
as accurate. The observer's online bias learner is better suited here.

## Effect with GNSS available

`checkFusedMode.m`, `fused_mode_check.csv`. These are the same substitutions
in the production GNSS-aided closed loop:

| Variant | RMSE all (cm) | After 2 s | P95 | Max | Heading (deg) |
|---|---:|---:|---:|---:|---:|
| Production | 8.22 | 7.88 | 16.49 | 23.58 | 0.386 |
| Lever arm + bias 2 m/s + gain | 7.06 | 6.68 | 12.34 | 19.32 | 0.389 |
| + relative height | 6.84 | 6.44 | 12.44 | 19.40 | 0.334 |

The remedies also improve the normal fused mode. They are not a LiDAR-only
trade-off.

## Adoption

The lever-arm transport was adopted into production on 2026-09-24: the
lateral observer now exports at the configured output point
(`lateralObserverConfig("mncav").outputPoint`), so the global observer and the
source-window odometry both receive the transported velocity. See
[the production update](../output_point_transport_20260924/README.md). The
gain scale, bias threshold and relative-height settings tested here were not
adopted.

## Answer and remaining limit

Without any GNSS, position RMSE improves from 24.1 cm to **16.5 cm**, and the
P95 from 52 to 37 cm, using only the independent-drive lever arm, observer
settings and the existing optional height association. This is within 1 cm of
the RMSE obtained with perfect lateral velocity. The heading remains 0.45–0.50 deg.

The remaining gap to the 11.5 cm reference-seeded ceiling is a matching
mode-selection problem, and it concentrates in the segments at 13–21 s and
83–89 s. There, 65–77 accepted matches still exceed 30 cm and the maximum error
is about 71 cm. The next step is ambiguity handling in the matcher, such as
joint association or multi-hypothesis tracking, not further observer tuning.

Limits: a single 117 s evaluation drive; lever arm from one other drive; the
constant lateral offset is unexplained; initialization and scan tilt are
reference-derived (shown negligible in `../localization_evaluation_20260923`).
The gain scale 4 and the 2 m/s bias threshold were each tried as one value on
this drive, with no sweep. Adoption into the production observer would replace
the current lateral-velocity input path and requires the usual tests.

## Reproduction

From the repository root, after `../localization_evaluation_20260923` has
produced its source cache and observer inputs:

```matlab
addpath(pwd); setupVehicleLocalization; addpath('research/lidar_only_20260923');
diagnoseLidarOnly;          % seed/matching separation, reference-seeded ceiling
calibrateMncavMotionOutputPoint;  % separate-drive lever arm -> config/mncavMotionOutputPoint.json
runLidarOnlyVariants;       % no-GNSS observer variants
runLidarOnlyGraph;          % no-GNSS switchable pose graph
checkFusedMode;             % remedies with GNSS available
```

MATLAB R2026a. Deterministic; no random numbers.
