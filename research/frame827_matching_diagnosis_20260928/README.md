# Mississippi frame 827: map merging selects the wrong association basin

The latest deployed replay has its largest post-initialization position error
at frame 827 (82.5992 s): **56.304 cm**, mostly **56.081 cm backward**, with
5.004 cm left error and -1.0507 degrees heading error. The initial frame has
64.031 cm configured initialization error and is not a successful match.
This diagnosis uses production commit `7b2d8cfc52d7e85a47a38b75f4aed0ce20d8ff5a`.
No production setting, perception selection, map file or label was changed.

## Main finding

The 1.5 m map canonicalization radius combines two traffic-sign modes whose
centers are 0.96448 m apart. The merged target shifts the coarse solution into
an association basin that is inconsistent with the recorded reference pose.
This occurs even when starting exactly at the reference pose. Fine refinement
from that displaced coarse pose chooses the other original map mode and makes
position error worse. The 0.15 m refinement trust gate limits that further
failure, but cannot repair the biased coarse pose.

The decisive source component (17) has reference-body mean
(13.83689, -2.70832) m. Map coordinates below are transformed into the same
frame for diagnostic comparison; the reference is not supplied to production
matching.

| Representation | Forward X (m) | Left Y (m) |
|---|---:|---:|
| Source component 17 | 13.83689 | -2.70832 |
| Original map 1263 | 13.84727 | -2.70622 |
| Original map 1264 | 12.99612 | -3.15984 |
| Canonical merged target | 13.41126 | -2.93840 |

The source is only **1.059 cm** from map 1263, but **48.383 cm** from the
canonical target. Its merged group contains 1263, 1264, 1277 and 1278; the last
two nearly duplicate the first two. Map 1264 has about 1.5 times the prior
weight of 1263. The two separated modes need not represent two distinct physical
signs: this analysis establishes a map-mixture geometry/association problem,
not their physical identity or the origin of their separation.

With the original unmerged map, starting from the recorded prediction selects
1263 for nearby source sign components 17, 18 and 20, and gives 3.991 cm error.
Starting from the deployed coarse solution instead selects 1264 for all three,
and gives 81.747 cm error. The distant sign remains associated with 1281 in
both solutions. Therefore the coarse stage changes the subsequent optimization
basin; merely allowing unrestricted refinement does not resolve the error.

## Controlled experiments

All controls retain the same frame inputs unless explicitly stated. Errors
are compared with the recorded pose reference; oracle controls are diagnostic
only and were never deployed.

| Control | Position error | Heading error |
|---|---:|---:|
| Deployed production | 56.304 cm | -1.0507 deg |
| Exact reference initial pose | 56.304 cm | -1.0507 deg |
| Unlimited fine refinement | 81.747 cm | -1.3677 deg |
| Disable only map canonicalization | 3.991 cm | +0.1064 deg |
| Disable only source canonicalization | 56.304 cm | -1.0507 deg |
| Disable new curb direction factor | 44.304 cm | -0.7096 deg |
| Current scan alone | 40.981 cm | -0.4108 deg |
| Reference motion for the source window | 55.489 cm | -0.9926 deg |
| Traffic signs alone | 46.764 cm | -0.6700 deg |

The prediction already has 26.681 cm / -0.5699 degree error; production matching
increases it. The reference-seed and reference-window controls show that poor
prediction or window-motion error alone does not explain the 56 cm peak.
The coarse/fine displacement is 27.936 cm, exceeding the 15 cm trust radius.

A radius sweep gives about 3.991 cm for 0, 0.25, 0.5 and 0.75 m, and 56.304 cm
for 1, 1.25, 1.5 and 2 m. This transition agrees with the 0.964 m map-mode
separation. It is evidence for the mechanism on this frame, not a validated
universal radius selection.

## Why the other features do not prevent it

The actual five-scan matching input contains **23 curb Gaussians, four sign
Gaussians and no poles**. These are pooled matching components, not individual
0.6 m perception pillars. Poles disappear from the matching source at frame
827 and remain absent through 830. Their absence removes an additional class
of constraints during this error burst; it is not proof that recovering one
pole alone would fix localization.

The curb geometry is curved, whereas target Gaussians model line segments.
Curb-only registration has rank two and fails the overlap acceptance gate.
At the deployed pose, many right-side curb direction residuals are strongly
robust-downweighted. The new direction factor contributes to this particular
peak (44.3 cm without it versus 56.3 cm with it), but the large error remains
when that factor is removed. Thus it is not the sole cause, and feature count
alone does not establish a correct, fully constrained pose.

The accepted solution passes existing class-consistency checks: information-
weighted corrections are 0.0827 for curb and 0.0469 for signs, below 0.5.
Its curvature eigenvalues are approximately 1.466, 2.573 and 20.603, giving full
rank under the current threshold. These are local geometry checks, not tests
of global association correctness or calibrated pose accuracy.

## Visualization and local validation

The interactive `showMississippiFeaturePillars(827)` window was opened and
focused in niri. It retains all 65,536 raw points, with uniform marker size 4
and in-place reference coloring (orange pole, magenta signs, cyan curb), plus
the current 0.6 m selections on a nearby floor grid. No feature-point overlay
is added. Full-cloud and camera-focused PNGs and the exact JSON selections are
included; focusing the camera does not remove source points.

The current scan has 0 selected pole cells, 5 sign cells and 28 curb cells.
Frozen fine reference labels contain 192 pole points, 40 sign points and 65
curb points. Coarse-cell coverage is 0/192, 40/40 and 50/65, respectively.
Nine selected curb cells contain no frozen reference curb point. These are
reference-detector disagreements, not independently adjudicated physical
false positives. Stored fine labels are not fed into the matching controls.

- Frame 827 reproduces the recorded production pose within 5.83e-11 maximum
  coordinate discrepancy; all 16 frames 820--835 reproduce within 1e-7.
- Fifteen primary controls, eight merge radii, three unmerged starting poses,
  and a reference-window-motion control were executed.
- No-map-merge controls across 820--835 use each frame's original stored seed,
  not a recursively corrected alternate trajectory. In particular, frames
  828 and 829 still reach the wrong basin (80.43 and 77.37 cm), demonstrating
  why a one-frame improvement cannot justify changing the global default.
- Both diagnostic MATLAB files have zero factory Code Analyzer issues.
- `analyzeDiagnosis.py` independently checks the key geometric distances,
  correspondences, reproduced error, controls and viewer metadata.
- No new unit suite is needed for this diagnostic-only change. No full-route
  alternative replay or Downtown evaluation was run in this task.

The resulting design direction is to preserve separated map modes and avoid
unconditionally trusting a canonical coarse seed. Candidate solutions should
be checked against original map components; any replacement needs recursive
full-route validation, especially after already-biased preceding poses.

## Reproduction

```bash
matlab -batch "addpath('research/frame827_matching_diagnosis_20260928'); diagnoseFrame827"
matlab -batch "addpath('research/frame827_matching_diagnosis_20260928'); inspectBasins827"
uv run --offline --with numpy --with pandas --with matplotlib python research/frame827_matching_diagnosis_20260928/analyzeDiagnosis.py
```

The MATLAB controls use the unchanged source cache and production report under
`output/line_direction_matching_20260928/`. Heavy diagnostic MAT files remain
under `output/frame827_matching_diagnosis_20260928/`. Committed CSV/JSON exports
and plots are sufficient to review the reported comparisons. The input and
artifact hash manifest identifies local inputs without committing recordings,
large caches or external dependencies.
