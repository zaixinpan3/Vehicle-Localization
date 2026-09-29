# Fresh production matching replay and frame 856 diagnosis

A fresh raw replay of all **1,170 Mississippi frames** with current production
perception and registration confirms **frame 856** as the largest error after
initialization: **0.410501029 m**, **+0.650671 degrees**, at **85.500238 s**.
The pose error is approximately +0.40264 m forward and -0.07998 m left.
Frame 1 remains the overall maximum at its configured 0.640312424 m initial
offset, without an accepted match. Full-route RMSE is 0.132903197 m; MATLAB
`prctile(...,95)` is 0.244939135 m. There are 1,152 full updates, ten directional
updates and eight frames without an update.

This rerun uses production commit `f0568d430de197dd442dda6a555617b52b7ba403`,
including minority-shaft protection and the production specialist JSON. The
map is loaded from the current configured map file. Every raw frame is
reprocessed; no previous perception or matching result is substituted.
Independent wheel/gyro/observer odometry and the initial XY/yaw remain frozen
for comparison with the preceding experiment. Existing recorded INS tilt is
used for projection. There are no intermediate reference XY/yaw resets.
The entire resulting trajectory reproduces the preceding specialist replay
within 1e-9 in exported coordinates. No production code or parameter was
changed during this task.

## Visualization

Niri window `Mississippi 856 | current maximum matching error 41.05 cm` shows
all 65,536 points, with original fine-reference points recolored in place at
uniform size 4. Orange is pole, magenta traffic sign and cyan curb. Current
0.6 m selections are drawn on the close floor at Z = -3.1153 m. No point overlay
is added. Full-scene/focused exports and exact viewer metadata are included;
`diagnosis.png` and `.pdf` separately illustrate the matching controls.

Current single-frame perception versus stored fine labels:

| Feature | Selected pillars | Reference-empty pillars | Covered reference points |
|---|---:|---:|---:|
| Pole | 0 | 0 | 0/18 |
| Traffic sign | 5 | 0 | 49/49 in ROI; 51 total |
| Curb | 27 | 7 | 60/83 |

Fine labels remain detector references, not independently surveyed truth.
Temporal matching uses 25 curb, zero pole and four sign distributions.

## Error source 1: a missed, split pole removes a useful constraint

The reference pole is near sensor XY **(17.9116, -3.5405) m**. Its 18 reference
points cross the Y=-3.5 m boundary between global pillars **7944 and 7945**,
with 11 and seven points respectively. In the compact off-ground raster those
same owners are numbered 7864 and 7865; both ID systems are exported explicitly.

The detector generates a shaft hypothesis, but its support height is only
1.976 m and owner support heights are 1.483 and 1.155 m. Owner support counts
are ten and five; whole-owner off-ground counts are 14 and 13. Learned owner
scores are **0.2366545** and **0.0239940**, well below **0.8747229**. This is a
final owner rejection, not missing input points or a near-threshold miss.
The geometry is split and sparsely supported; the exact contribution of each
feature to the nonlinear classifier score is not isolated by this study.
It would be incorrect to claim that moving the grid boundary alone is proven
to fix the detector.

The nearby shaft is detected at frame 855, but has only one detection in the
five-scan window then. Frame 856 supplies none, so it still fails the existing
two-acquisition confirmation requirement. At frame 857 the shaft is detected
again, confirms with 855, and contributes a pole match. Neighborhood component
counts document this gap. The earlier confirmed pole track expires at 854;
it does not replace the missing evidence at 856.

An oracle control identifies only this pole's reference owners in frames
852:856 using frozen labels and recorded poses, changes their selection masks
and probabilities, and retains all whole-pillar means/covariances, curb/sign
inputs, independent motion and temporal rules. It creates **one confirmed and
matched pole**. At evidence probabilities 0.9 and 1.0 the error falls to
**0.072210797 m** with the unchanged matcher. This demonstrates that recovering
the shaft could greatly improve this frame; it is not a detector fix or a
new recursive trajectory. The zero-intervention rebuild exactly reproduces
the current temporal source.

## Error source 2: map merging biases coarse registration; the gate keeps it

The 1.5 m map merge combines traffic-sign modes **1263 and 1264**, separated
by **0.964483 m**, with near duplicates 1277 and 1278. For source component 14,
the original 1264 center is 0.1813 m away at the reference pose, while the
merged target is **0.5452 m** away. A second merged group contains 1287, 1286,
1279 and 1280; source 15 is 0.2025 m from original 1279 but 0.5087 m from the
merged target. The physical identities or mapping origin of these separated
modes are not established here; the demonstrated issue is their merged
representation and resulting matching bias.

The coarse solution has 41.05 cm error. **Fine refinement already finds
10.16 cm**, but its **31.74 cm** displacement from coarse exceeds the **15 cm**
trust radius. Production therefore discards the fine result and retains the
biased coarse pose. Starting exactly at the recorded reference pose yields
the same 41.05 cm result, so this is not merely inherited prediction drift.
The prediction itself has 41.03 cm error: current matching fails to remove it.

| Fixed-frame control | Position error (cm) | Signed heading error (deg) |
|---|---:|---:|
| Current production | 41.050 | +0.6507 |
| Exact reference initial pose, oracle | 41.050 | +0.6507 |
| Allow existing unrestricted refinement | 10.162 | -0.0601 |
| Disable map merging | 10.162 | -0.0601 |
| Curb only | 14.445 | +0.7846 |
| Sign only | 67.715 | +1.3314 |
| Current scan only | 47.517 | +1.2772 |
| Reference temporal-window motion, oracle | 38.235 | +0.7576 |
| Add missed pole owners, oracle | 7.221 | +0.2067 |
| Add missed pole and disable map merging | 9.044 | +0.0735 |

Map radii 0 through 0.75 m give about 10.16 cm, while 1 through 2 m give
41.05 cm. Fine solves from prediction, reference and coarse seeds all reach
approximately 10.16 cm. The pole and map/gate mechanisms interact, so their
error reductions must not be added together. No map-radius or trust-gate
change is deployed: the prior frame-827 experiment showed that unrestricted
refinement can also be harmful. Curb has real reference disagreements, but
removing it increases this frame's error to 67.72 cm; it is still useful.

## Reproduction, checks and limits

From the root, add `pwd`, call `setupVehicleLocalization`, add this directory,
then run `replayCurrentRoute`, `diagnoseMaximum`, `inspectMaximumBasins`, and
`testMissingPoleMaximum`. The last diagnostic was repeated to export both
compact and global owner IDs; matching outcomes stayed unchanged.
`analyzeCurrentRoute.py` uses numpy/pandas/matplotlib to independently verify
all trajectory errors, compare the previous trajectory, validate controls,
check runtime/frozen pole scores and viewer invariants, and render the plots.

Four MATLAB files have zero factory Code Analyzer findings. Exact production
pose/source reproduction assertions passed, along with Python compilation,
independent numerical assertions and scoped whitespace checks. The replay
contains no unexecuted test claims or inferred user work hours. Source/cache
MATs remain local under `output/current_matching_20260929/`. The reused map,
route, fine labels and recorded pose reference limit independent validation.
This task diagnoses the remaining failure; it does not fix the split-pole
classifier, map merging, trust gate, or curb disagreements.
