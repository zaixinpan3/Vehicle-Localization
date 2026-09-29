# Frame 827 matching after Mississippi pole calibration

Current perception and unchanged production matching reduce frame 827 position
error from **0.563038 m to 0.093794 m** against the recorded pose reference.
Signed heading error changes from -1.050740 degrees to +0.007923 degrees.
This is a measured replay result, not a new matching implementation.

## Method and controls

`evaluateRevised827` recomputes raw perception for frames 1 through 827 using
the current Mississippi profile from commit
`b73e0c7c26867c1fbf63cf6d510a277035f3136a`. It reconstructs the causal five-scan
source window, requiring detections in at least two scans. Alignment and pose
propagation use the frozen wheel/gyro/observer odometry from the previous
production experiment. Map corrections propagate recursively; the initial
pose is supplied only at frame 1. Reference poses are used for error scoring
and the existing roll/pitch projection convention, not XY/yaw initialization
or intermediate resets. This reproduces the existing pipeline convention,
not an independent evaluation without INS tilt.

The semantic map, registration configuration, map/source merge radii and
curb-direction factor are unchanged. The script also re-solves the previous
frame 827 source from its recorded prediction and asserts agreement with the
previous production pose within 1e-7. Updating only the source at that same
prediction produces the same 9.3794 cm result as the new causal prefix.

| Frame 827 variant | Position error (m) | Heading error (deg) | Result |
|---|---:|---:|---|
| Previous source and prediction | 0.563038 | -1.050740 | Coarse retained |
| Revised source, previous prediction | 0.093794 | +0.007923 | Fine accepted |
| Revised causal prefix | 0.093794 | +0.007923 | Fine accepted |

The revised prediction has 0.2338 m position error. The final error in vehicle
coordinates is approximately -0.0577 m forward and -0.0740 m left. There are
29 matched distributions: 23 curb, 2 pole and 4 traffic-sign components.
Both poles survive temporal confirmation and participate in the final solve;
their map component IDs are 1009 and 1016. These are Gaussian distributions,
not point counts or necessarily distinct physical objects. Their source/map
center distances at the reference pose are 0.1114 and 0.2356 m.

Previously the coarse-to-fine shift was 0.2794 m, exceeding the 0.15 m trust
radius and retaining the biased coarse result. With the additional pole
constraints it is 0.1093 m, so refinement is accepted. The same-seed control
shows that changed current source evidence is sufficient for this improvement;
it is not solely an effect of accumulated trajectory changes.

## Validation and limits

The raw 827-frame causal prefix completed with finite errors. The old result
reproduction assertion passed. Both MATLAB scripts have zero factory Code
Analyzer findings. `inspectRevised827` exports actual final, unmerged
associations in `solution_pairs.csv`; these are not representative indices
of merged coarse map groups. Recorded matching-only runtime at frame 827 is
11.465 ms for one run, excluding perception/window construction; it is not
a performance benchmark. No new production code or parameters were changed.

This does not resolve the known frame 827 curb misselections/misses or the
map-mode merging issue. Only frames 1:827 were replayed, so no claim about
the revised full-route maximum is made. The map and development route are
reused, and the reference is the recorded pose table rather than independent
survey truth. Frozen fine-perception point labels are separate from the pose
reference used for localization errors.

## Reproduction and artifacts

Run from the repository root with the original local data and prior cache:

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/frame827_revised_matching_20260928');
evaluateRevised827;
inspectRevised827;
```

Inputs are `data/raw/MissisipiPointClouds.mat`, the synchronized pose table
named by `featureMapBuildConfig`, and
`output/line_direction_matching_20260928/{sources.mat,production/report.mat}`.
The cache contains the projected calibrated semantic map and independent
odometry. Local detailed results are stored in
`output/frame827_revised_matching_20260928/results.mat`.
Committed CSV/JSON files contain the causal prefix, three-way comparison,
class diagnostics, association geometry and summary. Large raw/cache MAT
files remain local and are excluded from version control.
