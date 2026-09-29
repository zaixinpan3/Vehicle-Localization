# Current Mississippi route maximum

Completed the current causal trajectory through all 1,170 frames. Excluding
frame 1, the maximum is **frame 857**, at 85.5988 seconds:
**0.473865025 m** position error and
**1.115077 degrees** signed heading error against the recorded
pose reference. The overall maximum remains frame 1's configured
0.640312424 m initialization error, with no accepted match.

Frame 857 is accepted with `coarseRetainedByTrustRadius`. Its temporal source
has 26 curb, zero pole and five traffic-sign distributions. These are source
component counts, not verified association counts or physical object counts.
Frames 858 and 859 follow at 0.46903 and 0.43751 m; the error is concentrated
in a short neighborhood. This experiment locates the failure, not its cause.

Full-route RMSE is 0.134649813 m and P95 is 0.244939135 m.
There are 1150 full updates, 12 directional
updates and 8 frames without an update. Frame 827 remains at
0.093794451 m. No production code or parameters were changed.

## Method and validation

`evaluateRouteMaximum` reuses the verified current 1:827 causal prefix from
`output/frame827_revised_matching_20260928/results.mat`. It verifies all
perception/registration/window configuration values, normalizing only the
frame-dependent projection rotation. Raw frames 823:827 rebuild the five-scan
window; exact source-component equality at 827 is asserted before resuming
from the saved recursively corrected pose. Raw frames 828:1170 then extend
that same trajectory with frozen independent wheel/gyro/observer odometry.
There are no reference XY/yaw resets. Existing recorded INS tilt projection
and the original calibrated map are retained. This reused route/map and
recorded pose reference do not constitute independent survey validation.

An initial configuration comparison removed the projection field on only one
side and failed before replay. The corrected comparison normalizes it on the
saved configuration. Configuration/window assertions and replay then passed.
The script has zero factory MATLAB Code Analyzer findings. Independent Python
checks verified all 1,170 errors from trajectory/reference XY values, exact
prefix equality, the maximum frame, RMSE and top-20 ranking. Whitespace checks
passed. Timing columns combine runs and are not a timing benchmark.

## Reproduction

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/revised_route_max_20260928');
evaluateRouteMaximum;
```

Inputs also include the raw Mississippi scan/pose files and the prior
`output/line_direction_matching_20260928/` map/motion cache and production
report. CSV/JSON exports here contain the full trajectory, top-20 ranking,
reference neighborhood and summary. Local `output/revised_route_max_20260928/results.mat`
contains detailed results; its `maximum` object is the suffix maximum (which
in this run is also the post-initialization full-route maximum at 857).
Large datasets and MAT caches remain local.
