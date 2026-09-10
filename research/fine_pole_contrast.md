# Fine pole point validation under weak neighborhood contrast

On 2026-09-10 the user identified Mississippi frame 91 original point 43363,
XYZ `[8.540121 -13.106604 -1.550120]` m, as a false-positive pole point.
Baseline: `1ab91a81a307f38ca94954d4a7bd7fb38540b91f`.

## Diagnosis and current rule

Its candidate has 3.154 m total height, 2.5 m qualified vertical support,
2.532 degrees fitted tilt, mean qualified neighborhood support fraction
0.7648, and radial RMS 0.1195 m. The point lies 0.1084 m from the fitted
axis. The preceding robust radial limit was 0.2558 m, allowing it through.
The candidate has 50 supported points, of which 48 previously passed the
robust point test. At the reported point's support interval, the object has
8 of the neighborhood's 10 returns; simply increasing that interval's
support threshold would remove other well-supported portions or entire
candidates on this frame.

The existing line fit and short-support gate remain. After computing the
robust per-point radius, the current fine validator applies:

- If mean qualified neighborhood support is below
  `poleLowContrastSupportRatio = 0.80`, cap the accepted radial distance at
  `poleLowContrastMaximumRadius = 0.10` m.
- Otherwise retain the existing robust radial limit.

This point-level rule retains the narrow core instead of rejecting the whole
candidate. Its accepted count changes from 48 to 28, and 43363 is excluded.
These surviving points are not independently confirmed true poles. Original
indices are test/display metadata only; no coordinate, ring, frame, or index
exception enters classification. Coarse processing remains whole-pillar based.

## Executed comparisons

The new rule was compared with the same pipeline using an infinite
low-contrast radius cap on the 30 reference frames plus Mississippi 91 and
425. The complete non-pole masks and complete coarse probability clouds
match on all 32 frames. No pole points are added; 20 frames have unchanged
complete masks.

The 30 reference frames retain 6,033 of 6,348 preceding pole points (95.04%),
with 315 removed. Frame 91 changes from 269 to 225, removing 44 points across
low-contrast candidates; all three user-reported indices 47906, 16728 and
43363 are rejected. Frame 260 retains all 136 pole points including confirmed
63010 and its recovered neighborhood; frame 425 retains all 21. The largest
reference-frame decrease is Mississippi 900, from 506 to 418. Additional
removals have no independent labels confirming correctness. These are
baseline-retention measurements, not precision or recall.

The cumulative current removal fixture is updated without overwriting the
frozen original perception/recovery fixtures. Exact-mask regressions still
cover every point. The frame-91 test now requires all three reported points
to be rejected and 225 pole points to remain, including after permutation
into unorganized XYZ vectors (seed 91047906).

The combined implementation and viewer-interaction suite passed 330 tests,
with zero failures and two synthesis tests filtered for unavailable YALMIP/SDP
dependencies. Factory Code Analyzer found zero issues in all eight changed/new
MATLAB files. The native niri preview contains 225 pole, 73 curb and 126 sign
points, with 43363 displayed as source, direct RGB coloring, uniform markers,
original-index datatips and the current camera retained. No full-route
regeneration or new accuracy benchmark was performed.

See `research/results/pole_contrast_20260910/` for check summaries and
`output/pole_contrast_20260910/` for native/diagnostic originals. The companion
[point-rotation repair](perception_viewer_interaction.md) is independently
specified and does not change semantic masks.

A later [candidate-isolation review](fine_pole_isolation.md) rejects a weakly
separated candidate as a whole when its shaft neighborhood is also cluttered.
The point-radius rule above remains active for candidates that pass that check.
