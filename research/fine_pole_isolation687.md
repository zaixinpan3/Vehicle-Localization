# Tighten fine pole isolation on Mississippi frame 687

Date: 2026-09-10. Baseline: `3b21d86082e25a010b068e75391c645d194988ca`.

The user identified original point 47135, XYZ `[4.088118 -18.437752
-0.693202]` m, as an unwanted pole detection. Its candidate has 2.0 m of
qualified support, 1.7069 m actual supported Z span, 2.7799 degrees fitted
axis tilt, 0.078827 m radial RMS, and 0.76042 mean local support fraction.
At qualified shaft heights, 19 of 24 retained nonground neighborhood returns
are inside the 0.25 m core around the fitted axis, within the 0.75 m search
radius: 79.167%. It passes the preceding 75% isolation requirement.

Raise `poleIsolationMinimumCoreFraction` from 0.75 to 0.80. This tightens the
existing fine-only candidate rejection rule without introducing another
algorithm, a coordinate exception, or a ring/index dependency. Isolation is
still required when mean local support is below 0.80. Candidate rejection
precedes final point trimming. Whole-pillar coarse processing is unchanged.

## Observed effects

Compare current settings against the same settings with isolation fraction
0.75, on the 30 frozen-reference frames plus Mississippi 91, 425 and 687:

- Frame 687: pole points 136 to 124. All 12 points in the reported candidate
  are removed, including 47135. Curbs remain 133 and signs remain 8.
- Frame 1050: pole points 71 to 61. These additional ten removals are
  algorithm effects without independent user labels.
- All other 31 frame masks are unchanged. Frame 260 retains all 136 poles,
  including known true point 63010; frame 425 retains all 21; frame 91
  retains all 147 and continues rejecting previously reported false poles.
- All 33 coarse probability clouds and non-pole masks are exactly unchanged.
  No new poles are added. Combined pole counts change 6091 to 6069;
  frozen-reference retention is 5777/5787 (99.83%). Retention is not accuracy.

The cumulative rejection fixture explicitly records the changed indices;
original baseline evidence remains intact. Tests exercise the 19/24 rejection
and inclusive 20/25 acceptance boundary, recorded frame 687, and shuffled
unorganized XYZ input with seed 68747135, alongside existing reference,
whole-pillar, pipeline and viewer tests. Results are exported under
`research/results/pole_isolation687_20260910/`; raw masks, test logs and
native figure remain in `output/pole_isolation687_20260910/`.

Native niri frame 687 was recomputed and refreshed with original-point colors,
frame counter, point-index tips and the current camera unchanged. No complete
route/video/mapping rerun or new accuracy/timing measurement was performed.

Validation completed: 100 affected MATLAB tests passed, zero failed and zero
incomplete. Factory Code Analyzer reported zero findings in five checked
MATLAB files. The initial temporary diagnostic tracer used unavailable
`range()`; replacing it with `max()-min()` allowed the successful measurement.
All diagnostic breakpoints were cleared.
