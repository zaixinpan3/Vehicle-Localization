# Local ridge continuation of revalidated curb models

## User target and diagnosis

After orientation-guided recovery in commit
`58123bb6c7a5a2895db53122af2df647c6c01ab7`, the user identified 41 remaining
missed points on Mississippi frame 855, around XY (5.4, 13.1) to (2.1, 14.7) m,
and requested that at least half receive curb labels. All 41 are ground points
and have already been evaluated, but the baseline labels none of them.

The revalidation neighborhoods produce usable geometric candidates. The
remaining failure occurs when a single global boundary proposal and curve fit
select a shortened or displaced subset of a bending ridge. Local region
probes with the existing guided geometry and global model select 11--15 of
the reported points; following a local ridge instead selects up to 24 in the
same probe region. These angle probes were diagnostic only; no fixed angle or
scene coordinates enter production inference.

## Implementation

Orientation-guided revalidation now uses the existing local ridge tracer,
which follows connected metric boundary evidence through bends, instead of
re-fitting one global line/curve. Existing ordinary proposals remain unchanged.
The change is limited to the revalidation pass for models rejected by the
normal-consistency gate. Guided ridge revalidation continues to enforce the
ordinary neighborhood-relief cap; the pre-existing unguided trace mode retains
its separate allowance for tall neighboring terrain.

No threshold was loosened. Ground segmentation, whole-pillar coarse perception,
other semantic classes, original-point coloring and point size are unchanged.
Production logic uses neither ring indices nor frame-specific annotations.

## Results

Frame 855 now labels 27 of the 41 newly reported points (65.9%), compared with
zero previously, exceeding the requested half-coverage target. The full curb
mask changes from 153 to 177 points: 45 additions and 21 removals as the thin
boundary is reselected. The removals are not independently annotated as false
positives. Pole and traffic-sign counts remain 23 and 60. The native niri
preview preserves the user's camera and frame counter.

The recorded-point regression requires at least 21 of the 41 points to be
classified directly as curb. Existing vicinity coverage, original-label
preservation, unorganized permutation, reported false-positive and pipeline
checks remain part of validation. The full-route video exported earlier is
not re-rendered by this correction.

For the 41 new picks, nearest-curb XY distance quantiles (median, 90th
percentile, maximum) are now 0, 0.2641 and 0.4367 m. Of the earlier 137 vicinity
picks, 125 now have a curb point within 0.35 m, up from 108. Their maximum
nearest-curb distance decreases from 1.9460 to 1.2089 m; an endpoint remains
partly missed.

Across 61 frames, 58 curb masks remain identical. Total curb labels change
from 6512 to 6547 (69 additions, 34 removals). Besides frame 855, frame 384
changes 161 to 163 (-4, +6), and frame 850 changes 140 to 149 (-9, +18).
Those changes remain unannotated accuracy-review limitations. The six reported
frame-384 false positives stay rejected. Every compared non-curb mask and
coarse probability product remains identical.

All 106 tests pass: 38 curb tests plus 68 current-reference, feature-selection,
pipeline and whole-pillar tests. Code Analyzer reports zero findings in the
three modified MATLAB files.

Measurements, per-frame changes, tests and the native preview are stored in
`output/curb855_completion_20260911/`.
