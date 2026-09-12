# Short pole support with an elongated transverse footprint

## Diagnosis

The user identified Mississippi frame 28, OriginalIndex 37328, at
`[21.250158, -8.343867, 1.128219] m` as an unwanted pole detection.
The baseline is commit `23220c2aa8d93b266c9884b3d57f5daddf2d6bad`.
A runtime trace captured its 46-point candidate, which produced 14 pole labels.
It had 2.0 m of qualified fine support, a fitted tilt of 3.279 degrees and
radial RMS of 0.099991 m. The transverse principal standard deviations were
0.099660 and 0.022378 m, giving an aspect ratio of 4.453. It therefore fit an
elongated strip while passing the previous 0.12 m broad-surface width gate.
The neighborhood core fraction was 0.84375 and the returns were vertically
continuous; neither isolation nor a missing vertical run explained this case.

## Change and comparison

The existing transverse-shape rejection now uses a 0.09 m major-axis standard
deviation threshold for support at or below the existing 2.0 m short-support
boundary. The aspect-ratio threshold remains 3.0. Longer support retains the
0.12 m width threshold. Both width and anisotropy must exceed their limits,
measured before radial trimming. Configuration and inference contain no frame
IDs, original indices, scan rings or scene-specific coordinates.

A uniform 0.09 m threshold was evaluated first. Across 62 frames it removed
203 labels, including 189 labels in Mississippi 687, 75 and 900 and downTown
300. These additional candidates had 2.5 to 3.0 m of support. That experiment
was replaced with the short-support condition to limit changes to accepted
baseline behavior.

The final 62-frame comparison changes 9734 to 9720 pole labels: exactly the
14 labels of the reported frame-28 candidate are removed, with no additions.
All 61 other pole masks are identical. All non-pole masks and coarse
probability products are identical in all 62 comparisons. Frame 28 retains
35 curb points, 127 pole points and 453 traffic-sign points. Only 37328 is
explicitly user-annotated; the other 13 removals are inferred members of the
same candidate. This comparison measures output stability, not dataset-wide
precision or recall.

The native niri preview was refreshed in place with all ten camera/axis
properties unchanged. Original-point coloring, uniform point size, the frame
counter and datatips remain available. The existing full-route video remains
a record of the preceding version and was not regenerated during this fix.

## Validation and artifacts

A recorded regression checks the reported false positive, the complete removal
count, locality of the 14 removed points, unchanged other outputs and exact
invariance under unorganized XYZ permutation with seed 2837328. Existing
ablation tests that disable transverse-surface filtering explicitly disable
both its long- and short-support thresholds; their expected results remain
unchanged. No historical expected-output fixture was revised.

All 93 tests passed: 25 pole-specific tests and 68 current-reference,
feature-selection, pipeline and whole-pillar tests. Code Analyzer found zero
issues in the five changed MATLAB files; `git diff --check` passed.

Diagnostics, comparison results, test logs, static-analysis results and the
native preview are under `output/pole28_review_20260911/`. Temporary runtime
tracers were cleared after capture. The production change only affects fine
pole validation; coarse processing still uses whole pillars.
