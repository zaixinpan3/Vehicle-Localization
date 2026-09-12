# Measured support for fine curb endpoints

## Diagnosis and change

The user identified Mississippi frame 28 OriginalIndex 36671 at
`[5.039316, -1.828372, -2.103029] m` as a false curb return. The baseline is
commit `f6d4116e3323e49bd23d6134a3f18c37dfed7838`.
Runtime tracing found this point at the end of an ordinary fitted boundary.
Its local plane residual was 0.062468 m, score 0.9697 and detrended height
slope 0.103512. Thus broad neighborhood confidence was insufficient to reject
it. Its height gradient deviated 28.53 degrees from the normal to the measured
local boundary tangent. No boundary return extended that gradient's predicted
tangent within the existing 0.10 m transverse band and 0.30--2.0 m distance.
The aggregate boundary-normal check nevertheless accepted the model.

Fine validation now checks the terminal 0.15 m of each fitted boundary once,
after ordinary model validation. A terminal point is removed only when its
gradient deviates more than 25 degrees from a supported local boundary normal
and no measured return of that boundary supports its predicted tangent.
Sparse endpoints with fewer than three local returns or less than 0.30 m
local extent remain inconclusive and are retained. Interior points are not
trimmed by this check. The global boundary-normal limit remains 45 degrees.
All lengths reuse existing metric configuration values; the new endpoint
angular limit is in `finePerceptionConfig.m`. No ring, row, original-index or
scene-coordinate condition is used in inference. Coarse processing remains
whole-pillar based.

## Comparison

Across 62 frames, curb labels change from 6582 to 6559: 25 removals and two
additions, with 46 unchanged masks. Frame 28 changes from 35 to 34 curb labels
by removing exactly OriginalIndex 36671. Its 127 pole and 453 traffic-sign
labels remain unchanged. All non-curb masks and coarse probability products
remain identical across all 62 comparisons. Changes in other frames are
unannotated and are recorded as output differences, not verified accuracy
improvements. Removing primary endpoints can change subsequent continuation
anchors; this is the inferred mechanism for the two additions in other frames.

The native frame-28 preview was refreshed in place. All ten camera/axis
properties remained identical; original-point colors, uniform marker size,
frame counter and datatips are retained. The earlier full-route video remains
an artifact of the preceding perception version.

## Validation artifacts

The new tests cover rejection of an unsupported tip, retention of interior and
inconclusive sparse points, rotation and permutation invariance, and the exact
frame-28 mask change. The recorded-data shuffle uses seed 2836671. Historical
reference files remain intact; `fineCurbEndpointSupport.json` records explicit
new expected-output revisions, which are regression data rather than ground
truth. Runtime tracers were cleared after diagnosis.

Three existing ablation tests initially failed because the independent endpoint
check removed some of the false positives those tests deliberately exposed.
Their isolated configurations now disable endpoint trimming, preserving the
original expected results. The default pipeline is validated separately by
the recorded comparison and current-reference tests.

Outputs are in `output/curb28_review_20260911/`: `baseline.mat`,
`comparison.mat`, `comparison.json`, regression logs/results, static-analysis
results, and native preview files with `desktop_validation.json`.

All 110 final tests passed: 42 curb tests plus 68 current-reference,
feature-selection, pipeline and whole-pillar tests. Code Analyzer reported
zero findings in eight changed MATLAB files; `git diff --check` passed.
