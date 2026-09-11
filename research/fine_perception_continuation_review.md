# Fine perception: measured curb continuations and sparse pole recovery

Activity date: 2026-09-10. Baseline project revision:
`304465cc18ac1d510ce0752dfa1b50257785fe60`.

## Measured curb continuation

The earlier [frame 538 extent review](curb538_extent_review.md) retained a short
boundary after the user said shorter output was acceptable. The user then
clarified that improvement was wanted. The following implementation supersedes
that decision without changing the historical diagnostic record.

Primary curb extraction keeps its existing metric geometry and boundary model.
Endpoint refinement now also revisits previously rejected candidates, using
an overlapping section of the established curb as an anchor. The existing
unexamined-ground continuation still handles its current cases.

For sparse, approximately straight continuation, the measured endpoint tangent
sets the local strip direction. A metric ellipse extends the neighborhood to
2.0 m along the curb while retaining the existing 0.65 m transverse radius.
Relief, plane residual, strip width, population and surface-slope thresholds
remain unchanged. For a bending continuation, the same local geometry can
instead feed the existing ridge tracer, which connects compatible measured
returns rather than relying on one fitted curve or a fixed endpoint direction.

Additional boundary models must overlap at least three accepted-tail neighbors
within 0.15 m, spanning at least 0.75 m. Forward additions start beyond half the
0.15 m sampling interval; existing primary labels are excluded. Stop at the
first measured 3D gap over 0.75 m, and require six spatial support cells and
1.5 m forward span. Existing endpoint proximity and 30-degree tangent checks
also apply. Deduplicate evaluated/accepted point indices because a rejected
candidate can now be reconsidered. Output remains a subset of original points,
with uniform marker size and per-point RGB; there is no ring-based inference.

Frame 538 changes from 131 to 166 curb points, retaining all existing labels.
Left maximum sensor-frame X changes 13.052 to 17.902 m, right 12.241 to
17.842 m. All six supplied right-side indices are selected; 19 of 28 left-side
indices are selected. The left sample near X=18.4 m remains outside the final
supported extension; the detector stops before a larger measurement gap.
The user accepted the updated preview. The previously accepted left boundary
already contained a scan row with nine points, and the added extension has
one row with six; density validation checks preservation of the prior maximum
and that at least 80 percent of occupied extension rows have at most five
points. This is an output audit, never an inference input.

Frame 214 changes from 92 to 110 curb points, retaining the existing labels.
Seventeen of 18 supplied vicinity samples already passed local geometry, but
the prior model did not connect their bend. Ten supplied indices are now
selected and all 18 lie within 0.168 m XY of an actual selected point. No
unmeasured points are synthesized to fill the boundary.

## Reject a reversal created by terrain correction

Frame 963's 12 user-reported false curb points belonged to a preexisting
boundary; they were not introduced by continuation. Eleven have opposite
raw and terrain-corrected local gradient directions. Reject a boundary only
when at least 80 percent of its selected points have a nonpositive gradient
dot product, with at least six such points in three independent XY cells.
This is a boundary-level consistency test, not a blanket per-point sign rule.

An intermediate per-point rejection removed valid frame-538 labels and was
replaced by the boundary consensus rule. The final frame-963 result removes
exactly the 12 reported false points and preserves all 60 points of the other
boundary. The user accepted that preview. Coarse processing is unaffected.

## Recover a sparse but strongly supported pole

Frame 214 original index 34450 lies in a whole-pillar distribution with 24
returns, 5.3735 m vertical span, 0.1715-degree inclination and 0.0490 m radial
RMS. It passed the existing coarse pole and strong recovery-statistic gates,
but a second detailed seed filter rejected its two-cell longest support run,
even though five fine height cells had enough returns overall.

Keep the existing detailed recovery gate for ordinary cases. Add a narrowly
qualified alternative for a whole-pillar shaft with at least 4 m span,
at most 1-degree inclination and 0.06 m radial RMS. This alternative still
requires the existing recovery gates (12 returns, 3 m span, 3-degree tilt,
0.10 m RMS and coarse pole evidence), shared-axis neighbor completion and
facade exclusion. An intermediate removal of the detailed gate added 1075
pole labels across 52 frames and was rejected as too broad.

The five subsequently reported false positives in frame 214 were already
present before sparse recovery. They belong to a 13-point supported fragment
of a 63-point candidate, with only 1.5 m qualified support. Its immediate-cell
support ratio is 0.8214, but the wider isolation test measures 15 core returns
out of 19 neighborhood returns (0.7895), below the existing 0.80 requirement.
Apply that isolation check to every shaft with at most 2 m supported height,
in addition to the existing longer, weak-contrast case. No isolation threshold
is relaxed. The resulting frame-214 pole count is 117: 34 true-shaft additions
and 13 removals from the prior 96. Index 34450 is selected; all five reported
false points are rejected. Curbs remain 110 and traffic signs remain eight.
The user accepted the refreshed native preview.

## Recover a nearby shaft split at an XY cell boundary

In frame 746, the user identified 41 distinct indices around XY=(-3.14,-4.10)
m. The original fine candidate included only 38 neighboring edge returns,
while the denser body was omitted and then counted as clutter. Two body
pillars hold 140 and 127 returns, with spans 3.115 and 3.071 m, tilts 1.986
and 1.095 degrees, and radial RMS 0.0542 and 0.0352 m. An existing detailed
seed has 24 returns, 3.099 m span, 1.035-degree tilt and 0.0303 m radial RMS.

Fine-only boundary completion now starts from an existing detailed seed that
meets the tall recovery limits and the tighter 0.06 m radial RMS limit. It
also requires an adjacent similarly strong pillar with at least three times
the seed's count. This targets a sparse edge fragment alongside a dense shaft,
not ordinary fully represented poles. Apply the shared-axis completion twice
to join the two sides, then clip to the original seed's immediate XY neighbors;
there is no unbounded transitive growth. Move the entire touched base candidate
component into joint validation with the added shaft points. All final support,
tilt, radial and isolation checks remain active.

An intermediate unrestricted completion changed unrelated candidates and was
replaced by this density-imbalance condition. Before the subsequent trunk rejection, frame-746 pole output
changes from 422 to 717: all 295 additions lie on the reported shaft, all old
labels are retained, and 40 of 41 supplied indices are selected. The remaining
supplied point 55941 does not pass point-level selection. Curbs remain 98 and
traffic signs remain seven. The native preview preserves camera properties,
uniform original-point RGB display, frame number and datatips. Coarse inference
continues to use whole-pillar XYZ statistics, with no height subdivision. No
frame number, original index, organized row or fixed scene coordinate is used
by the detector.

## Reject a broad transverse trunk surface

The subsequent frame-746 false positive at index 19993 lies on a thick trunk
surface. Its candidate has 168 returns, with 132 supported across 3 m height.
It is nearly vertical (0.915 degrees) and dominates its immediate neighborhood
(mean support ratio 0.9467), so the existing short-support restrictions do not
apply. The supported transverse radial RMS is 0.1667 m. After detrending the
axis, covariance eigenvalues are approximately 0.0025585 and 0.025434 square
meters: major-axis standard deviation 0.1595 m and aspect ratio 3.153.

Reject supported groups whose major transverse standard deviation exceeds
0.12 m and whose major/minor standard-deviation ratio exceeds three. The two
conditions distinguish a broad elongated surface from a compact narrow shaft;
verticality alone is insufficient. Measure all supported returns before any
robust point trimming. This is not a botanical classifier or a complete thick
trunk detector, and it does not assume organized scan structure.

This removes exactly 131 prior pole labels near the annotated trunk. The new
295-point narrow shaft and 40/41 supplied positive indices are unchanged.
Final frame-746 counts are 98 curb, 586 pole and seven traffic signs. Relative
to the original displayed frame, there are 295 additions and 131 removals.
The user accepted the corrected preview and requested another frame.

## Validation and limitations

A frozen 52-frame comparison covers 42 Mississippi and 10 downTown frames.
Frames 214 and 963 were also reproduced with the baseline commit's original
functions in a temporary isolated path to verify their pre-change masks.
The curb changes alone retain 36 complete frame masks, add 265 labels and
remove 20, changing the total 5128 to 5373. The removals are the 12 annotated
frame-963 false positives and eight unlabeled frame-800 points. Other curb
additions outside the reviewed frames are regression effects, not measured
precision improvements. All non-curb masks and coarse probability clouds
match before the subsequent pole change.

The subsequent pole-only comparison changes 9272 to 8345 labels (+34/-961),
with 35 of 52 masks unchanged. Of the removals, 515 result from the short-support
isolation requirement and a further 446 from broad transverse surface rejection.
The boundary-fragment completion has no effect on these 52 frames and is
separately verified on frame 746. Known annotated positive poles in frames 260,
1047 and 214 and prior negative examples are checked by the regression tests.
The roughly ten-percent decrease in pole labels across this sample is not a
measured precision improvement; unlabeled removals require further visual or
ground-truth review. In particular, downTown frame 250 changes 170 to zero pole
labels. Curb, facade, sign and ground masks and coarse probability clouds are
unchanged by the pole changes.

Local artifacts are in `output/curb538_extension_20260910`,
`output/curb963_review_20260910`, `output/curb214_review_20260910` and
`output/pole214_review_20260910` and `output/pole746_review_20260910`. Tests cover plane rejection, disconnected
steps, a nearby parallel curb, rejected-candidate recovery, recorded vicinity
coverage, prior false positives and unorganized permutations. Immutable
reference outputs are retained, with explicit intended deltas in separate
fixtures. No full-route video or mapping rerun, new accuracy qualification,
or real-time performance claim is made.

## Follow local curb steps beside taller terrain in frame 1137

The supplied neighborhoods contain 38 left and 26 distinct right curb vicinity
samples, plus 13 right-side false positives. All 64 positive vicinity samples
are ground returns already evaluated by fine perception, but only one is a
primary seed. Seventy-five of all 77 supplied samples pass the local geometry
gates. The left neighborhood's detrended relief reaches roughly 0.32--0.62 m,
while the narrow local strip is predominantly 0.11--0.23 m. The larger
neighborhood includes taller terrain, preventing valid local steps from
seeding continuation. On the right, the competing upper response lies less
than the previous 0.4 m minimum edge separation from the lower curb.

Keep the primary seed's full-neighborhood relief cap. For an anchored endpoint
continuation only, permit the existing local-strip relief cap to govern
seeding even when surrounding terrain is taller. All strip, residual, slope,
mid-height, thin sampling, overlap and continuity gates still apply. Reduce
the minimum competing-edge separation from 0.4 to 0.2 m so a supported lower
step can suppress a nearby weak raised response. This does not widen the
output band or use scan organization.

Frame 1137 changes 102 to 142 curb labels, selecting 21 of 38 left and 15 of
26 right vicinity indices and rejecting all 13 supplied false positives.
The annotations describe neighborhoods; these exact-index counts are not
recall against a complete ground-truth boundary.

## Require actual vertical continuity for pole points

Frame 1137 index 54471 belongs to a candidate with qualified height cells 14,
16, 17 and 20. Adding their individual heights produced 2 m support even
though the top fragment was disconnected. The accepted core has a 1.205 m
vertical gap before that fragment. The previous validator lacked a test that
an individual connected run itself represents a shaft.

Partition axis-consistent candidate returns at actual Z gaps over 0.75 m.
Each run must satisfy the existing six-return and 1 m height minimums. Output
points must also retain their prior qualified-cell support and robust radial
acceptance. Use actual axis-consistent returns, including sparse cells that
are not independently sufficient for output, to establish continuity. An
intermediate version used only output-qualified returns and unnecessarily
truncated sparse frame-214 support; it was replaced before completion.

This removes five isolated top-fragment labels in frame 1137, including 54471,
changing its pole count 103 to 98. Frame 214 remains 117 pole points with the
recovered true shaft intact. Frame 746 remains 586 pole points and retains the
295-point nearby shaft; all 14 reported frame-1047 shaft indices remain selected.
The native frame-1137 preview preserves the camera and displays 142 curb,
98 pole and 18 traffic-sign points. These checks do not label or validate
all remaining objects in the sequence.

## Final cumulative comparison

After the frame-1137 follow-up, the same 52 recorded cases change 5128 to
5563 curb labels (+567/-132), with 34 unchanged curb masks, and 9272 to 8323
pole labels (+34/-983), with 33 unchanged pole masks. All ground, facade and
traffic-sign masks and coarse probability clouds match the frozen original
snapshot. The preceding comparison totals describe intermediate stages.

The changes outside reviewed examples are not ground truth. Mississippi
frame 370 changes 112 to 255 curb labels and downTown frame 250 changes 170
to zero pole labels; these particularly large unlabeled changes warrant
further visual or labeled-data review. The annotated frames and permutation
tests constrain regression, but cannot establish overall precision or recall.
Final explicit regression deltas are in `fineCurbGuidedExtension.json` and
`finePoleBoundaryRecovery.json`; the original immutable mask evidence is
preserved. Frame 1137's supplied neighborhoods are at most 0.274 m (left) and
0.135 m (right) XY from selected curb returns. No complete-route video,
map-building run or runtime optimization was performed in this iteration.

Final validation: 117 tests passed, zero failed or incomplete, using
`/tmp/test_final_perception.m`. MATLAB Code Analyzer reported zero findings
across the 12 changed MATLAB files. The suite covers the pipeline reference,
immutable-mask revisions, synthetic geometry, previously annotated false
positives, whole-pillar invariants, viewer behavior and input permutations.
Initial test harness failures (an unavailable `range` function and an absent
frame-91 immutable base entry) were corrected; old ablation configurations
were updated to isolate their intended gates from the new independent filters.
Final logs and analyzer results are in `output/curb1137_review_20260910`.
