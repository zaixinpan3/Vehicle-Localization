# Short pole support boundary: Mississippi frame 901

The reported false pole point is OriginalIndex 17708, approximately
[0.581737, 10.563892, -0.943924] m. Its connected candidate contains 102
points; the previous fine detector labels 85 of them as pole. The candidate
has exactly 2.0 m of qualified height support and an axis radial RMS of
0.11979 m. Its axis tilt is approximately 1.43 degrees. Passing the local isolation check does not establish that its broad XY
cross-section represents a narrow shaft.

The short-support continuity and tilt gates already included the 2.0 m
boundary, while the radial compactness gate used a strict less-than test.
At exactly 2.0 m, only weak-context candidates received the 0.10 m RMS check.
This strongly separated candidate therefore bypassed the intended compactness
limit. `refinePerceptionCandidates` now uses the inclusive short-support
classification consistently. The redundant weak-boundary exception is removed.
No threshold values, coarse pillar statistics, ring-dependent rules, or
scene-specific exclusions are introduced.

Frame 901 changes from 240 to 155 pole labels, removing exactly this 85-point
candidate including the supplied false positive. The other pole labels remain;
74 curb and 11 traffic-sign labels are unchanged. The native niri preview
preserves camera properties, frame counter, original-point colors and datatips.

The regression test isolates radial compactness at the exact support boundary,
verifies rejection of the reported point and its candidate, preserves
other masks and the coarse probability cloud, and checks an unorganized point
permutation using seed 90117708. Recorded cross-frame differences are regression
evidence, not a precision/recall measurement or a complete accuracy annotation.

The 56-frame comparison against commit
`60e4ed58e464c19fc1f82fceba4917ece4f9869f` changes 9043 to 8717 pole labels
(-326, no additions), with 50 frames unchanged. Besides frame 901, removals
occur in Mississippi frames 100 (120), 775 (46), 825 (21), 1150 (16), and
downTown frame 100 (38). These additional unannotated removals are an accuracy
review limitation. Previously reviewed frames 91, 214, 260, 384, 425, 687, 746,
832, 1047 and 1137 retain their pole masks. All non-pole feature masks and coarse
probability clouds remain identical across the comparison.

Validation finishes with 91 passing tests and zero Code Analyzer findings in
three checked MATLAB files. An initial ablation lowered the shared support-height
threshold and inadvertently bypassed isolation, admitting 26 additional points
elsewhere. The corrected ablation disables only radial compactness; its focused
rerun passes alongside the other 90 regression tests.
