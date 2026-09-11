# Spatial support review: Mississippi frames 1137 and 384

The implementation operates on unorganized XYZ coordinates. Original indices below
identify review annotations and regression evidence only; production code does
not use frame numbers, rings, rows, columns, or scene-specific coordinates.
The coarse product remains whole-pillar statistics and probabilities. All changes
in this review affect offline fine detection.

## Curb boundary orientation

Frame 384 contained an eight-point false boundary, including the six reported
indices 21753, 21628, 20477, 19007, 17664 and 17600. These returns approximately
formed a line, but most local height-transition directions were longitudinal
rather than transverse to it. The existing boundary-level gradient-reversal
check was insufficient: four of the eight returns reversed their gradient,
below its 80% rejection threshold.

`hasConsistentCurbNormals` estimates a local XY tangent within 2 m using SVD.
An assessed height gradient must lie within 45 degrees of the transverse
normal; a boundary requires 60% agreement. Fewer than three assessable points
remain inconclusive, avoiding a decision from an unsupported tangent. This
local construction accommodates curved boundaries without scan organization.
The new gate removes exactly eight frame-384 labels, including all six supplied
false positives. The other 127 curb labels remain unchanged.

## Compact pole support and isolation

Frame 1137's true near shaft was already a fine candidate. Its 199 core returns
were outnumbered in proportion by the much larger 0.75 m neighborhood: the raw
core fraction was 199/300, below 0.8. Correcting for the area of the 0.25 m core
and surrounding annulus gives a density contrast of approximately 15.76.
A dense-core alternative now requires at least 100 core points, a density
contrast of 10, tilt at most one degree, and axis RMS at most 0.06 m. It does
not globally lower the ordinary isolation requirement.

Short candidates with at most 2 m of qualified fine support require a contiguous
qualified support run of at least 1.5 m and tilt at most six degrees. Disjoint
support cannot simply be summed. The six-degree limit retains the previously
accepted frame-425 shaft (approximately 5.53 degrees), which an initial
three-degree prototype incorrectly removed. Existing point-run continuity and
wide-surface rejection remain active. Frame 1137 gains 195 near-shaft labels,
removes 59 labels on the two reported false objects, and changes from 98 to 234
pole points. All 28 positive samples and all 11 negative samples have the desired
classification; previously rejected high fragment 54471 remains rejected.

## Independently recovering a split shaft

The frame-384 shaft near XY [-12.8, -3.8] m never entered point validation.
Returns crossed neighboring XY pillars and included higher surrounding returns.
Its individual candidate seeds failed despite compatible whole-pillar axes.

The fine stage can now propose a missing component from at least two neighboring
compatible whole-pillar axes. Each seed requires at least 12 points, 3 m height,
tilt at most three degrees and radial RMS at most 0.10 m. Neighbor completion
uses a 0.25 m axis RMS bound. Components touching existing candidates are
excluded from this additional recovery path, preserving their validation.

New components still pass the complete fine validator. Their final labels also
require actual vertical runs, at least 3 m of output extent, axis tilt at most
three degrees, axis RMS at most 0.12 m, and a core fraction of at least 0.95.
The stronger final isolation requirement is specific to independent recovery.
Earlier wider proposals added many labels on unannotated frames and were not
adopted. Upper fragments without enough output height are discarded.

Frame 384 gains 58 labels near the supplied shaft, spanning Z approximately
[-1.6224, 1.5304] m. All eleven supplied positive indices are selected. The
33 existing pole labels remain, giving 91 in total. The upper twelve-point
fragment produced by a wider prototype is excluded. Curb and sign output are
unchanged by this pole recovery.

## Validation scope

Recorded comparisons use Mississippi and downTown frames. Label differences
measure regression effects, not precision or recall: complete ground truth is
unavailable. The tests cover reported positives and negatives, geometry-based
ablation, preservation of other channels and whole-pillar probabilities, and
permuted unorganized input. Original reference masks remain immutable; explicit
revision fixtures record intended behavior changes separately.

The initial 54-frame comparison against commit
`3361dfaf16685d62169c90cb7fe74b540e7542d2` changes pole labels from
9007 to 8712 (+195/-490); 44 frames are unchanged. Frame 900 removes 284
labels on a tilted candidate and smaller structures; this unannotated change
remains an accuracy-review limitation. The curb gate changes 5803 to 5784
labels (+14/-33), with 52 of 54 frames unchanged.
The additional independent-pole recovery comparison covers those frames plus
384: only frame 384 changes (+58), and the other 54 pole masks are identical.
Other feature masks and the coarse probability products are unchanged by
each applicable comparison.

Final validation records 125 passing tests, zero failures/incomplete tests, and
zero MATLAB Code Analyzer findings across 15 files. Results combine retained
unchanged curb checks with rerun affected pole and integration suites and the
current-reference suite. Two historical gate-isolation ablations required
disabling newly independent paths in their baseline configurations; their
original assertions remain and the corrected tests pass. The native frame-384
preview asserts 127 curb, 91 pole and 16 sign labels, all eleven supplied pole
positives, zero supplied curb false positives, and unchanged camera properties.
