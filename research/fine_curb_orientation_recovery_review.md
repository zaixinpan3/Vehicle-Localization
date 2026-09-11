# Orientation-guided fine curb revalidation

## Observed failure

The user marked 137 vicinity points on the missing left curb in Mississippi
frame 855, extending approximately from XY (15.4, 10.4) to (1.7, 15.2) m.
These are boundary vicinity annotations, not an exhaustive point-level truth
mask. The baseline is `d3699ac2ebe72ffe2ea84b6e375431165d0211ae`.
The user explicitly accepts some false positives and missed points, provided
there is no large overall degradation.

The baseline emits 65 curb points: 59 on the right and six on the distant
left section. The missing region contains ground points and geometric seeds.
Two initial left models with 14 and nine sampled points fail the boundary
normal consistency gate (fractions approximately 0.43 and 0.38). Their
isotropic neighborhoods do not reliably recover a transverse height gradient.
Sparse support at the surviving far-left endpoint also prevents the existing
guided extension from anchoring to enough previously accepted returns.

Disabling normal validation recovers only 14 region points. Changing the output
plane residual, competing-edge strength, gradient-reversal gate or coarse
candidate dilation does not recover the segment. Increasing the endpoint
support length alone does not resolve the failure. These diagnostic ablations
were not adopted as production changes.

## Implementation

Before discarding a spatial model for inconsistent normals, fine refinement
estimates its XY tangent with PCA and re-evaluates its nearby candidate points
using the existing elliptical, orientation-guided geometric neighborhoods.
The search is bounded by the existing one-meter half-width and curve endpoint
margin. Revalidation uses the same relief, surface slope, plane residual,
mid-height, boundary support, competition, normal consistency and thin-point
sampling requirements. Only validated original returns enter the final mask.

This additional pass applies only to ordinary models rejected by the normal
check. Guided and trace-only calls do not recursively trigger it. Existing
accepted models are preserved. No threshold changes, ring information, frame
numbers, original indices or scene coordinates enter production inference.
Coarse whole-pillar statistics and candidate probabilities are unchanged.

## Measurements and limitations

Frame 855 changes from 65 to 153 curb labels: 88 additions and no removals.
The user-marked region gains 86 labels. Of the 137 vicinity picks, 67 become
curb labels directly and 108 have a curb return within 0.35 m in XY.
Nearest-curb XY distance quantiles (median, 90th percentile, maximum) improve
from (9.1557, 14.1799, 15.5717) m to (0.0555, 0.7029, 1.9460) m.
The near end remains partly missed. Pole labels remain 23 and sign labels 60.

Across 61 recorded frames, 58 curb masks remain identical. Total curb labels
change from 6350 to 6512: 163 additions and one removal. The other changes are
Mississippi frame 384 (+34) and 850 (+41, -1). All six previously reported
frame-384 false-positive indices remain rejected. The 75 additional labels and
one removal outside frame 855 are unannotated accuracy-review limitations,
not independently verified precision or recall improvements. Every compared
non-curb mask and coarse probability product remains identical.

The new regression checks spatial vicinity coverage, preservation of original
frame-855 labels and exact invariance under shuffled unorganized XYZ input
(seed 85526534). Frame-384 rejection assertions are retained; its obsolete
no-additions expectation is replaced by the explicit revision manifest.
Final validation passes all 105 tests (37 curb geometry and 68 reference,
feature-selection, pipeline and whole-pillar tests). Code Analyzer reports zero
findings in all three modified MATLAB files. An initial test-runner path setup
failure was corrected and the affected suites rerun.
Original reference fixtures remain immutable. Measurements, comparison masks,
logs and the preserved-camera niri preview are stored under
`output/curb855_review_20260911/`.

The earlier 1170-frame video remains an artifact of its recorded algorithm
version; it has not been re-rendered for this correction.
