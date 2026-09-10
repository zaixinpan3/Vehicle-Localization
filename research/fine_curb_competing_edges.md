# Resolve competing fine curb boundaries on frame 832

Date: 2026-09-10. Baseline commit:
`92110b2f5344546e923038c7e223e4a1fde4b42e`.

## Report and diagnosis

The user supplied 75 distinct Mississippi frame-832 points describing the
vicinity of a true right curb, and ten explicit false curb labels. The
annotation is retained in `tests/reference/curbBoundaryVicinity832.json`.
Vicinity points are not mandatory pointwise positives: the desired output is
a thin boundary of original XYZ returns, independent of rings or scan order.

The true vicinity already passes local feature scoring: most sampled points
have detrended gradient magnitude near 0.30 and plane residual about 0.04 m.
The false points mostly have much weaker gradients (approximately 0.09--0.21)
and lie on a higher edge outside the true boundary. Spatial consensus can
connect a long, far curb to those outer returns. The existing 1.2 m duplicate
band then suppresses the nearby measured bend of the real curb. This is a
boundary-selection conflict, not simply a missing coarse candidate.

## Current implementation

Keep the existing point geometry, initial proposals, curve fit and thin
sampling. First obtain the ordinary fine boundaries. Compare qualified
candidate gradients using all valid fine candidates:

- Search within 1.5 m in XY; require at least 0.4 m cross-edge separation and
  less than 0.6 m along-edge displacement relative to the candidate gradient.
- A competing edge must be at least 0.07 m lower, have gradient magnitude
  greater than 1.4 times the candidate's, and have absolute normal alignment
  above 0.8. Require at least six returns in three independent 0.30 m XY cells.
- Reconsider the initial boundary selection only if already selected outputs
  fail this comparison in at least three cells spanning at least 1.5 m in XY.
  Weak unselected candidates alone cannot trigger a refit.
- Reject the dominated candidates and rerun the same boundary selector.
  Stronger lower-edge supporters remain eligible beyond a 0.25 m duplicate
  band, allowing measured bends to survive. Other candidates retain the
  existing 1.2 m duplicate band.

The same selector serves both passes; no previous algorithm is retained as a
legacy production path. The output half-band remains 0.06 m and along-boundary
spacing 0.15 m. No point indices, fixed scene coordinates, rings or forced
positive labels appear in the detector. Coarse processing remains whole
pillars; the additional search is fine-only.

## Validation

Forty-eight frames were frozen before editing: 38 Mississippi frames including
91, 260, 425, 687 and 832, plus ten Downtown frames. Comparing the final
implementation with that baseline gives:

| Dataset/frame | Before | After | Removed | Added |
|---|---:|---:|---:|---:|
| Mississippi 350 | 133 | 150 | 48 | 65 |
| Mississippi 650 | 139 | 121 | 39 | 21 |
| Mississippi 775 | 94 | 105 | 3 | 14 |
| Mississippi 832 | 122 | 111 | 59 | 48 |

All other 44 complete feature masks are unchanged, including 91, 260, 425 and
687. All 48 non-curb masks and coarse probability clouds are identical. Total
curb points change 4701 to 4700, with 4552 retained (96.83%), 149 removed and
148 added. The three other changed frames are algorithm effects without new
user labels; point retention is not accuracy.

All ten explicit false positives are rejected. All 75 distinct true-vicinity
references have a selected curb return within 0.20 m XY, with maximum nearest
XY distance 0.17673 m. This measures proximity to the supplied vicinity,
not exhaustive precision or recall. The native niri viewer reports 111 curb,
216 pole and 101 traffic-sign points, with camera unchanged, direct original
point colors, frame counter and point-index datatips retained.

Nineteen focused tests passed, covering competing-edge evidence, absence of
support, equal-height/equal-strength edges, frame-832 vicinity, input order
(seed 83234801), earlier curb annotations, planes, slopes, disconnected
fragments, close median edges, curvature and transforms. Additional regression
and analyzer outcomes are recorded in the accompanying result exports.

A globally narrowed duplicate band was rejected: it changed 39/48 frames and
made frame 425 grow from 133 to 256 points. Unconditionally protecting all
stronger candidates was also rejected: it changed 19/48 frames and added 22
points on frame 425. Those explorations are not production options. A temporary
trace initially sampled inside the point loop; it was moved after that loop
before deriving the reported diagnostics, and all breakpoints were cleared.

Results and changed indices are exported to
`research/results/curb832_20260910/`. Raw baseline masks, diagnostic traces,
validation outputs and native figure files remain in
`output/curb832_20260910/`. There was no full-route video/mapping rerun or new
accuracy or runtime qualification. Candidate competition assumes a stronger
nearby lower step is the relevant road boundary; this remains a heuristic for
complex multi-level surfaces and needs further scene review.

Final checks: 89 unique affected tests passed (19 focused curb tests and 70
reference/integration tests), with zero failures or incomplete tests. Factory
Code Analyzer reported zero findings in the four changed/new MATLAB files.
