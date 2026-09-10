# Fine curb output validation and measured continuation

Date: 2026-09-10. Baseline commit: `efda5337918078f0a34a1b13b770a8b0af35ad20`.

The user identified Mississippi frame-91 points 19904, 18559 and 17727 as
false-positive curbs, later identified 22080 and 18560 on the same short
false boundary, and supplied 110 vicinity points along a right-side
curb extending beyond the previously detected short segment. Vicinity annotations
describe a spatial boundary, not a requirement to classify each listed return.

## Point validation

The three false positives have neighborhood plane-fit RMS residuals 0.019948,
0.019005 and 0.019627 m. They passed the existing 0.018 m seed-support test.
The final sampled output now requires at least 0.020 m RMS departure from its
local plane. The weaker support threshold remains available to fit boundaries;
filtering after metric sampling avoids moving or replacing established points.
The later two false positives survive the point threshold (RMS 0.026109 and
0.020399 m). Their boundary retains only three output points in two spatial
cells. A final spatial-support gate rejects an entire boundary when too few
independent 0.30 m output cells survive (at least three are required), so isolated remnants of a rejected
hypothesis cannot seed continuation.
A trial that globally raised the seed threshold disturbed boundary fits and
was not retained. There is one current detector, with distinct support and
output criteria; no frame, index, ring or coordinate exception.

## Missed-boundary diagnosis and continuation

All 110 vicinity returns passed ground segmentation, but only the first 15
entered the fine candidate set inherited from coarse curb pillars. Direct local
geometry evaluation of the omitted region showed strong step support; the
candidate restriction was suppressing fine recall.

`extendCurbBoundaries` searches unexamined ground returns beyond endpoints of
already validated boundaries. A local two-meter tail estimates an outward XY
tangent. The bounded search extends up to 12 m within a one-meter transverse
half-width. These are search bounds, not labeling bands. The existing fine
geometry validator applies the same height, detrended surface, consensus,
0.06 m output half-band and 0.15 m tangent-sampling rules to measured returns.
A recovered boundary must connect within 0.75 m in XYZ and align within 30
degrees of the endpoint tangent. Overlapping continuation outputs are spaced
against previously admitted extensions. No recursive or unsupported line
extrapolation labels points. Original primary outputs are preserved except for
the independent plane-residual rejection; evaluated extension points are
included in fine decision metadata. Coarse processing remains whole pillars,
with no height subdivision or additional raw-point pass.

## Results and limits

Across the 45 frozen thin-curb reference frames plus Mississippi 91:

- 41 of 46 full fine masks are unchanged. All coarse probability clouds and
  non-curb fine masks are exactly unchanged on every frame.
- Original curb points: 4353; retained: 4346; removed: seven; added: 100;
  final: 4446. Five frame-91 removals are user-labeled false positives.
  Frame-91 point 18496 on the same rejected fragment and Downtown 350
  point 25127 have not been independently labeled.
- Mississippi 91 changes 73 to 132: six points on the weak fragment are
  removed, including all five reported false positives; 65 measured points
  are added. Of these, 56 continue the annotated
  right boundary to approximately X=1.23 m, Y=-8.95 m. Nine extend another
  boundary. Pole 225 and traffic-sign 126 are unchanged.
- Mississippi 425 retains exactly its approved 133 curb points, including its
  existing thin boundary, optional accepted picks and false-positive exclusions.
- Other additions: Mississippi 350 +18, Mississippi 1000 +12, Downtown 250 +5.
  These are algorithm effects without independent point labels.
- 109 of the 110 supplied vicinity points lie within 0.20 m XY of a selected
  curb; median nearest distance is 0.0500 m. The remaining point is about
  0.805 m away. This is a vicinity-coverage diagnostic, not precision or recall.
  The detector does not force all annotations, apply ring quotas, or fabricate
  points. Tight bends, larger gaps and boundaries without an initial supported
  segment can remain undetected.

The live niri frame-91 preview was updated with original-point RGB colors,
original indices, frame counter and exact current camera preserved. Rotation
menu behavior is unchanged. No full-route video or mapping run was repeated.

## Validation and evidence

The initial per-point output gate passed the full suite (331 passed, zero failed,
two filtered synthesis tests for unavailable YALMIP/SDP dependencies). The
continuation was then added in response to the user's new vicinity annotation.
After all changes, 78 affected tests passed with zero failures and no filters;
factory Code Analyzer found zero issues across six changed/new MATLAB files.
All 45 baseline curb masks also match the frozen pre-change fixture exactly.
The final affected-suite results and analyzer report are exported alongside
46-frame comparisons under `research/results/curb_review_20260910/`.
Tests cover connected synthetic continuation, rejection of disconnected
segments, original-point membership, frame-91 vicinity coverage, preservation
of existing points, and flattened/permuted point clouds (seed 9135826).
Existing synthetic slope, tall-step, thinness, curvature and frame-425 checks
remain in the same suite. The original thin-curb fixture is preserved; a
separate explicit delta fixture records the current correction.

Initial temporary scripts wrote into /tmp because MATLAB run changed the
working directory; the comparison artifacts were relocated and scripts fixed.
An initial test invocation found zero tests and was discarded; the actual full
and final runs assert suite size. A diagnostic pdist2 call required an absent
Statistics Toolbox and was replaced with base-MATLAB pairwise XY distances.
Neither issue changed the production algorithm. Diagnostic artifacts remain
under `output/curb_plane_20260910/`; raw recordings and generated binaries are
excluded from Git.
