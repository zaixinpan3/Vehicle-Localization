# Thin curb geometry from unorganized point clouds

Date: 2026-09-09. Baseline revision:
`d925e1c5b4579b64f5e70e8481b3fe0981cd1997`.

The fine detector now locates narrow curb boundaries using XYZ neighborhoods
and retains measured source returns. Mississippi frame 425 has **133 curb
points** (72 left, 61 right), compared with the original broad mask's 861.
The final masks exactly reproduce the niri MATLAB preview that the user
approved. Marking, pole, and sign counts remain 85, 21, and 240.

## Representation and algorithm

The online product remains whole XY pillars with their existing statistics.
Fine curb work runs only in the offline branch. The candidate search includes
one neighboring pillar around coarse curb cells, using ground returns only.
The obsolete point selection and dominant-boundary thinning functions and
unused configuration fields were removed. No alternative executable detector
or frame-specific exception is retained.

`refineCurbGeometry` takes XYZ, candidate point indices, a ground mask, and fine
configuration. It does not read rings, rows, azimuth, scan order, or organized
array dimensions. Indices identify output points; they do not influence scores.
Coordinate sorting makes point and neighborhood tie handling deterministic.

1. Query a 0.65 m XY neighborhood with at least eight returns. Require height
   variation, departure from a local plane (RMS at least 0.018 m), and slope.
2. Estimate terrain grade from supported side surfaces. Four orientations
   relative to the local height gradient and both sides are evaluated; the
   lowest-residual eligible plane supplies the grade. A surface needs six
   returns, RMS below 0.025 m, and slope magnitude at most 0.35. With no eligible
   surface the grade estimate is zero. Subtract the grade before testing curb
   height, and remove it from the local step direction.
3. In a 0.12 m half-width strip, require detrended 90th-minus-10th percentile
   relief of 0.07–0.30 m. Returns within 0.07 m of the strip's middle height may
   join a supported boundary. Boundary seeds additionally need a middle-height
   offset at most 0.06 m and full-neighborhood detrended relief at most 0.30 m.
   Thus a locally plausible point cannot establish a boundary by itself.
4. Generate deterministic line proposals from at most 128 spatial anchors.
   Count one support vote per 0.30 m XY cell; require six support cells and
   at least 1.5 m length. Split supports across gaps larger than 3 m. Fit a
   weighted line or constrained quadratic to a narrow consensus. Quadratics
   must improve squared residual by at least 20% and have coefficient magnitude
   at most 0.05 per meter in the local tangent coordinate. Their support extent
   may extend by 1 m, but only measured eligible returns can be selected.
5. Select returns in a 0.06 m transverse half-band and retain one per 0.15 m
   projected tangent interval. This is metric interval sampling, not a strict
   Euclidean minimum-separation guarantee. Suppress nearby repeated boundary
   hypotheses; opposite-facing median edges remain separate.

The synthetic grade test exposed the error in capping raw neighborhood Z
variation: grade can exceed that cap while the curb step remains valid.
Merely relaxing the middle-height band also displaced curved boundaries in
synthetic tests. The selected implementation uses compensated surface geometry
and separate boundary-seed and point-acceptance tests.

## User annotations and local result

The user supplied 127 unique left and 99 unique right vicinity anchors and
three explicit false positives. The anchors describe neighborhoods of true
curbs; they are not an exhaustive point segmentation or a requirement to label
every picked return. The final mask excludes all three false positives:
17536, 34467, and 34214.

The latest optional batch contains 22 points. These 11 are accepted:
`29031, 29352, 29224, 29160, 26033, 35492, 34405, 35113, 36719, 36847, 37360`.
The other 11 remain unclassified as curb. All 33 distinct optional picks across
the two batches, their source coordinates, final decisions, and nearest selected
curb distances are exported in `optional_points_425.csv`.

For frame 425, the left boundary represents 28 original rings and the right
represents 29; each represented ring has one to four selected points per side.
These are output-density measurements only. There is no detector quota, and
rings without sufficient supported geometry remain empty. Within the annotated
X extents, every selected point is within 0.15 m transverse distance of a
quadratic fit to the corresponding user vicinity anchors. Those anchor fits
are used only by the diagnostic test.

The desktop preview uses orange-red curb, cyan pole, yellow marking, magenta
traffic sign, and gray source points. Datatips retain the source-index mapping
and print original index, row, column, XYZ, and clicked layer in the desktop
Command Window. A `layer=source` pick can hit the gray underlay of an accepted
feature; the exported mask is the classification authority.

## Baseline comparison and limits

Forty-five frozen frames were evaluated against the baseline revision:

- Mississippi: 50, 75, 100, 150, 175, 200, 225, 250, 260, 300, 326, 350, 370,
  400, 425, 450, 475, 500, 525, 550, 600, 650, 700, 725, 775, 800, 825, 850,
  900, 950, 1000, 1050, 1100, 1125, 1150.
- Downtown: 75, 100, 150, 200, 250, 300, 350, 400, 450, 500.

Every coarse candidate structure and every non-curb fine mask was exactly
unchanged across all 45 frames. Curbs intentionally change in point count and
membership. The original baseline JSON/MAT files were not regenerated;
`tests/reference/fineCurbGeometry.json` separately records the reviewed new
outputs. It is a regression expectation, not labeled accuracy evidence.

On the 35 Mississippi frames, the median fraction of new curb points within
0.30 m XY of an old curb point is 97.7%. The median fraction of the old broad
mask within 1 m XY of a selected new point is 94.7%. Frame 425 has corresponding
fractions 97.0% and 90.7%. These are geometric proximity diagnostics, not
precision or recall.

Coverage is less consistent on short, sparse, or complex boundaries. On the
nine Downtown frames with a nonempty baseline, the median old-mask proximity
within 1 m is 61.7%. Downtown 250 changes 373 to 10 points and has 26.3% on that
measure; Mississippi 50 changes 381 to 27 and has 62.7%. Local candidate XYZ,
height, and seed views were inspected for these cases and Downtown 400. Some
baseline regions lack a supported narrow height step; some short supports do
not establish a boundary. Their correctness is not resolved by the available
labels, and they remain a coverage limitation rather than a claimed accuracy
improvement. No universal baseline-accuracy preservation claim is made.

The final 1.5 m minimum length retains a supported short segment on Downtown
400 (10 points) and exactly preserves every frame-425 mask in the approved
preview. Synthetic 1.2 m disconnected fragments remain rejected. Very tight
curves, nearby same-facing boundaries, low curbs, or locally ambiguous terrain
can still be missed or merged. No full mapping/localization trajectory replay,
independent labeled benchmark, or online timing qualification was performed.
Single-call offline times in the CSV are descriptive measurements only.

## Reproduction and validation artifacts

```matlab
setupVehicleLocalization;
frame = loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',425);
cfg = perceptionConfig();
cfg.executionMode = "offline";
result = perceiveFrame(frame,cfg);
assert(nnz(result.featureMasks.curb)==133);
assert(~any(result.featureMasks.curb([17536 34467 34214])));
runtests('tests/curbGeometryRefinementTest.m');
setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(pwd,'data'));
runtests('tests');
```

Eleven focused tests passed: original-return thinning; shuffled points and
candidates; flat/sloped planes; a curb on 25% additional longitudinal grade;
rejection of a tall step; opposite-facing median edges; rigid XY transforms;
disconnected fragments; curvature; complete frame flattening/permutation; and
recorded frame-425 vicinity, false-positive, and density checks. The point-order
tests use a local random stream with seed 425 and restore output by source
index; the detector itself uses no random sampling.

The full suite returned **338 passed, zero failed, and two assumption-filtered
synthesis tests** because YALMIP/SDP dependencies were unavailable. Factory
Code Analyzer found zero issues in all eight changed/new MATLAB files. An
explicit square-allocation cleanup and a loop-progress guard were followed by
another passing run of all 11 focused tests and exact reproduction of all 45
validated outputs. Diff whitespace checks passed.

CSV exports are under `research/results/curb_geometry_refinement_20260909/`.
Raw recorded datasets and generated MAT/figure files remain outside Git.
