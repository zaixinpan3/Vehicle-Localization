# Trace measured curb ridges through bends and endpoints

Date: 2026-09-10. Baseline commit:
`b1cc2ddd404dbef74516dc9af48d2a22c8bd4404`.

## Follow-up finding

Two follow-up reviews of Mississippi frame 832 supplied 28 vicinity points
on a misplaced central segment, another 28 on the missed near end, and 13
additional explicit false positives, including 39295. Together with the
original ten, all 23 explicit false positives are retained in
`tests/reference/curbBoundaryVicinity832.json`.

The preceding 0.20 m nearest-point check was too permissive: a displaced
boundary could count as recovered curb. The long proposal still connected
outer returns, and a second fitted boundary did not consistently follow the
actual bend or near end. Earlier proximity measurements remain actual
measurements; they did not establish correct boundary placement.

A first follow-up constrained global proposals to local gradient normals and
filtered sparse output runs. It corrected the central segment but failed the
original vicinity regression (89/90 tests passed), consistent with the user's
subsequent near-end report. That attempt is not a production mode. A global
curve is no longer used to resolve an established competing-edge conflict.

## Current implementation

Retain ordinary boundary proposals and the existing trigger requiring
initially selected weak raised points in at least three cells spanning 1.5 m.
Only when that trigger fires, remove dominated candidates and call
`traceCurbRidges` using original XYZ returns and their existing local scores:

1. Keep seed-qualified points passing the 0.020 m output plane-residual gate.
   Process decreasing score, with canonical XYZ tie ordering. Suppress weaker
   same-facing responses within a 0.30 m cross-face half-width and half the
   0.15 m output sampling interval along the local tangent. This finds the
   local middle-height response without fitting one curve through the frame.
2. Connect retained points only within 0.75 m XY. Displacement must project
   by less than 0.10 m on **both** local height-gradient normals; the normals
   must agree within 40 degrees. These mutual tangent tests reject a jump
   across the curb, even when points are close in Euclidean distance.
3. Require Z differences to fit the existing terrain-slope allowance plus
   minimum curb relief. Retain only connected components occupying at least
   six independent 0.30 m cells and spanning at least 1.5 m XY.

The ridge returns themselves become output features; no synthesized curve
points are introduced. The obsolete protected-candidate output and its
special duplicate-band configuration were removed. The initial selector is
still used to identify supported conflicts; there is no retained alternative
legacy refit. No ring, scan order, original index, frame number or fixed scene
coordinates enter the detector. The added search remains fine-only; coarse
perception still processes whole pillars.

## Geometry and regression

For the combined 56 follow-up references, measure distance to line segments
between neighboring actual detected points, ignoring gaps over 1.0 m:

| Metric | Baseline | Updated |
|---|---:|---:|
| Frame-832 curb points | 111 | 112 |
| Follow-up samples selected exactly | 6/56 | 16/56 |
| Median XY distance to measured boundary segments | 0.0403 m | 0.0082 m |
| Maximum XY distance to measured boundary segments | 0.1454 m | 0.0546 m |
| Maximum XY distance to a detected point | 0.1987 m | 0.0990 m |
| Explicit false positives retained, both follow-ups | 12/13 | 0/13 |

Point 39295 was introduced by the intermediate global-normal refit and
reported during that unfinished attempt; it was absent in the committed
baseline and is absent in the final result. The original ten explicit false
positives also remain absent. Vicinity samples
are not mandatory per-point labels: thin sampling selects a subset of a curb
face. These measurements do not replace visual review or exhaustive labeling.

Forty-eight frames were frozen before this coherent follow-up:

| Mississippi frame | Before | After | Removed | Added |
|---|---:|---:|---:|---:|
| 350 | 150 | 129 | 83 | 62 |
| 650 | 121 | 149 | 40 | 68 |
| 775 | 105 | 102 | 61 | 58 |
| 832 | 111 | 112 | 61 | 62 |

All other 44 full feature masks remain identical, including 91, 260, 425 and
687. All 48 non-curb masks and coarse probability clouds are identical. Total
curb count changes 4700 to 4705; 4455 are retained, 245 removed and 250 added.
Changes on 350, 650 and 775 are unlabeled algorithm effects, not independently
confirmed improvements. Point-count retention is not accuracy.

The native niri viewer reports 112 curb, 216 pole and 101 sign points with
unchanged current camera, original-point coloring, frame counter and datatips.
The annotation regressions require all 23 false positives to be absent, both
follow-up groups' maximum boundary error below 0.06 m, median below 0.02 m,
and maximum nearest-point distance below 0.10 m. The original 75-point
vicinity check remains active. Synthetic tests cover a curved ridge with
sparse parallel clutter and deterministic input permutation (seed 83239295),
alongside the frame-832 permutation test (seed 83234801) and earlier curb tests.

Raw masks, traces, native figures and checks remain under
`output/curb832_boundary_20260910/`; compact exports are under
`research/results/curb832_boundary_20260910/`. No full-route video/mapping
rerun or new runtime/general-accuracy qualification was performed. Sparse
sampling, ambiguous terrain gradients and multi-level surfaces remain limits
of the local geometric heuristic.

Final validation: 92 affected MATLAB tests passed, zero failed or incomplete.
Factory Code Analyzer reported zero findings in five changed/new MATLAB files.
