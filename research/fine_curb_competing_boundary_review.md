# Competing curb boundary review: Mississippi frame 276

The user supplied 78 locations near the true left curb and 20 explicit false
positive indices on a higher, outward boundary. These are spatial-vicinity
annotations: not every supplied positive index must become a feature point.
Production inference uses unorganized XYZ, without frame, index, row or ring
exceptions.

## Diagnosis

All 78 vicinity points entered the fine curb candidate stage. Of these, 67
passed local geometry and 45 were seeds, but none received a final curb label.
The initial boundary model mixed the far true curb with a higher outward edge
in the near field. Greedy model selection suppressed the nearby true boundary.
Several true candidates had terrain-corrected transverse gradients around
0.31--0.34 and plane residuals around 0.04 m. The competing-edge test required
40% greater gradient strength on the lower edge, so the available replacement
evidence did not trigger a correction. All 20 reported false points remained
selected in the baseline (118 curb points).

## Implementation

Strong competing-edge evidence keeps the existing reconstruction path, which
can recover boundaries from ambiguous initial models. An additional correction
uses a strictly stronger lower edge beside an already geometrically supported
boundary. The additional path retains the same separation, height difference,
parallel-normal, point-support and independent XY-cell requirements. It only
activates when the conflict has sufficient spatial extent.

The weaker strength advantage is not sufficient to replace an unsupported
model. The original candidate boundary must first pass normal agreement and
terrain-gradient reversal checks. Retracing then uses candidates within the
existing duplicate band around the affected boundary, preserves other models,
and subjects replacement output to the ordinary boundary checks. This avoids
resampling the opposite roadside merely because the left boundary is wrong.

A prototype that globally relaxed competition, and another that discarded
ambiguous models before strong-evidence reconstruction, lost previously
reviewed frame-538 curb extensions. Those prototypes were rejected. The final
implementation retains the strong-evidence reconstruction and confines the
additional correction to a supported local conflict.

## Frame 276 result

The final output contains 144 curb points, compared with 118 before the change.
All 20 supplied false positives are rejected. Twenty-three of the 78 vicinity
indices themselves are selected; every vicinity sample is within 0.11613 m XY
of a selected curb return. This measures spatial coverage, not annotation
recall. The right curb mask is exactly unchanged. Pole and traffic-sign labels,
and the coarse whole-pillar probability product, are also unchanged.

The new recorded test checks the complete set of negative samples, coverage of
all 78 vicinity locations, exact preservation of the right curb and other
channels, and a shuffled unorganized input using seed 27625329. A synthetic
case checks that a modestly stronger lower step is usable while equally strong
or equal-height edges remain protected by the existing tests.

## Validation and limits

All 103 tests passed: 35 curb geometry tests and 68 current-reference,
feature-selection, pipeline and whole-pillar tests. Code Analyzer reported no
findings in the five changed MATLAB files. The existing gradient-reversal
ablation now holds the alternative competition ratio at 1.4 to isolate that
mechanism; its original assertions are unchanged.

A 57-frame comparison against commit
`605315685bfe4cd1c1e7522b476408c24e962429` preserves 55 curb masks exactly.
Frame 276 adds 65 and removes 39 curb labels (118 to 144). Mississippi frame
1000 adds 42 and removes 62 (135 to 115); this unannotated change remains an
accuracy-review limitation. The combined curb count changes from 6103 to 6109.
All other feature masks and coarse probability products are identical across
the comparison. Previously reviewed frame-538 extensions and frames 832, 963
and 1137 retain their baseline results.

The native niri preview shows frame 276 with the same camera, original-point
coloring, frame counter and datatips. These checks do not establish full-route
accuracy or real-time performance.
