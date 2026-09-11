# Residual curb competition after reconstruction

## Mississippi frame 615 evidence

The initial output contained 132 curb labels. The user identified 14 false
positives: 27884, 27692, 26413, 26285, 26542, 26799, 26671, 26543,
26864, 26544, 25393, 25265, 25073 and 24945. These points formed one
complete reconstructed ridge component, spanning approximately 3.2 m in X.
The other two components contained 55 left-curb points and 63 right-curb
points. All three passed the existing boundary-normal and gradient-direction
checks.

The false component survived because strong competing-edge reconstruction
and the modest-advantage local competition path were mutually exclusive.
After strong reconstruction, 13 of its 14 points still had supported lower
competitors under the existing 1.0 gradient ratio; none of the other 118
selected points had that evidence. The 1.4 strong-evidence ratio rejected none
of the 14 residual false points.

## Algorithm decision

The strong reconstruction path now also checks the resulting valid components
for residual competition. A component is removed only when at least 60% of its
points favor a supported lower competitor and those conflicting points meet
the existing independent XY-cell and boundary-length requirements. The lower
competitor must still satisfy the existing separation, height difference,
gradient strength, parallel normal and local support conditions.

A reconstructed component is already a measured ridge chain. This check
therefore removes a supported competing component without running another
trace. Initial fitted models that did not undergo strong reconstruction retain
the previous local retracing behavior. The change affects only fine perception;
coarse whole-pillar products and unorganized XYZ inference remain unchanged.

## Rejected prototype and follow-up

A prototype ran local retracing after strong reconstruction. It removed the
14 initial false positives but added 18 labels, producing 136 curb points.
The user identified four of those additions as false positives: 23354, 23419,
23484 and 21885. This showed that retracing could admit an unrelated branch
while correcting the competing component. The final component-rejection
approach replaces that prototype; no alternative implementation is retained
in production.

The final frame-615 result contains 118 curb labels: exactly the original
mask minus the 14 initial false positives, with no additions. The four
follow-up false points are also absent. The right curb, 212 pole labels,
zero traffic-sign labels and coarse probability product remain unchanged.
The native niri preview preserves the camera, original-point coloring,
frame counter and datatips.

The recorded test checks all 18 user negatives, exact retention of the other
original curb labels, unchanged non-curb products, and unorganized input
permutation with seed 61527884. Its ablation holds the alternative ratio at
1.4 to reproduce the residual component without changing strong reconstruction.

## Validation and limits

All 104 final tests passed: 36 curb geometry tests and 68 current-reference,
feature-selection, pipeline and whole-pillar tests. Code Analyzer reported
zero findings in the four changed MATLAB files. The old frame-370 output
expectation initially failed; a separate explicit output-delta fixture records
the change, and its focused exact-reference rerun passed. This fixture records
algorithm output, not ground-truth annotation. Original fixtures are unchanged.

The 58-frame comparison against commit
`01ad27b640cf0c8fc29982a4bb533b79b802a8f3` preserves 56 curb masks exactly.
Besides the 14 removals in frame 615, Mississippi frame 370 loses 76 labels
(255 to 179) on a higher boundary spanning approximately X 0--10.1 m,
Y 6.67--6.95 m and Z -1.99-- -1.73 m. These points have not been annotated
by the user, so their removal remains an accuracy-review limitation. No
frame gains curb labels. The aggregate curb count changes from 6241 to 6151.
All non-curb masks and coarse probability products remain identical across
all 58 comparisons, including previously accepted frames.

The discarded retracing prototype and its initial checks are retained only as
diagnostic output evidence. No full-route accuracy or real-time performance
claim follows from these checks.
