# Mississippi frame 538: accepted short curb extent

Date: 2026-09-10. Algorithm revision:
`21f402e30358bc373074f52e1c98b2ef78b4b7f5`.

The user reported no visible false detections in the current frame and provided
28 left-side and six right-side vicinity samples beyond the detected curb
ends. These are indications of nearby curb space, not a requirement to label
every supplied return. The user explicitly accepts shorter curb detection.
The decision is to retain the current algorithm and parameters for this frame.

The current output contains 131 curb, 52 pole and 56 traffic-sign points.
Its two curb boundaries contain 71 left-side and 60 right-side points, reaching
approximately x=13.052 m and x=12.241 m respectively. The supplied left vicinity
reaches x=18.408 m and the right vicinity x=17.347 m. These X coordinates are
sensor-frame extents, not fitted arc lengths.

All 34 supplied returns are ground points already evaluated as curb candidates.
None is selected. Four pass local geometry as seeds; 11 fail the minimum
strip-neighbor requirement, 15 fail the strip-relief gate and four fail a
slope gate. All have plane residuals above the 0.020 m final output threshold,
so that threshold alone does not explain the omission. The strip is defined
in local metric XY geometry; this diagnosis uses no ring-dependent inference.

The current neighborhood radius is 0.65 m, strip half-width 0.12 m, minimum
strip population four points, acceptable detrended strip relief 0.07--0.30 m
and minimum gradient magnitude 0.08. Failed slope gates occur either before
or after estimated terrain-grade correction. Relief-gate diagnostics here
combine the lower and upper bound; no claim is made about which bound failed.
The diagnostic preserves the gate order of `refineCurbGeometry` and records
the first failed condition. The four passing samples alone do not establish
a sufficiently supported, thin continuation. `extendCurbBoundaries` adds zero
points in this frame, and only searches previously unevaluated ground returns;
it does not reevaluate these rejected candidates with weaker thresholds.

This finding rules out a simple maximum-range crop for the supplied samples.
It does not establish that their neighborhoods contain no recoverable curb, or
that extending the detector is impossible. Widening the strip or relaxing
geometry would require a new cross-frame precision review. Given the user's
stated preference, no threshold sweep or production change is made now.

Validation used a fresh offline `perceiveFrame` call on
`data/raw/MissisipiPointClouds.mat`, frame 538, and a temporary diagnostic copy
of the current local geometry evaluator that reports each first failing gate.
The diagnostic copy is retained only as a technical archive export, not as an
alternative production algorithm. Local artifacts are
`output/curb538_review_20260910/{review.csv,metrics.json,diagnostic.mat}`;
`review.csv` records all supplied indices, XYZ coordinates and gate outcomes.
The native niri preview remains on frame 538 with the prior camera preserved.
No dataset-wide precision measurement, video or mapping rerun is claimed.
