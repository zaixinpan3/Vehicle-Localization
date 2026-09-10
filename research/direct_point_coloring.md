# Direct point-cloud coloring

Date: 2026-09-09. Baseline: `42cd9dbe4c3d86c830e7ef8324dd60eb7b48df5b`.

The visualization now represents every finite original point exactly once in
one `pcshow` scatter. Semantic RGB values replace its gray color in place;
feature points use the same marker and size as the source cloud. The extra
large semantic scatter overlays have been removed.

`updatePerceptionDisplay` is shared by the single-frame preview and video/map
entry point. It updates XYZ, RGB, original-point index mappings, legend counts
and the frame counter. Legend-only line proxies contain NaN coordinates, so
they add no finite point geometry. Overlapping masks retain their original
values; the last requested class supplies the one visible color.

`perceptionPointTip` reports the original point index, frame, XYZ, row/column
and visible semantic label in the MATLAB Command Window. Frame changes rebuild
this index mapping and clear stale tips. Row/column values are display metadata
only; no perception algorithm uses them. Camera and axes properties are fixed
before updating point data, preventing automatic axes scaling from changing
the chosen view. An initial validation exposed this automatic camera change;
the explicit camera preservation passes the repeated update check.

## Validation

- Native Mississippi frame 91: 65,536 finite input points and exactly 65,536
  plotted points in one scatter; uniform marker `.` with SizeData 8.
- RGB equals the selected semantic masks exactly, including unchanged gray
  background points. Legend proxies have no finite point coordinates.
- Native frame-91 camera and marker settings are retained. A frame-92 update
  preserves the camera, and its pole datatip reports the correct new frame and
  original point index.
- A frames-91--93 video/map pilot decodes three video frames, ends at
  `Frame 93 / 1170`, keeps one point-cloud scatter and preserves the complete
  registered mapping observations exactly relative to the preceding pilot.
- Factory Code Analyzer reports zero findings in all four changed/new MATLAB
  files. The headless pilot's software graphics can reduce rendered markers;
  native desktop visualization was inspected separately.

The detector, its parameters, feature masks, and mapping implementation did not
change. Existing full-sequence video files remain the recordings produced at
their original revision; this operation changes current preview and future
recordings. Compact checks are under
[`results/direct_point_coloring_20260909/`](results/direct_point_coloring_20260909/),
with native screenshots and pilot binaries under
`output/direct_point_colors_20260909/`.
