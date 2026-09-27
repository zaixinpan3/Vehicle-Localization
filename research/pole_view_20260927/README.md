# Inspect stored fine pole references and coarse pillar selections

Two interactive MATLAB viewers were opened on the niri desktop:

- `showMississippiPolePillars(900)` displays every finite original point,
  highlights stored fine pole points in orange, and draws the actual 0.6 m
  pillar grid below the cloud. Current coarse pole cells use the same orange.
- `showMississippiPoleReference(500)` displays the full source cloud with
  only the stored fine pole points highlighted. It reads the original cache
  directly and performs no perception inference or relabeling.

Both offer **Full scene** and **Pole region** buttons, native point-cloud
rotation, mouse-centered source-point zoom, and coordinate datatips. The
complete point set remains available after zooming. A zoomed viewport
naturally shows a smaller part of the scene; Full scene restores the view.

## Source and diagnostic interpretation

Input frames come from `data/raw/MissisipiPointClouds.mat`. Pole indices come
from `output/pole_precision_20260927/mississippi.mat`, whose references retain
the original offline 0.3 m fine detector output. Each plotted reference XYZ
is checked against its original source index. These are **algorithm outputs,
not manually annotated physical-object ground truth**.

Frame 900 contains 65,536 finite source points and 96 reference pole points.
The precision-first coarse configuration at commit
`0a24e509eae35a2d9c7ef5458b6cf9071d99204f` selects pillar 5744. Its footprint is
X in `[4.3,4.9)` m and Y in `[-4.1,-3.5)` m, so the inspected point
`[4.6174,-3.8088]` lies inside it. This pillar contains no stored reference
pole points, and none of the 96 reference points is covered in that frame.
This is an exact reference disagreement, not proof that the selected
physical structure is unsuitable as a pole feature. No reference labels or
perception parameters were changed during this visualization task.

Frame 500 is a separate reference-only inspection. Its JSON export retains
the exact original indices highlighted in the viewer. All finite source
points are drawn without ROI filtering or downsampling. Frame 500 contains
65,536 finite source points and 296 stored fine pole points.

Both viewers recolor the original source returns directly in a single
`pcshow` scatter. All returns use the same small marker size (4); feature
points are never duplicated or enlarged. A NaN-only legend proxy draws no
scene points. This replaces the earlier redundant orange overlay, which
obscured the underlying pole structure.

## Zoom defect and validation

The initial comparison used generic `scatter3` limit-based zoom. In a scene
containing a distant lowered grid and scattered source returns, magnification
could crop the point cloud out of the data limits. The viewer now enters
MATLAB's native `pcshow` path, which configures camera pan/zoom. The installed
R2026a `initializePCSceneControl` explicitly selects that mode for point clouds.
Its native `pcviewer` tag and XYZ/RGB restoration caches are preserved.

Magnifier pre-actions and inward wheel actions aim the camera at an original
return nearest the pointer ray. They translate camera position and target
together; they do not shrink data limits or delete points. The lowered grid
is five meters below the minimum source Z and keeps the exact production XY
origin, spacing and selected cells. Its vertical offset is for display only.

`validatePolePillarView` exercises six public magnifier zoom operations, the
installed wheel callback, source-point targeting, both view buttons and exact
home-camera recovery on frame 900. All checks pass: all 65,536 XYZ values and
their RGB colors remain unchanged, as do pillar vertices and data limits.
The camera view angle decreases from 11.2293 to 0.9890 degrees. A separate
eightfold source-centered close-up is retained in `zoomed_reference.png`.
The live niri window was also inspected in a zoomed state. This is callback
and rendered-window validation, not an automated physical mouse-injection test.

Validation also checks a single source scatter, uniform marker size and exact
highlight counts in both viewers. Factory MATLAB Code Analyzer reports no
findings for both viewers, the focus helper and interaction validator. The
reference-only viewer checks exact source/reference counts, source XYZ and
native RGB caches at runtime. The
static PNG exports omit UI buttons; the live windows contain them.

```matlab
setupVehicleLocalization;
comparison = showMississippiPolePillars(900);
reference = showMississippiPoleReference(500);
addpath('research/pole_view_20260927');
validatePolePillarView;
```

This task changes visualization only. Recorded datasets, cached references,
perception thresholds and semantic decisions remain untouched.
