# Restore native point-centered perception viewing

The user reported an inability to rotate the pcshow scene around a point.
The user then identified double-click as the cause of returning to the default
view and explicitly excluded that behavior from the repair. Double-click
reset and the fixed-camera recording policy are not changed.

## Cause and repair

The project had replaced pcshow's native scatter tag `pcviewer` with
`PerceptionPointCloud`. Inspection of MATLAB R2026a's installed point-cloud
interaction implementation showed that point-center selection searches for
scatter objects tagged `pcviewer`. Renaming that tag therefore hid the
original points from its picker. The preview now leaves the native tag alone;
color updates and video recording identify the same native scatter.
The updater also synchronizes pcshow's transient PointCloud/ColorData caches
and user-specified color mode with the current XYZ and exact RGB data. Native
drag/zoom release can restore from these caches; leaving them stale would
restore old coordinates or height-based colors. These cache fields are
renderer-specific and are checked when a saved viewer is rebuilt.

The visible frame-91 figure had also been reopened from a saved FIG file.
Its Rotate 3D callback was a generic axes-rotation handler even though the
point-cloud context menu remained. Point-cloud scene state is transient, and
restoring a FIG does not reconstruct the complete live viewer interaction.

`restorePerceptionFigure(fig)` rebuilds the live point-cloud renderer through
the public pcshow entry point using the saved original XYZ and masks. It
restores the current camera, axes limits, marker size, colors, labels and
frame counter, reconnects original-index datatips, and enables native Rotate
3D. It does not add a custom orbit algorithm or override double-click reset.
The transient interaction state is supplied by the installed pcshow renderer.

To reopen a saved perception figure:

```matlab
setupVehicleLocalization;
fig = openfig('path/to/perception.fig');
restorePerceptionFigure(fig);
```

In Rotate 3D mode, right-click and choose **Rotate Around a Point**, then drag
from the point of interest. Choose **Rotate Around Axes Center** to switch
back. The current native frame-91 figure was placed in point-centered mode.
The toolbar's Data Tips mode still prints original indices and visible
semantic classes. These are native controls described in the
[MathWorks pcshow documentation](https://www.mathworks.com/help/vision/ref/pcshow.html).

## Executed validation

- The native frame-91 repair preserved exact XYZ, RGB, camera properties and
  uniform SizeData 8, while restoring the pcviewer tag and point-center mode.
- Public-interface tests cover the new preview's native tag and a saved FIG
  round trip preserving camera, colors, indices, frame counter and datatips.
  A subsequent frame update retains the native tag and current camera.
- In the native desktop, invoking the installed mouse-down callback selected
  `[4.9799876 -12.7090702 -2.2915421]` m as its pivot: distance to an actual
  source point was exactly zero, and it differed from the axes center.
  Exercising the installed orbit kernel moved the camera with zero measured
  change in distance to that pivot. The test restored the preceding camera
  and removed its temporary action callback afterward. This is callback/kernel
  validation; physical XTest mouse injection under XWayland was inconclusive.
- A native release-callback probe deliberately reduced the displayed cloud
  and verified that release restored the latest frame XYZ and exact semantic
  RGB values from the updated caches. The two viewer tests were rerun after
  the final cache synchronization, as was the changed updater's analyzer check.
- A headless 91--93 video/map pilot decoded three frames, retained one native
  pcviewer scatter and ended at `Frame 93 / 1170`. Headless graphics warned
  about software marker decimation, so native appearance was checked separately.
- The combined full suite passed 330 tests with zero failures and two synthesis
  dependency filters. All eight changed/new MATLAB files passed factory Code
  Analyzer. An initial test assertion compared an OnOffSwitchState enum with a
  character array; the assertion was corrected before the passing full run.

The inspection used installed R2026a source and runtime state. No MATLAB
installation files were modified. Renderer-specific interaction discovery
should be rechecked when changing MATLAB releases. The repair does not change
perception or mapping inference; the separately documented pole correction
was validated in the same working revision. No full-route recording was rerun.

Check exports are in `research/results/viewer_interaction_20260910/`; native
figures, callback evidence and the pilot remain in
`output/viewer_interaction_20260910/` and the identified archive export bundle.
