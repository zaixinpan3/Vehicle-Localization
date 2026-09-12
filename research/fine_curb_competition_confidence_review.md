# Fine curb competition confidence and anchored continuation

## Observation and diagnosis

Mississippi frame 600 had 30 curb labels while all 76 distinct user-indicated right-curb vicinity points were more than 0.20 m from a detected curb return. Their median nearest XY distance was 0.41174 m, with a maximum of 0.69734 m. These annotations identify the space near the curb rather than requiring every selected return to be labeled.

Runtime tracing of `refineCurbGeometry` found 139 valid local candidates in the broad target region. In a tighter comparison band, the lower edge near y=-3.4 m had median midpoint confidence 0.884781 and gradient magnitude 0.250242; the raised parallel edge near y=-3.75 m had confidence 0.686648 and gradient magnitude 0.267792. Gradient magnitude alone preferred the raised response. Only four selected points had stronger lower competitors under the original comparison, over about 0.4 m, which did not establish the required 1.5 m conflict extent.

## Implementation

`rejectWeakerRaisedCurbEdges` optionally scores competing strength as the terrain-corrected gradient magnitude multiplied by squared midpoint confidence. Normals still derive from the original gradient. `refineCurbGeometry` enables this confidence term only when comparing alternatives beside a validated boundary; the initial strong-conflict reconstruction uses its existing gradient-only comparison. A lower competitor must still have a height difference, transverse separation, compatible normal, multiple points and independent occupied XY cells.

A first prototype applied confidence in both comparisons. Its 62-frame results are preserved as `all_competitors_prototype.*`; the final implementation confines confidence to the validated-alternative comparison.

The corrected primary boundary supplied an anchored guided continuation, but six measured continuation returns occupied only five proposal cells. They were rejected by the six-cell discovery threshold despite overlapping the existing boundary in 13 returns spanning 1.945 m. `extendCurbBoundaries` now applies the existing three-cell output-support threshold after that anchored overlap is established. The 1.5 m continuation length, 0.75 m maximum consecutive 3D gap, tangent agreement and overlap gates remain. The measured chain stops at a 1.270 m gap rather than bridging it.

All inference remains based on unorganized metric XYZ. Point indices appear only in annotations and regression checks. Coarse whole-pillar processing, pole and traffic-sign inference are unaffected by the implementation.

## Frame result and verification artifacts

The revised frame has 27 curb, 64 pole and five traffic-sign labels. Of 76 annotated vicinity positions, 64 now have a detected curb return within 0.20 m, compared with zero before. Median nearest XY distance is 0.063687 m; maximum distance is 0.88330 m, so the more distant end still has misses. This is vicinity coverage, not a labeled-data precision or recall estimate.

The native niri MATLAB view was refreshed with unchanged camera properties, original-point coloring, datatips and frame counter. The export is `output/curb600_review_20260911/frame_600.png`. Comparison outputs, metrics, test results and diagnostic exports are kept in the same output directory. The existing 1170-frame video has not been regenerated for this change.

The intermediate result restored the inner boundary but retained an outer component. The user identified 47 points in this outer false-positive vicinity. Increasing the confidence exponent from 0.5 to 2 removes the outer component while retaining all 27 inner detections and the 64/76 vicinity coverage. All 47 newly indicated points are unlabelled in the final result. Intermediate comparisons and tests are preserved under `before_outer_edge_feedback/`.

The final 62-frame comparison changes 6528 to 6519 curb labels: 107 removals, 98 additions and 53 identical masks. Frame 600 accounts for 30 removals and 27 additions. The remaining 77 removals and 71 additions are unannotated differences, not demonstrated accuracy improvements. The largest other change is Mississippi 1125 (109 to 100 labels; 49 removed and 40 added), and frame 250 loses 19 labels. These changes limit conclusions about preservation of the baseline: output identity is verified on 53 frames, but annotation-based accuracy is not available for all changed regions. Non-curb masks and coarse probability products remain exactly identical in all 62 comparisons. Frame 28 retains its preceding correction.

Final validation has 114 passing tests, zero failures or incomplete tests, and zero Code Analyzer findings across eight changed MATLAB files. The geometry group contains 46 tests and the remaining reference, feature-selection, pipeline and whole-pillar groups contain 68. Three earlier ablation scenarios (frames 276, 384 and 963) initially failed because confidence comparison affected their intentionally exposed false-positive baselines. Their isolated configurations disable confidence weighting while preserving the original expected outputs. Default results remain covered independently by the 62-frame comparison, frame-600 annotation test and current-reference suite.
