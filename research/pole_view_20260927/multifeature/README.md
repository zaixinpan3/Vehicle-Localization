# Three-feature inspection of Mississippi frame 500

Run `showMississippiFeaturePillars(500)` after `setupVehicleLocalization`.
The upper point cloud retains all 65,536 finite original returns at uniform
marker size 4. Orange denotes pole, magenta traffic sign, and cyan curb.
The lower 0.6 m grid uses the same colors for current coarse selections.
Two cells have multiple class memberships: color strips show each class
within the same cell footprint, without introducing finer detector cells.
Original points are never duplicated or enlarged. Overlapping reference
point labels would use white; frame 500 has no such overlap.

## Reference provenance

All three reference channels come directly from original 0.3 m offline fine
perception indices in `output/fine_matching_20260919/inputs_2.mat`. The viewer
checks frame membership, partition completion, 0.3 m configuration, unique
indices and stored counts. Pole indices also exactly equal the cache used by
the earlier pole-only viewer. There is no reference inference rerun or change
to any perception threshold. These stored algorithm outputs are the user's
comparison ground truth, not independently annotated physical labels.

| Class | Original reference points | Reference points inside coarse ROI | Coarse selected cells | Covered reference points in ROI | Cells with no class reference point |
| --- | ---: | ---: | ---: | ---: | ---: |
| Pole | 296 | 296 | 2 | 296 | 0 |
| Traffic sign | 507 | 495 | 12 | 495 | 0 |
| Curb | 64 | 64 | 81 | 62 | 54 |

The 12 traffic sign points outside the coarse ROI remain visible in the full
cloud and are reported separately from its coverage denominator. Curb
coverage is 62/64 (96.875%) and reference-empty selection is 54/81 (66.667%).
These are single-frame comparisons, not dataset-wide quality estimates.
No selections are hidden because they disagree with the reference.

The display grid is one meter below the fifth percentile of source Z within
its XY footprint, giving Z = -3.1327 m here. Low outliers no longer push the
grid tens of meters below the main cloud. All source points are retained,
including any below this display plane. This changes display placement only.

## Validation

`validateFeaturePillarView` verifies exact class counts, source XYZ, native
RGB restoration caches, uniform marker size and one source scatter. Rendered
cell IDs match current coarse membership for each class, including shared
cells. Six magnifier steps, the installed wheel callback, Feature region and
Full scene preserve source/color data, grid vertices and data limits. Factory
Code Analyzer results are included in `validation.json`. The live niri window
was also inspected; no physical mouse-injection test is claimed.

`features_0500.json` preserves original point indices, selected pillar IDs,
class colors and alignment counts. `features_0500.png` is an export of the
rendered view (UI buttons are omitted by MATLAB exportgraphics). Raw MAT
recordings and original large reference caches remain local and unmodified.
