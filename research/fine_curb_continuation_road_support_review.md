# Road-support validation after curb continuation

The accepted baseline is commit `73161a137c5a5e0f57a4c30de228d1ca7c94a251`.
In Mississippi frame 458, 19 user-identified false curb points belonged to
one 25-point raised continuation. The baseline contained 208 curb labels,
102 pole labels and 11 traffic-sign labels.

## Diagnosis and implementation

`extendCurbBoundaries` reconstructs boundaries independently inside endpoint
search windows. Road support was validated inside each reconstruction, but
there was no comparison of the completed continuation against all primary
and other continued boundaries. A competitor outside that window could not
reject a raised branch. The erroneous reconstructed model had a median
lower-surface fit residual of 0.03011 m. After selection and spacing, its
25 emitted points had residual 0.0346 m, below the 0.035 m absolute limit.
The valid neighboring boundary models had residuals of approximately
0.0018--0.0073 m.

`refinePerceptionCandidates` now applies the existing `validateCurbRoadSupport`
check to the combined primary and continued boundaries before point labels
are finalized. The same measured lower-surface residual, parallel direction,
relative height, spatial witnesses and support-cell requirements apply.
The point mask, retained boundary models and continuation indices all derive
from that validated set. Existing thresholds are unchanged. Detection uses
unorganized XYZ; annotation indices are used only for validation. Coarse
whole-pillar processing and other feature channels are unchanged.

## Target-frame outcome

All 19 reported false labels are removed together with six other points of
the same geometric boundary. The other 183 curb labels are retained exactly,
with no new curb labels. The six additional removals were not individually
annotated. This result does not establish exhaustive ground truth for the
frame. Pole and traffic-sign masks remain unchanged.

The recorded-frame test also checks boundary/mask consistency and shuffled
unorganized input with random seed 45836260. Earlier road-support synthetic
tests exercise rough lower surfaces, rough elevated terrain, competing
boundaries, inconclusive support, and horizontal rigid transformations.

## Artifacts

- `output/curb458_review_20260912/frame458.mat`: source frame, configuration,
  baseline and updated results, and reported false-positive indices.
- `tests/reference/curbFalsePositive458.json`: user annotations, excluded from
  inference.
- `output/curb458_review_20260912/comparison.mat` and `.json`: comparison with
  the accepted baseline on prior frames plus frame 458.

The existing full-route video represents its original algorithm version;
this correction does not regenerate the video or mapping outputs.

## Baseline comparison

Across 64 recorded frames, 61 curb masks are identical. Total curb labels
change from 6,766 to 6,709, with 57 removals and zero additions. Apart from
frame 458, Mississippi frame 350 changes 188 to 173 and frame 1125 changes
100 to 83. Those 32 other-frame removals have no user annotations and are
not claimed as accuracy improvements. All non-curb masks and coarse
probability products are exactly unchanged. Previously reviewed frames,
including 943 and 600, are unchanged. Historical reference fixtures remain
intact; a separate revision fixture records the changed expectations.

## Validation and recording preparation

All 122 geometry, current-reference, feature-selection, pipeline and
whole-pillar tests pass with zero failures or incomplete tests. The first
harness invocation lacked the repository root on the explicit MATLAB path
when tests changed directories; 74 tests could not complete setup. Adding
that path fixed the harness without changing test expectations. Initial
results are retained separately.

The requested video regeneration exposed software-interactive marker
subsampling despite a full scatter XData array. The recording runner now
uses a complete `print(..., '-RGBImage', '-r72')` export with an explicit
paper rectangle matched to the requested pixel dimensions, instead of
capturing the interactive framebuffer. A two-frame 1910 by 2086 export
passed camera, source-point-count, image-size and frame-count assertions;
its dense raster was visually inspected. This changes recording only,
not semantic decisions or marker size between feature classes.
