# Remove road-marking perception

Date: 2026-09-09. Baseline: `8ec16499323291e20dbc917af15589f715490606`.

The user removed road markings from the perception task. Mississippi now
selects curb, pole and traffic sign; Downtown additionally selects facade.
The removed channel is rejected by the invocation validator, rather than
retained as an empty mask or disabled detector.

## Implementation

Removed the coarse marking mask/probability calculation, fine reflectivity
classification, ground reflectivity threshold and configuration, unused ground
scalar extraction, and marking-specific semantic-product branch. The unused
`resolveRoadReflectivityThreshold` and `sampleCellMapAtPoints` functions were
deleted. The shared road-surface and curb-adjacency calculations remain because
curb detection needs them. Whole-pillar XYZ statistics remain unchanged.

Mapping defaults and invocation scripts now use the retained classes.
Registration's line-feature classification and synthetic registration tests
use supported line classes. Diagnostic plotting obtains class labels from its
input tables instead of hard-coding the removed channel. Point-cloud colors
and video recording discard unrequested scatter layers, including a marking
layer in a previously saved figure.

Original regression evidence and previously generated videos/maps remain
historical outputs. The expected-mask adapter excludes the removed field but
does not rewrite any retained mask or immutable fixture. This operation did
not regenerate the earlier 1170-frame video or full-route map.

## Validation

- All 325 executed tests passed; zero failed. Two observer synthesis tests were
  filtered because YALMIP/SDP solver dependencies were unavailable.
- All 30 recorded point-mask cases in the current-reference suite retained
  their exact remaining masks. The removed-channel rejection and all 16
  subsets of the four supported channels are exercised.
- Additional before/after comparisons on Mississippi frames 91, 260 and 425
  preserve every retained fine/ground mask exactly. Their coarse component
  means, XYZ means, XY/XYZ covariances, counts, semantic/occupancy probabilities
  and unnormalized weights also match exactly per retained class. Semantic IDs
  shift and mixture weights renormalize when the marking class is removed.
- Factory Code Analyzer reports zero findings across 24 changed MATLAB files.
- The niri frame-91 preview contains source, curb, pole and traffic-sign layers,
  preserves its camera, shows `Frame 91 / 1170`, and retains verified original
  point-index mappings and datatip output. Counts are 73 curb, 315 pole and
  143 traffic-sign points.
- A three-frame video/map pilot on frames 91--93 prunes the marking layer from
  an old four-class figure, decodes exactly three video frames, ends at
  `Frame 93 / 1170`, and maps only the three retained Mississippi classes.
  The initial desktop pilot was stopped by the fixed-image-size guard after
  window geometry changed. The completed repeat used stable headless capture
  for interface validation; software graphics reduced displayed markers there.
  The visible frame-91 preview was checked separately in native desktop MATLAB.

Compact test and comparison records are in
[`results/road_marking_removal_20260909/`](results/road_marking_removal_20260909/).
Generated preview and pilot artifacts remain in
`output/remove_road_markings_20260909/`. Detection counts and numerical
regression equivalence do not constitute an independent accuracy assessment.
