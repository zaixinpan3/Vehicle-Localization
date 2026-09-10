# Traffic-sign intensity threshold: 2026-09-10

The default traffic-sign intensity threshold is **1800**, increased from
1600 (12.5%) at the user's request. Both the whole-pillar structural default
and detailed structural default use this value. The public invocation setting
remains `cfg.offGroundFeatures.trafficSignIntensityThreshold`.

Coarse perception selects a whole XY pillar when its maximum finite return
intensity is strictly greater than the threshold. Its Gaussian continues to
use all nonground members of that pillar, including less reflective returns.
Fine perception classifies individual candidate points only when their finite
intensity is strictly greater than the same threshold. A return equal to
1800 is rejected. These are raw recorded sensor intensity values, not a
physical reflectance unit or a calibrated cross-sensor threshold.

The existing fine structural preprocessing also excludes returns above this
threshold from pole/facade structural statistics. The parameter change keeps
that existing coupling. Sampled regression checks found no changes to the
other point masks; this does not prove that every unseen frame is unaffected.
No ring-based logic, point-index exceptions, or subpillar coarse processing
was introduced.

## Executed comparisons

Baseline: `51d47a41ad4bbb439cc23a5a86b303897b64b744`. The same current pipeline
was evaluated with thresholds 1600 and 1800 on the 30 reference frames plus
Mississippi 91 and 425. For every frame, the new traffic-sign mask equals the
old mask intersected with `intensity > 1800`, and every retained coarse sign
pillar belongs to the old candidate set. No new sign points or pillars appear.

| Scope | Fine sign points before | After | Coarse sign pillars before | After |
| --- | ---: | ---: | ---: | ---: |
| 30 reference frames combined | 2004 | 1864 | 360 | 317 |
| Mississippi 91 | 143 | 126 | 34 | 30 |
| Mississippi 425 | 240 | 238 | 13 | 13 |

The original 30 frames retain 93.01% of their previous traffic-sign points.
Sign point counts change on 24 of the 32 frames. All other fine masks,
including ground, curb, pole and facade, match exactly on all 32 frames.
Frame 91 keeps 269 pole and 73 curb points. The coarse sign confidence and
cross-class normalized probabilities may change along with the sign candidate
set; complete coarse probability products are not claimed to be unchanged.

These are threshold effects and baseline-retention measurements, not
independently labeled false-positive removals or precision/recall estimates.
No new full-route video/map run or real-time timing experiment was performed.

## Validation and visualization

Frozen historical perception fixtures remain untouched. The explicit
`tests/reference/trafficSignThreshold.json` correction records removed sign
indices for exact current-mask regression. Structural semantic tests now
compare against those current expectations. Synthetic positive examples lie
above the new threshold; an additional public-interface test requires an
intensity of exactly 1800 to produce neither a fine sign point nor a coarse
sign candidate. Existing tests still require each sign Gaussian to retain
all nonground members of the selected pillar and verify custom threshold
configuration.

The full suite passed 328 tests with zero failures; two synthesis tests were
filtered for unavailable YALMIP/SDP dependencies. Factory Code Analyzer found
zero issues in all four changed MATLAB files.

The native niri frame-91 preview was recomputed at the new default. It retains
the current/saved camera, one original-point scatter, uniform marker size 8,
frame counter and original-index datatips. Traffic signs remain magenta;
points removed only from that class return to their applicable remaining
class color or source gray.

Validation summaries are in `research/results/sign_threshold_20260910/`.
The native figure/screenshot and local comparison/test artifacts remain in
`output/sign_threshold_20260910/` and the identified archive export bundle.
