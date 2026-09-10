# Fine pole rejection for short, diffuse support

This report records the 2026-09-09 configuration and experiment. The current
0.10 m short-support RMS threshold and subsequent validation are documented
in [the 2026-09-10 refinement](fine_pole_threshold_refinement.md).

The default fine detector rejects Mississippi frame 91 point 47906
(XYZ `[1.825752 -17.053820 -0.926266]` m), reported by the user as a false
positive. The source index is used only for validation, never as a detector
condition. Baseline: `d21fccb635f71f5ce80c8cf24b5388e05ffe6a72`.

## Decision and diagnosis

The candidate's total height is 3.622 m, fitted tilt 2.897 degrees, and mean
qualified-neighborhood support ratio 0.9095. However, only 1.5 m of its vertical
support meets the existing fine structural qualification. Its 31 supported
points have radial RMS 0.144 m about the fitted axis. The reported point is
only 0.095 m from that axis, so merely reducing the per-point radius would
retain it while removing some surrounding returns.

`validatePolePoints` now rejects a candidate when **both** conditions hold:

- Qualified support height is less than `poleShortSupportHeight = 2.0` m.
- RMS transverse residual over all supported points exceeds
  `poleShortSupportMaximumRadialRms = 0.12` m.

The gate runs before robust point trimming. Stronger vertical support can
still validate wider objects, while narrow sparse poles retain the existing
1.5 m minimum support. The recovered true pole at frame 260 has 2.0 m support
and approximately 0.078 m radial RMS. Both its 17 recovered points, including
63010, and the other 119 pole points are preserved.

This is a fine-only XYZ geometry condition. No ring/row/column metadata,
frame number, point index, or spatial blacklist enters detection. Coarse
perception still uses whole XY pillars with their point statistics.

Raising the unconditional support-height threshold from 1.5 to 2.0 m was
probed: it would remove 399 points from the 30 reference frames, including
some earlier narrow-pole recoveries. An unconditional 0.12 m radial RMS cap
would remove 1,154 points. Both alternatives were rejected in favor of the
combined condition's smaller baseline change. These are parameter-selection
comparisons on the same frames, not held-out accuracy measurements.

## Executed validation

Thirty existing reference frames plus Mississippi 91 and 425 were compared
against the same pipeline with the new radial cap set to `Inf`, reproducing
the pre-change pole decision rule. The original historical mask fixtures are
unchanged; `finePoleRejection.json` records the exact intended removals for
current regression tests.

| Dataset | Frame | Before | After | Removed |
| --- | ---: | ---: | ---: | ---: |
| Mississippi | 75 | 201 | 184 | 17 |
| Mississippi | 700 | 222 | 196 | 26 |
| Mississippi | 1100 | 429 | 365 | 64 |
| Mississippi | 1150 | 95 | 72 | 23 |
| Downtown | 200 | 557 | 539 | 18 |
| Mississippi | 91 | 315 | 284 | 31 |

The remaining 26 of 32 frames are unchanged. On the original 30 reference
frames, 6,493 of 6,641 baseline pole points remain (97.77%); 148 are removed
and none are added. Frame 260 retains all 136 pole points and frame 425
retains all 21. All other point masks and complete coarse probability clouds
match exactly on all 32 comparisons. Removed points outside the user-reported
example are threshold effects, not independently confirmed false positives.
This is baseline retention, not measured precision or recall.

The new public-interface regression checks frame 91's exact removed set,
unchanged other masks/coarse cloud, and identical pole decisions after a
random permutation into unorganized XYZ vectors (seed 91047906). Existing
frame-260 recovery checks remain unchanged. The full suite passed 327 tests
with zero failures; two synthesis tests were filtered because YALMIP/SDP
dependencies were unavailable. Factory Code Analyzer reported zero findings
in the five changed/new MATLAB files. Current reference comparisons
still require exact masks, including the explicit removal fixture.

The native niri frame-91 preview was refreshed in the saved/current view.
Datatips identify 47906 as `source`; one original-point scatter, uniform size
8, frame counter and camera are retained. No full-route video/map regeneration
was performed. The current pipeline will apply these defaults on subsequent
runs; existing recordings retain the results of their original revision.

Validation exports are under `research/results/fine_pole_rejection_20260909/`.
Native screenshots/figures and local diagnostic probes remain under
`output/pole_threshold_20260909/` and in the identified archive export bundle.
