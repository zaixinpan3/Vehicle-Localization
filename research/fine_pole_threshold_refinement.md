# Fine pole threshold refinement: 2026-09-10

The current default rejects short-support pole candidates whose radial RMS
about their fitted axis exceeds **0.10 m**, tightened from 0.12 m. The
existing condition applies only below 2.0 m qualified vertical support.
No detector branch, coordinate exception, ring dependency, or coarse pillar
change was added. Baseline: `7d8c6cfb83ce9706400029c42bb9ae6e2a649e39`.

## Reported false positive and decision

The user identified Mississippi frame 91 original index 16728, XYZ
`[-0.495967 23.122290 2.267180]` m, as another false-positive pole return.
Its candidate has 15 supported points, 4.321 m total height, only 1.5 m
qualified support, 7.784 degrees fitted tilt, and 0.1070 m radial RMS. It
passed the previous 0.12 m short-support cap. The point itself is 0.1524 m
from the fitted axis; the prior robust per-point limit is 0.2892 m.

Reducing the short-support RMS cap by 0.02 m rejects this candidate as a
whole. It also retains rejection of the earlier reported point 47906.
Frame/index metadata are used only in validation and datatips. Production
classification still uses unorganized XYZ geometry. The prior 0.12 m study
is preserved in `fine_pole_rejection.md` as a dated technical record.

## Baseline comparison

The same pipeline was evaluated with 0.12 m and 0.10 m on the 30 existing
reference frames plus Mississippi 91 and 425. Only these frames changed:

| Dataset | Frame | Pole points before | After | Removed |
| --- | ---: | ---: | ---: | ---: |
| Mississippi | 900 | 632 | 506 | 126 |
| Downtown | 100 | 283 | 264 | 19 |
| Mississippi | 91 | 284 | 269 | 15 |

The other 29 of 32 frames have identical complete masks. All non-pole masks
and complete coarse probability clouds remain identical on all 32 frames,
and no new pole points appear. The reference frames retain 6,348 of 6,493
baseline pole points (97.77%). Relative to the earlier pre-rejection
revision's 6,641 points, cumulative retention is 95.59%.

Mississippi frame 260 retains all 136 pole points, including the confirmed
point 63010 and its recovered 17-point neighborhood. Frame 425 retains all
21 pole points. The 126 removals in frame 900 are a material local change:
there are no user labels confirming that these, or the 19 Downtown removals,
are false positives. These numbers measure baseline retention, not precision,
recall or independently validated detection improvement.

`finePoleRejection.json` now records cumulative intended removals relative
to its original baseline, with the latest revision baseline explicitly named.
The frozen original perception and recovery fixtures remain untouched. Exact
reference-mask tests still cover every point; the frame-91 regression now
requires both user-reported points to be rejected and 269 poles to remain.
The permutation check also requires both points to remain rejected after
reordering the input into unorganized XYZ vectors (seed 91047906).

The native niri frame-91 preview was recomputed and refreshed with direct
original-point coloring, uniform point size, frame counter, original-index
datatips, and the current/saved camera. Point 16728 now displays as `source`.
Existing full-sequence videos/maps are historical outputs; no full-route
regeneration was performed.

The full test suite passed 327 tests with zero failures; two synthesis tests
were filtered for unavailable YALMIP/SDP dependencies. Factory Code Analyzer
reported zero findings in the two changed MATLAB files.

Executed test/analyzer summaries and 32-frame comparisons are exported under
`research/results/fine_pole_threshold_20260910/`. Native figures, screenshots,
and diagnostic artifacts remain under `output/pole_threshold_20260910/` and
in the identified archive export bundle.
