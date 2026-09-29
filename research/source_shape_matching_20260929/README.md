# Joint position and shape compatibility before temporal confirmation

The temporal source window now rejects position-compatible observations when
their Gaussian covariance shapes disagree. It compares every original
acquisition in the prospective track before association and pooling, using
the existing probability-cloud means and covariances. No additional shape
covariance is exported or stored in the source-cloud schema.

This repairs a missing association condition. It does **not** substantially
reduce the current Mississippi route maximum: the adopted 0.5 threshold keeps
the peak at **15.986 cm, frame 932**. All-frame RMSE changes from 6.10627 to
6.11234 cm; P95 changes from 10.45378 to 10.44644 cm. The measured effect is
association consistency, not a demonstrated semantic false-selection reduction
or a new localization accuracy breakthrough.

## Previous failure and new rule

The previous source-window association used class identity, Euclidean center
distance and a center Mahalanobis distance with summed covariances. When two
means coincide, that center distance is zero even if one distribution is long
along X and the other along Y. Repetition could then confirm an incompatible
track. Once moment-matched, its widened covariance concealed the disagreement.

Each scan is first transported to the current XY frame using the existing
independent wheel/gyro motion. The same transform rotates the full covariance.
The existing 0.75 m center gate and standardized-distance gate of three remain.
For each position-compatible track/observation candidate, let the original
covariances be A and B. Apply a common spatial regularizer:

\[
 \bar A=A+\epsilon I,\quad \bar B=B+\epsilon I,
 \qquad \epsilon=0.0004\;\mathrm{m}^2.
\]

The covariance-only Bhattacharyya distance is

\[
 d_S(A,B)=\frac12\log\left[
 \frac{\det((\bar A+\bar B)/2)}{\sqrt{\det\bar A\det\bar B}}
 \right].
\]

This uses the existing covariance to consider direction and scale together.
It does not normalize away real scale differences. A common XY rotation leaves
the score unchanged, and nearly circular distributions do not invent a precise
axis angle. The 2 cm spatial floor stabilizes small empirical scatters; it is
separate from the existing 10 cm positional association-noise scale.

The Gaussian position/covariance decomposition is given in Section 3.3,
Equation (7), of Jeffri M. Llerena, Luis Felipe Zeni, Lucas N. Kristen and Claudio
Jung, [Gaussian Bounding Boxes and Probabilistic Intersection-over-Union for
Object Detection](https://arxiv.org/abs/2106.06072). Only the analytical Gaussian
distance is used here, not that paper's object detector or training method.
The production score uses separate engineering regularizers for motion-related
position error and spatial shape, so it is not a calibrated joint likelihood.

The candidate's shape distance is the maximum against **all** original
acquisitions already assigned to that track. It must be at most 0.5.
Comparing only the pooled covariance or the newest member would allow a chain
of individually modest changes to connect strongly conflicting endpoints.
Ownership indices refer to the original Gaussian observations already retained
in the five-scan history; no second set of shape moments is maintained.

Eligible pairs are ranked by center standardized squared distance plus four
times this shape distance, then assigned one-to-one within each class as before.
An incompatible observation starts another candidate track and needs at least
two compatible acquisitions before contributing. Existing equal-acquisition
mixture-moment pooling and spatial-scatter semantics remain unchanged; scatter
is never divided by the number of scans as if it were mean-estimation noise.
The two new configuration fields are `maximumShapeDistance` and
`shapeVarianceFloor` in `localizationSourceWindowConfig`.

## Complete-route controls

All controls use 1,170 Mississippi acquisitions, the same current 0.6 m detector
clouds, view-conditioned offline map, independent motion, original initialization
and existing INS tilt. Reference XY/yaw only score the recursively estimated
route. There are no fine query labels, reference resets, randomized draws,
frame-specific rules or desktop visualizations.

| Shape threshold | Peak after initialization | RMSE | P95 | Rejected pair tests | Full updates |
|---|---:|---:|---:|---:|---:|
| Disabled, prior behavior | 15.986 cm | 6.10627 cm | 10.45378 cm | 0 | 1147 |
| 0.25 | 16.162 cm | 6.10642 cm | 10.48625 cm | 15441 | 1145 |
| **0.50, adopted** | **15.986 cm** | **6.11234 cm** | **10.44644 cm** | **238** | **1148** |
| 0.75 | 15.986 cm | 6.11356 cm | 10.44644 cm | 0 | 1148 |
| 1.00 | 15.986 cm | 6.11356 cm | 10.44644 cm | 0 | 1148 |

The disabled control reproduces every previous source cloud exactly and every
previous route pose within export precision. Threshold 0.25 splits substantially
more tracks and worsens peak/P95; 0.75/1.00 reject no real candidate pairs on this
recording. The adopted threshold rejects 238 of 117,397 position-compatible pair
tests, across 186 output windows. The counts include repeated checks in overlapping
windows, not 238 distinct objects or independently verified false associations.
The shape term also changes ranking, so source clouds change in 650 frames.

Total confirmed component occurrences across all output frames change from
31,052 to 31,152; splitting a track can produce multiple separately confirmed
components, so this count is not a recall measure. Full updates increase by one,
while directional updates remain 11. The first configured 0.640312424 m error
is excluded only from the peak and remains in all-frame RMSE/P95.

## Distinguishing association from partial-sign measurement bias

Frame 178 stays near 2.776 cm with the preceding partial-sign measurement rule.
A full-route negative control uses the adopted shape gate but disables that
rule: frame 178 becomes the maximum again, at 16.224 cm. This demonstrates that
cross-frame shape gating does not replace the partial-sign correction. A real
partial patch can remain stable across acquisitions while its center differs
from the complete offline landmark's center. The new gate does not establish
that the earlier frame-178 error was caused by incompatible cross-frame shapes.

This task changes temporal association. The previously deployed map-conditioning
and partial-sign residual models remain in place. No new source shape statistic
is introduced. Further work on a unified map/source observation model remains
distinct from the association consistency repair reported here.

## Validation and timing

Ten new public-interface class tests cover colocated orthogonal anisotropic
clouds, incompatible scale, moderate shape variation, odometry rotation,
near-circular shape, cumulative shape bridging, position ties, retained center
gates and invalid settings. The initial new/existing source-window run passes
31 tests. The final selected 23-suite regression and raw replay results are
reported in `final_tests.csv`, `code_analysis.csv` and `summary.json`.
The final run passes **322 tests across 23 suites**. Factory Code Analyzer
reports zero findings in the two changed production files and the new test file.

`replayRawShape` recomputes perception on every original scan. It asserts exact
equality with the prior **current-acquisition** clouds, exact equality of the new
window clouds against the independently rebuilt cached-current-cloud replay,
and recursive-pose agreement below 1e-7. Window clouds intentionally differ from
the previous algorithm. The raw verification is counted only after all 1,170
scans complete and export their audit.
The final raw replay completes all 1,170 scans. Every current cloud and adopted
window cloud passes exact equality; the maximum raw/cached pose difference is
4.66e-9. The disabled-shape control reproduces all previous source clouds and
exported poses exactly. An initial Python summary inverted an integer 0/1 audit
column as bits; that derived count was corrected to a Boolean comparison and
a bounded-count assertion added before final export. MATLAB results were unaffected.

Paired alternating-order timing maintains independent previous/new histories
and asserts every resulting source cloud. Window-update median is 0.898 versus
1.269 ms, paired median difference +0.346 ms; P95 is 1.398 versus 2.525 ms.
Raw replay and regression work were concurrent. These local relative timings
are not a real-time guarantee; unchanged single-frame perception dominates the
raw processing cost. Percentiles use MATLAB-compatible midpoint positions.

```matlab
addpath(pwd); setupVehicleLocalization;
addpath('research/source_shape_matching_20260929');
screenShapeMatching;
checkShapeImplementation;
replayRawShape;
benchmarkShapeWindow;
```

```sh
uv run --offline --with numpy --with pandas --with matplotlib \
  python research/source_shape_matching_20260929/analyzeShapeMatching.py
```

Compact scripts, CSVs, JSON validation, plots and technical hashes are retained
in this study. Original scans, large MAT caches, final raw products and the
primary paper download remain local under `data/` and `output/`. Results reuse
the mapping recording and existing INSPVA reference; no independent-drive or
surveyed-ground-truth validation, physical error floor, or all-class semantic
precision improvement is claimed.
