# Root cause of Mississippi frame 178

This diagnosis uses production commit
`4985af121073f32d659df49a67c751167fbd0c05`, the adopted view-conditioned map,
unchanged 0.6 m pillar perception, and the saved causal trajectory. The current
frame-178 horizontal discrepancy is **16.224 cm**, comprising **-9.466 cm
forward and -13.176 cm left**, with **+0.117 degrees yaw**. The incoming
prediction already has 16.821 cm discrepancy. These quantities are differences
from the existing INSPVA comparison reference, not surveyed physical errors.

## Main mechanism: partial sign centers treated as complete landmark centers

The pooled source contains 27 curb distributions, three sign distributions,
and no pole. All three signs associate to the **same** map component, global
ID 1074. These are Gaussian matching components, not necessarily individual
0.6 m pillars: the production cloud aggregates selected pillar statistics into
1.2 m output cells before temporal tracking.

In reference body coordinates, the conditioned map center is
`[7.221781, -3.434401]` m. Source sign 22 has mean
`[7.432372, -3.067492]` m: **+21.06 cm forward and +36.69 cm left**, or
**42.30 cm** away. Signs 21 and 30 are only 2.48 and 8.72 cm away.
The algorithm treats all these partial distributions as observations of the
same complete point-landmark center. Moving the vehicle backwards and right
reduces sign 22's residual at the expense of the better-aligned observations.

The following interventions keep the production incoming seed and freeze
the exact same conditioned map, including its view-support weights:

| Intervention | Horizontal discrepancy |
|---|---:|
| Production | 16.224 cm |
| Disable sign 22's force, preserve every other residual weight | **1.824 cm** |
| Remove sign 22, allowing normal within-class rebalancing | **0.878 cm** |
| Keep only sign 21 plus the original curbs | 0.704 cm |
| Keep only sign 22 plus the original curbs | 37.389 cm |
| Replace sign 22's center with its original fine-labeled subset center | 14.599 cm |
| Merge nearby source signs into one moment-matched Gaussian | 15.737 cm |
| Admit the current pole at full temporal weight, keep original signs/curbs | 2.220 cm |
| Increase maximum iterations from 40 to 400 | 16.224 cm, exactly unchanged |

The weight-preserving control is stronger than simple deletion: deletion
normally increases the remaining signs' within-class weights. The harness
reduces their temporal multipliers to exactly cancel that renormalization,
then asserts equality of all other correspondence weights at the reference.
Correspondences, robust influence and optimizer basin can still change as a
consequence of the intervention. These reductions are interacting
counterfactual outcomes, not additive percentages of error attribution.

## Raw-point and temporal evidence

All five original scans 174--178 were reprocessed, reproducing their cached
current clouds exactly. Every selected sign component's point count and
projected XY mean were independently reconstructed from its original selected
pillars. A diagnostic copy of the class-local greedy association reproduces
the production sign-track means **and covariances**. A nearest-final-center
shortcut was initially rejected because it does not reproduce temporal
membership when a new partial sign track appears.

Source 22 is confirmed in all five acquisitions. Across its 245 returns,
139 (56.73%) carry the original map's fine sign labels. At frame 178,
56 of its 106 returns are fine sign points. Thus this is **not an empty
false-selected pillar under the user's definition**. Unlabeled returns
contribute contamination relative to that reference, but the original fine
annotations are not asserted to be exhaustive physical labels.

Removing the unlabeled returns only for the diagnostic center leaves that
partial-sign center **31.61 cm** from the map center; the matching discrepancy
remains 14.60 cm. Contamination alone therefore does not explain the error.
The fine labels in the local complete sign extend over approximately 0.47 m
forward and 0.61 m laterally. A partial slice has a different center from the
complete target. The current local sign has 158 fine returns and its complete
center is 5.86 cm from the conditional map center. A separate distant singleton
fine sign return is excluded from that local-center comparison, and the
158-return membership/mean is checked against the actual map observation.

Equal-scan tracking preserves the partial-center bias instead of averaging it
away. Original sign-22 target distances over frames 174--178 are
56.81, 41.23, 41.18, 36.58 and 36.55 cm after production odometry transport.
Full five-frame support gives this persistently biased track temporal weight
one. Simply merging the signs preserves a displaced aggregate center and does
not repair the measurement definition.

## Why existing safeguards accept the biased pose

At the reference pose, sign 22 has squared standardized residual 2.222;
the Cauchy factor retains **73.77%** of its nominal influence. At the biased
production solution that factor rises to **86.92%**. The summed source/map
scatter makes this physically consequential offset look statistically ordinary
to the configured robust loss. Its reference-pose objective contribution is
0.30564, compared with 0.28211 from all 27 curbs combined. At the final pose,
opposing individual sign gradients largely cancel within the sign class.
The existing class-consistency check therefore cannot isolate this component.

The curb-only scaled normal matrix at the production pose has eigenvalues
approximately `[0.00443, 4.406, 21.514]`. Its weakest direction falls below
the 1% observability threshold: the largely parallel curbs cannot supply an
independent longitudinal anchor. The complete system is numerically rank
three, with eigenvalues `[4.764, 9.391, 27.760]`, but observable biased geometry
is still biased. Rank alone is not an accuracy certificate.

Production objective cost is **0.46731**, below **0.59421 at the reference**.
It converges in four fine iterations; allowing 400 iterations changes no pose
coordinate. Disabling source canonicalization gives 16.225 cm. This is not
primarily an iteration-limit or coarse-refinement guard problem.

## Secondary contributors and the pole handover

With the conditional map frozen at the original prediction, a diagnostic
reference seed converges to 10.175 cm. Four curb sources (2, 11, 17, 18)
change map targets between that solution and production. Reconditioning the
map at the reference instead gives 10.193 cm. Thus there is genuine
correspondence-basin sensitivity, largely separate from the small map-view
change, but even the better basin retains substantial bias from sign geometry.

Using reference relative motion only for the five-frame source pool reduces
the discrepancy to 8.374 cm. For the frame-174 sign-22 observation, the
reference-motion transport differs from wheel/gyro transport by +1.77 cm
forward and -14.15 cm laterally; that difference shrinks to zero at frame 178.
This identifies a contributing inter-scan transport mismatch. It does not
separate wheel/gyro errors, reference errors, calibration, or actual lateral
motion, and is not a deployable use of reference poses.

The pole absence is a **temporal admission gap**, not a current-frame detector
failure. The old pole is near `[-16.44, 6.64]` m in frame-178 axes and is seen
at frames 173 and 175. The new pole is near `[16.30, 7.25]` m and first appears
at 178 in this window. When the horizon advances to 174--178, each physical
pole has only one observation, so both fail two-acquisition confirmation.
At 179 the new pole is confirmed and the saved production discrepancy drops
to 4.12 cm. Admitting the frame-178 pole diagnostically reduces that frame
to 2.22 cm, but changes the admission and class-balance rules. It is not a
validated all-route replacement for temporal confirmation.

The earlier single-scan result of 4.57 cm changes geometry, admission,
stability, and available classes together. The single scan also contains a
distant sign component absent from the confirmed pool. That result by itself
cannot establish that simply removing history solves this problem.

## Consequence for algorithm design

The primary issue is a mismatch between the geometry represented by a source
observation and the geometry represented by its map target. More permissive
detection or stronger temporal confirmation cannot guarantee compatible
centers. A subsequent implementation should distinguish feature-existence
confidence from localization-center confidence, handle partial sign support
consistently on both map and source sides, and avoid treating adjacent portions
of one physical sign as independent complete-center measurements. Within-class
conflict checks and a carefully qualified pole-handover rule are candidates
for evaluation. Earlier broad surface-model replacements did not improve the
whole route; this diagnosis does not claim that they now work automatically.

Source IDs and reference-assisted interventions here are retrospective probes,
not production rules. No frame-specific removal, online fine labeling or
reference-pose injection has been deployed. No production file was changed,
and no new whole-route accuracy improvement is claimed.

## Reproduction and validation

```matlab
addpath('research/frame178_diagnosis_20260929');
runDiagnosis;
```

```bash
uv run --offline --with numpy --with pandas --with matplotlib python \
  research/frame178_diagnosis_20260929/analyzeDiagnosis.py
```

The harness executes 31 fixed-frame registration controls, asserts production
pose parity, exact cached/raw cloud parity for five scans, exact source-window
reconstruction, sign-track mean/scatter parity, raw membership/count/mean
parity, local fine-map observation parity and preservation of other residual
weights in the stricter ablation. Four MATLAB research files have zero factory
Code Analyzer findings. No full-route rerun or production test-suite rerun is
claimed for this analysis-only change. Initial harness field-shape and scalar
expansion errors were corrected; a stale user Code Analyzer settings path was
resolved by explicitly using factory settings. Failed attempts are not passing
validation results.

CSVs and JSON preserve the results; `diagnosis.png` and `diagnosis.pdf` are
static exports. No desktop window is opened. Large inputs and MAT diagnostics
remain local under `data/` and `output/`; technical input/artifact hashes identify
them without publishing recordings. The map/query recording is shared, labels
are existing fine-perception output and poses are the existing INSPVA reference.
These are diagnostic findings on this recording, not independent-drive or
surveyed-ground-truth validation.
