# Mississippi frame 932: motion transport and conflicting curb geometry

The live raw replay at commit
`5411244a2294848bb92431a40c2fed44f73d6d53` has its post-initialization
maximum at frame 932: **0.159861270 m position error and +0.672590030 degrees
yaw error**. In reference body axes, the error is -0.015715 m longitudinal
and -0.159087 m lateral. This study reproduces that result and diagnoses it;
it does not change production detection, registration, motion, or map code.

The original full point cloud and fine/coarse feature comparison were opened
on niri with `showMississippiFeaturePillars(932, ...)`. Original points retain
the same small marker size and are recolored: orange pole, magenta sign, cyan
curb. The close lower grid shows current 0.6 m selections. A second interactive
window shows matching geometry and history-transport discrepancies.
Both windows were verified in niri's window list and explicitly focused.

## Principal finding

**Historical translation compensation is the strongest isolated source of
this frame's error. Curb geometry also contributes. Reinstating the missed
current pole alone does not improve this frame.**

The five-scan window spans 0.400790 s. From frame 928 to 932, its odometry
translation expressed in frame 932 body axes is [-4.603662, -0.004335] m,
whereas the recorded reference trajectory gives [-4.570399, -0.098931] m.
The difference is approximately +3.326 cm longitudinal and -9.460 cm lateral.
The yaw-increment discrepancy is only 0.032872 degrees. Transported feature
centers from the oldest acquisition consequently differ by about 10.4--10.9 cm
from reference-motion transport. The discrepancy decreases steadily with age.

Replacing only the historical translation with reference translation reduces
the final error to **6.253 cm**; replacing only historical yaw leaves
**15.512 cm**. Replacing both gives **5.761 cm**. These interventions keep the
original current coarse distributions, confirmation rules, predicted pose,
conditioned map, and registration configuration. The translation-only and
yaw-only histories preserve the other motion increment exactly. They isolate
transport before pooling, rather than resetting the output pose to reference.

The saved motion comes from the preceding recorded four-wheel-speed,
corrected-gyro, lateral-observer replay, integrated causally. This study
establishes a relative translation disagreement with the evaluation reference;
it does not establish which underlying velocity estimate, reference heading,
calibration, or reference trajectory is physically correct. The roughly
0.24 m/s lateral discrepancy over this short interval identifies a concrete
next diagnostic for the motion pipeline. It is not evidence that lateral
velocity was omitted from the integration.

## Controlled registrations

Map conditioning is frozen at the causal production prediction except the
explicit dynamic-reference-seed control. Every source component remains a
distribution from the existing whole-pillar coarse pipeline.

| Diagnostic intervention | Position error (cm) | Yaw error (deg) | Update |
| --- | ---: | ---: | --- |
| Production, reproduced | 15.986 | +0.67259 | Full, rank 3 |
| Start at reference; frozen map | 15.986 | +0.67260 | Full |
| Start at reference; recondition map | 15.992 | +0.67260 | Full |
| Increase iteration budget to 400 | 15.986 | +0.67259 | Full |
| Reference translation transport only | 6.253 | +0.34183 | Full |
| Reference yaw transport only | 15.512 | +0.67302 | Full |
| Reference full motion transport | 5.761 | +0.34159 | Full |
| Remove curb 711 sources; preserve other weights | 10.610 | +0.26612 | Full |
| Remove all line-direction constraints | 19.896 | +0.90017 | Full |
| Remove pole support | 20.928 | +0.60992 | Directional, rank 2 |
| Restore current near-pole detection at threshold 0.87 | 16.027 | +0.67246 | Full |

Results are separate interventions, not an additive error decomposition.
Single-scan and confirmed-current-only controls are directional, with errors
13.938 and 15.869 cm. They lack the historical pole and cannot substitute for
a fully observable registration. Other controls are preserved in `controls.csv`.

## Curb model and objective bias

The confirmed source contains 15 curb components and one pole component.
At the production pose, those curbs associate to map components 710, 711,
and 714. The reference pose uses some neighboring component 709 as well.

Four source components on the upper curb associate to map curb 711. Their
inferred local direction disagrees with the map Gaussian's major axis by
**3.14008 degrees at reference**, with an engineering angular scale of
**1.01216 degrees**. The resulting standardized direction residual is about
-3.10. All four components carry that common direction discrepancy.
The source direction is inferred from neighboring selected centers, while
the target direction comes from a larger accumulated map covariance. These
are not necessarily descriptions of the same local length of curb.

Suppressing only those four sources reduces position error to 10.610 cm and
yaw error to 0.26612 degrees. The diagnostic uses temporal weights to suppress
the group and rescales the remaining same-class weights to prevent semantic
class normalization from increasing their force. `checkDiagnosis` verifies
that every retained residual and weight at reference is identical to the
baseline. This particular upper-curb removal also leaves the other source
direction neighborhoods unchanged. Removing all directions makes the result
worse, so a blanket removal or broad weakening is unsupported.

Lower curb 710 has another mismatch: fine curb points near the selected source
centers lie roughly **17--19 cm** from the map Gaussian's infinite-line normal
location. The corresponding coarse-versus-current-fine local normal differences
are about 0.7--2.8 cm. This is evidence of a local map-line representation bias,
not proof that the current detector invented those curb points. The neighborhood
fits are diagnostics from only 3--23 fine returns, and their tangents are noisy.
The map view model currently conditions point landmarks (pole/sign), not curb
Gaussians. A single accumulated curb Gaussian is still treated as a straight
local line despite finite extent, curvature, and acquisition differences.

The current geometric objective is lower at the biased solution (0.48454) than
at reference (0.65617). Starting at reference converges back to the same biased
pose, and extra iteration budget has no effect. The geometric normal matrix
has rank three; its scaled eigenvalues at reference are approximately
[5.331, 8.814, 43.883]. This is a model-consistency problem even with available
geometric information, not evidence of a calibrated pose-uncertainty bound.

## Current pole rejection is real, but is not the dominant pose cause

Current coarse perception selects zero pole cells. The matching source retains
the near pole from frames 928--931 (four acquisitions, stability 0.8), so the
matcher does have a pole. Its mean in frame 932 body axes is
[8.292496, -2.948730] m; the matched map pole is
[8.296481, -2.978258] m at reference. Their center difference is about 3.0 cm.

The current shaft hypothesis near [8.255099, -2.918454] m has 3.730 m supported
height, 204 accepted support returns, 0.0921 m radial RMS and 0.9733 isolation.
Its owner score is **0.870262339**, below **0.874722905**. That shaft lies
1.14 cm from a current original-fine pole return. Another owner scores 0.77381.
The table also records a farther fine-supported shaft near
[19.308224, -9.984011] m with much lower learned scores; the original fine pole
reference includes more than the near localization landmark.

Reducing the threshold to 0.87 only for frame 932 restores one current coarse
pole, then the production association/confirmation rules update the five-scan
window. Position error becomes **16.027 cm**, slightly worse than baseline.
This local diagnostic does not justify lowering the global precision-oriented
threshold. It also does not mean pole evidence is unhelpful: removing the
historical pole worsens the result and removes a geometric direction of
observability.

The full-cloud comparison reports 192 fine pole points, zero coarse pole
cells, and 67 fine curb points with 65 covered by 21 coarse cells. Three current
curb cells contain no original fine curb point. The stored reference is the
original detector output, not a surveyed physical annotation. Counts of fine
points, current cells, and confirmed matching components are distinct products.

## Next algorithmic work supported by these findings

1. Audit body translation used in temporal transport, particularly lateral
   velocity and reference/body convention around frames 928--932. Estimate
   transport consistency from repeated feature distributions before treating
   their pooled centers as fixed geometry. A rigid repeated shape can pass the
   existing covariance gate while a biased translation moves all its centers.
2. Model local curb segments at a comparable source/target extent. Account for
   curvature and shared directional evidence; four nearby components should
   not turn one shared direction mismatch into four apparently separate votes.
3. Improve the near-pole rejection rule under the existing low-false-selection
   requirement, but assess it on the full route. This frame's threshold control
   shows that recovering the pole alone will not address the main error.

No production algorithm change or new full-route improvement is claimed here.
Reference-motion controls use unavailable runtime reference information.
The map and query recording overlap; the LiDAR origin correction is empirical
and fitted on this drive. All accuracy statements are within this evaluation.

## Reproduction and artifacts

From the repository root:

```matlab
setupVehicleLocalization;
addpath('research/frame932_diagnosis_20260929');
diagnoseFrame932;
traceFrame932;
checkDiagnosis;
view = showMississippiFeaturePillars(932, ...
    'output/frame932_diagnosis_20260929/viewer');
fig = showFrame932Matching;
```

The study needs the existing raw replay, cached original coarse distributions,
recorded motion, fine map observations and recorded poses. Large inputs and
MAT products remain under `data/` and `output/`, excluded from Git. Compact
controls, source-track masks, correspondence/objective tables, motion traces,
pole scores/features, diagnostic assertions, and the matching PNG/PDF are kept
here. `input_hashes.json` and `artifact_hashes.json` identify technical inputs
and exports. Code Analyzer outcomes are recorded without claiming a new
repository-wide regression run for a diagnostics-only change.
