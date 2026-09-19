# Attribution of the remaining Mississippi localization errors

Date: 2026-09-19. Runtime revision examined:
`2700c10b414a86b0cb1824144d97c943f980909f`.

This investigation changes no production perception, map, matching, or observer
parameters. It separates matching errors from fusion and prediction effects by
executing controlled offline interventions on the current results.

## Main finding

The large errors already exist in the LiDAR pose measurements. Conflicting
semantic Gaussian constraints, changing hard associations and local optima are
confirmed contributors. The matcher can prefer a displaced pose even when
initialized at the recorded reference. A nominally full-rank local information
matrix does not identify this error. Neither coarse perception alone nor the
three-scan motion transport explains the large errors.

The current production matching outputs have 0.18253 m XY RMSE and 0.84467 m
maximum discrepancy. Only 31 of 1170 outputs exceed 0.5 m, but they account for
34.50% of total squared error. Frames 80--100 and 800--842 jointly account for
42.37%. The first interval is predominantly lateral (0.52416 m lateral versus
0.06742 m longitudinal RMSE); the second is predominantly longitudinal
(0.45817 versus 0.19434 m).

On the same 1165 valid aligned measurement epochs, LiDAR XY RMSE is 0.18233 m,
while receiver BESTPOS corrected to the observer output point is 0.06399 m.
These are measurement discrepancies, distinct from observer output errors.
The evaluation reference is recorded INSPVA, not independent surveyed truth;
BESTPOS and INSPVA share a receiver, and BESTPOS is INS aided.

## Full-sequence matching controls

`runDiagnostics` executed eight fits at each of the 1170 frames. Except for
the explicit initialization oracle, every fit uses the production recursive
initial guess recorded for that frame. Changes do not propagate to subsequent
initial guesses. These are per-frame interventions, not eight independent
end-to-end localization replays. Reported values below include every **candidate**,
even when its acceptance gate fails; production substitutes predictions on
four failed frames. The production all-output RMSE therefore differs slightly
from the candidate-control RMSE.

| Intervention | Candidate RMSE (m) | Maximum (m) | Accepted |
|---|---:|---:|---:|
| Reproduce current inputs | 0.18258 | 0.84467 | 1166 |
| Initialize each solve exactly at reference | 0.16158 | 0.80015 | 1166 |
| Use exact reference-relative motion for scan transport | 0.17685 | 0.78929 | 1165 |
| Use only the current scan | 0.20942 | 1.31246 | 1167 |
| Replace stored map priors with uniform association priors | 0.18105 | 0.81826 | 1168 |
| Omit curb | 0.20964 | 0.95892 | 1164 |
| Omit pole | 0.28922 | 1.83407 | 1087 |
| Omit traffic sign | 0.19494 | 0.96806 | 1164 |

The cached control reproduces every accepted production XY pose within
9.34e-10 m. This validates the cached perception inputs and diagnostic map crop.
Reference initialization improves average error, but 21 candidate errors still
exceed 0.5 m. At frame 820, even exact initialization returns the same 0.68028 m
error. At frame 92 it changes the solution from 0.84467 to 0.41000 m: this frame
has a material basin/association effect, as well as residual-model bias.
Exact scan transport makes only a modest change overall, and removing the
window worsens the sequence. The map stability prior also does not explain
most of the tail: uniform priors leave an 0.81826 m maximum. This is not a
recommendation to discard the existing stability weights.

## Which implementation stages contribute

### 1. Semantic association and the pose objective

`registerSemanticProbabilityCloud` chooses one same-class target for every
eligible source Gaussian using covariance-normalized geometry and stored map
mass. The association gate admits point-like centers within 2.5 m. Multiple
source components may choose the same target. There is no explicit competing
clutter/no-correspondence likelihood, object identity or visibility model in
this objective. Cauchy weighting reduces large residuals, but a coherent set
of wrong constraints can become small-residual constraints at a displaced pose.

The solver recomputes hard associations and whitening during iteration. Each
line search reduces a **frozen-pair** geometric cost; this is not optimization
of a single globally marginalized GMM likelihood. Convergence means a local
stationary configuration, not a physically correct registration.

At frame 92 the recomputed class-balanced robust residual score is 4.31685 at
reference and 3.78210 at the displaced production pose. The sign contribution
falls from 0.79237 to 0.22172. The latter pose has 0.84467 m error. These scores
use each pose's own associations and normalization; they are engineering fit
scores, not comparable calibrated likelihoods or proof of a global optimum.

`analyzeAssociations` additionally freezes both association identities and
whitening at the reference, then fits the same robust mean residual. At frame
92 this diagnostic yields 0.16407 m error. At frame 820 it still yields
0.60740 m, and at frame 830, 0.58378 m. Thus association/metric changes matter
strongly for some peaks, while other peaks remain biased even with fixed pairs.
Pairs chosen at reference are an oracle baseline, not labeled correct matches.
This control changes two coupled mechanisms and cannot assign a numerical
percentage to association alone.

### 2. Equal class budgets can give inconsistent sign constraints too much influence

The matcher normalizes source quality within each class and gives each shared
class one third of the base residual weight. This protects sparse landmarks
from numerous curb cells, but does not account for the current reliability of
a class's detections, visibility or association.

At frame 820, reference-pose pairs comprise 167 curb components, 12 pole
components and 12 sign components. Each class receives base weight 1/3. The
12 sign components associate with only three map components; three pooled
scans do not imply 12 independent landmarks. Removing only signs changes the
candidate error from **0.68028 to 0.05571 m**. At frame 816 the change is
**0.76915 to 0.10624 m**. This is strong evidence that sign constraints drive
these particular errors. It does not label every sign detection false.

Signs are useful elsewhere: deleting them for the full per-frame control raises
RMSE from 0.18258 to 0.19494 m, and at frame 842 changes 0.12220 to 0.55768 m.
Poles are also essential: omitting them worsens full-sequence RMSE to 0.28922 m.
The needed improvement is evidence-dependent constraint reliability, not blanket
class deletion or simply increasing all pole weights.

### 3. Perception statistics and map Gaussian geometry represent different support

Online `analyzeGroundPillars` computes moments from ground returns in accepted
cells before fine boundary thinning. `analyzeStructuralPillars` computes whole
off-ground pillar moments; its sign eligibility uses maximum pillar intensity,
and sign moments include all off-ground returns in that accepted pillar.
`buildCoarseSemanticProbabilityCloud` then moment-matches these distributions
into 0.9 m output cells. Their means are empirical point centroids, **not cell
centers**; a 0.9 m grid does not automatically imply 0.45 m position bias.

The map uses fine point masks registered over many frames, followed by temporal
GMM fitting. The map covariance includes spatial scatter, stable offsets and
reference-mean uncertainty. A visible portion of an extended object and its
multi-view Gaussian mean need not have the same centroid. Curb/facade normal
residuals address the line-tangent ambiguity, but signs and poles use full XY
mean residuals. Stable detectability does not guarantee an unbiased centroid
under partial observation or a correct current correspondence.

To test whether coarse classification alone explains the peaks, `runFineControl`
reuses fine observations that built the map, undoes their known registration,
and bins them at the same 0.9 m resolution with the same covariance bounds.
It replaces each class separately and all classes on 73 diagnostic frames,
including both entire peak intervals. All-fine candidate RMSE on this selected
set is 0.47484 m versus production 0.48313 m. At frame 92 all-fine still gives
0.80074 m, and at frame 820, 0.67203 m. It improves the first interval but worsens
the second (0.52331 versus 0.49768 m). Some candidates fail gates.

This is a joint support/geometry/quality substitution, not a pure resolution
ablation. Reusing mapping observations is deliberately optimistic and is not
independent validation. Nevertheless, simply returning to fine perception is
not supported as a solution to these peaks. These experiments do not separately
identify GMM compression error, upstream scan distortion or map-reference error.

### 4. Local geometry information fails to report a coherent registration bias

The exported information matrix is a robust Gaussian Gauss--Newton normal
matrix. It describes local curvature conditional on the selected associations,
source scatter and map scatter. It does not estimate the probability of a wrong
association, systematic source/map displacement, temporal correlation or the
existence of a second solution basin. At frame 92 the result is full rank and
all three information-weighted class corrections pass the 0.5 gate, despite
the 0.84467 m position error. Merely checking eigenvalues cannot catch this case.

Fusion inherits part of this error. `runObserverControls` keeps measurement
validity, information, common initial state and all motion inputs unchanged:

| LiDAR intervention in the observer | Fused XY RMSE (m) | Maximum (m) |
|---|---:|---:|
| Current measurements | 0.08103 | 0.35361 |
| Replace LiDAR XY only with reference | 0.05087 | 0.17066 |
| Replace LiDAR yaw only with reference | 0.08612 | 0.35002 |
| Replace complete LiDAR pose with reference | 0.05691 | 0.20270 |

These coupled observer controls confirm that inaccurate LiDAR position is a
major source of the fused peak. They are not an additive variance decomposition
or performance bounds. Yaw and the LiDAR-derived velocity-bias adaptation
interact, which is why replacing more inputs does not monotonically reduce
error. GNSS, motion/IMU errors and sampled observer dynamics remain present.

The preceding implementation report separately measured the near-saturated
LiDAR information gain: restoring scale 0.001 on the same final inputs worsened
fused RMSE from 0.08103 to 0.11072 m and maximum from 0.35361 to 0.51660 m.
Current scale 16 attenuates this problem but is not empirical confidence
calibration. Fusion currently has a lower RMSE but a higher maximum than
GNSS-only (0.08815 m RMSE and 0.26083 m maximum).

### 5. Motion and scan timing: distinguish confirmed effects from a remaining hypothesis

Prediction and source-window transport are secondary contributors in the full
controls above. In complete measurement outages their role is different:
the preceding same-input ablation found that omitting the longitudinal
velocity-bias correction increased the 20 s dual-outage maximum from 0.61189
to 1.44889 m. That previously fixed mechanism should not be presented as the
cause of the remaining continuously measured peaks.

The present data also warrant a scan-motion audit. Epochs with absolute gyro
rate above 10 deg/s have 0.44257 m matching RMSE (88 epochs), versus 0.13392 m
below 3 deg/s (882 epochs). The 88 high-rate epochs contribute 44.26% of aligned
squared matching error. This is correlation, not proof of scan distortion.

Read-only examination of original ROS PointCloud2 messages at frames 92 and
820 found a per-point uint32 field `t`, spanning respectively 0--99804570 and
0--99859800 raw units. Current MAT frames retain XYZ, intensity, reflectivity,
ambient, range and a single frame timestamp, but no per-point time. The current
perception path performs no per-point motion deskew. This establishes a missing
input/preprocessing capability. It does not establish whether the original
driver already compensated XYZ, the correct `t` convention, or the size of its
localization effect. Those must be verified before claiming causation or
applying a deskew correction. Three-scan transport does not by itself correct
motion within one scan.

## Priorities supported by the evidence

1. Improve semantic association and per-class/per-object constraint reliability,
   especially inconsistent signs; retain stored temporal-stability map weights.
   Include an explicit unmatched explanation and detect competing pose solutions.
2. Audit the original scan time convention and upstream compensation, then test
   deskew consistently for both mapping and localization. This is a high-priority
   hypothesis, not an already measured correction.
3. Make source-versus-map observation geometry compatible under partial visibility
   and calibrate resulting pose confidence empirically. Preserve whole-pillar
   sufficient statistics; do not infer that finer grids or fine online perception
   are required.
4. Validate on withheld frames/drives and retain both RMSE and tail/availability
   metrics. Arbitrarily withholding difficult measurements can exchange a matching
   peak for prediction drift; measure both outcomes.

## Reproduction and validation

```matlab
setupVehicleLocalization; addpath('research/error_attribution_20260919');
runDiagnostics; runFineControl; runObserverControls; analyzeAssociations;
```

```bash
uv run --offline --with numpy --with rosbags python research/error_attribution_20260919/auditPointTimes.py
uv run --offline --with numpy --with pandas --with matplotlib python research/error_attribution_20260919/analyze.py
```

The runs consume the existing cached coarse inputs, frozen map, fine map-building
observations and final observer inputs listed in the scripts. There is no random
sampling, rigid trajectory alignment, reference-based clipping or production
parameter edit. Raw result tables, snapshots and plots are under
`output/error_attribution_20260919/`; compact summaries and selected controls are
committed here. The complete diagnostic figure is `error_attribution.png`/`.pdf`.
The raw timing-field audit reads original bag messages by their recorded frame
timestamps and serializes field definitions and time ranges to `raw_time_fields.json`.

Checks assert complete 1170-frame coverage for all eight controls, accepted-pose
reproduction, and successful convergence of all 36 frozen-pair optimizations.
MATLAB factory Code Analyzer reported zero messages for the four diagnostic
functions. Independent Python recomputed the reported summaries and generated
the inspected plot. No production code changed, so prior production unit tests
were not needlessly rerun. The first diagnostic invocation stopped on a map-mass
validation error when cropping an unprojected map; using the production XY
projection before cropping fixed the harness, and the complete rerun passed.

The map includes query observations from this same drive, known recorded tilt
is used, and source reference data share a receiver with GNSS. Oracle controls
are explicitly diagnostic and must never be substituted into deployed inputs.
The present experiments identify mechanisms and counterexamples, not independent
ground-truth accuracy or exact additive percentages for each module's error.
