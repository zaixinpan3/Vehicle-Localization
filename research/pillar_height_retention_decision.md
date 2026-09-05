# Retaining height in pillar perception and distribution registration

Date: 2026-09-04. Decision study at production baseline
`61e70b54ead9aa8050f6fb59644a3b5d972afb37`.

Follow-on implementation and its negative XYZ-matching findings are recorded
in [the implementation study](height_probability_cloud_implementation.md).
The measurements and proposal below describe the original decision study.

## Decision

Retain three-dimensional first and second moments of the returns supporting
each semantic pillar/component. Continue to classify XY pillars; retaining
height does not require point-level semantic validation, semantic Z voxels, a
dense 3D raster, or a six-degree-of-freedom vehicle observer.

The desired representation is a gravity-referenced XYZ distribution with an
exact XY marginal. The next registration design should exploit height only
after accounting for vertical alignment, tilt uncertainty, and partial vertical
visibility. Preserve the current planar registration as the evaluation baseline
and as a usable marginal when height is unavailable or unreliable. This study
recommends that direction; it does not enable a new production matcher or rebuild
the map.

Confidence is high in retaining the information, but an improvement in vehicle
localization accuracy has not been measured. Strong absolute-height matching
with the current unmodified map is not justified.

## Existing implementation

- `pillarizePointCloud` already retains sparse height-bin membership.
  Structural detection uses occupied-height count, continuous vertical runs,
  and vertical span. Ground detection uses elevation and terrain geometry.
- `aggregatePlanarCellMoments` reads XYZ and optionally applies known tilt,
  then saves only XY means and the xx/xy/yy scatter entries. The outgoing
  Gaussian cloud therefore loses z/xz/yz/zz information used upstream.
- Online feature decisions remain on pillars. Ground-feature moments describe
  ground members of candidate pillars, while pole moments describe eligible
  non-ground structural members. They are not fine-feature distributions.
- Offline `collectFeatureObservations` preserves XYZ after the full pose
  transform. `assembleMapInput` drops Z before fitting the current temporal
  Gaussian map. Existing saved XY components cannot reconstruct lost height;
  a new map requires retained XYZ observations or reprocessing raw frames.
- `poseRowToPlanarPose` supplies map x/y/yaw and a tilt rotation, but does not
  provide the vertical translation needed for absolute XYZ matching.
  `localizeLidarFrame` currently accepts only an SE(2) initial pose. Changing
  component dimensions alone cannot make a correct 3D registration path.
- Current D2D uses a one-window sum-of-support surrogate for the original
  noisy-OR/max map. A future 3D conversion must retain that explicit semantic
  distinction; its Gaussian mass factor would be
  `(2*pi)^(3/2)*sqrt(det(SigmaXYZ))`, rather than the current planar area factor.

## Primary-source check

This was a targeted design review, not a systematic review or a literature
claim that 3D will outperform 2D on this dataset.

1. Alex H. Lang, Sourabh Vora, Holger Caesar, Lubing Zhou, Jiong Yang, and
   Oscar Beijbom, *PointPillars: Fast Encoders for Object Detection from Point
   Clouds*, CVPR 2019. The pillar feature encoder in Section 2.1 retains point
   z and the z offset from the pillar mean while indexing pillars on XY.
   This supports separating the indexing dimension from feature dimension.
   It is a learned object detector, not evidence for this hand-designed D2D
   algorithm's localization accuracy.
   [Author manuscript](https://arxiv.org/pdf/1812.05784).
2. Todor Stoyanov, Martin Magnusson, and Achim J. Lilienthal, *Point set
   registration through minimization of the L2 distance between 3D-NDT
   models*, ICRA 2012, pp. 5196--5201,
   DOI [10.1109/ICRA.2012.6224717](https://doi.org/10.1109/ICRA.2012.6224717).
   The institutional abstract verifies the 3D distribution-to-distribution
   formulation and analytic optimization. Only metadata and the abstract
   were inspected in this study; no uninspected paper equations or performance
   results are used here.
   [Institutional record](https://portal.fis.tum.de/en/publications/point-set-registration-through-minimization-of-the-lsub2sub-dista/).
3. Aleksandr V. Segal, Dirk Haehnel, and Sebastian Thrun, *Generalized-ICP*,
   Robotics: Science and Systems 2009,
   DOI [10.15607/RSS.2009.V.021](https://doi.org/10.15607/RSS.2009.V.021).
   Section III.B explains anisotropic surface constraints and why scans from
   different viewpoints need not sample identical surface locations. This
   supports accounting for geometry and visibility instead of treating every
   observed extent as an invariant object size. Its modeled surface covariances
   differ from our empirical distribution scatter; they are not interchangeable.
   [Conference paper](https://www.roboticsproceedings.org/rss05/p21.pdf).

These sources support complementary representation and modeling principles.
Their different tasks do not constitute a controlled 2D-versus-3D comparison
for this project. The recommendation below is an engineering inference from
those principles, the code, and the bounded local probe.

## What to retain

For each relevant source pillar, retain count n, mean m in R^3, and population
scatter S in R^(3x3). When aggregating source pillars into an output component:

\[
N=\sum_j n_j,\qquad \mu=N^{-1}\sum_j n_jm_j,
\]
\[
\Sigma=N^{-1}\sum_jn_j\left[S_j+(m_j-\mu)(m_j-\mu)^T\right].
\]

This carries within-pillar and between-pillar scatter without recovering fine
point labels. Store six independent covariance entries, including xz and yz;
storing only a z variance loses tilt and slope coupling. Spatial covariance
must not be divided by N again as if it were centroid-estimation uncertainty.
Sensor, pose, terrain-reference, and sampling uncertainty are separate concepts.

With packed symmetric storage, geometry increases from five doubles
(two means and three covariance entries) to nine doubles (three means and six
covariance entries): 32 additional bytes per component, excluding metadata,
diagnostics, inverse caches, and temporary arrays. Full MATLAB covariance
matrices instead add 48 bytes per component. Dense working-raster storage
increases too; this arithmetic describes retained component geometry only.

Keep the existing vertical support/continuity descriptors. Height quantiles
or a compact relative-height histogram can distinguish disconnected vertical
support that one Gaussian cannot represent. They are secondary extensions to
evaluate, not a requirement to create a GMM or many Z layers in every online
pillar. A single Gaussian retains only first/second moments, not the complete
height distribution.

## How height should affect matching

The Gaussian identity

\[
p(x,y,z)=p(x,y)\,p(z\mid x,y)
\]

makes the distinction explicit: retain a full distribution and use its exact
XY marginal when the conditional height model is unreliable. More dimensions
do not necessarily provide more horizontal observability. A flat road's normal
constrains height and tilt; it does not by itself resolve translation along the
road. If every matched component has the same independent vertical distribution,
its common vertical factor cancels from normalized overlap, leaving the same
horizontal objective. Distinct heights, cross-covariances, and shape differences
can instead change correspondence ambiguity and pose sensitivity.

Recommended next design:

1. Keep gravity-aligned XYZ moments in the source and world XYZ moments in the
   new offline map. Preserve semantic layers and temporal support.
2. Continue to publish planar pose corrections. Use an externally supported
   vertical translation with uncertainty, or estimate a common frame-level
   vertical offset as a nuisance parameter with a justified prior. Do not
   silently assume the local sensor origin is at map z=0. Uncertain tilt also
   needs propagation or a bounded nuisance treatment.
3. Downweight uncertain vertical agreement using a modeled covariance or a
   visibility-aware height compatibility factor. Avoid hard rejection from
   matching raw minimum/maximum height. If no defensible vertical reference is
   available, use the XY marginal rather than an arbitrary very large variance.
4. Use height relative to local ground mainly as a shape descriptor. A local
   terrain subtraction changes the coordinate model; independently normalized
   heights cannot be inserted into rigid XYZ D2D as though they were global z.
   Ground reference and its uncertainty must be defined consistently.
5. Preserve distinct observed structures in mixed pillars. Whole-candidate
   moments may contain curb-adjacent road points or a partly visible pole;
   adding Z does not make coarse distributions equal to fine map distributions.

Feature implications are not uniform. Pole vertical continuity/anisotropy is
valuable, but visible top/bottom change with occlusion and range. Curb relative
height and xz/yz structure can help separate boundary geometry from surrounding
terrain, but a local step is not necessarily apparent in one pillar's z span.
Markings are nearly coplanar with the road; reflectivity and XY layout remain
their main evidence, while height mainly checks road consistency. Sharp marking
z variance makes naive absolute-height likelihood particularly sensitive.

For scale, two equal independent vertical Gaussians with standard deviation
0.01 m and a 0.10 m mean mismatch have normalized vertical overlap
`exp(-0.10^2/(4*0.01^2)) = exp(-25)`, about 1.39e-11. This is an analytic
illustration, not measured localization error. A 1 degree tilt discrepancy at
30 m also creates about 0.52 m vertical discrepancy. Such offsets must not be
interpreted as evidence that the correct horizontal match is wrong.

## Bounded Mississippi probe

Executed `evaluatePillarHeightRetention` in the existing MATLAB R2026a Update 3
desktop on frames 260, 550, and 900. No random sampling was used. All production
perception defaults were retained. Source pillars are 0.3 m in XY.

The cost comparison uses all ROI-retained returns on the same dense XY raster
and compares the current two-pass XY moment accumulator with a research-only
XYZ extension. There are three interleaved `timeit` estimates per method/frame;
the table reports their median. Loading, pillar indexing, semantics, IMU rotation,
registration, covariance regularization/inversion, and output serialization are
excluded. This is not the full two-branch perception runtime.

| Frame | Returns | Occupied pillars | XY moments (ms) | XYZ moments (ms) | Added (ms) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 260 | 51,577 | 10,366 | 1.177 | 1.781 | 0.604 |
| 550 | 51,237 | 15,210 | 1.201 | 1.880 | 0.679 |
| 900 | 53,451 | 13,745 | 1.231 | 1.942 | 0.711 |

Counts, XY means, and XY scatter are exactly equal between accumulators on all
three frames. The measured additional cost supports retaining XYZ moments;
it does not establish the cost of the eventual complete 3D map/matcher.

Candidate statistics use all retained members of each semantic candidate pillar
having at least four returns. Per-frame median vertical standard deviations are
0.015/0.014/0.039 m for curb, 0.0045/0.0049/0.0062 m for marking, and
1.254/1.401/0.943 m for pole. Pole-to-horizontal RMS ratios have medians
13.62/12.61/10.78. There are only 1/2/15 eligible pole pillars, so these are
descriptive examples, not stable population estimates. Candidate selection
already uses height, making this a check of retained information rather than
independent evidence of classification improvement. No point labels, tracking
of the same object across frames, or accuracy comparison were performed.

Artifacts:

- [Reproduction function](experiments/evaluatePillarHeightRetention.m)
- [Moment runtimes and equality checks](results/pillar_height_retention_20260904/moment_runtime.csv)
- [Candidate height summaries](results/pillar_height_retention_20260904/candidate_height_summary.csv)

Run after repository path setup:

```matlab
addpath(fullfile(pwd, "research", "experiments"));
evaluatePillarHeightRetention(fullfile(pwd, "research", "results", "pillar_height_retention_20260904"));
```

Code Analyzer reported no issues for the experiment function. No production
code changed, and no full regression suite or 3D localization test was run.

## Acceptance evidence required before changing the default matcher

Compare current XY D2D, uncertainty-aware XYZ D2D, and XY plus relative-height
compatibility on the same independently built map windows. Exclude query scans
from map fitting; use fixed initial-pose perturbations. Include vertical offsets,
tilt errors, ramps, pole truncation, sparse distant returns, missing classes,
and mixed-height negative controls. Measure horizontal/yaw errors, accepted
incorrect matches, rejection rate, runtime P50/P90, and memory. Lock tolerances
before using held-out cases, and distinguish map-pose consistency from independent
accuracy. New thresholds or a full-trajectory improvement are not established
by this study.

Research assistance: source verification, the reproducible statistics probe,
and synthesis used coding-agent assistance. Reported measurements came from
the executed local probe; proposed matcher changes remain unimplemented.
