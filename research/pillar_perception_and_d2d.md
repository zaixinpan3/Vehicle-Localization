# Shared Pillar Perception and Semantic D2D Registration

Date: 2026-09-04. This study supersedes the execution-boundary and registration
choices in [the earlier voxel-cloud study](coarse_probability_cloud_design.md).

## Decision and implemented boundary

Use **XY pillars as the common analysis unit**. Keep the useful geometric ideas
of the existing detector, correct their implementation boundaries, and expose
real distribution statistics. Default perception is coarse; offline mapping
recovers and validates only the members of candidate pillars.

```text
returns -> ROI/range filtering -> XY pillars + sparse height counts
        -> common terrain segmentation
        -> ground geometry/radiometry and structural pillar neighborhoods
        -> semantic candidate pillar footprints
             -> online: empirical Gaussian cloud -> semantic D2D -> pose event
             -> offline: candidate-member decisions -> fine points -> map
```

The internal binning utility still has voxel-named membership fields because it
shares the historical ROI, boundary, and height-bin conventions. In sparse mode
all ten dense 3D statistic fields are empty. No height bin receives a semantic
classification. Ground/non-ground separation and conditional radiometry
statistics read individual returns as preprocessing; online feature validation
operates on pillar evidence, not per-point curb/pole/marking labels.

Production entry points are `perceiveFrame` and `perceiveCoarseProbabilityCloud`.
`analyzeGroundPillars` and `analyzeStructuralPillars` replace the former coarse
voxel-named analyzers. The common candidate product has `pillarIndices`, semantic
names, and `[Ny Nx]` XY geometry; it has no point semantic masks. `offline`
(alias `full`) adds fine masks and explicit candidate/evaluated/accepted arrays.
`legacyFull` is an explicit historical comparison path. The immutable stored
pipeline reference is tested through that path, not regenerated to bless changes.
The modern map defaults contain curb, roadMarking, and pole; facade and traffic
sign remain available in the historical pipeline.

## Research and selection of retained ideas

This was a targeted primary-source design review followed by implementation and
negative-control experiments, not an exhaustive systematic review. Searches
combined NDT, Gaussian overlap, Cauchy-Schwarz divergence, ground segmentation,
road reflectivity, semantic registration, and density mismatch. Sources were
checked through author manuscripts, arXiv, and publisher/institution metadata.

| Primary source and portion examined | Resulting decision |
| --- | --- |
| Stoyanov, Magnusson, and Lilienthal, *Point Set Registration through Minimization of the L2 Distance between 3D-NDT Models* (ICRA 2012), [DOI 10.1109/ICRA.2012.6224717](https://doi.org/10.1109/ICRA.2012.6224717); distribution overlap and analytic optimization formulation | Use analytic Gaussian distribution overlap, transforming covariance as well as mean. Adapt the representation to the existing planar map rather than assume a 3D map. |
| Lee, Lim, and Myung, *Patchwork++: Fast and Robust Ground Segmentation Solving Partial Under-Segmentation Using 3D Point Cloud* (IROS 2022), [arXiv:2207.11919](https://arxiv.org/abs/2207.11919), III.D adaptive ground likelihood | Local elevation and flatness statistics support adaptive terrain reasoning. Retain slope-grid terrain and relative geometry for this stateless change; do not claim to reproduce Patchwork++ or silently introduce its temporal state. |
| Certad, Morales-Alvarez, and Olaverri-Monreal, *Road Markings Segmentation from LIDAR Point Clouds using Reflectivity Information*, [arXiv:2211.01105](https://arxiv.org/html/2211.01105v1), reflectivity and thresholding method | Radiometry matters, but a global threshold substitution is not sensor-independent calibration. Retain the road-conditioned reflectivity rule after the global-Otsu ablation failed. |
| He, Emami, Ranka, and Rangarajan, *Self-Supervised Robust Scene Flow Estimation via the Alignment of Probability Density Functions*, [arXiv:2203.12193](https://arxiv.org/abs/2203.12193), PDF alignment formulation | Normalized overlap/Cauchy-Schwarz geometry offers a closed-form comparison of Gaussian mixtures. The present rigid solver and semantic class balancing are this project's engineering derivation, not a reproduction of their learned scene-flow method. |

Retained ideas include terrain-relative curb height/roughness, road adjacency
and connected boundary topology, reflectivity contrast on the road, and pole
vertical support relative to neighboring pillars. Removed duplication includes
the additional final curb-energy threshold and independent per-pillar pole
point/line gates. The pole shape test now evaluates each complete footprint
once in the coarse stage, using the previous mean-shape criterion (0.70).

Diagnostic findings before the final implementation:

- Simple height-step/relative-height/roughness curb selection on frames
  260/300/326 gave precision 0.130/0.115/0.095 and recall
  0.441/0.528/0.425 against the old cell support. Adding road adjacency alone
  restored high precision but lost most curb support. Topology was retained.
- Global Otsu road-reflectivity thresholds selected 149/1189/134 marking points
  versus the old 83/26/60 on those frames. Median-plus-MAD replacements also
  reduced fidelity. These trials do not establish a universal reflectivity rule.
- Initial pole refinement retained only 357 of 638 old pole points in frame 900.
  The cause was a boundary-crossing pole footprint: one constituent pillar had
  point-shape score 0.5 and was deleted by an independent post-gate. The remaining
  half then failed neighborhood support. Whole-footprint selection recovers
  632 of those points while actually testing their line residuals.
- Global mixture-energy normalization allowed extensive curb support to dominate
  sparse landmarks, producing several-meter along-road shifts. Equalizing each
  shared class's self energy fixes the semantic mass imbalance. Comparing direct
  local optimization with covariance continuation also prevents smoothing from
  discarding a better fine-scale basin. Neither change uses a reference pose as
  an optimization target.

## Fine validation

Ground features recover only ground members of candidate pillars. Curbs use the
established terrain-residual and dominant-boundary point tests, and markings
compare each candidate return with the road-derived reflectivity threshold.
Poles recover non-ground candidate members, test sparse object/neighborhood
height support, fit XY position versus height, reject excessive tilt, and apply
a metric radial limit with a median/MAD residual scale. The line fit itself is
least squares; the residual gate is robustly scaled. Metric defaults are in
`finePerceptionConfig`, including 0.35 m maximum radius and 20 degree maximum tilt.

Every recovered member has a decision. Group rejection can reject all members
before a line fit; no later fallback republishes the candidate set. Noncandidate
raw returns are not passed to fine feature classifiers. Existing neighborhood
statistics may still be consulted. This is a real coarse-to-fine dependency,
not a full detector followed by mask intersection.

## Distribution contract and map conversion

For pillar j, accumulate count n_j, actual mean m_j, and population scatter S_j
before feature selection. When pillars are grouped into a 0.9 m output cell:

\[
 n=\sum_jn_j,\quad \mu=\frac{\sum_jn_jm_j}{n},\quad
 \Sigma=\frac{\sum_j n_j[S_j+(m_j-\mu)(m_j-\mu)^T]}{n}.
\]

Covariance is regularized and its eigenvalues bounded. It is **spatial extent**,
not the covariance of an estimated centroid; dividing it by n would misrepresent
the point distribution. A known proper IMU tilt rotation is applied during
moment accumulation before projecting to XY. `poseRowToPlanarPose` decomposes the
recorded full rotation as R_global = R_yaw R_tilt, consistent with the offline
quaternion transform. Sensor extrinsics must already put returns in the vehicle
frame. Neither extrinsics nor roll/pitch are estimated by the SE(2) solver.

Each coarse component contains its class, actual mean, SPD covariance, support
count, evidence weight, and normalized mixture mass. Compatibility fields
`semanticProbability` and `occupancyProbability` represent uncalibrated semantic
evidence and hit support. They are not learned class posteriors or occupancy
posteriors incorporating free-space rays. Geometry retains within-pillar scatter
and between-pillar scatter, replacing the former cell-center/uniform-cell proxy.

The offline temporal map's query combines peak-normalized Gaussian support by
noisy-OR and combines overlapping windows by max. It is not an ordinary density
mixture. `temporalMapToProbabilityCloud` exports **one explicitly selected window**
as the normalized sum-of-support registration surrogate:

\[
 a_k=\text{priorScore}\,\text{supportAmplitude}_k,\qquad
 w_k\propto a_k\,2\pi\sqrt{\det\Sigma_k}.
\]

The area factor converts peak amplitude to integrated Gaussian mass. EM mixture
weights are not substituted for support amplitudes. The original query and saved
map are unchanged, and overlapping windows are not concatenated as independent
evidence. Class-wide prior scales cancel under the subsequent class-conditional
normalization; relative temporal support *within* each class still contributes.

## D2D objective and local solver

For shared semantic class c, let F_c and M_c be the fixed and transformed moving
Gaussian fields. The objective is the mean normalized overlap:

\[
 S(T)=\frac{1}{|C|}\sum_{c\in C}
 \frac{\langle F_c,M_c\circ T^{-1}\rangle}
 {\sqrt{\langle F_c,F_c\rangle\langle M_c,M_c\rangle}},
\]

\[
 \langle\mathcal N(\mu_f,\Sigma_f),
 \mathcal N(R\mu_m+t,R\Sigma_mR^T)\rangle
 =\mathcal N(\mu_f;R\mu_m+t,\Sigma_f+R\Sigma_mR^T).
\]

Self energies are constant under rigid motion. Analytic derivatives include the
rotating-covariance term. Class balancing is internal Hilbert-space scaling,
not a change to the exported probability masses. It prevents component count or
class-wide weight scale from dictating the relative influence of curb and pole.

`registerSemanticProbabilityCloud` uses BFGS with Armijo search, bounded local
corrections, covariance smoothing [1.0, 0.35, 0] m, and a direct unsmoothed branch;
it selects the higher final score. Map means are recentered at the initial
translation for numerical stability at UTM coordinates. Yaw uses a 10 m lever
arm for numerical scaling. The defaults describe a local prediction basin
(3 m per translation axis, 12 degrees), not global relocalization.

Empty/incompatible support, failure to converge, search-boundary solutions,
weak overlap, or degenerate scaled curvature reject a result.
`localizeLidarFrame` publishes an observer-compatible pose event only after
acceptance. Acquisition time and delivery time remain distinct; the caller sets
actual `arrivalTime` before fusion. The objective Hessian is an uncalibrated
observability diagnostic and is deliberately not supplied as sensor information.

## Evaluation and limits

The reference implementation is commit
`1ac86ba5c3c92021c83934f750ebe93546b36d8d`; development snapshots were captured
before edits. Original reference MAT data remain unchanged. Precision/recall
below mean **agreement with that detector**, not ground-truth accuracy.

Development: Mississippi 260, 300, 326, 370, 450, 550, 700, 850, 1000, 1150,
plus Downtown 400. Diagnostic expansion: 150, 600, 900, 1100; these exposed and
informed the footprint correction, so they are not an untouched validation set.
Final checks: 200, 500, 800, 1050, not used to adjust this redesign. Frames 500 and
800 also appeared in the earlier coarse-cloud study, so no cross-study holdout
claim is made. All use the same physical defaults; Downtown is only one transfer
sanity check, not evidence of broad generalization.

| Dataset | Feature | Shared points | Added vs baseline | Removed vs baseline | Precision | Recall |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| mississippi | curb | 13954 | 0 | 0 | 1.0000 | 1.0000 |
| mississippi | roadMarking | 1649 | 0 | 0 | 1.0000 | 1.0000 |
| mississippi | pole | 3189 | 64 | 67 | 0.9803 | 0.9794 |
| downtown | curb | 103 | 0 | 0 | 1.0000 | 1.0000 |
| downtown | roadMarking | 24 | 0 | 0 | 1.0000 | 1.0000 |
| downtown | pole | 273 | 0 | 4 | 1.0000 | 0.9856 |

Across the 18 Mississippi frames, warmed median-of-three runtimes have medians
149.0 ms (historical full perception),
124.5 ms (coarse), and 127.3 ms (offline).
Coarse per-frame medians range from 94.8 to
130.5 ms. Median retained outputs are
171.34 MB, 34.00 kB, and
332.63 kB respectively (decimal units). These results do
not establish a sustained 10 Hz deadline, especially after D2D and sensor I/O.
MATLAB R2026a Update 3 ran in the existing desktop session; timings exclude file
loading and plotting and vary with session load. Fine-filter cost is small beside
shared geometric analysis, so noise can make isolated offline runs faster.

All 21 saved-map perturbation cases were accepted. Their translation differences
from mapping poses range 0.002--0.300 m;
absolute heading differences range 0.158--2.027 degrees.
The new map excluding query frame 260 produced 0.182 m and
0.671 degree differences, and was accepted.
Median D2D solver runtime over all 22 cases was
19.5 ms. Convergence and
acceptance here are engineering observations, not independent accuracy certification.

Machine-readable artifacts: [point fidelity](results/pillar_redesign_20260904/feature_fidelity.csv),
[timing/storage](results/pillar_redesign_20260904/runtime_and_storage.csv),
[map consistency](results/pillar_redesign_20260904/registration_consistency.csv).
Intermediate failure tables are retained as `initial_*.csv` in the same folder;
they describe explicitly superseded prototypes, not the final algorithm.


Final verification: **54 tests passed, zero failed, zero incomplete**, including
historical perception/mapping, the new execution boundary and D2D mathematics,
and both observer suites. Code Analyzer reported no remaining issues across 33
changed MATLAB files after one indexing suggestion was corrected. A profile of
frame 260 contained 223 functions and zero matches for the six prohibited
point-feature refinement function prefixes. The coarse display colors candidate
pillar membership over all 65,536 finite source points; it does not claim those
colors are fine point decisions. See [test outcomes](results/pillar_redesign_20260904/test_results.csv)
and [the profile audit](results/pillar_redesign_20260904/online_profile_audit.json).

The candidate-support tests intentionally allow a modest precision tradeoff:
curb source precision floor 0.69 -> 0.65 and NDT precision floor 0.80 -> 0.79;
pole source floor 0.58 -> 0.55 on dispersed frames. Complete candidate footprints
and retained topology add support for offline testing. Recall bounds remain,
and a separate 18-frame point test enforces exact curb/marking agreement,
aggregate pole precision/recall >=0.95, and per-frame pole F1 >=0.80.

The saved-map registration experiment uses frames 150, 260, 300, 326, 600, 900,
1100 with exact, [+0.5,-0.4,+2 degrees], and [-0.5,+0.4,-2 degrees] initial
perturbations. These scans belong to the same trajectory that created the map:
map-pose differences are consistency measurements, not independent accuracy.
The second experiment rebuilds a six-frame map from 261:266 through the new fine
path, then registers frame 260, which is excluded from map fitting. It remains
same-route evaluation. No complete trajectory observer replay is claimed here.

Reproduction:

```matlab
setupVehicleLocalization();
setenv("VEHICLE_LOCALIZATION_DATA_ROOT", fullfile(pwd,"data"));
runtests("tests");
evaluatePillarPerception();
evaluatePillarRegistration();
showMississippiPerception(260,"","coarseProbabilityCloud");
```

The design simplifies stage boundaries and several redundant gates; it does not
eliminate the retained curb topology parameters or sensor-specific radiometry
assumptions. No labeled cross-sensor benchmark, automated radiometric calibration,
confidence calibration, 6-DoF recovery, automatic map-window selection, or hard
real-time guarantee is established. Those require separate experiments. The
reported output sizes are retained MATLAB values, not peak process memory.

Research assistance: literature triage, implementation, and experiment reporting
used coding-agent assistance. Every reported numerical result comes from an
executed local experiment; unsuccessful prototypes are identified as such.
