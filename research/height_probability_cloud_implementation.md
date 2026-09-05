# Height-preserving semantic probability clouds

Implementation and experiments: 2026-09-04 through 2026-09-05, America/Chicago.
Reference: `8ff55cbb41c785b7e7755ce41c8df4a554ad7f2c`.
This implements the retention decision in
[the height study](pillar_height_retention_decision.md).

The subsequent [bias diagnosis](height_registration_bias_diagnosis.md) separates
a vertical geometry inconsistency from view-dependent planar density alignment
using 435 controlled registrations and original-bag checks.

## Delivered behavior

Online coarse perception now retains XYZ Gaussian statistics, including xz and
yz covariance, without changing candidate pillars or performing point-feature
validation. Newly built offline maps retain height too. Existing XY fields and
map queries remain compatible. An explicit experimental XYZ D2D path is
implemented and tested mathematically, but **default localization still uses
XY** because recorded perturbation experiments did not justify enabling XYZ.

This separates the supported improvement (retaining information at modest
cost) from an unestablished claim (more accurate localization from direct XYZ
overlap). The two are not conflated by the default configuration.

## Data contract

The normal coarse/map export keeps its existing `components.mean` [N,2] and
`components.covariance` [2,2,N] as the XY marginal. It adds:

| Field | Shape | Meaning |
| --- | --- | --- |
| `components.meanXYZ` | N by 3 | Spatial mean in the declared frame |
| `components.covarianceXYZ` | 3 by 3 by N | SPD spatial scatter with the exact existing XY block |
| `components.heightAvailable` | N logical | Whether actual height statistics are present |
| `spatialDimension` | scalar | Spatial dimension; `dimension=2` describes the compatible primary arrays |
| `spatialCoordinateFrame` | string | Sensor XYZ after the supplied projection rotation, or map XYZ |
| `heightModel` | string | Joint source Gaussian, conditional map Gaussian, or unavailable |

`projectSemanticProbabilityCloud(cloud,3)` returns ordinary `mean` [N,3] and
`covariance` [3,3,N] arrays. Dimension 2 returns the exact marginal. Both retain
mixture mass. Unknown legacy height is flagged rather than fabricated; explicit
XYZ projection fails when required height is unavailable. Empty products remain
valid. XYZ validation checks dimensions, finite geometry, symmetry, positive
definiteness, and agreement with the stored XY marginal.

The existing `aggregatePlanarCellMoments` name is retained for compatibility.
XYZ input adds `meanZ` and packed `heightCovariance=[xz,yz,zz]`; XY input remains
supported. Known tilt acts on XYZ before accumulation. Ground-feature moments
come from ground members of candidate pillars, and pole moments from eligible
non-ground structural members. The visual candidate membership is not a fine
point label.

## Source covariance and exact XY preservation

Height moment matching includes both within-pillar and between-pillar scatter.
The implementation recenters height residuals at each output-cell mean rather
than subtracting large squared height coordinates. Let A be the existing
regularized XY block, b the empirical xz/yz cross vector, and c the empirical
z variance. Complete the spatial covariance as

\[
\Sigma_{XYZ}=\begin{bmatrix}A&b\\b^T&c_*\end{bmatrix},\qquad
c_*=\max(c+\lambda,b^TA^{-1}b+\epsilon_z).
\]

Here lambda is the existing regularization variance and epsilon_z is the new
minimum conditional height variance (1e-4 square meters by default). The Schur
complement is positive, and the XY block does not change. This does not claim
an eigenvalue floor of epsilon_z for the entire 3D matrix. No raw point
distribution covariance is divided by its support count a second time.

The synthetic one-row pillar fixture exposed an existing MATLAB vector-shape
implicit expansion in source-cell selection. Selectors now consistently use
column vectors for row/column coordinates, counts, and probabilities. Ordinary
recorded raster outputs remain exactly equal to the reference.

## Offline map height model

`assembleMapInput` now preserves `pointsXYZ` in addition to its compatible XY
array, and `buildSlidingWindowMap` passes XYZ to the temporal map builder.
Its original XY patching, temporal-reliability resampling, EM, pruning, and
query logic are unchanged.

After the final XY components exist, `fitConditionalHeight` fits

\[
p(z\mid u,k)=\mathcal N(z;a_k+\beta_k^T(u-\mu_{u,k}),s_k^2),
\quad u=(x,y),
\]

by weighted least squares on the same resampled XYZ returns, using the final
XY GMM responsibilities. Coordinates and height are recentered for numerical
stability. A pseudoinverse handles underdetermined horizontal slope directions,
and residual variance has a positive floor. For XY covariance A, the resulting
joint covariance is

\[
\begin{bmatrix}A&A\beta_k\\\beta_k^TA&s_k^2+\beta_k^TA\beta_k\end{bmatrix}.
\]

The layer retains `componentMeansXYZ` and `componentCovariancesXYZ` next to
the existing XY components. Height fitting does not change the XY GMM or feed
back into its EM objective. The height distribution is unimodal conditional
on each existing XY component; it does not discover additional vertically
separated mixture modes at the same XY location.

`temporalMapToProbabilityCloud` exports this joint distribution with the same
mixture mass as its XY marginal. Because p(z|u,k) integrates to one, the map's
existing planar integrated-support weight is preserved. This differs from a
new independently peak-normalized 3D support map, whose amplitude-to-mass
conversion would require the 3D volume factor. Multiplying our conditional
lift by another height-extent factor would double-count height normalization.
The original noisy-OR/max query remains an XY support query; D2D still uses
the explicitly documented one-window sum surrogate.

Old saved XY maps cannot reconstruct Z. They remain readable, with
`heightAvailable=false`. Three new local maps were built for validation; the
existing full-route saved map was not overwritten or represented as rebuilt.
New full-route builds through `buildFeatureMap` use the height-preserving path.

## Registration and vertical reference

`semanticGaussianOverlap` dispatches planar and spatial overlap. The spatial
kernel evaluates the exact Gaussian integral with summed covariances, including
analytic x/y/yaw derivatives of rotated xz/yz terms. The optimizer and observer
event remain planar; no height state is added to the observer.

`prepareSemanticRegistration` resolves the representation and adds optional
moving-frame height/tilt uncertainty before comparison. A finite
`heightTranslation` is the sensor/vehicle origin's map Z. It is now the third
output of `poseRowToPlanarPose`; it is not guessed from cloud means.
The roll/pitch perturbation Jacobian of a gravity-aligned component mean is
used to propagate the specified small-angle tilt uncertainty. These are
per-component covariance approximations, not a calibrated correlated pose
likelihood or full integration over common scan-level nuisance variables.

Configuration:

- `heightMode="xy"` is the production default, even with height available.
- `heightMode="xyz"` explicitly enables the experimental path and requires
  complete source/map height and a finite vertical reference.
- `heightMode="auto"` is also opt-in: it uses XYZ only when those inputs exist,
  otherwise it chooses XY. Availability is not an accuracy/visibility test.
- `heightStandardDeviation=0.20` m and `tiltStandardDeviation=0.5` degrees are
  engineering experiment defaults, not measured sensor specifications.
- `result.height` reports the selected dimension, reference, uncertainties,
  and reason. An accepted result is still subject to the existing uncalibrated
  convergence/overlap/curvature checks.

Example for deliberate research evaluation, after rebuilding a height map:

```matlab
[predictedPose, tilt, height] = poseRowToPlanarPose(poseRow);
cfg.perception = perceptionConfig();
cfg.perception.coarseProbabilityCloud.projectionRotation = tilt;
cfg.registration = distributionRegistrationConfig();
cfg.registration.heightMode = "xyz";
cfg.registration.heightTranslation = height;
[measurement, result] = localizeLidarFrame( ...
    frame, mapCloud, predictedPose, acquisitionTimestamp, cfg);
```

## Executed validation

The existing MATLAB R2026a Update 3 desktop ran the full test directory:
91 passed, zero failed, zero incomplete. Final targeted reruns cover the height,
D2D, temporal-map, and stored-pipeline tests, including two added cases. The
combined latest results contain **93 passed, zero failed, zero incomplete**.
The new tests cover empirical tilt/cross covariance, empty/singleton support,
within/between-pillar moment matching, exact map marginal and weight
preservation, analytic derivatives, large map coordinates, vertical-reference
consistency, legacy maps, explicit rejection of unavailable height, height
discrimination, uncertainty softening, and invalid retained covariance.

### Perception regression and runtime

`evaluateHeightPerceptionRegression` compares an immutable export of the
reference commit with current code, using the same compiled native kernels.
It alternates implementations for three `timeit` estimates per frame/path,
excluding data loading and path changes. Median timings are:

| Mississippi frame | Before (ms) | With height (ms) | Added (ms) |
| --- | ---: | ---: | ---: |
| 260 | 57.802 | 59.674 | 1.872 |
| 550 | 70.388 | 71.022 | 0.634 |
| 900 | 72.202 | 73.741 | 1.539 |

For all three frames, candidate products, offline fine masks, and every
previously existing Gaussian component field are exactly equal. Timings are
warmed desktop measurements, not a hard deadline or a broad distributional
latency claim. The earlier three-frame standalone-accumulator timings measured
a narrower operation and should not replace this full coarse-path comparison.

### Map and registration experiment, including negative results

`evaluateHeightProbabilityCloud` builds three six-frame maps from 261:266,
551:556, and 901:906, excluding query frames 260/550/900 respectively. Each
query uses two initial offsets, [+0.5,-0.4,+2 degrees] and its negative. Both
XY and XYZ modes use identical maps, semantic supports, and optimizer settings.
Five vertical-reference/extra-pitch cases are tested: (0 m,0 degrees),
(+0.1 m,0 degrees), (-0.1 m,0 degrees), (+0.3 m,0 degrees), and
(+0.3 m,+0.5 degrees). Pose differences reference the mapping trajectory,
not independent ground truth. No random perturbations or post-hoc tuning
were used.

Of 30 XY cases, ten were accepted, all at frame 260, with translation
differences 0.182--0.190 m. The six-frame windows at 550 and 900 mostly drove
the old XY objective to its search boundary and were rejected; this is a
limitation of this small-window experiment, not a successful route benchmark.

XYZ accepted 14/30 cases. At frame 260, nominal XYZ translation/yaw differences
were 0.356 m/1.318 degrees versus XY 0.182 m/0.671 degrees. At frame 550,
the +0.3 m perturbations produced four XYZ acceptances with 1.762--2.629 m
translation differences, despite XY rejection. The same engineering gates
therefore do not justify promoting XYZ. Median solver runtimes across all
cases were 22.0 ms XY and 28.1 ms XYZ, excluding perception and loading.

The production default was explicitly kept at XY in response to this result.
No claim of improved localization, calibrated acceptance, or robust vertical
nuisance estimation is made. A future promotion requires visibility-aware
coarse/fine distribution modeling, trustworthy vertical reference uncertainty,
and independent held-out evaluation, including incorrect-acceptance rates.

## Artifacts and reproduction

- [Regression/performance table](results/height_implementation_20260905/perception_regression.csv)
- [All 60 registration cases](results/height_implementation_20260905/registration_height.csv)
- [Tilt-aware coarse timings](results/height_implementation_20260905/coarse_runtime.csv)
- [Latest test results](results/height_implementation_20260905/test_results.csv)
- [Code Analyzer results](results/height_implementation_20260905/code_analyzer.json)

The ignored `output/height_retention/height_map_evaluation.mat` contains the
three rebuilt maps and exact perturbation settings. Its source scripts are
versioned; raw LiDAR data, map binaries, native binaries, and reference exports
are deliberately excluded from the public commit.

```matlab
setupVehicleLocalization();
setenv("VEHICLE_LOCALIZATION_DATA_ROOT",fullfile(pwd,"data"));
runtests("tests");
evaluateHeightProbabilityCloud(fullfile(pwd,"output","height_retention"));
evaluateHeightPerceptionRegression(referenceSourceExport, ...
    fullfile(pwd,"output","height_retention"));
```

Code Analyzer reported no issues on 23 changed MATLAB files. The source export
used for the native performance comparison included a link to the same local
MEX binary. The build did not require changing that binary. Unrelated observer
file relocations, agent instructions, reference material, and other user work
were preserved.
