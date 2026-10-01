# Why continuous Gaussian matching regresses

The continuous anisotropy principle is sound, but the September 30 implementation
changed the observation model, association and information scale together.
It treats a locally visible Gaussian's center as the center of a complete map
Gaussian, weakens coherent neighborhood direction evidence, and lets the resulting
weak centroid constraints pass the existing relative observability threshold.
The optimizer then prefers a displaced pose. A separate shape-residual
linearization defect is real, but correcting it does not remove that displacement.

This study diagnoses the explicit `anisotropicD2D` experiment at source commit
`322eb398329f1e95491ae53b2d8061b114851d90`. Production code and the
`geometricD2D` default are unchanged. No new complete route replay or deployed
accuracy improvement is claimed. The preceding study already established the
1170-frame maximum increase from **15.986 cm to 58.028 cm**. These experiments
isolate its causes on frames 178, 894 and 932 using the same frozen inputs.

## Experimental contract

`diagnoseAnisotropicRegression` uses the raw-verified five-scan source windows,
the existing view-conditioned map, the preceding default pose export and the
recorded evaluation poses. For each frame:

- Seed from that frame's preceding default output; crop the map once and condition
  it once at that seed. The seed is an offline diagnostic initialization, not a
  new causal replay prediction. Source membership, points and semantic labels are
  identical between methods.
- Disable source/map pyramid merging to expose the original component factors.
  Dynamic old and new controls reproduce the public fine solver to `1e-7` in pose.
- Freeze old/new correspondences and base quality weights at the evaluation pose
  for factor interventions. These association anchors are **offline oracles**;
  the reference is not an online input or a newly obtained physical annotation.
- Use the same iteration budget, pose bounds, line search, yaw scaling and
  observability projection as the fine pose solver. Custom controls do not run
  the public acceptance gates or coarse search; their errors are diagnostic fits.
- Check all 51 geometry-control gradients against frozen-cost finite differences,
  plus 30 threshold controls, cost-preserving shape splits, solver parity and
  saved-result assertions. The controls use no random draws.

The original source caches were checked against raw perception in the preceding
study. This study does not rerun perception, motion estimation or the whole route.
The reference evaluates poses and anchors selected diagnostic pairs. Shared-recording
map/query, empirical calibration and detector-derived map labels remain limitations.

## 1. Weak center attraction does not preserve the prediction

The new pair metric is

\[
 q=\delta^T S^{-1}\delta+h,\qquad
 S=R\Sigma_sR^T+\Sigma_m+\sigma^2I,
\]

where `delta` is the transformed source center minus the target center and `h`
is the normalized Gaussian overlap's shape penalty. Increasing anisotropy reduces
the long-axis positional weight, as requested. However, it does not make a biased
center correspondence unbiased. For the elementary cost

\[
 E(x)=w(x-\mu)^2,\quad w>0,
\]

the optimum is `mu` for every positive `w`. In a Newton step the common weight
cancels between gradient and curvature. A flat objective can still have its
minimum far from the prediction. Motion fusion or an observability projection
must actually prevent unsupported displacement; weak weighting alone cannot.

The source cloud is a local visible subset, often transported over five scans.
The target is a larger accumulated Gaussian. A curb pillar and another patch of
the same continuous curb can have different centers without representing different
roads. Partial views of a sign likewise need not share the complete panel center.
The previous model partly protected these cases through line-normal residuals
and a partial-sign treatment. The new model restored finite center attraction
along every axis while removing those protections.

Frame 894 demonstrates this directly. Its median combined curb anisotropy is
**105.31**, so the issue is not an absence of elongated covariances. Nevertheless,
the dynamic fit's longitudinal discrepancy rises from **2.45 cm to 55.61 cm**;
its lateral discrepancy only changes from **14.56 cm to 16.59 cm**. With new
pairs frozen, removing the curb tangent positional term reduces total error
from **48.12 cm to 18.98 cm**. This intervention retains the Gaussian shape term
and all source membership/base quality weights.

At the evaluation pose, almost all of the new yaw gradient comes from transformed
center positions. For frame 894 curb factors, the scaled-yaw gradient components
are `+0.24295` from center displacement, `-0.0004544` from covariance rotation,
and `-0.0004577` from the shape term. Frame 178 has the same pattern for both
curbs and signs. Gradient parts add exactly; their squared information matrices
contain cross terms and must not be summed as an additive information decomposition.

## 2. A genuine sign produces a biased partial-center constraint

All three frame 178 sign components associate to the same map component, global
ID 1074. One source component, ID 22, was treated as a partial panel in the old
model. At the evaluation pose it has:

| Quantity | Value |
|---|---:|
| Offset along the map cloud major axis | -41.72 cm |
| Offset along its minor axis | 6.36 cm |
| Combined position covariance eigenvalue ratio | 2.096 |
| Map within-acquisition covariance eigenvalue ratio | 10.317 |
| Map total covariance eigenvalue ratio | 2.221 |
| Within-acquisition versus total principal-axis difference | 16.459 degrees |
| Remaining Cauchy weight relative to base quality | 73.56% |

Repeated detection establishes that the sign exists; it does not establish that
the visible patch center is an unbiased observation of the complete sign center.
The accumulated total covariance combines within-view point shape and between-view
center dispersion. Here it makes the map cloud much rounder and rotates its axis.
The combined metric's tangent positional weight is about **48%** of its normal
weight, despite the physical panel's elongated within-view shape. The robust loss
does not discard this factor: its displacement is plausible under that broadened
Gaussian, so it retains most of its weight.

On identical new fixed pairs, restoring partial-sign treatment alone reduces
frame 178 error from 35.39 to 23.46 cm. Restoring neighborhood direction alone
reduces it to 10.56 cm. Restoring both gives **3.15 cm**. This is evidence for
interacting center and direction-model errors, not evidence of a newly false sign
or a groundtruth correction. The old sign treatment is an intervention, not a
proposal to abandon the requested uniform distribution-based design.

## 3. Local shape overlap replaces much stronger coherent direction evidence

The legacy curb direction uses multiple nearby source component centers. The new
shape factor compares the covariance of individual local pillars with the map
Gaussian. These have different support extents and statistical meaning. Shape
overlap does not inherit the old neighborhood direction's information strength.

For frame 894, individual source/map curb axes differ by a median **2.30 degrees**;
the coherent neighborhood/map axes differ by **0.75 degrees**. Even after repairing
the shape residual's Gauss-Newton factorization below, summed curb angular geometry
information is about **1.02 rad^-2**, compared with **1701.77 rad^-2** from the old
neighborhood factors. The old factors use an engineering angular scale near one
degree. Neither number is a calibrated posterior precision; the comparison shows
an approximately 1665-fold change in the objective's relative constraint strength.
It does not justify copying that old weight as a statistical truth.

| Fixed-pair intervention | Frame 178 | Frame 894 | Frame 932 |
|---|---:|---:|---:|
| New full overlap, new pairs | 35.39 cm | 48.12 cm | 15.65 cm |
| Remove shape pose cost | 35.52 cm | 48.38 cm | 18.62 cm |
| Freeze covariance and remove shape pose cost | 35.22 cm | 48.41 cm | 18.53 cm |
| Full overlap, old pairs | 32.16 cm | 16.24 cm | 15.76 cm |
| Restore neighborhood direction, new pairs | 10.56 cm | 13.63 cm | 14.28 cm |
| Restore direction and partial-sign treatment, new pairs | 3.15 cm | 13.63 cm | 14.28 cm |

Base quality weights and source membership are verified unchanged. Robust weights
respond to each changed residual as in production. These fits are not additive
error contributions. In particular, direction restoration can change the retained
observable rank, so it is not a clean measurement of direction accuracy alone.

Removing the shape cost or freezing rotated covariance scarcely helps the two
large regressions. Covariance derivatives and direct shape torque are therefore
not their dominant cause. Removing all coherent direction information from the
old fixed-pair model also worsens these frames; normal-only geometry is insufficient.

## 4. Association and a relative rank cutoff amplify weak centroid bias

At the evaluation pose, correspondences change for 6/30, 2/16 and 1/16 source
components in frames 178, 894 and 932. Source membership and base quality weights
are exactly the same. All frame 894 targets remain eligible line components under
the old rule, ruling out new isotropic/ineligible curb targets as the explanation.

In frame 894, source 3 changes from map component 683 to 687 and source 5 from 685
to 687. Their candidate centers differ by **4.52 m** and **4.41 m**, predominantly
along the road. The exported old association scores prefer the old targets; the
new scores prefer the new targets. Both position/shape overlap and mixture priors
enter these scores. These may be different patches of the same continuous curb,
not physically false curb detections. Their centers are nevertheless not
interchangeable point landmarks. Holding the old pairs reduces the new fit from
48.12 to **16.24 cm**, including a change in observable rank.

The scaled information matrix uses yaw scale 0.1 and the existing relative cutoff
`lambda_min/lambda_max > 0.01`. At frame 894's dynamic fits:

| Model | Smallest/largest eigenvalue | Retained rank |
|---|---:|---:|
| Legacy | 0.0422% | 2 |
| New full overlap | 1.1636% | 3 |

With new fixed pairs the ratio is 1.1007%; with old fixed pairs it is 0.9287%.
Weakening the large angular eigenvalue can increase this ratio even if no reliable
longitudinal landmark has appeared. The old numeric cutoff then accepts all three
degrees of freedom, including the biased weak road direction. The recursive
matching replay directly assigns an accepted event pose to its state; it does not
combine that direction with a calibrated motion prior. This claim concerns that
matching benchmark, not a new audit of the downstream global observer.

A threshold control confirms the mechanism. With identical new fixed pairs and
objective, increasing the cutoff from 1% to 2% changes rank from three to two and
reduces frame 894 error from **48.12 to 17.17 cm**. With neighborhood direction
restored, a 0.5% cutoff yields rank three and **46.02 cm**, while the original 1%
cutoff yields rank two and **13.63 cm**. Thus part of the apparent direction-factor
benefit comes from preserving the prediction in the weak direction. Raising a
cutoff is a protective restriction, not a repaired observation model or evidence
of a whole-route improvement. A 5% full-overlap cutoff retains only one direction
and returns about the seed's original error; that is not improved localization.

The cross-objective table provides another caution. At frame 178, modern cost is
1.01212 at reference, 0.97651 at the legacy fit, and **0.84845 at its 46.60 cm fit**.
It is optimizing its defined objective rather than simply failing to converge.
At frame 894 even the legacy dynamic objective is slightly lower at the new far
fit than at its own rank-projected fit. Weak-direction protection is important
in both models; lower matching cost is not sufficient evidence of better position.

## 5. The square-root shape residual loses second-order information

For unequal Gaussian eigenvalues, the shape cost separates as

\[
 h=h_0+\log(1+\kappa\sin^2\theta),\qquad
 \kappa=\frac{(a_\ell-a_s)(b_\ell-b_s)}{(a_\ell+b_\ell)(a_s+b_s)},
\]

where the eigenvalues include the half-noise regularization. The previous
implementation uses one residual `sign(sin(theta))*sqrt(h)`. When the Gaussian
scales differ, `h0 > 0`; at aligned axes that residual's local derivative is zero
while the true shape cost has `h''(0)/2 = kappa > 0`. Its sign also jumps at
alignment, although the scalar cost and gradient remain continuous. The positive
angular curvature is lost from `J'J` into an ignored residual-second-derivative
term. First-order objective-gradient checks and equal-shape tests missed this.

The research-only repair splits `sqrt(h0)` from a smooth signed square root of
the angular term. It preserves the scalar cost and gradient, but supplies the
correct positive Gauss-Newton curvature at alignment. A synthetic unequal-scale
example has original angular `J'J = 0` versus true/split information **2.81073**.
Cost and gradient agree to `1e-12` across the four synthetic rotations.

On actual curb pairs, the split raises summed angular information from
0.00337 to 0.72901 (178), 0.00689 to 1.02179 (894), and 0.00208 to 0.61907 (932).
The true half-curvature comparison freezes robust weights; it is not the full
Cauchy Hessian. All dynamic and fixed-pair fitted errors change by **less than
1 micrometer** after this repair. It is an implementation weakness to correct,
but it cannot account for the large biased optimum. The repaired Gaussian
direction term is still far weaker than the old neighborhood term.

## Design implications and remaining limits

Continuous anisotropy should be retained. A reliable replacement also needs a
consistent meaning for the compared distributions: partial visible support and
accumulated support cannot automatically be treated as the same centered feature.
Point scatter, between-acquisition center dispersion and uncertainty of a
feature's direction/position estimate are different quantities even if they are
represented within one probabilistic model. A covariance broadening cannot, by
itself, resolve a systematic partial-center bias or determine observation
confidence.

The next algorithm needs a partial-support observation model or marginalized
sliding coordinate, shape association at compatible support scale, direction
uncertainty that decreases continuously with anisotropy, and pose updates that
preserve a motion prediction where geometry is weak. These controls do not
establish a unique implementation or calibrated thresholds. Blindly restoring
class rules, inflating major eigenvalues or increasing a relative cutoff would
leave the underlying mismatch unresolved. The shape residual factorization
should also be repaired with unequal-scale second-order coverage.

Frame 932 remains about 14--16 cm under these local controls. The earlier study
identified historical translation transport and accumulated curb geometry bias;
this turn does not remove those causes. Its preceding recursive new-mode error
of 7.74 cm is a trajectory-dependent result, not its isolated factor accuracy.
No universal accuracy floor, physical false-feature annotation, new detector
precision/recall, or independent-drive validation is inferred.

## Reproduction and artifacts

```matlab
addpath(pwd);
addpath('research/anisotropic_regression_20260930');
diagnoseAnisotropicRegression;
plotRegressionDiagnosis;
checkRegressionDiagnosis;
```

The scripts require the local cached inputs listed in `input_hashes.json`.
They overwrite this study's CSVs/plot and ignored local MAT product only.
`validation.json` records the completed checks; three diagnostic MATLAB files
have zero factory Code Analyzer findings. The earlier implementation's 336 tests
are not rerun or counted as fresh validation here. An initial final check found
three analyzer messages; unused output, transpose-product parentheses and a
stale suppression were corrected before the successful final check.

- `controls.csv`, `observability_controls.csv`: factor and threshold interventions.
- `reference_forces.csv`, `reference_geometry.csv`: gradient and distribution audit.
- `associations.csv`: changed candidates and both methods' association scores.
- `shape_curvature.csv`, `synthetic_curvature.csv`: second-order shape evidence.
- `cross_objectives.csv`: each model's cost at reference and the two fitted poses.
- `causal_controls.png` / `.pdf`: static control and information charts.
- `findings.json`, `validation.json`, `code_analysis.csv`: compact findings/checks.
- `input_hashes.json`, `artifact_hashes.json`: independent technical inputs and
  exports; no weekly/monthly report hashes.

Large recordings/MAT data remain local. This directory contains the complete
in-scope diagnostic implementation; unrelated observer, lattice, documentation
and instruction changes are excluded from its project commit.
