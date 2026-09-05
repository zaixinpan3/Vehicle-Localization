# Repeated-observation semantic Gaussian field

Implementation decision, September 5, 2026. Schema version 2 replaces temporal
Bernoulli sampling, ordinary mixture fitting, post-hoc support, and pruning/refit
with a hierarchical observation model. This is a new statistical target; old
noisy-OR scores and localization thresholds are not interchangeable with it.

## Estimation target and inputs

The target is geometric repeatability conditional on registered semantic
observations and the chosen observation blocks. It is neither occupancy nor
permanent object existence. Positive feature detections provide no visibility
or free-space model. A stationary object observed during a short interval may
be repeatable even if it subsequently moves.

`buildTemporalStabilityGmmMap(X, labels, observations, cfg)` accepts XY or XYZ.
XY must be finite; missing Z is NaN. The observation argument is a frame-ID
vector or a struct with `frameId`, `observationBlockId`, and optional `sourceId`.
Source IDs refer to original observations, not independent evidence. Repeated
IDs must agree in coordinates, frame, and block within a class. Source frames
must belong to one block. Provenance retains original rows, frame IDs and source
IDs for every representative. With a bare frame vector, generated source IDs
are input row identifiers, so callers needing persistent cross-call provenance
must supply IDs explicitly.

The default is one source frame per block. Numeric frames can instead use fixed
groups defined by `observationBlockSize` and `blockOriginFrame`; explicit block
IDs take precedence. Independence is an acquisition assumption and must be
calibrated against motion, frame rate, and correlated registration errors.

Within each class and block, the point closest to each 0.10 m XY voxel center
is retained, with lexical XY tie breaking. Height does not affect XY selection.
Duplicate returns cannot change that selection. Retained observations are the
likelihood's sampling units; raw return count is not an independent vote for
repeatability. More distinct representatives within a block improve geometric
precision; more independent blocks inform repeatability.

## Local generative model

For component k and observed block b,

\[
H_k\sim\mathrm{Bernoulli}(\rho),\quad
\mu_k\sim N(0,P_0),\quad u_{kb}\mid H_k=h\sim N(0,Q_k^h),
\]
\[
p_b(x)=\epsilon_b/|\Omega|+(1-\epsilon_b)
       \sum_k\pi_{bk}N(x;\mu_k+u_{kb},C_k),\qquad x\in\Omega.
\]

Coordinates are centered on the tile before inference. The proper mean prior
has standard deviation 8 m by default. `C` is within-block retained-return
spread including measurement noise; it is not an independently identified
intrinsic surface covariance. Its eigenvalues are constrained to [0.0025,16]
square meters. No extra epsilon inflation is applied in its M-step.

Defaults set stable offset standard deviation to 0.15 m and alternative added
standard deviation to 1 m. Compact features use `Qv = Qs + tau^2 I`. Elongated
features use `Qv = Qs + tau^2 nn'`, allowing a common 1 m tangential displacement
under both hypotheses. Normals and the elongated/compact decision come from
observed within-block scatter of the deterministic initialization. They remain
fixed within each optimization so the monitored objective stays fixed.
These values are engineering defaults, not fitted sensor or registration
calibration. Uniform background is proper over the fixed tile-plus-halo square.

Observation weights have a Dirichlet prior with entries `1 + 0.1/K`, and
background fractions have a Beta(2,10) prior. They are optimized by MAP updates.
Neither is used as a localization class-importance weight. Variable components
and the background remain available for the entire fit.

## Structured variational objective

The approximation is

\[
q(A)\prod_k q(H_k,\mu_k,U_k).
\]

Assignments are factorized, but each component's hypothesis, common mean and
all block offsets remain coupled. For fixed responsibilities, the entire
Gaussian hierarchy is integrated analytically under each hypothesis. Means
and offsets are not independently approximated and then scored after fitting.

For fractional component weights, define `n_b=sum_i gamma_bik`, weighted mean
`y_b`, and centered scatter `T_b`. With `n_b>0`, let

\[
W_b=C/n_b+Q^h,\quad L=P_0^{-1}+\sum_b W_b^{-1},\quad
g=\sum_b W_b^{-1}y_b.
\]

The exact fractional-likelihood normalizer is

\[
\begin{split}
\log Z_h={}&-\tfrac12\sum_b\{n_b[2\log(2\pi)+\log|C|]
 +\operatorname{tr}(C^{-1}T_b)+\log|C+n_bQ^h|-\log|C|\}\\
 &-\tfrac12\{\log|P_0|+\log|L|+
 \sum_b y_b^TW_b^{-1}y_b-g^TL^{-1}g\}.
\end{split}
\]

`W_b^-1` is computed as `n_b*(C+n_b*Qh)^-1` to avoid dividing by tiny
responsibilities. Zero-mass blocks contribute exactly zero under both
hypotheses. The posterior reference mean and covariance are `L^-1*g` and
`L^-1`. Conditional offset moments also retain their covariance with the mean.

The variational repeatability estimate is

\[
r_k=\frac{\rho Z_S}{\rho Z_S+(1-\rho)Z_V}.
\]

For unknown associations this is an approximate posterior, conditional on the
fitted covariance parameters, fixed geometric projectors, local model order,
and acquisition assumptions. It is not automatically calibrated.

After optimizing each structured factor, the monitored ELBO is the sum of
`log(rho*ZS+(1-rho)*ZV)`, assignment-weight terms, assignment entropy, and
Dirichlet/Beta log priors including their normalizers. The assignment step uses
expected squared residuals including uncertainty in `mu+u`; the covariance
step clips the eigenvalues of the expected residual second moment. The latter
is the constrained maximum for the stated covariance objective.

Each iteration updates assignments, structured factors, covariances and MAP
observation weights, then reoptimizes the structured factors. A material ELBO
decrease is an error. Stopping requires both relative objective progress and
relative changes in covariance, observation weights, assignments, hypothesis
probabilities, reference means/covariances and block location moments.
Iteration-limit termination is
retained as `converged=false`; publication is not a convergence certificate.
The dense stacked-covariance unit test independently checks the hypothesis
ratio for fixed fractional associations.

## Complexity, ownership and publication

The default nonoverlapping ownership tiles are 8 m squares with 2 m context
halos and a fixed origin. Proposals use deterministic farthest-point/Lloyd
initialization within each semantic tile. One through three components are
considered by default. When at least three blocks are available, every third
block is held out from candidate fitting. Selection averages marginal
predictive log scores within each held-out block and then across blocks. It
uses training-only nuisance-weight averages and integrates a fresh block's
offset under both hypotheses. It is explicitly a **composite marginal
predictive score**, not the joint density of a held-out block. A new component
must improve the score by 0.01 per representative. Selected complexity is
refitted on all construction blocks. These selection blocks cannot also serve
as independent evaluation data. With insufficient blocks, one component is
used and selection is labelled unavailable.

Every tile uses a union of unique representatives. Halo data provide context;
only owned reference cells contribute mass. Canonical component IDs include
tile coordinates and the local component index, scoped by semantic class.
Separate tiles are local fits, not an exact global joint posterior; changing
tile geometry changes that approximation. Frame-window stride does not.

Each unique owned 0.25 m reference cell has one area budget, `resolution^2`.
The cell's mean assignment vector divides its area among Gaussian candidates
and background. Background keeps its share; unexplained cells are not forced
to increase a surviving Gaussian's mass. For every cell,

\[
\sum_k a_{jk}+a_{j,\mathrm{background}}=\Delta^2.
\]

Summing allocated cell areas gives each candidate's `referenceMass = a_k`.
The layer's `referenceMass` includes background and unpublished geometry;
`backgroundReferenceMass` makes the distinction explicit. Reobserving the same
cells does not create another area budget, although new observations can
change its allocation and repeatability estimate.

Publication requires at least two blocks with effective assignment count at
least 0.5, adequate total assigned representatives, and
`r > C_FP/(C_FP+C_FN)` (0.5 by default). One-block components remain
`unconfirmed`. All fitted candidates remain in offline diagnostics; only
published components contribute field mass `m_k = a_k*r_k` and exported clouds.
Unpublished candidates have zero deployment mass. There is no pruning/refit
that transfers rejected observations back to surviving geometry.

`buildSlidingWindowMap` ingests the union of scheduled frames once. It rejects
stride greater than count and shifts the final window to avoid a short tail.
`canonicalMap` stores the field, while `batchMaps` stores only scheduling and
first-ingestion provenance. Fits run a tile at a time, but input observations
and compacted representatives remain in memory. This is not an out-of-core
implementation. The standalone semantic NDT baseline retains original
coordinates and now provides block/cell counts, means and centered scatters;
its covariance sums use local cell coordinates for numerical stability.

## One deployment field

For published candidates,

\[
\Sigma_k^{pred}=C_k+Q_k^S+\operatorname{Cov}_q(\mu_k\mid H_k=S),\quad
\Lambda_c(x)=\sum_{k\in c}m_k\phi_2(x;\hat\mu_k,\Sigma_k^{pred}).
\]

The future measurement spread is included once. Point queries return
`Lambda/(Lambda+kappa)` for a positive class clutter intensity. This is a
reference feature-versus-clutter interpretation conditional on an observation
at x. Class importance is separately stored metadata for a downstream loss;
it does not alter map mass or clutter. Existing registration objectives and
their own class-balancing policies are not changed by this map revision.

Valid coverage is the union of owned fitting regions, explicitly not a sensor
visibility model. Scores outside coverage or for unknown classes are NaN, with
`details.valid=false` and a named status. Known coverage with no published
geometry returns zero and `noPublishedStructure`. Empty maps and queries are
valid states. `details.intensity` remains the Gaussian field, including tails,
and must not itself be treated as a coverage indicator.

The export stores global `mixtureWeight=m_k/sum(m)` and `totalMass`, plus
`classMixtureWeight=m_k/M_c`, `classTotalMass`, clutter intensities, class
importance, coverage, and covariance semantics. Either representation exactly
reconstructs the corresponding untruncated intensity. Splitting an identical
Gaussian and partitioning its mass leaves that field unchanged. Remote map
growth does not renormalize local query support downwards.

For indexed evaluation, each positive component may omit at most `epsilon/K`
intensity. Its Mahalanobis cutoff is
`R_k^2=max(0,2*log(peak_k*K/epsilon))`; projected ellipsoid extents populate a
spatial cell index. Very large extents use a global candidate list to bound
index memory. Zero tolerance evaluates full Gaussians. Thus omitted intensity
is at most epsilon, and score error is at most `epsilon/kappa`. Finite
truncation can introduce small discontinuities bounded by this policy. Index
geometry, mass and tolerance snapshots detect stale indices. Recompile with
`mappingSupport.compileFieldIndex` after deliberate field edits.

## Height and compatibility

Height is a separate weighted conditional regression with XY associations and
geometry held fixed. It uses finite unique height observations within each
retained XY voxel and divides that representative's weight among them. The
joint covariance is `[Sigma, Sigma*b; b'*Sigma, sigma_z^2+b'*Sigma*b]`, preserving
the XY marginal and integrated mass exactly. This is fixed-association
conditional fitting, not conditional-mixture EM. A documented residual-mode
diagnostic marks sufficiently separated two-mode height residuals as
`inadequateMultipleModes`; such components do not export fabricated height.
This diagnostic does not detect every non-Gaussian height distribution.

Legacy saved maps retain their original query and export paths. Legacy cloud
exports remain labelled `sumSurrogateForNoisyOrAndMax`, with their historical
prior semantics. They are not relabelled as schema 2. Rebuild from source
observations to obtain the new field. Old builder sampling/pruning/prior
parameters are rejected rather than silently reused. On new scheduled maps,
the exporter always uses the canonical field; an optional batch index validates
a schedule entry and does not select a distinct statistical map. New cloud
`sourceBatchIndex` is zero. Old maps still select exactly one legacy window.

## Validation and remaining empirical questions

`temporalStabilityGmmMapTest` exercises the dense marginal-likelihood identity,
two-block confirmation, one-block nonpublication, normal versus tangential
variation, input permutation and duplicate-density invariance, explicit block
grouping, conserved cell mass, split-mass and remote-growth invariance,
query/export reconstruction, indexed error bounds, coverage, height inadequacy,
held-out complexity, and schedule ownership. Existing height and registration
tests retain legacy export coverage. The recorded regression retains the
original perception reference and builds the new field from Mississippi
frames 260--289; old noisy-OR numerical map targets are intentionally no longer
the expected output. The original reference MAT file is not rewritten.

`evaluateRepeatedObservationMap` writes actual recorded-map diagnostics,
including iteration-limit counts and query/export reconstruction error. These
are engineering validation artifacts. They do not establish route-level
localization accuracy, posterior calibration, independent-block validity or
online runtime suitability. Those require a separate locked evaluation set
and acquisition/registration calibration.

Structured variational inference is grounded in the optimization framework
reviewed by [Blei, Kucukelbir and McAuliffe (2017)](https://arxiv.org/abs/1601.00670).
Anisotropic probabilistic geometric residuals have precedent in
[Segal, Haehnel and Thrun, Generalized-ICP (2009)](https://www.roboticsproceedings.org/rss05/p21.html).
Neither source validates this particular implementation or its default
hyperparameters; its hierarchy, mass policy and coverage semantics are the
project's explicit design choices.
