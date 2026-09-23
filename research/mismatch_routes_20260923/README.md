# Routes to reduce semantic map-association errors

Research date: September 23, 2026. Code audited at
`375f6fe7db8b85d13ebcc124d45c26a302d4ed76`.

**Recommendation:** improve association before replacing the localization
backend. First evaluate explicit unmatched alternatives and joint geometric
compatibility; then retain a small number of distinct association/pose hypotheses
when ambiguity persists. Preserve the current observer as the controlled
baseline. These are proposed experiments, not implemented improvements.

## Question and evidence scope

Which methods can reduce wrong-object associations while retaining whole-pillar
coarse perception, the existing stability-weighted Gaussian map, and useful
localization coverage and runtime?

This is a targeted narrative review and code audit, not a systematic review or
meta-analysis. Primary papers, author/institutional pages and the local Zotero
library were examined. [sources.json](sources.json) records what was actually
read, including abstract-only screening and inaccessible full text.
[search_log.json](search_log.json) records the final reproducible search pass.
It does not claim an exhaustive search history. No new localization replay or
algorithm implementation was performed for this review.

## What the existing experiment establishes

The [completed pose-graph experiment](../robust_pose_graph_20260923/README.md)
used all 1170 coarse scans and 1169 synchronized localization outputs. On the
common 1167 accepted matching frames, graph proposals changed LiDAR RMSE from
11.5060 to 11.5021 cm, a reduction of only 0.039 mm. The approximately 33.35 cm
matching peak at frame 840 remained. Replacing the observer with the switchable
graph increased full-sequence fusion RMSE from 8.2156 to 10.9862 cm.

This establishes lack of benefit for those tested integrations, not a general
failure of graphs. The historical wrong-sign interval 950--959 was already
resolved by the GNSS-aided baseline. The remaining frame-840 error has not been
proven to be a wrong-object association. Target Gaussian changes and
reference-aligned center distances are not ground-truth mismatch labels.

Three distinct failure mechanisms require different remedies:

1. A false or unmapped observation has no correct map counterpart.
2. A real observation selects a similar but different physical object.
3. The object is correct, but visible-patch means, geometry, timing, calibration
   or uncertainty modeling produce biased pose estimates.

Residual robustification helps the first mechanism when it produces a large
residual. A displaced pose can make a wrong-object match have a small residual,
so residual magnitude alone cannot resolve the second. Association changes
will not necessarily fix the third. A stable false detection or a repeated
wrong association can also survive temporal confirmation.

## Code audit: what is already present and what is missing

| Current implementation | Consequence for the next experiment |
|---|---|
| [Geometry kernel](../../localization/prepareSemanticRegistrationGeometry.m) selects the minimum-cost eligible target separately for each source at every linearization. | Correspondences already change during optimization. Calling this process a max-mixture is not by itself an improvement. |
| Target selection combines geometry and `-2 log(mixtureWeight)` up to a common constant. | Existing map stability already affects association; do not multiply another copy of the same stability score. |
| Default planar gate is 2.5 m. Eligible candidates have no explicit unmatched competitor. Global acceptance gates and robust residual weights still apply. | A nearby plausible wrong object can win even when all eligible objects are wrong. Add an unmatched alternative inside association, not merely another final pose gate. |
| Selection and fitting use different terms: line selection includes a loose tangent cost; fitting uses the normal. Point selection includes a covariance-volume term; the fitting residual does not reproduce the entire selection cost. | Treat existing scores as engineering scores. A new probabilistic association objective must specify consistent normalization and optimization terms. |
| Quality is balanced per semantic class before source temporal weighting. | One sign can receive the nominal class mass of many curb components. This is a potential influence concentration, not a demonstrated bug; geometry and robustness also affect its actual leverage. |
| [Position aiding](../../localization/selectPositionAidedRegistration.m) compares prediction/GNSS seeds, deduplicates converged poses, reduces information for mode disagreement, and rejects GNSS conflicts. | GNSS guidance and within-frame mode disagreement already exist. Additional value must come from finding genuinely different associations or retaining alternatives across time. |
| [Temporal source window](../../localization/updateLocalizationSourceWindow.m) confirms at least two detections in five scans and weights by count/5; it pools XY distributions. | It confirms existence, not map identity. It does not retain multiple map-assignment histories. The 0.45 s age limit can shorten the nominal five-scan horizon. |
| [Coarse cloud builder](../../perception/semanticProduct/buildCoarseSemanticProbabilityCloud.m) retains XYZ moments before the window; registration defaults to XY. | Some discriminating evidence is lost at the interface, not necessarily absent from perception. Height use requires consistent tilt/vertical motion treatment. |
| [Matcher](../../localization/registerSemanticProbabilityCloud.m) already projects weak information directions; graph factors reselect hard associations. | Local degeneracy handling and robust graph edges do not represent several plausible object identities. |

## Candidate routes, in order of investigation

### A. Explicit unmatched alternatives and coherent likelihoods

For each source distribution, keep several eligible map candidates plus an
unmatched state. A candidate should win against both other objects and the
possibility that the observation is clutter or absent from the map.
[Doherty et al.](https://arxiv.org/abs/1909.11213) develop semantic mixture
association with a null hypothesis. Particularly close to this project,
[Stannartz et al.](https://doi.org/10.1109/ITSC48978.2021.9565092) combine semantic
landmark localization, a sliding window and a max-mixture with an outlier
alternative. Their localization experiments use CARLA with simulated semantic
errors; that is not direct evidence of gains on our LiDAR data.

**Proposed adaptation:** for a feature-type-specific measurement space,

\[
L_i(T)=\pi_{i0}p_{0,c_i}(z_i)+
\sum_{j\in\mathcal C_i}\pi_{ij}p_{c_i}(z_i\mid T,M_j),
\qquad \pi_{i0}+\sum_j\pi_{ij}=1.
\]

The existing map weights inform candidate prior ratios. The clutter model and
unmatched prior require held-out calibration; map stability is not a calibrated
detection probability. Use the same measurement units and dimensionality for
the matched and unmatched densities of each feature type. Do not compare a
one-dimensional curb density directly with a two-dimensional pole density.
Preserve out-of-gate probability mass rather than automatically renormalizing
every small candidate set into certainty.

Evaluate either a normalized sum mixture or its explicitly defined max
approximation. Specify covariance rotation, determinant terms and derivatives
consistently. Current Gaussian scatter is not calibrated pose-error covariance,
so posterior-probability claims require additional validation. Keep minimum
usable-support checks: selecting unmatched for everything provides no pose.

This route handles forced matches at modest candidate-evaluation cost. It
cannot reliably reject a wrong object whose likelihood is genuinely comparable
to the correct one. Soft association can still converge to a poor mode; it is
not a reason to average two incompatible pose solutions.

### B. Joint spatial compatibility rather than isolated nearest candidates

Check whether several proposed correspondences describe the same arrangement:
pole separation, sign-to-curb normal distance, and relative line orientation.
For comparable point landmarks, a useful pose-invariant discrepancy is

\[
e_{ab,jk}=\left|\|\mu_a-\mu_b\|-\|\nu_j-\nu_k\|\right|.
\]

[CLIPPER](https://arxiv.org/abs/2402.07284) selects mutually consistent
correspondences using a weighted consistency graph. The original
[2021 paper](https://arxiv.org/abs/2011.10202) also describes point, line and plane
applications. [Schlichting and Brenner](https://doi.org/10.1109/IVS.2014.6856460)
demonstrate local pole/plane patterns with a separate automotive scanning
vehicle. Their experiments also expose a substantial correctness-versus-coverage
tradeoff. Consequently, requiring a fixed number of poles on every frame would
be unsuitable for our sparse scenes.

**Proposed adaptation:** a bounded local consistency graph over candidate
associations, using uncertainty-aware tolerances and a soft preference for
supported groups. Keep the existing D2D solve after candidate pruning. Test
cross-class relations where available; do not discard an otherwise useful lone
feature solely because a full pattern is unavailable.

The critical adaptation is object granularity. Several source pillars and map
Gaussians may describe the same curb or pole. Raw Gaussian IDs are not object
IDs: imposing one-to-one assignment on every Gaussian would reject legitimate
support. Group compatible whole-pillar statistics and map components, with
explicit uncertainty, or use relations appropriate to surface patches. Do not
compare arbitrary curb-patch centers as though they were fixed landmarks.
Neighbor relations can be prepared offline without gridding the GMM map anew.

Shared pose uncertainty correlates residuals in joint tests. Treating all pair
checks as independent would exaggerate confidence. Pairwise consistency also
does not resolve a perfectly repeated arrangement; GNSS or new distinctive
observations must distinguish such alternatives. Dense candidate graphs cost
quadratic memory, so use spatial gating and a candidate budget.

### C. Retain competing association/pose histories across acquisitions

The current matcher selects one winner each frame. Several initial seeds that
converge to the same solution do not preserve association ambiguity. An
alternative is to retain distinct assignments and their pose trajectories until
later observations distinguish them.
[DC-SAM](https://arxiv.org/abs/2204.11936) explicitly models discrete and
continuous variables. [Michael et al.](https://arxiv.org/abs/2202.12802) study
k-best assignment enumeration for marginal association probabilities. The
latter evaluates association subproblems, not a demonstrated complete
localization replacement; k-best assignments are not automatically k pose modes.

**Proposed adaptation:** use an ambiguity-triggered bounded beam, initially
testing 3--5 distinct histories as an engineering budget, not a literature-derived
optimum. Each contains map assignments, pose and motion history. Update with
new acquisitions, GNSS and wheel/gyro motion; prune only after sufficient score
separation. Measure candidate recall before choosing the beam size. If the true
branch was never generated, later optimization cannot recover it.

Keep the user's five-scan perception horizon. For a multi-frame optimizer,
insert each confirmed acquisition once; pooled clouds may seed matching but
must not be reinserted as independent measurements in every overlapping window.
Repeated observations with common biases are not five independent identity
votes. Delay collapsing a still-ambiguous history into a single marginal prior,
or preserve a bounded mixture prior. The experimental graph's single local
linearization prior does not do this.

This route has the largest potential for persistent aliases and the largest
implementation/runtime cost. A particle filter or multi-hypothesis tracker is
an alternative realization, not a requirement to replace the current observer.

### D. Separate stability, distinctiveness and information credibility

An object may exist reliably but look like many other objects. Map repeatability
and recent count/5 describe stability; the candidate likelihood ratio or entropy
describes identity ambiguity. Local Hessian rank describes geometry around one
solution. These are different quantities.

**Proposed adaptation:** log candidate separation, distinct physical-object
support, and leave-one-object/class-out pose change. Use influence diagnostics
to find cases dominated by a single sign or pole. Then test ambiguity-dependent
information reduction, rather than blindly lowering the entire LiDAR gain.
Do not automatically suppress a unique informative pole just because it has
high leverage.

In a local tangent chart, the total covariance identity for calibrated mode
weights is

\[
P=\sum_h w_h\{P_h+(\xi_h-\bar\xi)(\xi_h-\bar\xi)^\top\}.
\]

The second term captures between-mode uncertainty that a single Hessian misses.
It is a diagnostic representation, not an instruction to output the mean pose
between two different roads. Existing mode-disagreement reduction is a partial
implementation; the missing inputs are richer modes and validated weights.
Rank diagnostics such as [X-ICP](https://arxiv.org/abs/2211.16335) concern local
observability, not semantic identity. That distinction limits the novelty of
adding another rank threshold here.

GNSS already guides candidate selection. Using its likelihood again in graph
weights and observer fusion can create dependence or double counting. A fused
graph Hessian must never be exported as independent LiDAR information. Keep
conditional LiDAR geometry separate from GNSS/history-conditioned selection;
a calibrated uncertainty model must account for that dependence.

### E. Preserve discriminating pillar statistics across the interface

Keep useful existing XYZ moments and, where consistently available on both
sides, vertical extent/continuity, XY shape and line orientation as compatibility
descriptors. Missing or viewpoint-sensitive descriptors should reduce their
own influence, not automatically reject an otherwise sound feature. Whole
pillars remain the only coarse perception units; no ring processing, vertical
subdivision or online fine point labeling is required.

[Huang et al.](https://arxiv.org/abs/2305.14038) investigate semantic pole-map
localization, providing motivation for richer feature identity. Their learned
point-semantic frontend is not proposed for adoption here. Our adaptation uses
existing statistics and needs its own validation. Raw visible height or
intensity should not be treated as an invariant object fingerprint.

Also audit correct-object bias: a partial pole surface mean need not equal its
map mean; a curb's top and road-facing edge are different geometries. Even
oracle object associations will not remove such offsets. Use reference poses
only in diagnostic experiments to separate these effects from wrong identity.

### F. Visibility evidence and stronger robust solvers as later extensions

An assignment predicts other map objects that should be visible. Their presence
or absence can distinguish repeated patterns. This is our proposed extension,
conditional on a trustworthy range/occlusion/detection model. Missing detection
alone is not evidence against a pose when occlusion or coarse recall is unknown.
The present Gaussian map does not establish that visibility model.

[GNC](https://arxiv.org/abs/1909.08605) and
[TEASER](https://arxiv.org/abs/2001.07715) offer robust optimization/registration
tools. They merit a bounded fallback trial for difficult candidate sets, not an
assumption that a global cost optimum identifies the correct physical object.
Their stated registration assumptions must be adapted to planar point/line
Gaussian features. They do not supply missing identity evidence in a symmetric
scene. Further unconditional robust-loss tuning is therefore lower priority
than A--C given the completed negative graph experiment.

## Minimal implementation sequence and decisive evaluation

| Stage | Proposed change | Decisive measurement |
|---|---|---|
| 0 | Label selected difficult associations by physical object; record all candidate scores and rejection reasons. | Determine whether correct candidates exist, whether the peak is an association error, and whether one object dominates. |
| 1 | A alone: unmatched alternative with a coherent score. | Wrong accepted associations decrease without excessive loss of valid constraints. |
| 2 | B alone, then A+B: local relational consistency. | Resolve geometrically inconsistent aliases; measure coverage when only sparse features exist. |
| 3 | C on ambiguous cases, with D diagnostics. | Reduce persistent wrong-branch duration and peak error within an explicit runtime budget. |
| 4 | E descriptors and visibility extensions only where earlier audits justify them. | Improve correct-candidate ranking on held-out data rather than merely reject more scans. |

Freeze the current observer, map weights, perception settings and synchronization
for the first comparisons. Do not deploy a replacement by default based only on
synthetic results or a better mean error. No runtime or improvement magnitude
is asserted before implementation and measurement.

The minimum test matrix should include:

- Synthetic repeated poles/signs; absent true counterparts; persistent false
  detections; biased seeds; partial visibility; the same physical object split
  into several Gaussians; and exact symmetries where the correct outcome is
  declared ambiguity rather than a claimed unique pose.
- The full Mississippi replay, the previously resolved 950--959 interval,
  rejected frame 169, and peak frame 840. Inspect additional unseen intervals;
  do not encode any frame or component IDs in production decisions.
- Accepted wrong-object rate and correct-candidate recall with auditable labels;
  valid matching coverage; common-population matching error; full fusion RMSE,
  P95/P99, maximum, time above thresholds and recovery duration. Report startup
  separately and retain rejected frames in full-trajectory evaluation.
- Identical timing scope, hardware and inputs: median/P95/P99 cost, candidate
  count and peak memory. Re-run the existing relevant tests after implementation;
  the prior 152 passing tests are not claimed as newly run here.
- Held-out contiguous segments with buffers wider than all temporal processing
  windows, and ideally a separate mapping drive. If uncertainty intervals are
  reported, use sequence/block methods rather than independent-frame resampling.

A useful counterexample to RMSE-only selection comes from Doherty et al.'s
Table I: their null-hypothesis mixture reduces KITTI translation RMSE relative
to ML (0.19 to 0.053 m), while the maximum is larger (0.42 to 0.58 m).
This is not a prediction for Mississippi, but it directly motivates measuring
tails alongside averages. [Primary table](https://arxiv.org/pdf/1909.11213).

Inherited limitations remain: the present map includes the query drive;
GNSS/reference share a receiver; reference-assisted initialization/tilt and
offline alignment remain in the existing evaluation. Success would first mean
an improvement on this controlled replay, not independent validation of field
localization accuracy.

## Decision and research integrity

Prioritize A+B as a focused front-end experiment, with C reserved for actual
multi-mode cases and D recorded throughout. Retain the current coarse-pillar
perception and observer baseline. The evidence supports testing this direction;
it does not establish a numerical accuracy gain or prove that every remaining
large error is a mismatch.

This review used AI-assisted source retrieval, code inspection and synthesis.
Paper mechanisms, project observations and proposed adaptations are identified
separately. No paid full text is redistributed, no secondary snippets are used
as experimental evidence, and no unexecuted experiment is reported as complete.
