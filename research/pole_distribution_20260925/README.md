# Pole pillar point distributions and confidence weighting

## Result and scope

The online 0.6 m detector now weights accepted pole pillars by metric point
concentration and vertical continuity. This replaces the previously saturated
XY point-versus-line confidence for the coarse profile. It does **not** change
the candidate masks or the complete-pillar Gaussian geometry. Offline 0.3 m
perception retains its previous confidence rule. No 0.3 m auxiliary detector,
vertical occupancy lattice, trained model, or fine mask enters online inference.

The broader migration is still incomplete. Against frozen 0.3 m coarse output,
full-drive pole precision/recall remain 0.623/0.608 for exact cells and
0.674/0.700 with one-cell tolerance; the requested 80% agreement is not reached.
The confidence-ranking result below is a different measurement and must not be
used to claim that the migration passed.

## Point-distribution findings

The raw study uses Mississippi frames `1:10:1170`, with 0.6 m off-ground pillars
and neighboring original returns. Reference groups are:

- 306 pillars confirmed by the existing offline fine detector;
- 668 pillars selected by the old coarse detector but not fine, within the
  descriptive study's minimum-six-return / 0.6 m height universe;
- 272 current coarse pillars selected by neither reference.

Fine is an algorithmic reference, not manually labelled physical truth. Its own
isolation rules favor concentrated points, so these descriptive differences are
not independent evidence of semantic accuracy. Unconfirmed pillars can contain
real shafts, and adjacent cells can describe the same object.

| Median statistic | Fine-confirmed | Old coarse only | Current only |
|---|---:|---:|---:|
| Own-pillar fraction within the 0.15 m density core | 0.862 | 0.632 | 0.655 |
| Largest height gap within that core and its neighbors [m] | 0.238 | 0.599 | 0.558 |
| Largest gap / full core height | 0.076 | 0.225 | 0.177 |
| Returns within 0.25 m / returns within 0.75 m | 0.904 | 0.511 | 0.473 |
| Returns within 0.15 m / returns within 0.45 m | 0.729 | 0.432 | 0.509 |

[Point examples](point_examples.png), [empirical distributions](distribution_statistics.png),
and [radial mass profiles](radial_mass_profiles.png) expose the underlying data.
Raw examples show both tilted shafts crossing pillar boundaries and short stems
sharing pillars with upper clutter. They also show cases where a low point blob
plus a few high returns passes a maximum-height test despite a large empty gap.

The useful distinction is therefore **concentrated mass with continuous vertical
support**, rather than height range alone. Residual radial standard deviation
by itself separates these groups poorly (medians approximately 0.072, 0.078,
and 0.078 m).

## Implemented evidence

For each already accepted pillar, retain its existing whole-pillar density peak
and gather original returns through neighboring 0.6 m pillars. Let `z` be the
sorted heights within a 0.25 m radius and let `N75` count returns within 0.75 m.
The geometric evidence is

```text
concentration = numel(z) / N75
continuity = 1 - max(diff(z)) / (max(z) - min(z))
robustHeight = quantile(z, 0.95) - quantile(z, 0.05)
evidence = concentration * continuity
         * min(1, robustHeight / 1.5 m) * min(1, numel(z) / 12)
```

Insufficient or constant-height support has zero evidence. The existing
minimum semantic probability still provides the output floor. Evidence is
bounded geometric confidence, not a calibrated probability of a physical pole.
A large gap or isolated extreme return reduces confidence without deleting a
possibly useful candidate. Neighbor evidence handles split pillars; all output
means and covariances still use all original returns assigned to each pillar.
The radii are metric kernel supports, not finer semantic cell sizes.

Configuration is in `config/structuralPillarConfig.m`:
`pole.probabilityEvidence="distribution"` for coarse spacing, with support scales
in `pole.distribution`. Set the evidence mode to `"shape"` to reproduce the
previous confidence. `scorePolePillarDistributions` implements the score.
`computePillarVerticalShape` and `computePillarRadialDistribution` provide
reusable whole-pillar research descriptors; their larger descriptor sets are
not computed by the default online path.

On 869 existing candidate cells in the 117-frame study, 260 match a fine cell
exactly. The descriptive ranking comparison is:

| Measure against the fine reference | Previous confidence | Distribution evidence |
|---|---:|---:|
| ROC AUC | 0.7113 | 0.8543 |
| Average precision | 0.5078 | 0.7656 |
| Fine-cell fraction among the top-scored 20% | 59.4% | 83.9% |
| Median score on fine-confirmed candidates | 1.000 | 0.870 |
| Median score on other candidates | 0.726 | 0.447 |

These are exploratory same-drive ranking measurements, not held-out physical
accuracy or an 80% recall result. The score was selected after examining this
study. Cutoff ties are handled by expected precision under uniform selection: the old
cutoff ties 234 cells, while the new cutoff ties one cell. The new column uses
raw geometric evidence; the existing affine probability floor preserves its
ranking. Stored values in `pole_evidence_scores.csv` permit independent checking.

## Full-drive validation

`validatePoleDistributionIntegration` compares the saved previous and updated
products for all 1,170 scans. It asserts identical candidate masks for every
channel, road/curb/sign cells, component semantic IDs, counts, means, covariances,
and XYZ means. All 8,909 accepted pole probabilities change. The mean L1 change
of the normalized mixture-weight vector is 0.02871; mean pole mixture mass moves
from 0.17354 to 0.17082. Changes in normalization can also affect other channels'
mixture weights even though their detections and geometric components agree.

A paired 117-frame run alternates evaluation order after three warmup pairs:
median 27.396 ms for previous confidence and 29.528 ms for distribution evidence;
the median within-frame increase is 2.262 ms. The complete updated capture has
median 30.6 ms. These are observed software timings, not a hard real-time bound.
See `integration_validation.json` and `paired_timing.csv`.

## Rejected or deferred detection changes

`measureAxialPillars` investigates near-vertical regression hypotheses from
whole-pillar height quantiles, robust residual cores, contiguous height runs,
and neighborhood isolation. It can recover the tilted/boundary and lower-stem
examples but also selects branches. The optional continuous height-window
support experiment uses empirical height CDFs, not vertical voxels. Neither
variant met the old-coarse agreement target, so neither replaces detection.

Full-drive descriptor ablations use 1,097,054 whole-pillar rows (at least six
returns and 1 m height), seed 42, 250 histogram-boosting iterations, 31 leaves,
minimum leaf 25, L2=5, and positive class weight 10. Train frames are 1 modulo 4,
threshold selection uses 3 modulo 4, and 585 even frames form the scoring
partition. Absolute XY coordinates and frame identifiers are excluded from
predictors. Scoring denominators include **all** projected baseline positives,
including the 133 absent from the candidate table. One-cell matching allows
0.6 m in each axis, up to 0.849 m diagonally; empty background is excluded.

| Predictor inputs | Test tolerant precision | Test tolerant recall |
|---|---:|---:|
| Previous whole-pillar descriptors | 0.750 | 0.736 |
| Plus whole-height distribution | 0.741 | 0.744 |
| Plus axial distributions | 0.747 | 0.730 |
| Plus radial distributions | 0.756 | 0.748 |
| Axial and radial distributions | 0.751 | 0.757 |

A fixed-threshold refit of the radial model on all odd frames reaches recall
0.810 but precision only 0.713 on even frames. Its former validation frames are
part of training and are explicitly labelled accordingly in `radial_refit.json`.
No learned model is deployed. The same drive and test partition have been
examined repeatedly across engineering iterations, so these exploratory results
are not an untouched final generalization test. They also do not prove that an
80% whole-pillar detector is impossible.

Fine-reference probes on 117 frames separately use modulo-three partitions of
the sampled frame index (39 frames each), 180 iterations and 15 leaves. Adding
axial features improves test tolerant F1 from 0.639 to 0.698, but no tested feature
set reaches 80% precision and recall together. Validation-only results near
80% did not persist on the test partition. See the three ablation JSON files
and `fine_distribution_ablation.json` for full thresholds and metrics.

## Reproduction and artifacts

Use MATLAB R2026a with the project paths and local raw data. The frozen coarse
reference is `output/coarse_lattice_20260924/baseline_sequence.mat`, originating
from revision `796c737344e6c29c3e7aa8869aab2630dc0ebbb1`. The previous 0.6 m result
is `metricCoreFull_sequence.mat`, associated with commit
`ca48dfd7d245e719d076ce89121bf3b6af871012`. The fine mask cache is
`fine_pole_pillars.mat` in the same output directory.

```matlab
addpath(pwd); setupVehicleLocalization;
setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(pwd,'data'));
addpath(fullfile(pwd,'research','pole_distribution_20260925'));
addpath(fullfile(pwd,'research','coarse_lattice_20260924'));
capturePolePointDistributions();
summarizePolePointDistributions();
plotPolePointDistributions(); plotPoleDistributionStatistics();
plotRadialMassDistributions(); evaluatePoleDistributionScores();
captureAxialSequenceDescriptors(); captureRadialSequenceDescriptors();
exportPoleComparisonReferences();
captureCoarseSequence('distributionEvidenceFull',1:1170,pwd);
validatePoleDistributionIntegration();
```

The all-frame axial capture exceeded the MCP response timeout, but MATLAB
continued to completion; the resulting CSV was checked for all 1,170 frames and
exact row-key agreement with the previous descriptor table before use. Large
raw-point captures, descriptor CSVs, prediction arrays and MAT test results stay
under ignored `output/`. `artifact_hashes.csv` identifies independent technical
artifacts. The Python probes use NumPy 2.5.3 and scikit-learn 1.9.1 in an isolated
temporary environment. No trained-model dependency is introduced online.

Validation checks: **99/99 MATLAB tests passed**, zero failures or incomplete
tests, including 16 new distribution/evidence cases, the recorded offline
perception regression, mapping repeatability, native/MATLAB parity, compact
rasters, feature selection and probability products. `tests.csv` lists actual
results. Factory Code Analyzer found zero issues in 20 changed MATLAB files;
all three Python drivers compile. Initial harness issues (unsupported `range`,
an over-tight near-zero covariance tolerance, and a synthetic pole inside the
sensor's excluded near range) were corrected before the successful final run.
No full localization/observer replay or repository-wide test run is claimed.

```sh
python research/pole_distribution_20260925/probeAxialConsistency.py
python research/pole_distribution_20260925/probeAxialConsistency.py --radial
python research/pole_distribution_20260925/probeAxialConsistency.py --radial --refit
python research/pole_distribution_20260925/probeFineDistributions.py
python research/pole_distribution_20260925/summarizePoleEvidenceScores.py
```
