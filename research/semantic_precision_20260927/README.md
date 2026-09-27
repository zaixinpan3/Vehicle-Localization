# Precision-first coarse semantic perception

The coarse output still uses whole 0.6 m XY pillars. Curbs, traffic signs and
facades now receive separate distribution-based acceptance gates before
candidate publication and Gaussian construction. Pole keeps its already
validated continuous-support classifier and its existing operating threshold.
No 0.3 m auxiliary lattice, height-bin detector, original point-mask lookup or
frame identifier is used during online inference. The original 0.3 m offline
perception path and frozen comparison labels are unchanged.

## Evaluation contract and limitations

A false selection is a selected coarse pillar containing **zero original fine
reference points of the same class**. The false-selection fraction is
`reference-empty selected pillars / all selected pillars`, pooled over frames.
It is not false-positive rate over all negative pillars, a point-wise error
rate, or a guarantee on every individual frame. Point coverage counts all fine
reference points inside the coarse XY ROI, including frames without proposals
or accepted selections. Fine points outside that ROI are reported separately.
The reference is an older detector's output, not independently annotated truth.

Mississippi includes all 1,170 frames. Downtown includes 128 frames:
`unique([1:5:539, round(linspace(1,539,24))])`. Mississippi reference indices
come from the frozen September 19 fine-perception cache. Downtown references
were captured once with the original offline configuration, before gate
fitting, then reused without recomputation. Pole totals agree with the previous
pole alignment study.

Only Mississippi frames <=780 and Downtown frames <=360 enter training.
Five contiguous prefix folds purge 20 Mississippi / 10 Downtown frames around
the held-out fold. Operating points are selected from these out-of-fold
predictions: at least 30 selections and at most 5% reference-empty selections
per profile, maximizing reference-point coverage. The suffixes are **reused
recordings**, not new independent drives; suffix labels do not enter fitting or
threshold selection. Many prefix feature/model alternatives were explored,
so out-of-fold tuning estimates should not be treated as an untouched test.
Full-sequence results include fitted prefixes and are not generalization rates.

The current policy substantially improves Mississippi curb selection and
retains traffic-sign coverage. Downtown curb and facade coverage remains poor
under the conservative policy. Zero observed errors among 20 curb or 18 facade
suffix selections does **not** establish a population error rate below 10%.
This is a functioning precision-first operating point with large abstention,
not a complete solution to facade perception.

## Frozen temporal suffix results

| Dataset / class | Baseline false / selected | Current false / selected | Current false fraction | Baseline point coverage | Current point coverage |
|---|---:|---:|---:|---:|---:|
| Mississippi curb | 20,724 / 32,963 | 724 / 9,199 | 7.87% | 88.04% | 69.61% |
| Mississippi pole | 20 / 397 | 20 / 397 | 5.04% | 62.38% | 62.38% |
| Mississippi traffic sign | 33 / 874 | 28 / 866 | 3.23% | 99.96% | 99.92% |
| Downtown curb | 905 / 1,080 | 0 / 20 | 0 observed | 73.82% | 10.80% |
| Downtown pole | 3 / 40 | 3 / 40 | 7.50% | 38.61% | 38.61% |
| Downtown traffic sign | 12 / 579 | 9 / 575 | 1.57% | 99.96% | 99.94% |
| Downtown facade | 3,022 / 3,962 | 0 / 18 | 0 observed | 54.46% | 1.63% |

Full-sequence replay (including fitted prefixes):

| Dataset / class | Baseline false / selected | Current false / selected | Current point coverage |
|---|---:|---:|---:|
| Mississippi curb | 60,209 / 99,361 | 1,504 / 30,067 (5.00%) | 100,146 / 133,836 (74.83%) |
| Mississippi pole | 30 / 1,222 | 30 / 1,222 (2.45%) | 136,943 / 194,300 (70.48%) |
| Mississippi traffic sign | 129 / 4,625 | 28 / 4,517 (0.62%) | 66,768 / 66,783 (99.98%) |
| Downtown curb | 3,736 / 5,414 | 0 / 193 | 727 / 5,933 (12.25%) |
| Downtown pole | 3 / 275 | 3 / 275 (1.09%) | 20,317 / 34,829 (58.33%) |
| Downtown traffic sign | 65 / 1,454 | 9 / 1,397 (0.64%) | 12,405 / 12,445 (99.68%) |
| Downtown facade | 9,100 / 12,576 | 0 / 959 | 99,078 / 446,145 (22.21%) |

The large difference between fitted-prefix and facade suffix coverage is a
material limitation. Its prefix out-of-fold gate retains only 34 cells, one
false, covering 4,647 / 358,020 points (1.30%). Do not use the fitted-prefix
27.27% coverage as evidence of performance on new scenes.

## Why the original coarse selection was excessive

The prior curb proposals emphasize height change, roughness, directional
support and road adjacency. Those signals also occur in sloped ground,
uneven surfaces and nearby clutter. Proposal propagation can retain a cell
next to a curb without any actual reference curb point inside that cell.
Whole-pillar residuals and the distribution of points between two side
surfaces help distinguish these cases; merely raising one scalar energy
threshold did not reach the prefix precision target.

Traffic-sign proposals previously accepted any sufficiently intense return.
The new gate also measures the number, fraction, height distribution and
intensity quantiles of bright returns, plus contextual statistics. The
radiometry features use the existing 1,800 intensity convention of these
sensor/configuration profiles; they are not calibrated to arbitrary sensors.

Facade proposals use count-weighted line voting, assignment and a support
halo. A strong line or a point near its extension does not imply that every
nearby occupied pillar contains fine facade points. The original fine facade
result also depends on a larger field of view and different vertical-support
and line competition statistics. Larger coarse context and raw robust plane
fits alone did not reproduce that decision reliably. The selected conservative
facade gate requires agreement between a tree ensemble and nearby training
feature distributions; it rejects unfamiliar distributions rather than
assigning them from line score alone.

## Implemented features and classifiers

- `measureSemanticPillarFeatures` supplies current whole-cell moments and
  neighboring raster statistics. It has no absolute XY or frame identity.
- `measureSemanticPointDistributions` adds 45 raw-support descriptors:
  covariance eigenvalues, continuous plane residuals, height quantiles,
  contextual residuals, curb side-surface steps and bright-return evidence.
  Conditional subsets are measured inside existing whole pillars; no smaller
  spatial or vertical lattice is created.
- Curb uses 157 descriptors and traffic sign uses 117. Their histogram boosted
  trees have 200 iterations, maximum depth 6, minimum leaf size 25, learning
  rate 0.05 and L2 regularization 2. Positive weighting accounts for covered
  reference-point count during offline fitting only. Each recording profile
  receives its own prefix-selected threshold; see `model_validation.json`.
- Facade adds 31 robust line/plane group and owner statistics, for 178 total.
  Its ensemble has 250 extra trees, maximum depth 12 and minimum leaf size 3.
  It must score at least 0.8504190548553759, and all nine nearest distributions
  under a prefix-fitted robust scaler must be positive. The numeric training
  bank stores feature vectors and binary classifier targets, with no frame,
  pillar or original point identifiers. The JSON model is approximately 35 MiB;
  it is loaded once per MATLAB process and the neighbor query only runs for
  candidates that pass the forest. This is a storage/startup cost of the
  conservative facade gate, not a claim of a compact deployment model.
- `filterSemanticPillarCandidates` only prunes existing class masks and zeros
  rejected semantic evidence before both output products are built. Accepted
  whole-pillar moments, source preprocessing and pole selections stay intact.
- An empty-candidate guard avoids unnecessary feature work. A scalar-invalid
  input exposed an existing empty-index orientation edge case; retained source
  indices are now consistently column vectors in `pillarizePointCloud`.

The models rank agreement with fine detector references. Their scores are not
calibrated probabilities of physical class correctness. Changing geometry,
profiles, sensor intensity scale or proposal generation requires recalibration.

## Alternatives explored and not promoted

`single_feature_gates.csv` records scalar threshold sweeps. Raster-only curb
features failed the precision target; adding raw distribution support produced
the selected gate. A dedicated Downtown curb model increased suffix coverage
to 17.90% but reached 4 / 40 false selections (10%); the shared conservative
model was retained because false selection is the priority. Curb component
shape/topology features did not improve its prefix operating point.

Facade experiments used raw local statistics (147 features), line-group
statistics (178), global context variants on a phase-aligned 0.6 m lattice
(280), and seed/line-placement geometry (191). Boosted-tree gates did not reach
the required prefix precision. Standalone extra trees, random forests and
nearest-neighbor classifiers also failed. The conservative forest/neighborhood
conjunction was selected using prefix predictions. Wide-context and placement
features are not executed by the production gate. Supporting exploratory
helpers remain available for reproducing the unsuccessful comparisons.

## Reproduction and validation

From the repository root, use MATLAB R2026a with the project's installed
Image Processing Toolbox and native perception kernels. Python fitting uses
scikit-learn 1.9.1, pandas 3.0.6 and seed 927. No Python process or Statistics
and Machine Learning Toolbox is needed for MATLAB inference.

```bash
matlab -batch "addpath('research/semantic_precision_20260927'); captureSemanticBaseline('Mississippi'); captureSemanticBaseline('Downtown')"
uv run --with scikit-learn==1.9.1 --with pandas python research/semantic_precision_20260927/trainSemanticModels.py curb trafficSign
uv run --with scikit-learn==1.9.1 --with pandas python research/semantic_precision_20260927/trainFacadeConsensus.py
```

Exported models are written to `output/semantic_precision_20260927/`. Copy
`curb_model.json`, `trafficSign_model.json` and `facade_model.json` to their
corresponding `config/*PillarPrecisionModel.json` paths before replaying.

```bash
matlab -batch "addpath('research/semantic_precision_20260927'); replaySemanticPrecision"
matlab -batch "addpath('research/semantic_precision_20260927'); validateSemanticPrecision"
matlab -batch "addpath('research/semantic_precision_20260927'); benchmarkSemanticPrecision; checkSemanticPrecisionCode"
python research/semantic_precision_20260927/summarizeSemanticPrecision.py
```

Both raw-data replays require exact exported-MATLAB/Python selected-pillar
agreement on every frame, unchanged preprocessing summaries, selection subsets
of the baseline, and exact unchanged pole IDs. All frozen reference denominators
are retained. The fine regression compares the original offline masks with its
immutable reference fixture. The historical high-recall coarse proposal tests
explicitly disable the new precision gate; precision and coverage of the new
operating policy are tested separately, rather than silently redefining that
old proposal contract.

Validation completed with **249 unique tests passed, zero failed or incomplete**.
The full run passed all other tests; after the scalar-invalid input fix, all six
semantic precision contract tests were rerun successfully. `validation_runs.json`
records that provenance and `tests.csv` contains each test's latest result.
Factory Code Analyzer reports no findings. Every selected-pillar prediction
matched the MATLAB replay on all 1,170 Mississippi and 128 Downtown frames.

Paired warm median perception time increased from **83.45 to 117.87 ms** on
Mississippi and **106.69 to 152.38 ms** on Downtown. The additional distribution
checks cost approximately 34.42 / 45.69 ms per frame on this machine. The
current implementation improves precision at a computational cost; it is not
a speed improvement. Timing is a small paired workload, not a full-sequence
latency guarantee, and does not include one-time model loading.

`summary.json` is the machine-readable count/timing/test summary;
`*_baseline.csv` and `*_replay.csv` provide every frame and class.
`model_validation.json` records prefix selection and fitted/suffix metrics.
`artifact_hashes.json` identifies large local reference/prediction artifacts;
raw recordings, large per-pillar feature CSVs and MAT caches are not committed.
`runtime_paired.csv` contains five alternating warm baseline/current repetitions
on six frames per dataset; timing excludes disk reads, offline references and
first-load model parsing. `tests.csv` and `code_analysis.json` contain actual
executed validation outcomes.

The live Mississippi frame-500 viewer uses complete original points and the
new coarse masks. Curb changes from 81 selected cells / 54 false / 62 of 64
reference points covered to 14 selected / zero false / 38 of 64 covered.
Pole remains 2 cells / zero false / 296 of 296 points; traffic sign remains
12 cells / zero false / 495 of 495 in-ROI points (12 extra reference points
outside the coarse ROI are still displayed). See `frame500/`.
