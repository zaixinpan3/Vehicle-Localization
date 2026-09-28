# Mississippi curb coverage recovery

This study addresses missed curb selections in the September 27 precision
policy. The deployed change is restricted to Mississippi curb acceptance.
It uses the existing 157 current-frame whole-pillar distribution descriptors
on the same 0.6 m XY lattice, with a Mississippi-specific extra-tree ensemble.
No finer grid or reference-point lookup is used at inference. Original fine
references, proposal generation, ground routing, pole and traffic-sign
decisions remain unchanged. Downtown keeps its previous model/configuration
and was not replayed or retuned in this study.

## Diagnosis and evaluation

Frame 500 contains 64 frozen fine curb points in 27 coarse cells. All 64
survive coarse ground routing. Original proposals cover 62; the previous
precision gate covers only 38. Thus most additional misses arise from the
final acceptance gate, not from ground segmentation. Some real-reference
curb cells contain very few points or nearly planar single-beam support;
individual height/residual thresholds cannot reliably distinguish these from
nearby road cells. See `diagnosis.json` and the point-level diagnostic CSV.

A false selected pillar contains **zero original 0.3 m fine reference curb
points**. False fraction is false selected pillars divided by all selected
pillars, pooled over the stated frames. Point coverage includes every fine
curb point inside the coarse XY ROI, including cells without proposals.
The reference is an older detector, not independent physical annotations.

Only frames 1:780 enter fitting. Five contiguous temporal folds exclude a
20-frame margin on either side of each held-out block. The operating threshold
maximizes covered reference points under 5% prefix out-of-fold false selection
and at least 30 selected cells. Seed is 928. The final model has 120 extra
trees, depth at most 18, minimum leaf size 3, 70% candidate features per split,
and balanced binary classes. It uses no frame, pillar ID or absolute XY
feature. The selected threshold is 0.7503924254836816.

The prefix out-of-fold result selects 20,490 cells, including 1,024 false
cells (4.998%), covering 68,635/91,697 reference points (74.85%). The suffix
781:1170 was excluded from fitting and threshold selection, but **was reused
to assess multiple alternatives**. It is validation on an existing recording,
not an untouched independent test. Frame 500 is in the training prefix.
Its large improvement must not be presented as new-scene performance.

| Evaluation set | Previous covered points | New covered points | Previous false / selected | New false / selected |
|---|---:|---:|---:|---:|
| Frame 500 (fitted) | 38/64 (59.38%) | 58/64 (90.63%) | 0/14 | 0/24 |
| Suffix 781:1170 | 29,333/42,139 (69.61%) | 31,210/42,139 (74.06%) | 724/9,199 (7.87%) | 924/10,045 (9.20%) |
| All 1,170 frames (includes fitted prefix) | 100,146/133,836 (74.83%) | 112,504/133,836 (84.06%) | 1,504/30,067 (5.00%) | 1,060/36,051 (2.94%) |

The suffix improvement is 4.45 coverage percentage points, at 1.33 additional
false-selection percentage points. It stays below the user's 10% aggregate
limit in this recording, but is not a per-frame or future-drive guarantee.
Suffix coverage remains 74.06%, so this does not eliminate curb misses.
Lower full-sequence false fraction largely reflects fitted-prefix behavior.

## Alternatives and tradeoffs

Experiments retain failed and less effective alternatives rather than silently
replacing their results. Prefix sweeps use 5%, 5.5% or 6% precision budgets as
recorded in their JSON files. Some subsequent frontier analyses explicitly
use suffix outcomes; those are exploratory validation, not held-out estimates.

| Alternative | Suffix coverage | Suffix false fraction | Frame 500 points | Decision |
|---|---:|---:|---:|---|
| One-pass nearby strong curb support | 73.06% | 9.29% | 41/64 | Limited recovery |
| Mississippi-only boosted trees | 75.51% | 10.38% | 43/64 | Exceeds limit |
| Boosted-tree rank consensus | 75.17% | 10.37% | 44/64 | Exceeds limit |
| Range-calibrated boosted thresholds | 75.14% | 10.43% | 42/64 | Exceeds limit |
| Slender connected chains | 72.10% | 9.32% | 41/64 | Limited recovery |
| Chains supported by both ranks | 73.90% | 10.18% | 41/64 | Exceeds limit |
| Stronger chain anchors | 72.28% | 9.51% | 38/64 | No displayed-frame recovery |
| Selected extra-tree ensemble | 74.06% | 9.20% | 58/64 | Deployed for Mississippi |

Relaxed connected chains can cover 52/64 points in frame 500, but reach
13.32% suffix false selection and were rejected. Spatial continuity alone
also extends selections into adjacent reference-empty ground cells. Connected
component features did not improve the boosted model's prefix operating point.
The forest provides a useful empirical compromise on the same descriptors;
these experiments do not establish a unique causal explanation for its gain.

## Implementation and reproduction

`semanticPillarPrecisionConfig` supplies an optional per-class model filename.
Only the Mississippi curb entry uses the new model. Clearing `modelFiles`
restores the previous gate for paired comparisons. `scoreSemanticPillarModel`
averages flattened forest leaf votes using float32 split inputs, matching
scikit-learn. The existing facade forest still applies its neighbor veto.
Scores rank reference agreement and are not calibrated physical probabilities.
Accepted cells retain the original full-pillar moments and Gaussian product.

The numeric model is 41,862,098 bytes (approximately 39.9 MiB), loaded once
per MATLAB process. It contains 845,918 nodes. This adds storage and cold-start
cost; the warm timing comparison excludes initial JSON loading and disk reads.
No Python or Statistics and Machine Learning Toolbox is needed for inference.

The frozen reference/proposal cache and descriptor CSV are the outputs of
`captureSemanticBaseline('Mississippi')` in the September 27 study. Reuse
the original frozen cache for comparisons; do not regenerate reference labels
from a subsequently changed fine detector. Large local recordings, feature
tables, Python pickles and prediction tables remain under ignored `data/`
and `output/`. Independent artifact hashes are in `artifact_hashes.json`.

```bash
uv run --with scikit-learn==1.9.1 --with pandas python research/curb_recovery_20260928/probeCurbForest.py
uv run --with scikit-learn==1.9.1 --with pandas python research/curb_recovery_20260928/exportCurbForest.py
matlab -batch "setupVehicleLocalization; addpath('research/curb_recovery_20260928'); validateCurbRecovery; replayCurbRecovery; benchmarkCurbRecovery"
uv run --with pandas python research/curb_recovery_20260928/summarizeCurbRecovery.py
```

The raw replay requires exact MATLAB/Python selected-pillar agreement in every
frame, unchanged preprocessing summaries, unchanged pole/sign selections and
subsets of original proposals. Tests compare original fine point masks in
frames 300, 500 and 900, exercise coarse profile isolation, backend/channel
agreement, empty input, float32 boundary behavior and the existing facade
consensus veto with a small synthetic model. No Downtown recording is loaded.
The initial nine tests passed but the harness stopped at two Code Analyzer
logical-indexing suggestions; these were corrected before final validation.
Final test, analysis, replay and paired timing results are stored alongside
this report. The paired benchmark uses six frames, five repeats and alternating
old/new execution order, with all configured feature channels enabled.

Final validation passed **11/11 tests**, with no failed or incomplete cases.
Factory Code Analyzer found no issues in eight changed MATLAB files. All
1,170 raw-frame comparisons passed. The 60-run warm benchmark gives:

| All-channel frame time | Previous gate | New Mississippi curb gate |
|---|---:|---:|
| Median | 118.561 ms | 120.124 ms |
| Mean | 115.191 ms | 117.640 ms |
| 95th percentile | 147.841 ms | 149.336 ms |

The difference between medians is 1.563 ms (1.32%); the median of the 30
paired differences is 2.533 ms. This is a small measured warm-time increase,
not a speedup. Model load time was excluded and not separately benchmarked.

The live comparison is reproduced by `showMississippiFeaturePillars(500)`.
It colors existing source points and places same-color selected cells directly
below the cloud. Cyan denotes curb; orange pole and magenta sign are retained.
