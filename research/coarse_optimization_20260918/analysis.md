# Preserve pillar statistics while reducing coarse perception work

Date: 2026-09-18. Baseline: `e4ae02980a4740d5044ea1788d8ea8081cccd5c2` (the current, previously validated perception implementation). Configuration, lattice geometry, semantic channels and thresholds are unchanged.

## Outcome

On ten warmed Mississippi frames, the median of per-frame median coarse latency changed from **66.75 ms to 58.82 ms**, a **11.9% reduction** (1.135x speedup). Full coarse outputs are exactly equal on all **1170/1170 Mississippi frames** and **12/12 Downtown frames**. The complete fine masks agree on all **78/78 Mississippi regression frames**, with zero changed pole, curb or traffic-sign points.

These are baseline-consistency results, not new ground-truth accuracy measurements or a proof of equivalence on every possible input. The statistical feature-recognition model is retained.

## Design and implementation

1. **Batch pole support decisions and reuse them within each frame.** Previously, compact-support selection repeatedly converted cell indices and summed the same singleton and edge-pair neighborhoods. `selectPillarFootprints` now forms a table for each core pillar and its four edge neighbors, computes each proposal's context test once, and reuses the table when reducing connected components. A component-label array replaces repeatedly allocating a membership raster per proposed component. The detector still uses one evidence statistic per complete pillar. Neither a height partition nor point classification is introduced.

   The test is still `componentEvidence/contextEvidence > minimumContextFraction`, with the same small-context rule. Neighborhoods retain clipped 3-by-3, 3-by-4 or 4-by-3 bounds. Pair order remains up, down, left, right; singleton-before-pair selection and `score > best + eps` ties are preserved. Batch summation can change floating-point reduction order: near a decision boundary, the same local rectangle is reduced in the original order before deciding. No full-frame result is cached.

2. **Fuse whole-pillar bounds and radiometry into the existing moment pass.** The native `cellMoments` kernel now optionally returns XYZ minima/maxima and per-attribute finite counts/maxima alongside count, mean and all six population covariance entries. It uses the existing first point traversal for the added reductions instead of six separate bounds reductions plus separate attribute reductions. Covariance remains a second centered pass, preserving small scatter at large coordinate offsets. `aggregatePillarStatistics` retains the same public fields and sorted pillar IDs.

   For every pillar, the retained statistics include count, mean XYZ, covariance `[xx xy yy xz yz zz]`, minimum/maximum XYZ, and available intensity/reflectivity summaries. All-return and nonground memberships remain distinct. No required statistic or diagnostic is removed, and no lower precision is introduced.

3. **Use one native prefix buffer for curb neighborhood sums.** The `boxSum` CPU kernel retains the first-dimension then second-dimension cumulative-sum order and four-corner subtraction order. It avoids MATLAB's intermediate full-raster indexed arrays. Curvature, height-step and linearity decision rules remain unchanged. The portable MATLAB backend remains a supported execution mode for the same algorithm; native availability is versioned at 3, with the binary rebuilt from source.

4. **Reuse existing pillar membership.** The ground XY view consumes `pointPillarSub` from the shared lattice rather than quantizing the same points a second time. Raster origin, spacing, compact halo, and public pillar IDs remain unchanged.

The previously considered removal of diagnostic-only all-return statistics was not adopted: the whole-pillar statistics contract is preserved. Existing footprint-selection helper implementations replaced by the batch algorithm were removed from production source. The immutable Git export used as a differential oracle lives only in ignored experiment output and is never called by the production pipeline.

## Validation

- **1170/1170 Mississippi coarse outputs:** `isequaln` on the entire result, including candidate pillar IDs, Gaussian components, semantic probabilities, covariance/inverse covariance and summaries. Results are in `sequence_validation.csv`.
- **12/12 Downtown coarse outputs:** frames 1, 50, 99, 148, 197, 246, 294, 343, 392, 441, 490 and 539, with all configured channels including facade. Results are in `downtown_validation.csv`.
- **78/78 fine outputs:** the established sequence-wide sample and previously reviewed frames, including 28, 91, 214, 260, 384, 425, 458, 538, 600, 615, 687, 746, 832, 855, 901, 943, 963, 1047 and 1137. The reference is the previously saved corrected-baseline masks from commit `4fa219e952c8e918ac020dacc2d4e3d79ad01d54`; perception source is unchanged between that commit and this experiment's baseline. The final installed kernel was used for the final full coarse and fine comparisons.
- **132/132 unique tests passed**, no incomplete tests. The 25 new parameterized cases were rerun against the final installed kernel. Coverage includes malformed radii, empty and one-dimensional rasters, clipped windows, strict threshold equality, pair ties, finite attribute summaries, complete XYZ correlations, and small scatter at large coordinates. Existing suites cover whole-pillar execution, native/portable parity, compact-raster parity and the reviewed pole cases. See `tests.csv`.
- **1000 randomized footprint comparisons**, seed 1979, five raster shapes including single rows/columns, integer/fractional/signed/nonfinite evidence and varying thresholds: all six outputs exactly matched the pre-change helper. This is a differential experiment, not an extra production implementation.
- MATLAB Code Analyzer with factory settings: zero issues in all seven modified/new MATLAB files. `git diff --check` passed.

The platform's MEX post-build inspection reported that its own linked ELF was not a MEX file. Compilation and linking produced a binary; independent MATLAB loading, version checks, execution of both new kernel interfaces, and all subsequent tests succeeded. The final tested binary was installed by atomic replacement, leaving already loaded binaries intact in other MATLAB processes. Binaries are ignored by Git; rebuild using `buildPerceptionKernels`.

## Timing protocol and limits

Hardware: AMD Ryzen 7 7800X3D; MATLAB 26.1.0.3276743 (R2026a) Update 3; 8 maximum computational threads, native backend. Frames: 28, 91, 214, 260, 384, 615, 746, 901, 1047, 1137, each with 65,536 input points. For each frame/version, run five warm-ups and nine timed calls. Alternate which version is measured first across frames; use the same cached raw inputs. Switch isolated baseline/current paths and reload the native binary outside timed regions. Verify complete output equality for every measured frame. No profiler or other experiment was running during final timing.

| Metric | Baseline | Optimized |
| --- | ---: | ---: |
| Median of per-frame medians | 66.75 ms | 58.82 ms |
| Per-frame median range | 52.18–71.27 ms | 41.55–63.12 ms |
| Reciprocal of median latency | 14.98 Hz | 17.00 Hz |

Loading, path changes, visualization, mapping and localization are excluded. The 1170-frame runs establish output consistency; they ran concurrently with other validation and are not used as timing benchmarks. The ten-frame timing sample does not establish a full-sequence worst-case or end-to-end deadline guarantee. The fine pipeline was validated for output preservation; no fine-runtime speedup is claimed.

An initial pole-only experiment showed only a few milliseconds of improvement despite the large profiler share. This motivated the measured multi-stage implementation; profiler overhead was not treated as recoverable production time. Further work should begin with a new profile of this version and preserve these same membership/statistics contracts.

## Reproduction artifacts

The experiment drivers are in `output/coarse_optimization_20260918/` and preserved as archive export copies: `sequence.m` (baseline/final full-sequence comparison), `downtown_validation.m`, `fine_validation.m`, `check_footprints.m`, `paired_timing.m`, `run_tests.m`, `final_kernel_tests.m`, and `code_checks.m`. Raw profiler/timing/reference MAT files, recordings, immutable source snapshots and native binaries stay outside the source commit. Set `COARSE_VALIDATION_MODE=baseline` or `optimized` for the sequence driver; set `COARSE_TIMING_LABEL=final_timing` for the paired timing driver. Scripts contain the actual local data/cache paths and require those inputs. Public CSV/JSON exports contain per-frame outcomes and timing results.
