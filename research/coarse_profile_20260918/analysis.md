# Coarse perception optimization opportunities

Date: 2026-09-18. This is a profiling result and optimization plan, not an implemented speedup.

## Measurement scope

Profiled repository commit: `9ef9798b13bc70452073ed055b58e13e71475452`; perception implementation is unchanged from `4fa219e952c8e918ac020dacc2d4e3d79ad01d54`.
MATLAB R2026a Update 3, AMD Ryzen 7 7800X3D, native backend, default Mississippi coarse configuration with curb, pole and traffic-sign channels.
Frames: 28, 91, 214, 260, 384, 615, 746, 901, 1047 and 1137, each with 65,536 input points. Raw cached frames come from `output/pole_consistency_20260918/height_diagnosis.mat` (`frames` and `cachedFrames`).

Executed `matlab -batch "run('output/coarse_profile_20260918/profile_coarse.m')"`: one warm-up for each frame, followed by five passes through ten frames under MATLAB profiler, giving 50 complete coarse calls. Loading and display are outside the profile. Raw profiler data and configuration are in `output/coarse_profile_20260918/profile.mat`; the exported function table is attached here. The actual driver is preserved with the archive export.

The profiled mean is 97.07 ms/call. Instrumentation adds overhead, particularly for small functions called frequently; these costs establish priorities, not production latency or recoverable time. The separate unprofiled 11-frame benchmark reports a 64.59 ms median of per-frame medians (51.68–68.18 ms range); see [the timing report](../perception_timing_20260918/analysis.md). The two experiments have different frame counts and estimators.

## Observed distribution

Inclusive times include children. The rows below are separate pipeline stages; nested helper times must not be added to their parent stage totals.

| Stage | Profiled ms/frame | Share of full profiled call |
| --- | ---: | ---: |
| `analyzeGroundPillars` | 36.98 | 38.1% |
| `analyzeStructuralPillars` | 32.82 | 33.8% |
| `segmentGround` | 11.61 | 12.0% |
| `pillarizePointCloud` | 7.04 | 7.3% |
| `buildCoarseSemanticProbabilityCloud` | 2.27 | 2.3% |

Other orchestration and output construction account for the remainder. Self time in the CSV is total time minus the sum of immediate child times.

## Recommended priorities

1. **Reuse pole candidate context calculations.** `selectPillarFootprints` accounts for 25.60 ms/frame, about 26.4% of the profiled call. `computePoleCandidateContextRatio` is called 153,090 times, or 3,061.8 times/frame. Each call converts indices, constructs clipped bounds, and sums evidence. A singleton uses a 3-by-3 context; an edge-adjacent pair uses 3-by-4 or 4-by-3. Precompute or batch the singleton and pair evidence tests within a frame, reuse them across selection and filtering, and retain a general path for larger components. This should reduce repeated function dispatch, index conversions and small allocations. Preserve strict threshold comparisons, boundary clipping, candidate order and tie-breaking. This helper is shared with fine perception, so both outputs require regression checks. Global `sub2ind` and `ind2sub` self costs total 9.50 ms/frame, but that global cost cannot all be assigned to this helper.
2. **Fuse curb neighborhood calculations.** The ground-feature stage is the largest stage (38.1%). Within it, `applyLocalLinearityEnergyGate` costs 9.46 ms/frame inclusive; `boxSumMap` costs 4.03 ms/frame self across 500 calls and `shiftMap` 1.72 ms/frame self across 600 calls. Review shared neighborhood sums and batch compatible channels or implement a fused native kernel. These helper timings overlap with the parent and are not additive savings. Keep the existing compact raster and halo conventions.
3. **Reduce unnecessary grouping and statistics work.** `aggregatePillarStatistics` runs twice/frame and costs 8.33 ms/frame inclusive overall. These are all-return and nonground statistics, with different memberships: they are not interchangeable. The all-return product appears to be consumed only by diagnostics in the current pipeline; consider computing it on demand when diagnostics are requested while preserving the public pillar interface and required XYZ statistics. Reuse existing XY memberships in `buildGroundXYView` where geometries match instead of computing bins again. That view currently costs 3.13 ms/frame inclusive. Confirm complete call-site coverage before changing contracts.
4. **Leave Gaussian export for later.** `buildCoarseSemanticProbabilityCloud` accounts for only 2.3% of the profiled call. It is a lower priority than candidate and neighborhood processing.

## Constraints and validation before adoption

Coarse inference must remain whole XY pillars with full required point-distribution statistics. No height subdivision, ring-dependent logic, frame-specific labels, altered thresholds, or grid-domain changes are proposed. The existing native kernels and compact ground raster are already enabled; switching them on again is not a new optimization. Shrinking or shifting the lattice risks undoing the recently validated perception baseline.

Implement and measure one change at a time. Compare coarse candidate masks and Gaussian statistics, and compare fine masks for shared helpers, on the established regression frames including difficult pole and curb cases. Then rerun unprofiled warmed timing with repeated calls. Exact mask agreement is the first target; any floating-point reduction differences require explicit numerical checks. No production code or thresholds were changed, and no speedup, target latency, or full-sequence worst-case bound has been demonstrated by this experiment.
