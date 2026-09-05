# Coarse Perception Runtime Optimization

Date: 2026-09-04. Behavioral reference:
`1e4f503efdb0052a6d636d079144bdd60532ee07`.

## Outcome

The median of per-frame median coarse latencies fell by **50.4%** on Mississippi
and **50.6%** on Downtown. Every measured frame retained the reference ground
labels, semantic candidate pillar IDs, offline curb/pole/marking point masks,
Gaussian support counts, means, semantic evidence, and mixture weights.
Covariance differences were at most 5.56e-17 square meters; inverse differences
were at most 2.14e-13 inverse square meters.

| Dataset | Frames | Reference coarse | Optimized MATLAB | Optimized native |
|---|---:|---:|---:|---:|
| Mississippi | 19 | 113.943 ms | 78.176 ms | **56.501 ms** |
| Downtown | 5 | 92.765 ms | 59.225 ms | **45.846 ms** |

Across all 216 native timing calls, pooled P50 was 56.2 ms and P90 was 67.8 ms.
These are loaded-frame perception timings, excluding disk reads, plotting,
compilation, and registration. They do not establish a complete localization
system deadline or independently labeled detection accuracy.

## Measurement and reference preservation

Measurements used the existing MATLAB R2026a Update 3 desktop, an AMD Ryzen 7
7800X3D CPU, and eight MATLAB compute threads. No parallel pool or GPU was used.
Frame 260 initially measured 112.986 ms using `timeit`. Profiling five calls
identified terrain propagation/component promotion, curb topology and raster
neighborhoods as the main costs; Gaussian-cloud aggregation was secondary.
Profiler timings include instrumentation overhead and were used for ranking,
not as reported latency measurements. One-thread execution was slower in an
early diagnostic (179.8 ms on frame 260), so the original eight-thread setting
was restored.

Before edits, 24 raw frames, coarse outputs, offline masks, configurations and
seven warmed timing calls per frame were frozen under
`output/coarse_optimization`. The final study reloaded the reference source
from a `git archive` export, verified that all 24 reference outputs exactly
reproduced their snapshots, and measured nine warmed reference calls per frame
in the same session. It then measured nine calls for each optimized backend,
alternating the backend order across frames. Every timed call recomputed the
entire coarse pipeline. Path changes, frame loading and warm-up were outside
the timed intervals. The original source path was restored after reference
evaluation.

Mississippi development frames were 150, 260, 300, 326, 370, 450, 550, 600, 700,
850, 900, 1000, 1100 and 1150. Preselected validation frames were 75, 225, 475,
775 and 1125; Downtown frames were 100, 200, 300, 400 and 500. The expanded
checks exposed a ground-propagation equivalence defect, which was corrected;
this final validation set therefore also informed implementation debugging.
It should not be described as an untouched accuracy test set. Each frame has
65,536 raw returns. The 0.3 m XY lattice, 0.5 m height bins, 0.9 m Gaussian
aggregation cells, ROI, feature gates and offline validation thresholds were
retained. README's former 0.3 m height-bin description was corrected to match
the existing configuration.

## Implemented changes

1. **Batch small reductions.** Ground-component medians now use one grouped
   sort and indexed midpoint extraction instead of thousands of scalar median
   callbacks. Already grounded components are promoted together. Road-side
   column bounds, curb peak selection and connected-component shape statistics
   use grouped reductions. Pole configuration and toolbox availability cache
   only configuration or capability data.
2. **Process a compact ground raster.** The ground branch trims empty margins,
   retaining every ground return and a halo derived from the neighborhood
   radii. Bins are computed on the original lattice before subtracting the
   integer crop offset. The structural branch keeps all off-ground returns.
   `pillarOffset` maps compact ground diagnostics back to public frame pillar
   IDs; offline point tests use the corresponding compact cell indices.
   `cfg.compactGroundRaster=false` provides a full-raster control.
3. **Omit unused inverse lookups.** Modern modes keep forward point-to-pillar
   and height-bin membership, but avoid sorting three separate inverse point
   lookups. Explicit inverse lookup requests and the historical dense path
   remain supported. An uncomputed occupied-voxel count is NaN, with
   `hasPointLookup=false`, rather than reporting an invented zero count.
4. **Use small compiled CPU kernels.** `perceptionKernelsMex.cpp` implements
   streaming count/minimum/second-minimum/maximum ground statistics, ordered
   terrain propagation, union-find smooth components, height-constrained road
   flood fill and occupied-pillar shape neighborhoods. Arrays are validated
   before indexing, input arrays are not mutated, and no frame state is cached.
   MATLAB implementations remain selectable and work without a compiler.
5. **Batch Gaussian regularization.** For symmetric 2-by-2 covariance matrices,
   an analytic spectral-projector formula replaces per-component `eig`,
   inverse and determinant calls. Eigenvalue bounds and regularization variance
   are unchanged; inverse and normalization fields retain their meanings.

For covariance entries a, b, d after diagonal regularization, let
c=(a+d)/2, h=(a-d)/2 and r=hypot(h,b). Clip c+r and c-r to the configured
eigenvalue interval, obtaining u and l. The bounded matrix has diagonal
(u+l)/2 +/- s*h and off-diagonal s*b, where s=(u-l)/(2*r), or zero for r=0.
This is the same spectral operation without separate tiny matrix factorizations.

## Equivalence defect found and fixed

The original terrain loop compared a column of neighbor-height residuals with
a row of distance-dependent tolerances. MATLAB implicitly expanded this into
a matrix; `if any(matrix)` effectively required each tolerance column to pass.
Thus the existing decision was equivalent to comparing the smallest residual
with the smallest tolerance, rather than pairing each residual with its own
neighbor distance. An initial native translation paired neighbors directly:
the three semantic masks stayed identical, but seven recorded frames changed
some ground labels and two frames changed curb evidence by up to 0.00281.

Both implementations now express the original conservative rule explicitly:
accept a flat cell when some ground-height residual is below the tolerance
based on the closest accepted ground neighbor. A synthetic mixed diagonal/axis
support test and recorded native/MATLAB ground-mask tests protect this behavior.
Final frozen-reference comparison found zero changed ground points and zero
changed semantic evidence or mixture weights.

Two historical comparison tests initially assumed that diagnostic ground maps
always had full-frame dimensions. They now expand compact candidates into the
original lattice. Reference points outside the compact raster still count as
false negatives. No quality threshold or stored regression reference was
relaxed or regenerated. A native error-handling test also caught overly broad
exception translation; only allocation-related standard exceptions are now
translated, preserving MATLAB's validation error identifiers.

## Validation and build behavior

Final validation contains **77 passed, zero failed and zero incomplete** tests:
the full suite plus a 23-test targeted rerun after exception handling and a
single-anchor vector-orientation edge case were fixed.
The suite covers the original perception/map regression, observer designs,
Gaussian registration math, sparse candidate/offline contracts, anisotropic
ground grids, one-row/one-column pillar neighborhoods and invalid native indices.
All 21 changed MATLAB files passed `checkcode` with factory settings; the local
saved analyzer configuration pointed to a missing R2025b settings file, so the
factory option was scoped to the check without changing user preferences.
The C++ source passed `g++ -std=c++17 -Wall -Wextra -Wpedantic -fsyntax-only`.
An online profile confirmed native calls and no point-feature refinement calls.

`buildPerceptionKernels` builds in a fresh temporary directory, loads that exact
binary for a deterministic road-growth smoke test, then installs it locally.
The current Linux `mex` command reported that its freshly linked ELF was not a
MEX file during post-build inspection, including for a minimal C probe. The
fresh binaries did load and execute correctly in the active MATLAB session.
The build script permits only this specific inspection error to proceed to
runtime verification, emits a warning, and propagates all compilation/link
errors. The tested compiler was GNU g++ 16 with C++17; other native build
environments were not validated here. The generated binary is not committed.

## Reproduction and artifacts

```bash
mkdir -p output/coarse_optimization/reference_code
git archive 1e4f503efdb0052a6d636d079144bdd60532ee07 | \
  tar -x -C output/coarse_optimization/reference_code
```

```matlab
setupVehicleLocalization;
buildPerceptionKernels;
root = pwd;
baseline = fullfile(root,'output','coarse_optimization','reference_code');
snapshots = fullfile(root,'output','coarse_optimization');
% Run capture only when creating a new snapshot set; existing files are protected.
captureCoarsePerceptionBaseline(baseline,snapshots, ...
    '1e4f503efdb0052a6d636d079144bdd60532ee07');
report = evaluateCoarsePerceptionSpeed(snapshots,baseline);
setenv('VEHICLE_LOCALIZATION_DATA_ROOT',fullfile(root,'data'));
runtests('tests');
```

Versioned exports: [per-frame timing](results/coarse_optimization_20260904/runtime.csv),
[semantic/ground fidelity](results/coarse_optimization_20260904/feature_fidelity.csv),
[Gaussian fidelity](results/coarse_optimization_20260904/numerical_fidelity.csv),
[test results](results/coarse_optimization_20260904/test_results.csv),
[analyzer checks](results/coarse_optimization_20260904/code_analyzer.json), and
[online audit](results/coarse_optimization_20260904/online_profile_audit.json).
Raw frames, frozen MAT snapshots, the reference source export, binaries and
full evaluation MAT files stay in their ignored local locations.

Implementation references consulted were MathWorks' documentation for
[grouped accumulation](https://www.mathworks.com/help/matlab/ref/accumarray.html),
[separable raster convolution](https://www.mathworks.com/help/matlab/ref/conv2.html),
[typed MEX array access](https://www.mathworks.com/help/matlab/matlab_external/c-matrix-api-typed-data-access.html),
and [MEX build options](https://www.mathworks.com/help/matlab/ref/mex.html).
The optimization choices and speed claims above come from the repository's
own profiles, implementation analysis and measurements.
