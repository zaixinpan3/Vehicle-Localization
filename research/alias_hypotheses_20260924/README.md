# Mode ambiguity in LiDAR-only map matching: hypotheses, canonicalization and a map pyramid

Date: 2026-09-24. Starting point: the
[frame-851 diagnosis](../frame851_longitudinal_diagnosis_20260924/README.md)
on the 2026-09-24 production chain (revision `1a0075a`, unchanged here).
Every experiment uses the cached 1,170 five-scan source clouds, the recorded
wheel/gyro/lateral odometry and the calibrated map. The recursive replay is
emulated exactly (the "current" mode reproduces the recorded 16.84 cm RMSE
and the 68.48 cm error at frame 851). Reference poses are used for evaluation
and for the reference-seeded ceiling only. Production code is unchanged; all
scripts are in this folder.

## 1. The question

The current solver is a local optimizer: hard nearest-neighbour association
per semantic class, Cauchy robust weights, Gauss–Newton from one seed. At
frame 851 the cost along the road has two attractors 60–70 cm apart. The seed
sits in the wrong one; the reference-seeded solve converges to 3.6 cm with
similarity 0.525 against 0.363. The task was a mathematically clean,
real-time route by which the matcher finds that solution on its own.

## 2. Where the second attractor comes from

`profile851`: along the road through the recursive seed, the similarity has a
local maximum at the seed (0.378), a global maximum at −0.75 m (0.404, the
truth is at −0.69 m) and a barrier at −0.25 m. Per class at the two poses,
curbs and poles favour the truth, signs favour the alias.

The reason is structural. For hard nearest-neighbour association, a local
minimum of the cost at displacement `d` requires that source components can
be re-associated to *different* map components at that displacement, so the
candidate minima are the pairwise offsets between same-class map components
within the association radius. The map has this structure everywhere:

| Class | Components | Same-class pairs 0.25–1.5 m apart | Both weights ≥ 5% of class max |
|---|---:|---:|---:|
| pole | 241 | 67 | 52 |
| trafficSign | 178 | 151 | 90 |

They are tile duplicates (same position, weight split between adjacent
tiles) and split sub-components of one object (0.4–1.2 m apart along the
road, often at different heights). The five-scan source window has the same
duplication (frame 851: three components for one sign, two for one pole).
Two duplicated structures alias against each other like a moiré pattern: at
the alias pose both sub-components find a partner, which is why the alias can
score a *higher* similarity than the truth (frames 175, 196–201, 961–966).

## 3. Route A: alias hypotheses from the map's self-similarity

`enumerateMapAliasSeeds.m` computes the weighted pairwise offsets of the
same-class point components near the seed, clusters them (0.15 m) and returns
seeds `seed ∓ Δ` for the three heaviest clusters. This is the map's own
autocorrelation structure; no grid and no tuning on localization error.

Frozen recorded seeds, 1,170 frames (`runFrozenSeedHypotheses.m`,
`frozen_seed_hypotheses.csv`):

| | RMSE (cm) | P95 | max | >30 cm | >50 cm |
|---|---:|---:|---:|---:|---:|
| current single seed | 16.84 | 34.72 | 68.48 | 77 | 14 |
| reference-seeded ceiling | 10.89 | 20.13 | 36.79 | 3 | 0 |
| alias seeds, best available hypothesis | **10.57** | 19.26 | 36.79 | 1 | 0 |
| alias seeds, selected by similarity | 13.67 | 26.26 | 46.21 | 40 | 0 |
| weak-direction grid, best available | 14.90 | 29.34 | 68.48 | 57 | 11 |

Hypothesis generation is solved: the correct basin is among the candidates
in essentially every frame (best-available 10.57 cm, better than the
reference-seeded solve). A one-dimensional grid along the weakest information
eigenvector is not (14.90). The bottleneck is *selection*: among 38 frames
where the recorded seed and the reference seed converge more than 30 cm
apart, the true basin has the higher similarity in 26. A discounted
cumulative score over time (λ = 0.95) reaches 31 of 38.

Closed loop (`runClosedLoopHypotheses.m`, `closed_loop_summary.csv`):

| Mode | RMSE | P95 | max | >30 cm | >50 cm | frame 851 | ms/frame |
|---|---:|---:|---:|---:|---:|---:|---:|
| current | 16.84 | 34.72 | 68.48 | 77 | 14 | 68.5 | 7.7 |
| alias seeds, similarity selection | 13.52 | 25.53 | 46.21 | 41 | 0 | **3.6** | 91.1 |
| alias seeds, two tracked modes (λ = 0.95) | 13.45 | 25.45 | 46.21 | 39 | 0 | 3.6 | 104.0 |
| alias seeds, screened by one linearization | 14.62 | 32.19 | 46.21 | 64 | 0 | 3.7 | 36.8 |

Route A reaches 3.6 cm at frame 851 and removes every error above 50 cm, but
it rescues 29 frames and harms 27 (new errors at 657–660, 960–966,
1027–1032), and the unscreened form costs 91 ms per frame.

`compareModeStatistics.m` (`mode_statistics.csv`, 54 frames with two accepted
modes) tested selection statistics. Fraction favouring the true mode:
similarity 0.59, robust cost 0.69, mean pole/sign residual 0.65, relative
height association cost 0.72, `−2 log(sim) + heightCost` 0.69. Height decides
the 175–178 and 196–201 segments (sign sub-components differ by 1.1–1.5 m in
z) but not 841–845. In the 961–966 and 1027–1032 segments the map's own pole
components sit 0.3 m from the reference (pole residuals 15 vs 10 at the
"true" pose), so no selector can prefer the reference there; that is a map
consistency defect, not a matcher defect.

## 4. Route B: canonicalize the clouds

If the sub-metre duplicate structure causes the aliases, remove it.
`canonicalizeSemanticCloud.m` merges same-class point components closer than
a radius by moment matching: mean = weighted mean, covariance = within
scatter + between scatter, weights summed; quality fields are weighted means,
counts maxima, masks unions; line classes untouched. The merged map is a
strictly coarser Gaussian mixture of the same field; nothing is invented.

Current single-seed solver on merged clouds (`runCanonicalizedMatching.m`,
`canonicalized_matching.csv`):

| Map r | Source r | Alias pairs left | Frozen RMSE (>30 cm) | Ceiling | Closed loop RMSE | P95 | max | >30 | >50 | frame 851 | ms |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 0 | 142 | 16.84 (77) | 10.89 | 16.84 | 34.72 | 68.48 | 77 | 14 | 68.5 | 6.5 |
| 0.6 | 0 | 28 | 15.89 (66) | 12.19 | 16.34 | 34.52 | 66.89 | 76 | 13 | 66.8 | 6.3 |
| 1.0 | 0 | 5 | 14.45 (45) | 12.82 | 13.97 | 24.74 | 58.78 | 33 | 4 | 18.1 | 6.2 |
| 1.0 | 0.5 | 5 | 14.52 (46) | 12.77 | 14.12 | 24.96 | 58.22 | 36 | 6 | 14.9 | 6.1 |
| **1.5** | **0.5** | **0** | **13.00 (17)** | 13.00 | **13.01** | **22.26** | **36.25** | **17** | **0** | 14.9 | 6.4 |
| 2.0 | 0.5 | 0 | 13.03 (16) | 13.02 | 13.03 | 22.37 | 36.25 | 16 | 0 | 14.9 | 6.2 |

At 1.5 m no same-class pairs remain within the aliasing range, and the frozen
result, the reference-seeded ceiling and the closed loop coincide at 13.00 cm:
within the seed uncertainty the cost is unimodal, so the answer no longer
depends on the seed. The price is precision: the ceiling on the original map
is 10.89 cm, on the merged map 13.00 cm (72 frames worse by more than 10 cm),
because merged Gaussians are wider and their centroids can be biased. At
frame 851 the merged sign centroid lies 0.46 m ahead of the sub-component
the source actually saw, so the error is 14.9 cm rather than 3.6 cm. The
radius is chosen from the map, not from localization error: it is the
smallest radius that leaves no same-class structure at the ambiguity scale.

## 5. Route C: canonical map pyramid (coarse-to-fine)

Solve on the canonical clouds first (basin selection, unimodal), then refine
on the original clouds from that pose (precision). Without position aid, a
refinement that moves more than a trust radius τ from the coarse pose is
replaced by the coarse pose: the fine level is a refinement, not a new
search. With position aid, the fine level keeps the existing GNSS hypothesis
selection unchanged. `runPyramidMatching.m`, `runGatedPyramid.m`
(`pyramid_matching.csv`, `gated_pyramid.csv`), recursive LiDAR-only loop:

| Levels | RMSE | P95 | max | >30 | >50 | frame 851 | ms/frame |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1.5/0.5 only | 13.00 | 22.26 | 36.25 | 17 | 0 | 14.9 | 8.4 |
| 1.5/0.5 → original, ungated | 13.08 | 23.75 | 53.09 | 27 | 1 | 4.9 | 12.6 |
| 2.0/0.5 → original, ungated | 12.79 | 23.39 | 44.26 | 22 | 0 | 4.9 | 12.5 |
| 1.5/0.5 → original, τ = 0.15 m | **12.65** | 22.97 | 39.34 | 18 | 0 | **4.9** | ≈12.6 |
| 2.0/0.5 → original, τ = 0.15 m | 12.66 | 23.15 | 39.34 | 17 | 0 | 4.9 | ≈12.5 |

|fine − coarse| has median 4 cm, P95 14 cm, maximum 32 cm; the gate replaces
the refinement in 46 of 1,170 frames. The ungated fine level re-enters wrong
sub-basins in a few frames (maximum 53 cm); the gate removes them while
keeping the precision gain. Frame 851 refines from 14.9 to 4.9 cm (the
reference-seeded 3.6 cm is the same basin; the remaining 1.3 cm is the coarse
seed's residual pull).

## 6. Observer level

`runObserverWithPyramid.m` runs the full synchronous observer with the
matcher replaced by the pyramid (`observer_*.csv`). Errors after the common
2 s initialization transient:

| Matcher | GNSS + LiDAR RMSE | P95 | max | LiDAR-only RMSE | P95 | max | >30 cm |
|---|---:|---:|---:|---:|---:|---:|---:|
| production (original clouds) | 5.53 | 9.83 | 17.47 | 18.39 | 37.33 | 70.56 | 101 |
| canonical 1.5/0.5 only | 5.78 | 10.91 | 15.96 | 14.81 | 26.50 | 37.85 | 36 |
| pyramid 1.5/0.5 → original, ungated | 5.42 | 9.90 | 17.45 | 14.30 | 27.13 | 42.42 | 39 |
| pyramid 1.5/0.5 → original, τ = 0.15 m | **5.42** | 9.90 | 17.45 | **14.24** | 26.70 | **38.10** | 39 |

Matching time per call (median): 13.0 ms with GNSS hypothesis selection,
6.4 ms LiDAR-only, against 8.8 ms for the production matcher. The canonical
map is built once at load; canonicalizing a source window costs well under
a millisecond.

## Adoption

Route C was adopted into production on 2026-09-24
([record](../canonical_pyramid_20260924/README.md)): `canonicalizeSemanticCloud`
moved to `localization/` and `registerSemanticProbabilityCloud` now runs the
canonical level before the original level with the trust radius on
planar-only refinement. The scripts in this folder evaluated the pyramid on
top of the then single-level driver (revision `1a0075a`); on the current
driver they would nest a second pyramid and are kept as the record of that
evaluation, not as runnable tools.

## 7. Recommended route

1. **Canonical map pyramid in the registration driver** (Route C, gated):
   one canonical level (map 1.5 m, source 0.5 m) solved first without aid,
   the original level refined from it with the existing hypothesis selection
   and a 0.15 m trust radius when no aid is present. It is the same solver
   applied to a coarser mixture of the same field, seed-independent at the
   coarse level, 12–13 ms per call, and it improves both the fused
   (5.53 → 5.42 cm) and the LiDAR-only (18.39 → 14.24 cm, maximum
   70.6 → 38.1 cm) trajectories. Frame 851: 68.5 → 4.9 cm.
2. **Canonical publication at the map builder**: merge tile duplicates and
   sub-components at publication so the coarse level is the map itself and
   the split structure never reaches the matcher; keep the fine components
   only where they are physically distinct objects.
3. **Alias hypotheses as a safety net, not the primary mechanism**: with a
   canonical coarse level the residual alias set is empty on this map; the
   hypothesis generator remains useful as a diagnostic and for maps that
   cannot be canonicalized.
4. **Map consistency audit** for the 961–966 and 1027–1032 stretches, where
   the map disagrees with the reference by about 0.3 m and no matcher can
   recover the reference.

Considered and not chosen: soft-association annealing (GNC/EM-ICP). It
attacks the same duplicate structure by blurring, but it replaces the tuned
solver and its information semantics, and it would converge to whichever
basin holds the larger blurred mass, which is what similarity selection
already measures. The canonical level achieves the blurring explicitly, with
the solver unchanged.

Limits: one drive, same-drive INSPVA map; the 1.5 m and 0.15 m radii were
chosen from the map's alias structure and the |fine − coarse| distribution
on this drive, not validated on another; the canonical level lowers the
per-frame precision ceiling from 10.9 to 13.0 cm, which the gated refinement
only partly recovers (12.65 cm).

## Reproduction

From the repository root, after `output/mncav_coarse_localization_20260924`
exists with its `sources.mat` cache:

```matlab
addpath(pwd); setupVehicleLocalization; addpath('research/alias_hypotheses_20260924');
runFrozenSeedHypotheses;      % Route A, frozen seeds
runClosedLoopHypotheses;      % Route A, closed loop (current, alias_select, alias_track)
runClosedLoopHypotheses("alias_screen");
compareModeStatistics;        % selection statistics on ambiguous frames
runCanonicalizedMatching;     % Route B
runPyramidMatching; runGatedPyramid;   % Route C
runObserverWithPyramid([1.5 0.5],'canonical150');
runObserverWithPyramid([1.5 0.5;0 0],'pyramid150_0');
runObserverWithPyramid([1.5 0.5;0 0],'pyramid150_0_gated015',0.15);
analyzeClosedLoop; plotAmbiguityStudy;
```

Deterministic; MATLAB R2026a. Large intermediate files are under
`output/alias_hypotheses_20260924`.
