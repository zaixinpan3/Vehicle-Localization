# Fast Voxel-Only Semantic Probability Cloud

Date: 2026-09-04

## Research question

Can one LiDAR frame be converted directly into a sparse, two-dimensional,
semantic NDT probability cloud without point-level semantic refinement, while
substantially reducing latency and retained memory and preserving the spatial
support of the current curb, road-marking, and pole outputs?

The question is feasible because the repository contains the implementation,
stored regression outputs, and recorded Mississippi frames. It is relevant to
the intended probability-cloud-to-probability-cloud mapping pipeline. The
engineering study is non-human-subject research; IRB review is not applicable.

In scope:

- cell-level curb and road-marking classification;
- column-level pole classification;
- sparse 2D Gaussian output with semantic and hit-support probabilities;
- semantic distribution-to-distribution scoring;
- fidelity, latency, and retained-output-size measurements.

Out of scope:

- replacement of the full point-level perception path;
- learned probability calibration;
- ray-traced free-space occupancy updates;
- pose optimization over the D2D score or multi-frame map fusion;
- claims of generalization beyond the evaluated Mississippi frames.

## Research method

This is a quantitative implementation benchmark. The current full perception
result is treated as a non-regression reference, not as ground truth. Frames
260, 300, and 326 are the stored reference/tuning set. Frames 270, 280, 290,
310, 320, 340, and 350, followed by frames 360, 420, 500, 650, 800, 950, and
1100, were exploratory development sets. In particular, the second set exposed
inadequate pole recall in the initial simplified gate and was used to select
the final pole thresholds. The thresholds were then locked. Frames 370, 450,
550, 700, 850, 1000, and 1150 form a fresh validation set that was not inspected
during parameter selection.

The primary quality unit is occupied semantic cell support, because comparing
raw point counts would contradict the voxel-only objective. Precision, recall,
and F1 are measured both on the 0.3 m source cells and on the final 0.9 m NDT
cells. Steady-state latency is measured with `timeit`; the first-call result is
reported separately. MATLAB `whos` supplies retained variable sizes.

## Literature search and verification

Searches were run on 2026-09-04 using IEEE/DOI metadata, institutional
publication pages, arXiv, CVF Open Access, and publisher full text. Search terms
combined `normal distributions transform`, `distribution-to-distribution NDT`,
`NDT occupancy map`, `semantic NDT`, `NDT map compression`, `road marking`,
`pole`, and `vehicle localization`. Seminal work was included despite age;
secondary summaries and papers without a direct representation or matching
implication were excluded.

The seven included records are all primary computational or experimental
engineering papers. That methodological concentration is intentional for this
algorithm-design question, but it means the review is not evidence about human
or operational outcomes.

| Source | Verification | Design implication |
| --- | --- | --- |
| Biber and Strasser (2003), *The Normal Distributions Transform: A New Approach to Laser Scan Matching*, DOI [10.1109/IROS.2003.1249285](https://doi.org/10.1109/IROS.2003.1249285) | DOI and paper metadata resolve | A 2D plane can be partitioned into cells and each occupied cell represented by a local Gaussian, avoiding explicit point correspondences. |
| Stoyanov, Magnusson, and Lilienthal (2012), *Point Set Registration through Minimization of the L2 Distance between 3D-NDT Models*, DOI [10.1109/ICRA.2012.6224717](https://doi.org/10.1109/ICRA.2012.6224717) | DOI and institutional metadata resolve | NDT mixtures can be compared directly with a closed-form Gaussian-overlap objective; a point-to-map interface is not required. |
| Saarinen et al. (2013), *3D Normal Distributions Transform Occupancy Maps*, DOI [10.1177/0278364913499415](https://doi.org/10.1177/0278364913499415) | Publisher metadata resolve | NDT geometry and occupancy evidence are complementary, but a true occupancy posterior needs an inverse sensor model and free-space evidence. |
| Seichter et al. (2022), *Efficient and Robust Semantic Mapping for Indoor Environments*, [arXiv:2203.05836](https://arxiv.org/abs/2203.05836) | arXiv record and paper resolve | Semantic probabilities can be stored with NDT cells; speed does not remove dependence on upstream segmentation quality. |
| Manninen et al. (2023), *Towards High-Definition Maps: A Framework Leveraging Semantic Segmentation to Improve NDT Map Compression and Descriptivity*, [arXiv:2301.03956](https://arxiv.org/abs/2301.03956) | arXiv record and author metadata resolve | Semantic-aware NDT grouping can improve compactness, and approximately 0.5--1.0 m cells are a plausible vehicle-map scale. |
| Kim and Jee (2020), *Free-Resolution Probability Distributions Map-Based Precise Vehicle Localization in Urban Areas*, DOI [10.3390/s20041220](https://doi.org/10.3390/s20041220) | Publisher full text and DOI resolve | Road markings and vertical structures are useful probability-map channels with complementary localization geometry. |
| Lee et al. (2023), *LiDAR-Based Localization on Highways Using Raw Data and Pole-Like Object Features*, [CVF Open Access](https://openaccess.thecvf.com/content/CVPR2023W/WAD/papers/Lee_LiDAR-Based_Localization_on_Highways_Using_Raw_Data_and_Pole-Like_Object_CVPRW_2023_paper.pdf) | CVF paper resolves | Pole features can widen the useful convergence region of NDT-like highway localization, supporting a distinct pole semantic channel. |

The evidence converges on a compact mixture-of-Gaussians representation and
same-semantic matching. It does not establish that the repository's heuristic
scores are calibrated posteriors. Consequently, the implementation names them
semantic evidence probabilities and labels occupancy as hit-based support.

## Selected architecture

```text
organized frame
  -> canonical voxelization
  -> slope-grid ground segmentation
  -> ground 2D cells
       -> curb energy + road adjacency (no curb-point selection/thinning)
       -> road-cell reflectivity maximum (no marking-point output)
  -> sparse off-ground occupied voxels
       -> 2D vertical runs + point/line shape
       -> pole candidate and 2D shape gate (no dense copy, no 3D slice refinement)
  -> selected 0.3 m source-cell sufficient statistics
  -> aligned 0.9 m sparse semantic NDT components
  -> semantic D2D Gaussian-overlap score
```

The full `perceiveFrame` behavior remains the default and is regression-tested.
`perceiveCoarseProbabilityCloud` explicitly selects the new path.

### Source-cell representation

An accepted source cell contributes only

\[
V_i = \{n_i,\;\boldsymbol\mu_i,\;\mathbf\Sigma_i^0,\;p_i^{sem}\},
\qquad
\mathbf\Sigma_i^0 =
\operatorname{diag}(\Delta x^2/12,\Delta y^2/12),
\]

where `n_i` is the hit count, the mean is the cell center, and the covariance
is the uniform-cell prior. Raw points are not given semantic labels and are not
expanded again during probability-cloud construction.

For all source cells assigned to output cell `c`, weighted moment matching gives

\[
N_c=\sum_i n_i,\qquad
\boldsymbol\mu_c=\frac{1}{N_c}\sum_i n_i\boldsymbol\mu_i,
\]

\[
\mathbf\Sigma_c=
\frac{1}{N_c}\sum_i n_i\left(
\mathbf\Sigma_i^0+\boldsymbol\mu_i\boldsymbol\mu_i^T\right)
-\boldsymbol\mu_c\boldsymbol\mu_c^T.
\]

Eigenvalue limits then make every covariance positive definite. The stored
hit-support probability is

\[
p_c^{hit}=1-\exp(-N_c/k),
\]

and the mixture mass is proportional to
`p_c^hit * p_c^sem`. This is deliberately not called a complete occupancy
posterior: no ray misses or free-space updates are available in a single-frame
feature extractor.

### D2D matching interface

For same-semantic fixed component `i` and transformed moving component `j`, the
implemented cross term is

\[
I_{ij}=\mathcal N(\boldsymbol\mu_i;
\mathbf R\boldsymbol\mu_j+\mathbf t,
\mathbf\Sigma_i+\mathbf R\mathbf\Sigma_j\mathbf R^T).
\]

`scoreSemanticProbabilityCloudAlignment` sums weighted same-class cross terms
and normalizes them by the two self inner products. Identical clouds at the
identity pose therefore score one. The function supplies a verified map-to-map
objective; pose search and analytic derivatives remain future work.

## Optimization decisions

1. The full point masks, curb-point selector, dominant-boundary thinning, and
   3D pole slice validation are absent from the fast path. Road-marking
   threshold estimation is shared with the full path, but the coarse branch
   applies it only after reflectivity has been reduced to source-cell maxima;
   it never constructs a road-marking point mask.
2. The off-ground branch sorts occupied voxel indices and computes vertical
   runs sparsely. It does not allocate the second dense `[Ny,Nx,Nz]` structural
   tensor. Its pillar count, z range, occupied-layer count, maximum run, point
   score, and line score are bit-for-bit equal to the dense implementation on
   all ten regression-evaluated frames.
3. Three curb-refinement passes that did not improve recall on the stored
   reference set are disabled only in the coarse configuration. The road-facing
   continuation and completion passes needed for recall remain enabled.
4. Pole candidates retain the existing global column logic and sparse
   component-wide layer test. The development-selected 2D terminal gate is
   `pointScore >= 0.60` and `lineScore <= 0.75`; after parameter lock it retained
   every full-result pole cell on the validation set, at the cost of additional
   low-confidence candidates.
5. The output resolution is 0.9 m, exactly three 0.3 m source cells. A 1.0 m
   grid cut source cells at changing phase offsets and reduced cell-support
   recall; the aligned resolution removes that avoidable aliasing.

An ablation that bypassed the entire road-adjacency curb refinement was
rejected. On the stored reference frames it reduced source-cell curb precision
from 0.7015 to 0.1234 and recall from 0.9764 to 0.8180; locked-validation curb
recall fell to 0.7882 and road-marking support also degraded because the road
mask changed. A five-call MATLAB profile on frame 260 located approximately
43% of profiled inclusive time in the coarse ground-feature branch, 26% in
ground segmentation, and 15% in the off-ground branch. Thus ground processing
is the next optimization target, but removing its spatial validation wholesale
does not satisfy the quality constraint.

## Results

The tables compare the optimized result with the unchanged full perception
output projected to the same cell grids.

### Stored reference frames 260, 300, and 326

| Channel | Source-cell precision | Source-cell recall | Source-cell F1 | 0.9 m NDT precision | 0.9 m NDT recall | 0.9 m NDT F1 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Curb | 0.7015 | 0.9764 | 0.8165 | 0.8030 | 0.9938 | 0.8883 |
| Road marking | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Pole | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |

### Locked validation frames 370, 450, 550, 700, 850, 1000, and 1150

| Channel | Source-cell precision | Source-cell recall | Source-cell F1 | 0.9 m NDT precision | 0.9 m NDT recall | 0.9 m NDT F1 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Curb | 0.6270 | 0.9399 | 0.7522 | 0.7500 | 0.9448 | 0.8362 |
| Road marking | 0.9950 | 1.0000 | 0.9975 | 0.9875 | 1.0000 | 0.9937 |
| Pole | 0.5833 | 1.0000 | 0.7368 | 0.6538 | 1.0000 | 0.7907 |

High recall is intentional at this stage: low-confidence extra components carry
less mixture mass, while dropping a true curb or pole removes information that
later map matching cannot recover. Pole precision on the locked validation set
is only moderate, so temporal map stability and probability-cloud matching must
reject persistent false candidates; the result does not justify treating every
pole component as a hard landmark.

### Runtime and retained output size

MATLAB R2026a Update 3, current desktop session, `timeit`, Mississippi frames:

| Frame | Full perception (s) | Dense semantic product (s) | Legacy total (s) | Optimized total (s) | Speedup | Dense product bytes | Optimized bytes |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 260 | 0.14288 | 0.03159 | 0.17447 | 0.10234 | 1.70x | 115,774,073 | 27,381 |
| 300 | 0.13466 | 0.03219 | 0.16684 | 0.09687 | 1.72x | 122,097,077 | 23,719 |
| 326 | 0.11973 | 0.02397 | 0.14370 | 0.09568 | 1.50x | 79,753,037 | 30,231 |

The median end-to-end steady-state speedup is 1.70x. The sparse retained output
is 2,638--5,148 times smaller than the dense semantic product alone. A separate
first-call measurement on frame 260 was 0.41557 s optimized versus 0.54523 s
for full perception plus dense-product construction (1.31x); first-call timing
includes MATLAB parsing and JIT effects.

The earlier observation that "fine validation" took less time than "coarse
validation" arose from mismatched stage boundaries: the former timed only
`refineSemanticPoints`, after the expensive point-level feature work had
already run in `perceiveFrame`. The benchmark above compares complete paths
from the same frame input to their respective retained outputs.

### Verification

MATLAB Code Analyzer reported no issues in the 14 changed or added MATLAB
files. The dedicated coarse-cloud suite passed all five tests, including the
stored reference set, the locked validation set, empty/missing-scalar inputs,
positive-definite sparse components, and identity-versus-shifted D2D scores.
The two full-pipeline regression tests and five existing semantic-NDT and
temporal-map tests also passed. The complete repository run reported 38 passed,
zero failed, and two assumption-filtered observer-synthesis tests because
YALMIP and the required SDP solvers were not on the MATLAB path.

As a concrete interface check, frame 260 produced 82 sparse components: 68
curb, 13 road-marking, and one pole component. Its identity D2D score was
1.000000, while a 1.8 m longitudinal shift reduced the score to 0.566316.

## Devil's-advocate review and limitations

Verdict: proceed as an optimized coarse path, but do not replace the full path
or claim calibrated occupancy.

The strongest counterargument is that agreement with the current full
algorithm can preserve its errors. This study establishes non-regression and
computational behavior, not absolute perception accuracy. The ten formal
reference and locked-validation frames come from one route, so weather, sensor,
route, and domain-shift validation remain open. Pole precision is moderate on
the locked set. The semantic evidence transfer is monotonic but not calibrated
against labeled ground truth. The hit-support probability omits free-space
evidence. Finally, the 0.9 m alignment assumes the configured 0.3 m source
cells; changing source resolution must trigger a configuration and regression
review.

These limitations are handled operationally by keeping the full path intact,
making the fast mode explicit, locking parameters before the validation run,
exposing probability fields separately, and documenting the occupancy
qualification.

## Reproduction

```matlab
setupVehicleLocalization();
setenv("VEHICLE_LOCALIZATION_DATA_ROOT", fullfile(pwd, "data"));
runtests("tests/coarseSemanticProbabilityCloudTest.m");
benchmarkCoarseProbabilityCloud(fullfile(pwd, "data"), [260 300 326]);
```
