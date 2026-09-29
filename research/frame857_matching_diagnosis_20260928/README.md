# Frame 857: missed pole and coarse-map/trust-gate bias

The current post-initialization maximum, frame 857, has **47.3865 cm** position
error and **+1.1151 degrees** heading error. Both the missed pole and map
canonicalization matter. Adding only the inspected pole's reference-bearing
owners, with whole-pillar moments and normal temporal confirmation, reduces
error to **6.8993 cm**. An ordinary detector-threshold experiment, without
reference-selected owners, produces **7.7977 cm**. Neither experiment is
deployed: lowering the global threshold violates the requested false-selection
limit on the reused route suffix.

## Visualization and perception

The niri window titled `Mississippi 857 | fine reference and coarse feature
pillars` displays all 65,536 points. Original fine-reference points are
recolored in place at uniform size 4: orange pole, magenta sign and cyan curb.
Current 0.6 m selections appear on the nearby floor at Z = -3.1306 m. No point
overlay is added. The feature-region camera is selected; full-cloud data are
retained for navigation. Full-scene and focused PNGs plus metadata are saved.

The inspected pole is near sensor XY (16.97, -3.71) m, in pillar 7844. Its 13
fine-reference points span 1.882 m vertically. The whole off-ground pillar
contains 24 points over 3.623 m: it includes support beyond the reference
subset. The shaft hypothesis survives proposal generation and has a 3.339 m
support height, 0.0732 m radial RMS and isolation 1. Its learned score is
**0.8301088521**, below the deployed Mississippi threshold **0.8701683227**.
Thus this is a final classification rejection, not an absent proposal.

Curbs have 32 selected pillars, four reference-empty, and 89/113 reference
points covered. Signs have five selected pillars, zero reference-empty,
and 51/51 in-ROI reference points covered (53 total reference points).
These are comparisons with stored fine-detector labels, not physical ground
truth. The temporal matching source contains 26 curb and five sign Gaussians,
with no pole. Canonicalization reduces the signs to three Gaussians.

## Controlled matching results

All comparisons retain the stored frame prediction and original registration
settings unless the intervention is explicitly named. Oracle reference seeds,
reference-motion alignment and reference-owner selection are diagnosis only.

| Intervention | Position error (cm) | Heading error (deg) |
|---|---:|---:|
| Current production | 47.387 | +1.1151 |
| Start exactly at reference pose (oracle) | 47.386 | +1.1151 |
| Accept existing unrestricted fine refinement | 12.039 | +0.2347 |
| Disable map merging | 12.039 | +0.2347 |
| Current scan only | 39.453 | +0.6478 |
| Reference motion for temporal alignment (oracle) | 42.547 | +1.0359 |
| Curb only | 55.404 | +1.7060 |
| Sign only | 63.244 | +1.4890 |
| Add inspected pole owners (oracle selection) | 6.899 | +0.2898 |
| Detector threshold 0.83, normal selection | 7.798 | +0.2854 |

The original prediction already has 42.674 cm error. Production increases it
to 47.387 cm, consisting of +44.956 cm forward and -14.981 cm left. Exact
reference initialization still converges to the same displaced pose, so prior
trajectory error alone cannot explain the result. Reference window motion
reduces error by only about 4.84 cm; odometry alignment is not sufficient to
resolve the failure.

The 1.5 m map merging radius combines sign-map components 1263 and 1264,
separated by 0.9645 m, with their near duplicates 1277 and 1278. A second
merged group contains 1287, 1286, 1279 and 1280. For example, canonical source
16 is 0.140 m from original target 1279 but 0.435 m from its merged target.
These merged distributions and the curb constraints yield a biased coarse
solution. Map radii up to 0.75 m yield about 12.04 cm error; radii at least
1 m yield about 47.39 cm. These modes are not asserted to represent different
physical signs; this experiment identifies a representation/association bias.

In contrast with the earlier frame 827 failure, **fine refinement here is
already better**: it reaches 12.039 cm from prediction, reference or the
biased coarse pose. However, its 35.51 cm displacement from the coarse pose
exceeds the 15 cm trust radius, so production discards it and retains the
47.39 cm result. Loosening that gate is not a general fix: the earlier 827
control showed that unrestricted refinement could worsen another frame.

## Missed-pole intervention and precision limit

`testMissingPole857` reconstructs frames 853:857. The oracle uses frozen fine
labels and recorded poses only to identify owners of the inspected pole
within 1 m of its reference world location. Those owners appear in frames
855, 856 and 857, and become one temporally confirmed source Gaussian.
It changes only their selection mask/probability; all off-ground pillar means,
covariances, curb/sign decisions, motion and matcher parameters are retained.
A rebuild with no intervention exactly reproduces the original source.
Probabilities 0.9 and 1.0 both yield 6.899 cm, with one actual pole match.
This is causal diagnostic evidence for the importance of the missed pole,
not a deployable oracle or an independent accuracy guarantee.

The non-oracle control recomputes all five raw scans with threshold 0.83 and
otherwise unchanged perception. It also supplies one confirmed/matched pole,
reaches 7.798 cm, and allows fine refinement (10.56 cm coarse-to-fine shift).
This is a fixed-prediction local experiment, not a new recursive full route.

Screening existing frozen model predictions across 1,170 scans gives:

| Threshold | All-frame reference-empty selections | Reused suffix 781:1170 |
|---|---:|---:|
| 0.8701683227 | 64/1533 = 4.17% | 39/505 = 7.72% |
| 0.83 | 96/1672 = 5.74% | 60/565 = 10.62% |

Therefore a blanket threshold reduction exceeds the 10% false-selection target on that suffix.
A future detector improvement should recover this concentrated continuous
shaft while separating it from the additional false owners; this diagnosis
makes no claim to have implemented that discriminator. It also does not
remove the separately demonstrated map/trust-gate problem.

## Validation and reproduction

From the repository root, add the root and call `setupVehicleLocalization`,
then add this study directory. Run `diagnoseFrame857`, `inspectBasins857`, and
`testMissingPole857` in MATLAB R2026a. The experiments use the saved maximum
object from `output/revised_route_max_20260928/results.mat`, calibrated map
cache, raw scans and original frozen fine references. No new model is trained.
Run `analyzeDiagnosis.py` with numpy/pandas/matplotlib for independent numeric
assertions, the threshold screen and PNG/PDF comparison plot.

Production pose reproduction is exact. Raw source reconstruction is exact.
Three MATLAB scripts have clean factory Code Analyzer results after removing
two unused inspection variables. An initial pole diagnostic used unavailable
`range`; replacing it with `max-min` allowed both pole-control runs to finish.
The owner CSV's frame-reference counts/heights describe the tracked reference
subset per frame; off-ground counts/heights are per owner. Large MAT caches
remain local in `output/frame857_matching_diagnosis_20260928/`.
No production algorithm or parameter is changed by this study.
