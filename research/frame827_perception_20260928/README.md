# Frame 827 perception: pole recovery and unresolved curb errors

The user's visual criticism is supported by the frozen-reference comparison.
The preceding map-matching diagnosis established an association problem; it
did not establish that the perception inputs were correct. This task deploys
a Mississippi-specific pole operating point and investigates curb ownership.
**Curb quality in frame 827 remains unresolved.** No unsuccessful curb model
or restrictive veto was installed merely to improve one reported fraction.

## Inspected frame

Reference-empty means a selected 0.6 m pillar contains zero original 0.3 m
fine-detector points of that class. Coverage counts all reference points in
the ROI, including those rejected by preprocessing or proposal generation.
These are detector-reference agreements, not independent physical labels.
The original fine masks are unchanged and reproduced exactly at frame 827.

| Feature | Before selected / reference-empty | After selected / reference-empty | Covered reference points before / after |
|---|---:|---:|---:|
| Pole | 0 / 0 | **2 / 0** | **0/192 -> 125/192** |
| Curb | 28 / 9 | 28 / 9 | 50/65 -> 50/65 |
| Traffic sign | 5 / 0 | 5 / 0 | 40/40 -> 40/40 |

An empty selection has undefined false-selection fraction, not perfect
precision. Curb reference-empty fraction is **32.14%**, and its missed-point
fraction is **23.08%**. There are nine missed reference-bearing curb pillars
and nine selected reference-empty curb pillars. The fact that the full-route
aggregate was below 10% does not make this frame satisfactory.

## Pole cause and deployed calibration

The existing continuous shaft generator already proposes four reference-
bearing owners. Their reference-point counts and maximum learned ranks are:

| Original coarse pillar | Reference points | Rank |
|---|---:|---:|
| 5027 | 49 | 0.264298 |
| 5833 | 74 | 0.913169 |
| 6670 | 17 | 0.726405 |
| 7047 | 51 | 0.876729 |

The previous common threshold, 0.9371209151674477, rejects every one. Another
one of the 192 reference points is outside these exported proposal-owner rows.
Thus the empty final output does not mean the shaft distribution search found
no pole-like support. It primarily reflects the final operating point.

`pillarPoleDistributionConfig(profile)` now supplies a Mississippi-only
threshold of **0.8701683227450074**; `perceptionConfig` applies it only on the
0.6 m coarse path. The shared/default structural configuration and Downtown
retain 0.9371209151674477. Offline 0.3 m mode is untouched. The numeric model's
trees, feature names, normalization and fitted weights are unchanged; only
per-profile threshold metadata is added. No fine labels or frame IDs enter
online inference, and no finer XY lattice is created.

Selection maximizes Mississippi reference-point coverage under a **7.5%
prefix out-of-fold reference-empty budget**, using the existing five purged
folds on frames 1--780 (20-frame purge; original model seed 927). The chosen
prefix result is 74/1,002 = 7.385% reference-empty, covering 97,395/124,708
points. This is a deliberate change from the common 5% margin; the user's
10% aggregate objective is retained, not a claim of unchanged precision.
The suffix and frame 827 are reused development checks, not independent tests.

| Evaluation | Before false / selected | After false / selected | Point coverage before -> after |
|---|---:|---:|---:|
| Reused suffix 781--1170 | 20/397 (5.04%) | 39/505 (**7.72%**) | 62.38% -> **70.46%** |
| All 1,170, includes fitted prefix | 30/1,222 (2.45%) | 64/1,533 (**4.17%**) | 70.48% -> **78.05%** |

The revision adds both correct-reference and reference-empty owners. It is
not a false-positive reduction for poles; it restores coverage within the
stated aggregate error ceiling. Some individual frames can exceed that ceiling.
Two selected pillars are not asserted to be exactly two distinct physical poles.
Frame 827 still misses 67/192 reference pole points after this improvement.

## Curb diagnosis

All **65/65 reference curb points survive ground routing and lie in original
coarse proposals**. The final classifier retains 50, while accepting nine
neighboring cells with no reference curb point. Neither relaxed ground
segmentation nor broader proposal generation addresses this frame's bottleneck.

For example, the reference-bearing cells centered at (4.6,-3.2) and
(5.2,-3.2) m have ranks 0.228 and 0.191, while their neighboring reference-empty
cells at Y=-2.6 m have ranks 0.958 and 0.968. Raising the threshold removes
additional true support without fixing that ordering; lowering it admits more
surrounding cells. The problem involves the spatial ownership of the boundary
and final discrimination, rather than simply a lack of geometric proposals.
`curb_cells.csv` preserves all candidate/reference-cell descriptors and gates.

The fine reference is sparse: its right-side output includes a gap between
approximately X=5.13 and 6.99 m. Not every reference-empty cell can therefore
be called a physically false curb without inspecting the raw geometry. This
limitation does not change the user's exact zero-reference-point metric or
justify reporting the current frame as acceptable.

## Implemented curb experiments, not deployed

All fitted models use only frames 1--780; no model is fitted on frame 827.
Thresholds for the two boosted models maximize prefix covered points under
5% out-of-fold reference-empty selections. They use five contiguous folds,
20-frame purges, seed 928, 250 boosting iterations, depth at most six and
point-count weighting as recorded in the scripts. Labels join features offline.

1. **Signed neighboring distributions:** add 80 descriptors from eight
   existing 0.6 m neighbors (presence and signed height, roughness, energy,
   plane residual and mid-height-support differences). The fitted 237-feature
   model still selects 28 cells with nine reference-empty and covers 51/65.
   Core suffix false fraction is 1,062/10,416 = 10.20%, before recovery additions.
2. **Whole-pillar joint moments:** capture 71 new descriptors over every
   original coarse proposal in all 1,170 raw scans. They preserve mixed XY-height
   polynomial moments through degree four, using owner-centered height and
   neighborhood-plane residuals. No extra bins or per-point semantic decisions
   are introduced. The 228-feature boosted model again selects 28 cells with
   nine reference-empty and covers 51/65. Core suffix false fraction is
   1,015/10,297 = 9.86%, before recovery additions. It fails to resolve this
   inspected failure and is not promoted.
3. **Distribution-neighbor evidence:** examine basic and full descriptor banks
   and a bounded 10,000-vector prefix bank. A 30-neighbor unanimous veto on the
   bounded bank reduces frame-827 errors to 1/10 but covers only 23/65 points.
   On the suffix it reduces core coverage to 15,510 points, before recovery.
   This is an unacceptable coverage loss for the inspected miss problem.
4. **Continuous profile inflection:** exploratory cubic height-profile fits
   within 0.75, 1.05 and 1.5 m metric neighborhoods use raw current-frame points,
   not a finer grid. `inflection_probe.csv` shows unstable/local out-of-owner
   inflection estimates; no acceptance rule or deployment claim is made.

These outcomes are negative results. More local descriptors or a stricter
neighbor veto did not simultaneously resolve reference-empty selections and
missing curb ownership here. A reliable boundary-location/support mechanism
remains an open task; the current default curb classifier and recovery are
retained. No full-route matching replay with the revised pole selections was
run, so no new localization-accuracy improvement is claimed.

## Validation and visualization

- All 1,170 raw Mississippi pole-frame replays exactly match exported owner
  scores at the new threshold. Reference denominators and the aggregate counts
  above independently match `pole_replay.csv`.
- Thirty tests pass across pole distributions, Mississippi curb recovery and
  semantic precision, including default/alias isolation, unchanged Downtown
  configuration, original fine masks, continuous shaft controls, empty inputs
  and backend/channel consistency. An initial test invocation omitted the root
  from the path when the runner changed directory; its setup errors were fixed
  by explicitly adding the root before rerunning. They are not counted as passes.
- Frame-827 source preprocessing and grid geometry are unchanged, curb/sign
  selected IDs are identical before/after, and original fine masks reproduce.
- Factory Code Analyzer reports no issues in eight implementation/test/study
  MATLAB files. The research Python scripts compile successfully.
- The existing niri viewer is refreshed with the two accepted pole pillars;
  its title explicitly states that curb is unchanged. All 65,536 source points
  remain present, reference returns are recolored in place at uniform size 4,
  and no oversized feature-point overlay is added.

No timing gain or independent-drive generalization claim is made. Downtown
recordings are not replayed here beyond the existing pole-context regression
fixture; its runtime defaults and model remain unchanged. Large data tables,
trained experimental pickles and MAT caches stay in ignored `output/`.

## Reproduction

```bash
uv run --offline --with numpy --with pandas python research/frame827_perception_20260928/selectMississippiPole.py
matlab -batch "addpath(pwd); addpath('research/frame827_perception_20260928'); diagnosePerception827; replayPoleCalibration; validatePerception827"
matlab -batch "addpath(pwd); addpath('research/frame827_perception_20260928'); captureCurbJointMoments"
uv run --offline --with scikit-learn==1.9.1 --with pandas python research/frame827_perception_20260928/probeJointCurb.py
```

Other alternative scripts and their recorded results are in this directory.
The before/after comparison is in `frame827_comparison.csv`; the exact revised
viewer geometry and selections are in `features_0827.json`.
