# Frame 94 raw LiDAR matching: reference-point geometry investigation

Prepared and executed on September 21, 2026. Sequence: Mississippi, 1,170
frames. This investigation follows the [fusion error diagnosis](../frame94_fusion_diagnosis_20260921/README.md).

## Finding

The reported **92.80 cm is frame 94's individual horizontal raw LiDAR matching
error**, not the sequence RMSE. A missing effective translation between the
stored point-cloud origin and the recorded pose reference point explains most
of this error in controlled experiments. Turning exposes the problem because
the missing translation rotates with vehicle attitude.

A translation fitted without frame 94, and then applied consistently to map
observations and online coarse distributions, reduces this frame's error from
**92.7991 cm to 13.9882 cm** using the original recorded prediction, three-scan
source window, feature selections, map inference settings and matching settings.
With a single coarse scan, the corresponding comparison is **70.9111 cm to
8.8015 cm**. These are diagnostic results; no production calibration, map,
perception threshold or localization configuration was changed.

The estimated increment is approximately **[2.2681, 0.1541, 0] m** in stored
point axes, after their existing rotation. This is an *effective coordinate
offset inferred from same-drive consistency*, not a measured physical mounting
position. Pose-reference conventions, uncalibrated mounting rotation, residual
latency and scan distortion can partly influence its fitted value.

## Coordinate-transform evidence

The current path is:

1. `scripts/extractPointCloudsFromBag.m` rotates XYZ before writing the stored
   cloud; it does not apply a translation. Rotating axes does not move their
   origin.
2. `config/lidarFrameCalibrationConfig.m` defaults to an identity rotation and
   **zero translation**. The synchronized experiment and map use that setting.
3. `scripts/collectFeatureObservations.m` applies that calibration and then the
   full recorded attitude and reference position to construct global map points.
4. `perception/perceiveFrame.m` composes the same calibration with tilt alignment
   when constructing the online coarse probability cloud.

Let `p` be a stored point, `R_i` the attitude and `t_i` the recorded reference
position in frame `i`. The current zero-translation assumption gives

```text
current:  p_map = R_i p + t_i
offset:   p_map = R_i (p + b) + t_i
```

If a nonzero offset `b` is needed, each current projection misses `R_i b`.
Its direction changes during a turn. A matching displacement between query
frame `q` and target frame `f` can then approximately follow

```text
delta_position(q, f) = (R_q - R_f) b.
```

A 2.27 m planar offset and a 30 degree heading difference yield about 1.18 m
of relative displacement. This illustration does not assume that all pairs
in the experiment have that exact heading difference.

## Evidence before GMM fitting

The mismatch already occurs when a coarse query is registered against a
**single other frame's fine-feature distributions**, without multi-frame GMM
fitting or temporal-stability weighting. For frame 94's three-scan coarse
query, using the recorded reference as a diagnostic seed:

| Target observation frame | Original error (cm) | Offset error (cm) |
|---|---:|---:|
| 70 | 120.46 | 17.50 |
| 80 | 88.59 | 3.67 |
| 90 | 24.54 | 2.46 |
| 93 | 11.82 | 5.37 |
| 95 | 7.21 | 5.34 |
| 100 | 40.16 | 1.20 |
| 105 | 63.79 | 15.02 |
| 110 | 73.77 | 22.61 |

All these listed matches are accepted. The original error grows across
different turning attitudes, while temporally adjacent observations align much
better. The complete table includes frame 115, whose before/after matches are
both rejected for insufficient overlap; its errors are not counted as accepted
localization results. This control points to cross-frame projection consistency
as a primary problem rather than treating every mismatch as a classifier error.

`investigate_geometry.m` fixes training frames to
`[60 65 70 75 80 85 90 100 105 110]`, pairs frames separated by at most 25 frames
with at least 0.08 rad yaw change, and performs reference-seeded fine-to-fine
registration. Of 28 pairs, 26 pass the declared quality screen: accepted match,
absolute yaw error below 1 degree and matched fraction at least 0.2. There is
no position-error screen. Frame 94 is absent from both sides of every fit pair.
A 20-iteration vector Huber fit with 0.10 m scale estimates `b`. The training
position disagreement RMSE changes from 78.70 cm to an 11.02 cm model residual.
This is a conditional geometric fit, not an independent accuracy score.

## Rebuilt-map control

`rebuild_control.m` uses all sequence frames contributing unchanged fine-feature
observations within a common 65 m radius of frame 94. It retains **74,883
observations**, their labels, original source IDs and original frame blocks.
For the candidate, it adds `R_i [b_x b_y 0]'` to each stored global observation
and rebuilds the temporal-stability GMM with the same inference settings. The
source is regenerated through the existing coarse-only path with the matching
calibration, using the original wheel/gyro/lateral-motion source window.

The original local rebuild reproduces frame 94's saved raw match **exactly**.
Across seven query frames, the maximum reproduction discrepancy is
`9.31e-10` in the saved pose coordinates. Thus the map crop is adequate for
these measured comparisons.

| Source and seed | Original error (cm) | Offset error (cm) |
|---|---:|---:|
| Three coarse scans, original prediction | 92.7991 | 13.9882 |
| Three coarse scans, reference seed | 42.2760 | 13.8011 |
| Single coarse scan, original prediction | 70.9111 | 8.8015 |

For the primary three-scan comparison, similarity increases from 0.3108 to
0.5538 and correspondence count from 138 to 208. No acceptance gate was
relaxed. The original map admits substantially different wrong solutions
depending on initialization. With the effective offset, these two seeds reach
similar estimates. Better optimization alone cannot correct an inconsistent
target map.

Calibration is composed after pillar classification. Selected source-pillar
counts and selected hit counts are identical in every one of the 21 source
scans checked. The secondary Gaussian aggregation grid can nevertheless change
partition when the projected means shift. A separate control translates the
**existing Gaussian components**, preserving their partition, covariance and
weights. Frame 94 then reaches **14.0578 cm**, closely agreeing with the
13.9882 cm regenerated-source result. The improvement is not contingent on
changing that secondary partition.

## Checks beyond frame 94

The seven query frames below are excluded from the translation fit. Each uses
its own original recorded prediction and original three-scan motion window.
They still share the drive and map with training observations: this is not a
separate mapping traversal or a complete recursive localization replay.
Some auxiliary frames in the three-scan windows also appear in the training
set; the exclusion applies to query indices, not all underlying observations.
Frame 94's entire source window (92:94) is absent from the fitting set.

| Query | Original (cm) | Rebuilt offset source (cm) | Fixed Gaussian partition (cm) |
|---|---:|---:|---:|
| 76 | 3.21 | 7.68 | 7.18 |
| 87 | 60.57 | 15.38 | 15.08 |
| 91 | 85.14 | 14.34 | 15.06 |
| 94 | 92.80 | 13.99 | 14.06 |
| 97 | 30.57 | 14.24 | 15.94 |
| 103 | 12.26 | 3.56 | 5.06 |
| 112 | 19.53 | 4.66 | 3.37 |

All matches are accepted. Six of seven errors decrease; **frame 76 regresses**.
There is no claim of guaranteed improvement for every frame or of a new
1,170-frame RMSE.

## Time and stability controls

`fit_motion_models.py` compares relative feature transforms with interpolated
native INS trajectories on the same 26 training pairs. It fits robust spatial
and temporal models with a 10 m angular lever and a constant epoch shift
bounded to +/-0.25 s.

| Diagnostic model | Translation disagreement RMSE (cm) | Epoch increment (ms) |
|---|---:|---:|
| Identity | 78.70 | 0 |
| Translation only | 10.98 | 0 |
| Time only | 80.09 | -56.68 |
| Translation and time | 7.93 | -52.38 |
| Translation, mounting yaw and time | 7.37 | -51.08 |

The objective includes yaw disagreement, so a time-only fit may improve its
yaw residual while worsening its position residual. **Time alone does not
explain away the spatial mismatch in this test.** Joint fitting changes the
estimated translation, indicating parameter coupling. Neither the fitted
negative epoch shift nor the translation is a separately identified physical
calibration. The shared receiver-clock repair from the preceding work remains
in use throughout these experiments.

Map stability also does not certify geometric accuracy. One accepted target
component, `trafficSign:60524:622184:1`, has repeatability 1 and effective
support from frames 1 through 87 (effective count above 0.5). Repeated
observations before the turn can agree with one another under the wrong origin
assumption. The later query then disagrees as heading changes. High stability
can give a systematically displaced component high influence; this does not
require a defect in the stability calculation itself.

The plot `output/frame94_geometry_investigation_20260921/geometry_diagnosis.png`
shows saved sign observations before and after the offset, query errors and
the principal frame-94 control. The sign panels use a fixed 2.5 m neighborhood
around that original component, not manually annotated object identities;
some residual spread remains. Panel limits are automatic, and positions are
relative to the original component, so the candidate panel also moves globally.

## Scope and next engineering decision

The evidence supports correcting the **reference-point transformation and map
projection together** before further tuning registration gains or perception.
The current default is an unverified zero increment. A production calibration
should resolve the physical sensor and INS output reference points, fit or
verify mounting rotation and translation with sufficiently varied motion,
separate residual timing effects, and validate on a separate drive or withheld
map traversal. The map must then be rebuilt with the same calibration used
online. Applying an output-position correction to matches against the old map
would not repair its internal cross-frame inconsistency.

Remaining errors can include feature geometry, map fitting, scan motion,
window transport and reference uncertainty. The single-scan improvement is
evidence that a large error survives without source-window accumulation; the
remaining difference between one and three scans does not isolate motion
error because their feature support also differs.

The recorded INS trajectory is the comparison reference, not independent
ground truth. Existing maps include query-frame observations. The seven
selected tests therefore establish a strong local diagnostic, with those
limitations, rather than a deployable accuracy guarantee.

## Reproduction and artifacts

With the existing data and output caches in place, run these MATLAB scripts
in order through the project environment:

```matlab
run('research/frame94_geometry_investigation_20260921/investigate_geometry.m');
run('research/frame94_geometry_investigation_20260921/rebuild_control.m');
run('research/frame94_geometry_investigation_20260921/validate_candidate.m');
```

Run the independent motion-model fit:

```sh
uv run --offline --with numpy --with scipy python research/frame94_geometry_investigation_20260921/fit_motion_models.py
python research/frame94_geometry_investigation_20260921/validate_artifacts.py
```

Inputs include the synchronized fine-feature observations and map under
`output/mississippi_mapping_synchronized/`, the raw
`data/raw/MissisipiPointClouds.mat`, original matching calls and motion under
`output/receiver_clock_20260921/coarse_pipeline/matching/`, native reference
under `output/receiver_synchronized_inputs/`, and the previous frame-94
diagnostic and saved fine-feature adapter configuration. Scripts preserve
production configurations and write only research results and ignored output
artifacts. Cached input data are not included in the source commit.

The CSV/JSON files alongside this report provide fit pairs, acceptance flags,
all numerical controls, pillar/hit-count checks and component provenance.
Large diagnostic MAT files and the plot remain in the corresponding output
directory. MATLAB Code Analyzer reported no issues in all three scripts;
the scripts and Python fit completed successfully. `validation.json` records
the final result consistency checks and source/input provenance.
