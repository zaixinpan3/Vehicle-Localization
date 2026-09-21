# Historical worst-match diagnosis: Mississippi frame 307

Activity date: 2026-09-21. This is an offline diagnosis, with no change to the
production matcher, perception, map, clock exporter, or reference data.

## Finding

The reported **57.5065 cm** maximum belongs to **frame 307**, at the original
receiver-relative time **30.5603932813 s**, in the historical per-frame
zero-offset experiment. It is the largest accepted full-pose error among
1,084 accepted frames out of 1,170 scans. The input consists of saved fine
perception aggregated into XY distributions; this is the old geometric D2D
matcher, not the later coarse-only NDT experiment or a fused observer result.

The dominant spike is strongly explained by an anomaly in the ROS-header to
INSPVA receiver-time bridge used to select the evaluation/reference pose.
An approximately **38.5 ms** timing difference at **14.5593 m/s** corresponds
to **56.00 cm** of vehicle motion. Keeping the computed matching pose fixed
and selecting the native INSPVA pose through independent, timestamp-only
clock controls gives **7.71-7.81 cm**, instead of 57.51 cm.

These are diagnostic comparisons, not a newly certified localization
accuracy. The physically correct clock synchronization still requires
validation; the map itself was constructed through the original bridge.
The retained 57.5065 cm historical metric has not been overwritten.

## Exact reproduction and geometric evidence

The unchanged runtime snapshot is Git revision
`d6502080b392e6d368696eeb0b7fd755f556752b`, prepared by the preceding
`research/reference_free_785_20260920` study. Its matching configuration is
the saved historical configuration. The frame-307 pose reproduces the old
CSV within `1e-7` in all three coordinates; the actual difference is recorded
in `verification.json`.

| Quantity | Value |
| --- | ---: |
| Initial/reference X, UTM m | 484245.486336832 |
| Initial/reference Y, UTM m | 4977245.924430100 |
| Matching X, UTM m | 484245.510886286 |
| Matching Y, UTM m | 4977245.349889420 |
| Position difference | 57.5064926 cm |
| Forward difference in reference body axes | +57.1919641 cm |
| Left difference in reference body axes | -6.0063246 cm |
| Heading difference | +0.3528632 degrees |
| Matched distributions | 37 / 37 |
| Observable rank | 3 |
| Final similarity | 0.9311751 |

The 37 correspondences comprise 32 curb distributions, three pole
distributions representing two targets, and two sign distributions representing
one target. At the original reference, the pole and sign map centers lie
0.47-0.74 m forward of their paired source centers. All three landmarks
therefore request a similar forward translation. The curb residual model
mostly constrains lateral position and heading; the curb-only result has
rank two and cannot independently determine a full pose.

The per-frame global landmark centroids also show a common backward jump at
307, with recovery at 308. For pole component 949 the forward offsets from
the map are -16.31, -74.34, -17.07 cm at frames 306, 307, 308. For pole 931
they are +0.33, -55.30, -4.97 cm; for sign 1203 they are +13.92, -49.04,
+6.29 cm. A common rigid displacement affecting separate landmarks is
consistent with a pose/time association problem. It does not support
attributing the spike to one falsely detected pole.

## Clock evidence and responsible code path

`scripts/prepareInspvaMappingPoses.py`, function `prepare`, computes:

```python
target = np.interp(query, ros, receiver)
```

It then interpolates the native INSPVA trajectory at `target`. This piecewise
clock conversion directly inherits local header-timestamp irregularities;
there is no rejection of delivery-like timing excursions. The diagnostic
reproduces both the stored receiver times and the original frame-307 pose
from the native CSVs, rather than assuming that this is the operative path.

At native INSPVA sample 1530, the ROS-header interval grows to **54.9324 ms**,
followed by intervals **12.1293, 6.5923, 2.1858 ms**. Throughout this sequence
the embedded receiver time advances by **20 ms per sample**. The local
ordinary ROS interval is about 18.3 ms; the two clocks have a local rate ratio
around 1.091, so a global unit-slope offset was not assumed. Bag arrival
timestamps lie close to these irregular header times. This supports a local
timing/transport irregularity, but does not identify its driver-level cause.

The original receiver-relative times for scans 306, 307, 308 are
30.4987022317, **30.5603932813**, 30.6990027390 s. Interpolating the time
mapping at scan 307 between the neighboring scan timestamps, while excluding
307, gives **30.5988936954 s**. The difference is **38.5004 ms**. Native
INSPVA interpolation at that timestamp moves the reference by 0.5600062 m.
The unchanged map match is 0.0771497 m from that reference.

Seven controls use timestamps only, with no matching pose or spatial error
used to select their shifts or fit parameters:

| Clock control | Later time (ms) | Fixed match difference (cm) |
| --- | ---: | ---: |
| Neighbor scans, radius 1 | 38.5004 | 7.7150 |
| Neighbor scans, radius 2 | 38.5524 | 7.7055 |
| Neighbor scans, radius 3 | 38.1181 | 7.8067 |
| Robust native affine fit, +/-0.3 ROS s | 38.4729 | 7.7203 |
| Robust native affine fit, +/-0.5 ROS s | 38.4807 | 7.7187 |
| Robust native affine fit, +/-1.0 ROS s | 38.2256 | 7.7770 |
| Robust native affine fit, +/-2.0 ROS s | 38.2556 | 7.7692 |

The affine fits iteratively reject timestamp residuals beyond three scaled
MADs, with a 1 ms minimum threshold. The scan-neighbor and native-sample
controls use different observations and produce consistent results. An
initial exploratory interpolation directly between neighboring XY poses
gave about 7.4 cm; the reported final comparisons instead interpolate native
INSPVA samples at the independently predicted timestamp and give 7.7 cm.

## Matching controls and acceptance

All class-removal controls start at the same original reference pose and use
the same historical solver and thresholds. Remaining classes are rebalanced
by the existing solver, so these are removal controls, not fixed-weight ones.

| Features used | Difference from old reference (cm) | Difference from neighbor-clock reference (cm) |
| --- | ---: | ---: |
| All classes | 57.51 | 7.71 |
| Without curb | 57.61 | 6.46 |
| Without pole | 53.00 | 7.85 |
| Without traffic sign | 68.98 | 16.40 |

Removing one class does not remove the original apparent spike. Pole-only
matching gives 68.81 cm; curb-only is rank two and is not an accepted full
pose. Its smaller original 25.12 cm difference must not be interpreted as a
better full-pose measurement.

The matcher accepts the combined result because geometric agreement improves:
the summed robust objective decreases from 2.15493 to 0.14506 and all 37
distributions have correspondences. The final per-class remaining corrections
are 0.231 m (curb), 0.119 m (pole), and 0.048 m (sign), below the 0.5 threshold.
That check measures remaining disagreement at the fitted pose; it does not
validate the reference clock. A rank-three information matrix likewise cannot
detect a common timing offset or independently establish absolute accuracy.

## Interpretation and next engineering step

For this frame, the evidence prioritizes robust clock synchronization and
reference-time auditing ahead of changing perception thresholds or switching
the registration objective. A future correction should first establish the
physical LiDAR/header/receiver time relationship, then regenerate consistently
timed mapping poses and re-evaluate matching with a rebuilt map and independent
evaluation where available. Merely replacing a reported reference after
observing an error is insufficient; the controls here were specified from
timestamps without using spatial agreement to choose the shift.

No claim is made that the remaining 7.7 cm is wholly matcher error, or that
other sequence outliers have the same cause. Residual map geometry, sensor
extrinsics, scan motion, and reference accuracy remain unresolved. This is a
same-drive map and does not establish out-of-sample localization accuracy.

## Reproduction and validation

Prepare the historical runtime using the preceding study's `prepare_runtime.py`
if absent. In MATLAB, preserve the original path and folder, enter the
historical runtime, and run:

```matlab
repoRoot = '/home/zai/Downloads/ResearchProjects/vehicleLocalization';
originalPath = path; originalFolder = pwd;
try
    cd(fullfile(repoRoot,'output/reference_free_785_20260920/historical_runtime'));
    addpath(fullfile(repoRoot,'research/frame307_matching_diagnosis_20260921'));
    runFrame307Diagnostics(repoRoot);
catch exception
    path(originalPath); cd(originalFolder); rethrow(exception);
end
path(originalPath); cd(originalFolder);
```

Then run the command in `analyze_clock.py`'s help string. It independently
checks all exported scan clock values and the native frame-307 pose before
producing clock controls. No randomness is used. Compact CSV/JSON exports are
checked in; MAT files, raw datasets, runtime snapshots and rendered figures
remain under ignored `output/frame307_matching_diagnosis_20260921`.
`artifact_manifest.json` records technical input/output hashes. The figure is
`output/frame307_matching_diagnosis_20260921/diagnosis.png` with a matching PDF.
Validation outcomes are recorded in `validation.json`. No production unit
suite rerun or production clock repair is claimed.
