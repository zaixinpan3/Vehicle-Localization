# Precomputed MnCAV LiDAR measurements with zero delay

Date: September 14, 2026. The requested offline experiment now completes all
1,170 real LiDAR matching calls before global estimation, then supplies each
accepted pose at its capture timestamp with **zero processing delay**.
The original continuous observer equations and feedback gains are retained.

## Implemented execution

`runMncavZeroDelayExperiment` has two explicit phases:

1. Design and run the nominal MnCAV lateral observer; process all raw frames
   recursively against the existing map, using recorded speed/corrected gyro
   and actual estimated lateral velocity for prediction. Save every result
   in `precomputed_calls.csv` and a measurement cache containing frame IDs,
   capture times, acceptance, original poses and full information matrices.
2. Load the completed cache. Set both the configured and supplied LiDAR delay
   to zero. Merge the 1,170 original frame timestamps with the motion grid
   using `reconstructFrameAlignedLidarSignals`, then integrate the existing
   global observer. No matching computation occurs inside this integration.

The fresh matching pass was actually executed; this run does not merely
reuse the previous cache. Its poses, acceptance flags and six independent
information entries match the preceding full experiment exactly. There are
1,098 accepted full poses and 72 rejected frames. All 1,170 frame times have
an observer output. At every accepted frame, the supplied pose and information
are identical to the original cached values: both maximum discrepancies are
zero. The global runner stores no delayed-state history, and the diagnostic
state used by its innovation equals the current state at every output.

With d=0, the pose correction is `T*K*W*(S\y(t)-S\zHatPose(t))`. The estimated
state remains continuous; a frame supplies the observation in this equation
and does not reset the state to the matched pose. Between accepted frames,
the existing observer uses explicitly offline linear interpolation of pose
and information. This can use adjacent precomputed frames. A rejected frame
has a NaN original measurement pose in the cache and is not relabeled as a
valid LiDAR measurement; its continuous input is interpolated from adjacent
accepted poses. `frame_injection_audit.csv` distinguishes those cases.

All frame times, including the last at 116.899543110952 s, are included in
the 12,859-node integration grid. The original motion/lateral grid ends at
116.89 s; its final values are held for 9.543 ms, within the declared 20 ms
edge allowance. No pose is extrapolated beyond accepted endpoint poses.
The evaluation's original 100 Hz ODOM reference is extrapolated by that
same fractional interval for the extra endpoint; common scan scoring uses
the existing scan-associated ODOM directly.

## Gains and verification

The nominal MnCAV lateral gains are freshly synthesized and checked at all
18 design-grid points. Actual lateral outputs are used throughout matching
and global estimation. Global theta remains 2, information normalization
lambda remains .001, and the course-rate bound remains .4 rad/s. The physical
translational gain chains remain [6;12;12], with yaw coefficient 2 and the
same auxiliary gain. This run isolates the zero-delay replay configuration.

An initial attempt to re-solve the LMI directly at d=0 returned a SeDuMi
numerical-problem status. No matrices from that failed solve are used.
Because the physical gains are unchanged, the experiment instead loads the
existing MnCAV certificate matrices and independently recomputes their
inequalities at d=0 using `improvedObserverReferenceDesign`. This passes
with uniform margin .127958119831288. No supplied course-rate excursion is
reported. The matrix check is not a complete physical cascade proof.

## Results

Whole-sequence uniform-grid evaluation uses the original **11,690 samples
at 100 Hz**, not all the additional integration knots. Native-frame scoring
uses all **1,170 scan times**, including rejection predictions in the matching
baseline. No frames are dropped from these whole-sequence metrics.

| Evaluation | Position RMSE (m) | Heading RMSE (degrees) |
|---|---:|---:|
| Global, 100 Hz, original ODOM reference | .254625 | .538750 |
| Global, 100 Hz, projected INSPVA reference | .195742 | .538750 |
| Matching/prediction, all frames, original ODOM | .288295 | .621195 |
| Global, all frames, original ODOM | .278524 | .543506 |
| Matching/prediction, all frames, projected INSPVA | .207317 | .621195 |
| Global, all frames, projected INSPVA | .195741 | .543506 |

Against the same native-frame reference, the zero-delay observer improves
position RMSE by 3.39% using ODOM and 5.58% using INSPVA; heading improves
by 12.51%. Position peaks remain: all-frame maximum is 1.582088 m against
ODOM and 1.133687 m against INSPVA. An improved RMSE is not an improvement
at every time or a complete removal of the previously diagnosed errors.

The previous delayed run starts at .15 s; this run starts at 0. Restricting
both to the previous 11,675-sample interval [.15,116.89] gives:

| Position RMSE on common 100 Hz interval (m) | Previous d=.15 s | New d=0 |
|---|---:|---:|
| Original ODOM reference | .338912 | .254361 |
| Projected INSPVA reference | .298229 | .195312 |

This is a replay-configuration comparison: the new run starts at the first
frame and preserves native frame knots, while the older run begins at .15 s
and reconstructs only on the 100 Hz grid. For the strictly fixed-start
delay-only control, see the preceding
[diagnosis](../mncav_error_diagnosis_20260914/diagnosis.md): its zero-delay
scores were .254786/.195862 m. The new values are consistent with that
control rather than evidence of a separate large algorithm improvement.

Fresh matching computation median/P95/maximum is 83.025/94.412/195.881 ms,
with 19 calls above 100 ms; disk read is separate. All of this computation
is completed before replay and contributes no simulated observation delay.
Global integration takes 10.217 s on this run. These wall-clock durations
are neither simulated sensor delay nor a worst-case runtime guarantee.

## Inputs and scope

The June 7, 2024 Mississippi sequence and the frozen 1,331-component map
remain unchanged. Map source:
`output/mississippi_mapping_20260912/probability_cloud_map.mat`, SHA-256
`74abe950f618abef4e0991181743b0ba74b1d689a15185db2f0c63c52b8ab104`.
The same sequence contributed map and query observations. The previously
audited ODOM source substitutions also remain in the original map and
reference; this task does not rebuild that map. INSPVA is an alternate
same-receiver diagnostic reference, not independent ground truth. Its
position-only projection comes from the preserved prior reference audit.

Zero delay is the user's explicit precomputation assumption. The remaining
inter-frame reconstruction is an offline input model, including accepted
pose gaps up to the explicit 1 s reconstruction limit; maximum actual gap
is .882857 s. Precomputation makes that future data available in this
experiment. The run does not establish live sensor delivery or a sampled
measurement-only observer theorem. Production ODE/DDE equations, the
historical delayed experiment, raw data, map and reference construction are
preserved.

## Checks and reproduction

53 tests pass: seven new frame-alignment tests and 46 existing continuous
observer tests. They include analytic moving truth with no frame lag,
non-grid timestamps, preservation of full information cross terms, rejection
handling, the final fractional frame, invalid nonzero delay and excessive
motion/gap coverage. Three new MATLAB files have zero factory Code Analyzer
findings. All 1,098 accepted input identities and current-state feedback are
checked in the actual full run; the exported figure was visually inspected.

From the repository root, with the existing local dependencies:

```matlab
addpath(pwd); setupVehicleLocalization;
addpath(genpath('../RobustVehicleLocalization/external/YALMIP'));
addpath(genpath('../RobustVehicleLocalization/external/sedumi'));
maxNumCompThreads(8);
report = runMncavZeroDelayExperiment; % Fresh complete matching, then replay.
% Repeat global replay using the completed cache of matching calls:
report = runMncavZeroDelayExperiment( ...
    "output/mncav_zero_delay_repeat", ...
    MatchingFolder="output/mncav_zero_delay_20260914/matching");
results = runtests({'tests/frameAlignedLidarReplayTest.m', ...
                   'tests/improvedObserverTest.m'});
```

Original results are retained in `output/mncav_zero_delay_20260914/`: the
completed measurement cache, original calls and information, per-frame
injection audit, full states/gains, trajectory, summaries, tests, initial
synthesis diagnostic and PNG/PDF figures. Compact metrics accompany this
report in Git. Generated MAT/figure files and recorded datasets are excluded
from Git and preserved as identified archive exports.
