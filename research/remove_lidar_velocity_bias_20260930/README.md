# Remove LiDAR-derived velocity-bias learning

Removed the additional displacement-window velocity-bias estimator at the
user's explicit request. The current observer consumes the supplied wheel
longitudinal velocity and lateral-observer output directly. There is no
LiDAR-derived additive velocity correction, low-speed bias participation,
extra sideslip-rate correction, or learned correction in the matching seed.

## Implementation

- Removed the estimator and its histories from both
  `runSynchronousLocalizationObserver` and the historical-transport branch of
  `runFullLocalizationObserver`. GNSS course reconstruction retains its own
  motion history and uses the supplied velocities.
- Deleted `correctLateralVelocityFromLidar` and its dedicated recovery tests.
  Removed the two adapter variants from the historical interface comparison.
- Removed `fullObserverConfig.bias`, including the 2 s window and 4 s time
  constant. Advanced generic/MnCAV configuration identifiers to v3/v5.
- Removed learned-bias diagnostics and their consumers in the current validation
  and synthetic-cascade scripts. Removed the synthetic gate that required the
  deleted learner to recover wheel bias; all remaining accuracy thresholds
  are retained.
- Updated the motion output-point description and its exporter to remove the
  assumption of an online bias learner. All numerical calibration values are
  unchanged. The separate lateral observer and its physical sensor-bias state
  are outside this removal; its pre-existing working-tree changes are preserved.

Historical results and dated studies retain their original evidence. Reproduce
them with their original source versions, including commit
`a27ae36e7fbd3472949f3f5ea6c460733949e81c` for the preceding bias diagnosis.
They do not describe current default behavior after this deletion.

## Validation actually executed

**115 tests pass across six suites**: `localizationMotionInputTest` (6),
`synchronousLocalizationTest` (19), `fullLocalizationObserverTest` (20),
`gnssOutputPointTest` (10), `motionAidedObserverTest` (14), and
`improvedObserverTest` (46).

The new motion-contract tests verify both runtimes: changing only LiDAR
translation changes position corrections but does not learn a new body velocity;
outages preserve the supplied wheel/lateral propagation; online matching seeds
use those same velocities; removed configuration/diagnostic fields are absent.
The first test launch lacked the existing YALMIP/SeDuMi paths and one test
errored. Adding those paths and rerunning the 46-test affected suite resolves
that environment issue. `tests.csv` records the final outcomes. This is not a
claim that every repository test was rerun; the prior unrelated lateral
gain-bound assertion is outside these six suites.

On **1,169 frozen Mississippi LiDAR-only packets**, the removed implementation
reproduces all seven state coordinates of the preceding explicitly disabled
learning control **exactly**. Future measurement mutations after 60 s leave
the entire preceding state sequence unchanged. All states are finite.

After the established 2 s startup split, fixed-packet position RMSE is
**19.7033 cm**, maximum **37.5154 cm at frame 1151**. On the 1,121 accepted
paired frames, RMSE is **19.7040 cm**, versus the preceding enabled-learning
control's 9.7796 cm. Frame 847 is **22.9951 cm**. These measurements honestly
retain the accuracy consequence of the requested removal. No gain retuning or
replacement correction mechanism is introduced. This is a downstream replay,
not a new raw-perception or closed-loop matching run.

The current synthetic cascade also runs successfully for `noisy_both`: a 90 s
nominal MnCAV plant, random seed 20260929, current configuration and freshly
synthesized lateral gains, with GNSS/LiDAR noise and wheel/IMU biases. Its 899
frame outputs have **3.3996 cm settled position RMSE**, **9.8297 cm settled
maximum**, and **0.12306 degrees settled heading RMSE**. This one-scenario
smoke run exercises the updated export schema; the full synthetic scenario
matrix and its aggregate pass/fail checks were not rerun.

Factory Code Analyzer reports zero findings in 13 changed MATLAB files.
Among the prior experiment's 310 source hashes, exactly the 14 intentionally
changed/deleted files differ; the other 296 remain unchanged. The new test and
research files are additional artifacts. Scoped whitespace checks pass.

## Reproduction

```matlab
addpath(pwd); setupVehicleLocalization;
addpath(genpath('../RobustVehicleLocalization/external/YALMIP'));
addpath(genpath('../RobustVehicleLocalization/external/sedumi'));
results = runtests({'tests/localizationMotionInputTest.m', ...
    'tests/synchronousLocalizationTest.m', 'tests/fullLocalizationObserverTest.m', ...
    'tests/gnssOutputPointTest.m', 'tests/motionAidedObserverTest.m', ...
    'tests/improvedObserverTest.m'});
assert(all([results.Passed]));
addpath('research/remove_lidar_velocity_bias_20260930');
validateRemoval;
validateSyntheticLocalizationCascade( ...
    'output/remove_lidar_velocity_bias_20260930/synthetic', Scenarios="noisy_both");
```

The replay consumes the preceding study's compact `inputs.mat` and
`diagnostic.mat`, identified by `input_hashes.json`. Generated MAT products,
raw recordings, external solvers and unrelated working-tree changes are
excluded from the commit.
