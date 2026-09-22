# Five-frame coarse-perception horizon

On September 22, 2026, the requested default was changed to the current scan
plus four preceding scans. At 10 Hz this spans approximately 0.4 seconds; the
age bound is 0.45 seconds. Startup and acquisition gaps can leave fewer scans
available. This is causal processing without waiting for future frames.

Only distributions confirmed in at least two distinct acquisitions enter map
matching. Detection counts 2, 3, 4 and 5 receive temporal stability weights
0.4, 0.6, 0.8 and 1.0 respectively. Singletons are excluded. Relative independent
wheel/gyro odometry aligns historical distributions before association and
pooling. The underlying whole-pillar coarse classifier remains unchanged.

The existing five-frame experiment already used exactly this configuration.
Its 1170-frame trajectory RMSE was 17.2856 cm; the 1169 emitted full-pose
measurements had RMSE 17.1913 cm. The maximum trajectory error was 68.2811 cm.
Frame 1 provided no measurement. These are **reused measurements**, not a new
full-sequence execution. They are worse in aggregate than the former
three-frame result (15.9610 cm trajectory RMSE). The longer window is selected
at the user's request, not claimed as a demonstrated accuracy improvement.
Same-drive map overlap, sequence-fitted origin calibration, recorded reference
tilt, offline synchronization and the existing initialization remain limitations.

Fresh validation performed for this change:

- All 39 temporal-window and geometric-registration tests passed. They cover
  five-frame retention/expiry, missing detections, singleton rejection, all four
  support weights, motion compensation and preservation of geometric information.
- Actual raw frames 955 through 959 passed through `localizeLidarFrame` with
  no explicit window override. Its default retained all five scans and emitted
  61 distributions with support between two and five scans, rejecting 47
  singleton tracks. The component count agrees with the existing full replay.
- Default configuration equality and unchanged relevant runtime/data hashes
  establish compatibility with that full replay. No perception or registration
  parameter was tuned. MATLAB Code Analyzer reported zero findings for edited
  MATLAB files and the validation helper.

The earlier three-frame experiment is retained as a historical research record;
its reproduction commands and validator explicitly select its saved configuration.
There is one production default, defined in `localizationSourceWindowConfig`.

From the repository root:

```matlab
setupVehicleLocalization;
results = runtests({'tests/localizationSourceWindowTest.m', ...
    'tests/geometricRegistrationTest.m'});
assertSuccess(results);
writetable(table(results),'research/five_frame_horizon_20260922/tests.csv');
addpath('research/five_frame_horizon_20260922');
validate_five_frame_horizon;
```

`validation.json` records fresh checks and labels reused sequence metrics.
`runtime_provenance.json` verifies saved runtime and data hashes against the
prior experiment manifest. Large recorded/generated datasets remain local.
