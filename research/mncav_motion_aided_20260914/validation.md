# MnCAV motion-aided localization validation

## Material passport

Date: September 14, 2026. Mode: executed design, recorded experiment and
synthetic validation. Verification status: VERIFIED for exact repeated
recorded states, independent numerical/noise checks and 82 passing tests.
Sources: the saved zero-delay experiment, all 1170 precomputed real matching
frames, nominal MnCAV parameters and new explicitly synthetic realizations.
The [design report](design.md) provides source review, changed equations,
gain search and a new conditional stability proof.

## Outcome and scope

The new observer improves position RMSE, maximum, P95 and heading RMSE against
the identical continuous LiDAR input using the continuous INSPVA diagnostic
reference. The improvement also holds on the reserved time interval and at
all native and accepted-only frame populations. Against mixed ODOM, RMSE,
maximum and heading improve, but full-run P95 increases by 5.18 mm. Therefore
the strict all-reference/all-metric flag in the machine-readable output is
false. No metric is omitted to turn that flag into a pass.

The subsequent [distribution audit](../mncav_baseline_audit_20260914/validation.md)
qualifies this four-metric result: full-run median position discrepancy
increases from 10.3945 to 10.5119 cm, and the fraction within 5 cm falls from
17.6561% to 12.6005%. Position error improves at 45.4577% of paired times.
The RMSE benefit therefore does not establish improvement at most times or
centimeter-level accuracy. These distribution measures were not gain-selection
objectives; the audit does not retune the gains using the evaluation data.

The new runtime uses the same seven states, course-rate-dependent prediction,
real motion inputs and actual lateral observer. It replaces weak quadratic
auxiliary correction with direct signed velocity and acceleration correction.
It does not output the raw pose or blend it with a reference. The physical
gains [kp,kv,ka,kpsi]=[4,4,12,4] /s are now available through
`motionAidedObserverConfig` and `runMotionAidedVehicleObserver`; the recorded
entry point is `runMncavMotionAidedExperiment`. Original GNSS/delayed
observer paths remain separate and their results remain reproducible.

## Recorded comparison

Map: unchanged `output/mississippi_mapping_20260912/probability_cloud_map.mat`.
Input drive: `raw_data_2024-06-07-12-09-31_0`, June 7, 2024, Mississippi.
The matching cache is the prior completed all-frame cache. No map matching
is rerun or selected according to new observer output. All 1170 frames remain,
with 1098 accepted measurements and 72 rejected matches. On rejected frames,
the raw frame baseline is the recorded matcher prediction; the continuous
baseline is the same offline interpolation of accepted poses used previously.
Accepted-only results separately compare actual accepted measurements.

The original 11690 uniform samples are used for continuous metrics, with
12859 integration knots including all native frames. No extra knots receive
extra evaluation weight. Both methods use identical times and reference
poses within each row. Native reference poses use the stored continuous
reference evaluated at those same knots; consequently their mixed-ODOM and
yaw metrics differ slightly from historical reports that used native source
reference rows. The complete metric definition and counts are in `metrics.csv`.

| Population / INSPVA metric | Direct LiDAR | Previous observer | New motion-aided observer |
|---|---:|---:|---:|
| Full uniform position RMSE (m) | .188465 | .195742 | .176934 |
| Full uniform position maximum (m) | .967273 | 1.135061 | .863912 |
| Full uniform position P95 (m) | .376068 | .348828 | .337687 |
| Full uniform heading RMSE (deg) | .584031 | .538750 | .552902 |
| Reserved >60 s position RMSE (m) | .209498 | .223669 | .190611 |
| Reserved >60 s position maximum (m) | .967273 | 1.135061 | .735488 |
| Reserved >60 s position P95 (m) | .433806 | .420158 | .407302 |
| Reserved >60 s heading RMSE (deg) | .648723 | .599994 | .619015 |
| All 1170 frames position RMSE (m) | .207317 | .195741 | .176697 |
| Accepted 1098 frames position RMSE (m) | .187445 | .181042 | .161506 |

Full uniform position RMSE decreases by 6.12% relative to direct LiDAR and
9.61% relative to the previous observer. The new heading error is better
than direct LiDAR but slightly worse than the previous observer. The stated
goal is assessed against direct matching, not universal superiority over
every previous output metric.

For mixed ODOM, full uniform RMSE is .251085 -> .242192 m, maximum
1.731318 -> 1.622882 m, and P95 .485653 -> .490829 m. The last is a 1.07%
increase and is retained explicitly. The previously identified ODOM source
switching limits that reference, but it is not used to erase the exception.

## Lateral design, initialization and numerical controls

Nominal MnCAV lateral gains were synthesized again using
`designLateralObserverGains(lateralObserverConfig("mncav"))`. Its matrix
certificate passes. Its scheduled gain array has size 2-by-2-by-9-by-2 and the
reported H2 bound is .406114777; numeric gains are exported in `gains.json`.
The original motion grid is reconstructed from the saved
uniform indices; rerunning the lateral observer gives exactly the saved
lateral velocity (maximum difference 0). These lateral outputs are used by
the new global observer. The existing matching cache therefore retains its
original motion-aid provenance.

Vehicle mass is 2273 kg, lf=1.374605 m, lr=1.714395 m,
Iz=5874.60733 kg m^2, Cf=108238.09524 and Cr=80817.77778 N/rad.
Mass/geometry are stock specifications or derived values; inertia and tire
stiffness remain unmeasured priors. No occupied/instrumented vehicle mass or
new sensor extrinsic calibration is asserted.

The global certificate gives mu=1.836149879936 for |q|<=.4 rad/s and
minimum weight .25. Actual maximum |q|=.354975312295 rad/s; no value is
clipped. An independent symmetrized-matrix test covers varying course rate
and rotated anisotropic weights. The new matrix proof and runtime diagnostics
do not establish all true-state or upstream error assumptions on real data.

An additional old-observer run uses exactly the new initial seven-state
estimate. Its INSPVA RMSE is .194513 m and its peak remains 1.135061 m.
Thus changing initialization does not reproduce the new .176934 m result.
New states reproduce exactly on an immediate second run. Halving RK4's
maximum step from .005 to .0025 s changes any state entry by at most
1.2236641e-7 in its corresponding units. These numerical effects are much
smaller than the measured accuracy differences.

The first experiment wrapper attempt detected that CSV round trips perturb
frame times by at most 4.9738e-13 s. It was corrected to use the existing
exact MAT frame indices and explicitly verify the CSV discrepancy below
1e-9 s. Inputs and timestamps were not shifted or resampled to improve scores.

## New synthetic validation

Thirty 40 s runs use ten fixed random seeds 20260915--20260924 at each true
plant tire-stiffness factor .7, 1 and 1.3. Estimator parameters and gains
remain nominal. The independent bicycle plant uses exact frozen-midpoint
linear steps, varying longitudinal speed, and smooth reversing steering.
Pose is constructed by integrating true global velocity. Both the real
lateral observer and new global observer run on noisy measurements.

Synthetic assumptions: 100 Hz Gaussian speed noise .05 m/s, each acceleration
noise .08 m/s^2, gyro noise .002 rad/s and steering noise .05 deg. LiDAR is
10 Hz with .1 m XY and .3 deg yaw noise, plus .15*sin(.7*t) m X and
.1*sin(.4*t) m Y disturbances, linearly reconstructed identically for baseline
and observer. These are synthetic values, not manufacturer/sensor specs.
The same seeds across stiffness factors provide paired noise conditions;
there are ten independently seeded realizations, not thirty independent
drives. No Gaussian distribution is claimed bounded for all realizations.

All 30 cases improve all four reported metrics over their direct LiDAR
baseline. Mean per-run position RMSE changes from .173935556 to .139983567 m.
All sampled course rates satisfy the declared .4 rad/s envelope. Full inputs,
true states, lateral states and estimated trajectories remain in
`synthetic/validation.mat`; all 30 metric rows are retained.

A 40 s actual-runner harmonic check at .3587165 Hz gives gain .871177109
versus .871213990 predicted analytically. The linear-reconstruction relative
error is 4.23e-5. Position-only noise leaves the new motion states unchanged
in a separate unit test. This is the specific amplification mechanism removed
by the revised injection structure.

A deliberately adverse case with perfect zero-position LiDAR and a false
constant 1 m/s velocity gives observer RMSE .248809965 m versus raw zero.
It is preserved as a counterexample to universal dominance, not counted in
the thirty noisy bicycle-plant cases or described as passing. An independent
real drive and calibrated reference would be needed for broader conclusions.

## Tests and checks

82/82 tests pass, none incomplete: 14 new motion-aided tests, 46 prior global
observer tests, 15 lateral tests and 7 frame-aligned replay tests. New checks
cover straight/accelerating/turning motion, heading wrap, initial-error decay,
non-amplification of a position ripple, actual motion influence, independent
dissipation verification, invalid delays/heading/information, alignment and
rate-envelope reporting.

Initially two motion tests used tolerances below the known error of linear
reconstruction of curved position signals. Their expectations now use the
analytic source bound max|p''|*dt^2/8; the observer was not altered to fit
those tests. Two suite-invocation attempts used nonexistent test filenames
and did not run tests; the final exact suite paths below ran successfully.
Seven MATLAB files have zero factory Code Analyzer findings. The four-panel
PNG/PDF was inspected; MATLAB emitted a nonfatal vector-export advisory.

## Statistical interpretation and limitations

All 11 ARS statistical fallacy checks were considered:

| Check | Treatment |
|---|---|
| Simpson's paradox | Report design/reserved/full and both reference definitions; mixed-ODOM P95 exception retained. |
| Ecological fallacy | No aggregate result implies improvement at every timestamp or for every vehicle. |
| Berkson's paradox | Accepted-only metrics supplement, not replace, all-frame metrics. |
| Collider bias | No reference-error gating or new acceptance selection is applied. |
| Base-rate neglect | No diagnostic classification sensitivity/specificity claim is made. |
| Regression to the mean | Fixed identical input baseline and same-initial-state control retained. |
| Survivorship bias | All 1170 frames, rejection counts, all candidates and adverse control are disclosed. |
| Look-elsewhere effect | All 144 formal candidate metrics are saved; no significance test is claimed. |
| Forking paths | Preliminary probes and prior inspection of the reserved drive segment are disclosed. |
| Correlation/causation | Controlled numerical mechanism is distinguished from attribution of real sensor errors. |
| Reverse causality | References are scoring inputs only and never runtime measurement or switching inputs. |

Time samples are dependent. Map and queries share a drive; INSPVA and ODOM
share a receiver and are not independent ground truth. Reused matching
already incorporates lateral prediction, so the result measures the benefit
of an additional global observer on those fixed measurements. No independent
map rebuild, new raw-frame matching, real-time causality, statistical
significance, calibrated sensor covariance or all-condition superiority is
claimed. The reserved portion was excluded from this gain selection but
previously seen in diagnostics. The mixed-ODOM P95 regression and the adverse
synthetic counterexample remain material limits.

## Reproduction and artifacts

With the existing YALMIP/SeDuMi dependencies on the MATLAB path:

```matlab
setupVehicleLocalization;
selection = designMncavMotionAidedGains;
report = runMncavMotionAidedExperiment( ...
    DesignFile="output/mncav_motion_aided_20260914/design/selected_design.mat");
synthetic = validateMotionAidedObserver;
results = runtests({'tests/motionAidedObserverTest.m', ...
    'tests/improvedObserverTest.m','tests/lateralObserverTest.m', ...
    'tests/frameAlignedLidarReplayTest.m'});
```

`output/mncav_motion_aided_20260914` contains exact experiment states, gain
selection, metrics, trajectories, figures, numerical controls, test/analyzer
results and synthetic runs. Compact JSON/CSV results and this report are
committed; generated binaries and recorded datasets remain outside Git and
are preserved as identified archive exports. Previous source/result artifacts
and unrelated agent instructions/reference papers remain unchanged.
