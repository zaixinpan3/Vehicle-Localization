# MnCAV localization RMSE: controlled error diagnosis

Prepared September 14, 2026. This investigation preserves the previous full
experiment and production configuration. It identifies a mixed reference
position source, substantial delayed-feedback amplification, and weak
effective motion correction. Eleven downstream runs, sixteen selected-frame
registration solves, six original-bag checks, and an independent sinusoidal
response experiment support the findings.

## Findings and their practical meaning

1. **The previous reference position changes source.** Most recorded
   `/novatel/oem7/odom` XY values equal projected INSPVA; 135 equal projected
   BESTPOS instead. These substitutions introduce discontinuities and held
   lower-rate positions. They affect both scoring and the input poses used
   to build the frozen map. A discrepancy from this reference is not
   automatically a physical localization error.
2. **The observer also degrades position estimates with a consistent
   alternative reference.** On 1,167 common scan times, matching/prediction
   RMSE is 0.207456 m and the full observer is 0.299338 m against projected
   INSPVA. Reference repair alone will not fix the estimator. This alternate
   reference comes from the same receiver and is not independent truth.
3. **The theta=2, 150 ms configuration amplifies a band of pose disturbances.**
   A frozen scalar analysis predicts 3.3493 amplitude gain near 1.0954 Hz;
   the actual continuous observer reproduces it in a separate experiment.
   Lower gain and shorter diagnostic delay reduce recorded RMSE.
4. **The main translational correction is effectively LiDAR driven.**
   Information normalization makes all admitted directions nearly full
   weight, while the invariant-channel gain is very small. Removing that
   auxiliary correction barely affects RMSE. Weak or biased admitted poses
   therefore receive little protection from motion consistency.
5. **The current evidence does not prioritize lateral parameter error or
   numerical integration.** Downstream lateral-zero and half-step controls
   barely change position RMSE. The lateral-zero control freezes matching;
   it does not test removing lateral estimation from upstream prediction.

The previous statement that accepted matches around 109 s have 1.46--1.58 m
*physical errors* is not established. Those are discrepancies from a reference
that switches position source. The old numerical scores remain reproducible;
their interpretation must be qualified by this audit.

## Material and controlled design

The baseline is `output/mncav_full_localization_20260914/full_experiment.mat`
and its 1,170-frame matching output, created by commit
`a60992f2ef5647caeca122f68b04e6f68fea43c9`. The bag is
`raw_data_2024-06-07-12-09-31_0`; no second drive is tested here. The frozen
1,331-component map is
`output/mississippi_mapping_20260912/probability_cloud_map.mat`, SHA-256
`74abe950f618abef4e0991181743b0ba74b1d689a15185db2f0c63c52b8ab104`.
Map and query observations use this same drive. The nominal MnCAV vehicle
and actual saved lateral observer outputs remain unchanged.

All downstream runs use the same 11,675 samples at 100 Hz on [.15,116.89] s,
the same initial state/history, and frozen matching unless the row explicitly
replaces the measurement. Baseline reproduction is exact in all saved states.
No random seed is needed: these are deterministic recorded-data runs.
Changed gain/delay designs pass the implemented matrix verification. This
does not establish every physical theorem hypothesis.

Reference-pose substitutions are counterfactual diagnostics. They supply
reference XY/yaw as the measurement to test the downstream observer, and
cannot be advertised as LiDAR localization accuracy. Delay variants change
both the source query to t-d and the DDE's d. A zero-delay counterfactual is
not an achieved sensor latency. The theta=1.5 case receives a fresh gain
synthesis; this is a sensitivity point, not an optimized or held-out design.

## Reference-source audit

INSPVA and BESTPOS latitude/longitude are projected directly using
EPSG:4326 to EPSG:32615. There is no fitted trajectory alignment, smoothing,
or displacement correction. Nearby message coordinates are compared with
each native odometry position at a 1 mm tolerance:

| Native data check | Result |
|---|---:|
| ODOM / INSPVA samples | 5,845 / 5,847 |
| ODOM positions matching nearby INSPVA | 5,710 |
| ODOM positions matching nearby BESTPOS | 135 |
| ODOM positions >0.1 m from nearby INSPVA | 135 |
| Those 135 positions matching BESTPOS | 135 |
| Largest native ODOM position step | 1.729735 m |
| Largest step-implied ODOM speed | 86.5666 m/s |
| Concurrent reported body speed | 12.1615 m/s |

The maximum implied speed comes from a 19.975 ms interval starting at
107.661420 receiver seconds. Six messages spanning the three largest suspect
transitions were deserialized again from the original bag. Position and
header values agree exactly with the CSV export; the jumps are not an export
artifact. Receiver elapsed time uses the baseline ROS-to-receiver bridge.

The BESTPOS-valued intervals are approximately [87.6814,88.5613],
[107.6814,108.5613], and [108.6814,109.5613] s. On the 100 Hz evaluation grid,
the two reference constructions disagree by up to 1.431030 m.
`readFramePoseTable` supplies ODOM position to mapping; the actual pose table
assigns BESTPOS-valued odometry to frames 878--887 and 1078--1097, thirty
frames in total. Thus mixed positions demonstrably enter mapping. The amount
of retained map distortion after temporal filtering has **not** been isolated
by rebuilding the map, so duplicate/broadened features remain a hypothesis.

The published NovAtel driver contains a matching mechanism: it can choose
BESTPOS or INSPVA position based on availability/quality, then publish ODOM
position from that selected GPSFix while obtaining orientation and velocity
from INSPVA. This implementation supports the mechanism, but the deployed
2024 driver version/configuration has not been recovered. The coordinate
audit, rather than an assumed software version, establishes the recorded
source substitutions. [Official driver source](https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_driver/src/bestpos_handler.cpp).

INSPVA itself is not flawless: its largest native projected step is .5023 m,
and it is a navigation solution from the same receiver. It is used only to
test sensitivity to source consistency. On unchanged observer outputs,
100 Hz RMSE changes from .338912 to .298229 m. The matcher all-frame score
changes from .288295 to .207317 m; that scan comparison additionally uses
INSPVA interpolated at the scan time instead of the old nearest associated
ODOM row. The fair 1,167-common-time INSPVA comparison is .207456 m versus
.299338 m, so the observer's position deficit persists.

## Downstream ablations

Every number below is whole-sequence 100 Hz position RMSE in meters. Both
reference columns evaluate identical states for each row; they are not
independent experimental replicates.

| Single change from baseline | Mixed ODOM reference | INSPVA reference |
|---|---:|---:|
| None: theta=2, d=.15 s | 0.338912 | 0.298229 |
| Auxiliary gain N=0 | 0.338913 | 0.298231 |
| Supplied lateral output = 0 | 0.338887 | 0.298198 |
| Theta=1.5, freshly synthesized | 0.293631 | 0.246292 |
| Delay=.075 s | 0.270447 | 0.216824 |
| Delay=0 | 0.254786 | 0.195862 |
| Maximum integration step 2.5 ms | 0.338912 | 0.298229 |
| Measurement = mixed ODOM reference | 0.141997 | 0.211671 |
| Measurement = mixed ODOM reference, d=0 | 0.086412 | 0.171989 |
| Measurement = INSPVA position / reference yaw | 0.164718 | 0.045789 |
| Measurement = INSPVA position / reference yaw, d=0 | 0.161609 | 0.034149 |

Exact exported values and additional metrics are in
`complete_ablation_metrics.csv`. Heading uses the original reference yaw;
the alternate reference changes position only. For the INSPVA/no-delay
measurement counterfactual, the baseline information time series is retained
to isolate the pose and delay change.

Three conclusions follow. First, source-consistent reference measurements
allow this observer to track the actual recorded variable-speed turning
inputs at .045789 m RMSE with the original 150 ms delay. This makes a gross
failure of the translational model an unlikely sole explanation on this
sequence, but does not validate that model for all maneuvers. Second,
feeding the mixed reference back as an allegedly perfect measurement still
produces .141997 m residual and a 1.490 m peak against itself. Tracking a
reference's discontinuities is a poor test of vehicle dynamics. Third,
theta reduction alone improves the original-reference score by about 13.4%;
it cannot eliminate reference/map inconsistency or matching disturbance.

Halving the integration step changes the largest saved state component by
only 4.281e-8 in its native units. The baseline and half-step RMSE agree at
six decimals. Startup is also insufficient to explain the error: samples
t>=5 s have baseline RMSE .343270 m. This mask starts at absolute receiver
time 5 s; the previous report's five-seconds-after-start mask begins at
5.15 s and gives .343454 m.

The raw seven-state model explicitly includes the q-dependent turning terms
`q^2*v + 2*q*J*a`, with q=r+betaDot. Speed jerk, course angular acceleration,
and q-estimation errors remain disturbances. Variable speed and turning are
not omitted altogether. The reference counterfactual bounds neither these
physical disturbances nor the effect of changing the upstream matcher.

## Why the present gains amplify pose disturbances

The physical translational injection gains at theta=2 are [6,12,12]. Set
q=0, N=0, and W=I to isolate one translational chain. For capture-time pose
noise n(t), the source supplies n(t-d); eliminating the velocity and
acceleration states gives

\[
H(s)=\frac{P(s)e^{-ds}}{s^3+P(s)e^{-ds}},\qquad
P(s)=3\theta s^2+3\theta^2s+1.5\theta^3.
\]

This is a local scalar model, not a transfer function for the full
time-varying turning trajectory.

| Theta | Delay (ms) | Peak amplitude gain | Peak frequency (Hz) |
|---:|---:|---:|---:|
| 2 | 150 | 3.3493 | 1.0954 |
| 2 | 75 | 1.4852 | .4916 |
| 2 | 0 | 1.3546 | .3587 |
| 1.5 | 150 | 1.8252 | .7547 |

An independent 30 s run of `runImprovedVehicleObserver` uses a stationary
zero state, q=0, N=0, 1e6*I information, and 1 cm sinusoidal capture noise
at 1.0953784 Hz. A sine/cosine fit over [20,30] s measures gain 3.3493;
relative disagreement with the analytic gain is 6.324e-9. This directly
checks the implemented delay convention and noise-amplification mechanism.
It does not assert that every recorded peak is magnified by exactly 3.35.

The baseline's normalized information gain is nearly identity: choosing
lambda=.001 with minimum J eigenvalue 1.2896876 gives minimum W=.9992252.
Raw information was preserved, but this normalization almost eliminates
direction-dependent attenuation. It was previously selected to enter the
retained [.998520,1] weight sector, not by noise/RMSE optimization. A
stability certificate over that sector does not imply desirable disturbance
attenuation. The design should admit measured quality variation and optimize
robustness within a corresponding verified sector.

The existing physical auxiliary gain is very small. Removing it changes the
position score by less than one micrometer in RMSE, so measured speed and
acceleration provide little corrective resistance to pose disturbance in
this configuration. Yaw rate still enters prediction; this is not a claim
that all inertial inputs are ignored.

## Matching and missing-pose evidence

Eight frames were reprocessed from raw point clouds with identical source
clouds and a map crop fixed at the original predicted pose. Each was solved
from the original predicted seed and from the old reference seed. These
sixteen diagnostic solves retain all accept/reject outcomes. Representative
results show why a reference-seeded fit alone is not a correctness test:

| Frame | Seed | Discrepancy from ODOM (m) | Discrepancy from INSPVA (m) |
|---:|---|---:|---:|
| 820 | Original prediction | .714 | .697 |
| 820 | ODOM reference | .661 | .644 |
| 879 | Original prediction | 1.379 | .138 |
| 879 | ODOM reference | .442 | .841 |
| 1090 | Original prediction | 1.464 | .247 |
| 1090 | ODOM reference | .103 | 1.330 |
| 1093 | Original prediction | 1.585 | .351 |
| 1093 | ODOM reference | .160 | 1.146 |

The smaller ODOM discrepancy in the last rows does not establish a better
physical match; it worsens agreement with the alternate consistent source.
At frame 1093 both solutions are accepted, with minimum information
eigenvalues about 8.54 and 8.02. The original-seed solution has 67 curb
matches of observable rank 2, nine pole matches of rank 3, and one sign
match of rank 2. Aggregate full rank and class consistency do not select a
unique physically correct pose. Frame 820 remains discrepant from both
references, indicating a residual local matching/map issue independent of
the large reference substitutions; its precise physical cause is unresolved.

The preceding experiment accepts 1,098/1,170 poses. Its maximum accepted-
pose gap is .882857 s, and the offline interpolation requires future capture
for 479 observer samples, up to .732240 s beyond current time. The strict
default .12 s gap limit fails. Here, 762 evaluation samples interpolate
across accepted-pose gaps exceeding .2 s; they contain 18.3% of baseline
squared error. This is a descriptive association, not an attributable causal
fraction. A real causal missing-pose interface is still required. Changing
the numerical delay in a diagnostic cannot supply that interface.

Errors are concentrated: the worst 1% of samples contribute 21.45% of
squared error, and the worst 5% contribute 48.49%. All samples remain in
headline scores. RMSE differences between controls are correlated; they must
not be added into independent error percentages. The exact vector-error
decomposition saved in `summary.json` has a nonzero negative cross term.

## Engineering priorities supported by this investigation

1. Preserve the original data and explicitly select one position solution
   with receiver time, status and covariance metadata. Audit body/antenna
   origin, UTM orientation, and sensor synchronization before calling it
   ground truth. Rebuild a separately versioned map with that consistent
   pose source and measure the effect of the thirty affected frames.
2. Re-run matching on the rebuilt map; use held-out observations/drives for
   an accuracy claim. Inspect surviving errors around frame 820 and the
   weakly constrained along-curb direction. Do not force convergence toward
   the old mixed reference to improve a score.
3. Design the observation-quality weights, allowed information sector, and
   gains together. Preserve attenuation for weak geometry and evaluate
   disturbance response as well as certificate feasibility. Theta=1.5 is
   a promising sensitivity result, not the final production setting.
4. Implement a causal interface for missing/delayed poses and verify its
   assumptions. Evaluate motion-channel correction and calibrate loaded
   MnCAV inertia/tire parameters afterward; the present downstream controls
   do not identify those vehicle priors as the dominant error source.

No production setting, original reference, map, or raw dataset is replaced
by this diagnostic task. The actual research outputs are the source audit,
controlled trajectories, local response validation, and revised attribution.

## Reproduction, validation, and interpretation limits

Run from the repository root, using the existing YALMIP/SeDuMi installation:

```matlab
addpath(pwd); setupVehicleLocalization;
addpath(genpath('../RobustVehicleLocalization/external/YALMIP'));
addpath(genpath('../RobustVehicleLocalization/external/sedumi'));
maxNumCompThreads(8);
diagnoseMncavLocalizationError;
diagnoseMncavMatchingFrames;
```

```bash
uv run --with numpy --with pyproj --with rosbags python scripts/analyzeMncavReference.py
```

```matlab
diagnoseMncavReferenceResponse;
```

```bash
uv run --with numpy --with matplotlib python scripts/plotMncavErrorDiagnosis.py
```

MATLAB R2026a Update 3 executes all eleven runs and sixteen matching solves.
All three new MATLAB functions have zero factory Code Analyzer findings;
both Python scripts parse and execute. Exact baseline reproduction,
independent harmonic validation, native bag re-extraction, and integration-
step sensitivity are the relevant checks. The prior 104-test suite is not
repeated because production code is unchanged. Original numerical outputs,
full states, and visually inspected PNG/PDF plots are retained locally in
`output/mncav_error_diagnosis_20260914/`; compact metrics accompany this
report in Git. Generated figures and MAT states are also exported to the
archive, not committed as generated binaries.

Material passport: local material, experiment-critical decision basis,
recorded single-sequence deterministic validation, English technical
deliverable. No fabricated data, physical truth, executed experiment, or
vehicle identification is claimed. The statistical interpretation check
covers significance and null-result claims (none made), effect magnitude
(reported with units), multiplicity and post-hoc choice (disclosed),
dependence (time samples are not independent replicates), selection (all
samples retained; eight frame selections are diagnostic), association versus
causation (bounded interventions identified), measurement validity (mixed
reference audited), model validity (local response scope stated), uncertainty
(no population confidence intervals), and generalization (no second drive
or independent ground truth). No additive root-cause percentages are claimed.
