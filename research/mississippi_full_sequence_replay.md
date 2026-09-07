# Mississippi full-sequence perception and observer experiment

Activity date: 2026-09-07. Evaluation sequence:
`raw_data_2024-06-07-12-09-31_0`, 1170 front LiDAR scans.

## Experiment definition

The map is newly built using `buildFeatureMap` and the current offline fine
perception. It contains curb, road marking, pole and traffic sign channels;
facade is intentionally excluded for Mississippi. Its repeated-observation
GMM uses the current geometric-stability posterior and publication gates.
Every query recomputes coarse pillar perception and performs XY geometric D2D.
No pointwise fine validation runs online. All localization outputs are
`[X,Y,psi]`.

This requested first experiment builds and queries the same drive, including
the query scan's contribution to the map. The reference is the matched
NovAtel odom body-origin pose used for mapping, not raw antenna BESTPOS. Results
measure within-sequence consistency and pipeline behavior. They do not establish
independent-dataset accuracy, generalization, or absolute GNSS-free accuracy.

Recursive D2D uses one GNSS initialization plus `[0.5,-0.4,2 degrees]` error.
Subsequent initial guesses integrate recorded `/vehicle/twist` and the last
accepted D2D estimate. Rejections propagate that prediction. The whole
trajectory is scored, including rejected frames, rather than reporting only
successful registrations. `referenceSeed` remains an explicitly named optional
local-registration diagnostic and is not the main experiment.

The actual `runLateralVelocityObserver` and `runImprovedVehicleObserver` functions
are executed. Recorded CAN speed, steering, accelerations and gyro feed the
lateral observer; its lateral velocity and slip interface feed the seven-state
global observer. Accepted D2D events provide pose and the matcher-generated
information matrix. See [information derivation and contract](d2d_pose_information.md).
The global output scored is **onlineZ**, never the subsequently revised state
history `z`. D2D and the observer form a feed-forward replay here; global observer
predictions do not yet feed back into D2D initial guesses.

A fixed 150 ms LiDAR delivery delay is an experiment setting, not a selected
production bound. The number of measured matching calls exceeding it is
reported. GPS positions retain recorded arrival stamps. Scenarios include
GPS only, GPS plus D2D, and a synthetic loss of the GPS XY input on `[40,60)`
seconds with and without D2D. The latter is a **position-channel outage**:
D2D retains recorded known roll/pitch for source/map tilt compatibility. It
must not be described as a test of complete GNSS/INS device loss.

## Clock and input validation

The bag/ROS span is about 107.2 s, whereas receiver GPS time spans about
116.9 s. The fitted receiver/ROS elapsed-time slope is approximately 1.090927;
ignoring that mismatch would integrate recorded speeds and rates over the
wrong physical duration. A piecewise linear bridge uses paired INSPVA ROS
headers and receiver GPS seconds, without using positions to fit time.
Vehicle-motion integration and observer times use receiver seconds. High-rate
sensor resampling uses previous-sample hold. GNSS reference interpolation may
extrapolate at the final edge by less than 40 ms; the actual maximum is in
metadata. This is distinct from nearest scan/odom matching in the map.

CAN axes and constant offsets are checked on the **different** adjacent
`12:11:24` drive. Receiver seconds `(1,40)` fit offsets; later data validate
them. Offline calibration uses 51-sample, cubic Savitzky-Golay derivatives of
INS heading and velocity. It does not introduce acausal filtering into the
online replay. With the chosen forward/left convention, this recorded CAN
lateral acceleration requires a sign reversal. Unit gains are retained;
constant offsets are fitted by median residual. The calibration JSON records
the independent validation RMSE and unconstrained slope diagnostics. Remaining
road bank, attitude and sensor-to-CG lever-arm errors are not asserted to be
removed by a constant offset.

## MnCAV parameters and limits

UMN identifies MnCAV as a **2021 Chrysler Pacifica Hybrid**.
[UMN vehicle introduction](https://www.cts.umn.edu/news-pubs/news/2021/august/mncav).
Manufacturer specifications give 3.089 m wheelbase, 16.2:1 overall steering
ratio, 2273 kg EPA curb mass and 55.5/44.5 percent front/rear static loading.
The resulting unladen CG distances are `lf=1.374605 m`, `lr=1.714395 m`.
These are stock specifications and derived geometry, not measurements of the
instrumented, occupied MnCAV.
[Manufacturer 2021 Pacifica Hybrid specifications, pp. 2--4](https://www.stellantisfleet.com/content/dam/fca-fleet/na/fleet/en_us/chrysler/2021/Pacifica/specifications/2021_CH_PacificaHybrid_Specifications.pdf).

No reliable public MnCAV yaw inertia or axle cornering-stiffness measurement
was located. A direct independent-drive bicycle fit returned a negative rear
stiffness and negative inertia and was rejected. A separate yaw-response
identifiability probe reached parameter bounds. The INS reference-point lever
arm is also unknown. These failed fits are not used as physical parameters.
The nominal model is explicit: stock mass/geometry, uniform-planform inertia
`m*(5.189^2+2.022^2)/12 = 5874.6073 kg m^2`, and the pre-existing generic
stiffnesses scaled by the stock/generic mass ratio. These inertia and stiffness
values are **unidentified priors**. Joint `0.7` and `1.3` scaling of inertia and
both stiffnesses is a limited sensitivity experiment, not validation of the
true loaded vehicle model or a comprehensive uncertainty set.

Lateral gains are re-synthesized for each nominal model. The global speed
envelope is increased from 8 to 16 m/s and its gains are re-synthesized; each
global design is checked at all 65,536 certificate vertices. Runtime envelope
violations are reported. This certificate concerns the nominal continuous
model; it does not certify sample/hold pose pulses, delayed replay, GNSS
outages, physical-parameter uncertainty, or same-sequence map errors.

## Reproduction

Run from the repository root; original bags and GNSS exports remain unchanged.

```bash
uv run --with rosbags python scripts/extractVehicleReplaySensors.py \
  --output-dir output/mississippi_20240607_120931_20260907/sensors
uv run --with rosbags python scripts/extractVehicleReplaySensors.py \
  --bag data/raw/Missisipi/raw_data_2024-06-07-12-11-24_0.bag \
  --output-dir output/mississippi_20240607_120931_20260907/calibration_sensors
uv run --with numpy --with scipy --with pandas python scripts/calibrateMncavReplayInputs.py
```

Add the existing YALMIP and SeDuMi installations to the MATLAB path, then:

```matlab
setupVehicleLocalization;
maxNumCompThreads(8);
p=fullfile(pwd,'output','mississippi_20240607_120931_20260907');
results=runMississippiFullSequenceExperiment(p,fullfile(p,'vehicle_parameters.json'));
```

`rebuildMap=false` reuses the frozen map artifact. The replay also accepts a
compact MAT export containing `probabilityCloud=temporalMapToProbabilityCloud(map)`.
Frame blocks are loaded outside the timed computation and their actual read
times are recorded separately. Timing covers map selection, fresh coarse
perception, D2D and event construction; it excludes disk I/O, offline mapping,
one-time map preparation and plotting. Observer computation includes lateral
estimation and delayed replay; an amortized per-sample value is not a maximum
single-step deadline. No recorded timing establishes a hard real-time bound.

## Measured results

The map build completed in **1131.183 s**, including the full-map save. It
published 1723 components: 654 curb, 522 road marking, 347 pole and 200 traffic
sign components. The full history-rich map and compact registration export
are retained locally; file hashes and sizes are in the artifact manifest.

All 1170 scans were processed twice. The initial one-thread diagnostic used
individual file reads; the subsequent eight-thread run used blocks of 50.
Acceptance flags, position-error values and yaw-error values agree exactly
between the runs. These are unchanged algorithm outputs under different
execution/IO settings, not a before/after perception-quality comparison.

| Result | Value |
|---|---:|
| D2D accepted frames | 963 / 1170 (82.31%) |
| All-frame D2D/prediction position RMSE | 0.28138 m |
| All-frame position median / P95 / maximum | 0.10454 / 0.48866 / 1.56316 m |
| All-frame yaw RMSE / maximum absolute error | 0.60830 / 3.37680 degrees |
| One-thread computation median / maximum | 111.801 / 281.822 ms |
| Eight-thread computation median / P95 / P99 | 80.147 / 89.395 / 96.252 ms |
| Eight-thread maximum | 243.217 ms |
| Eight-thread calls above 100 ms | 4 / 1170 |
| Eight-thread perception / registration medians | 71.796 / 8.083 ms |

The four calls above 100 ms are frames 1, 818, 834 and 907. Frame 1 is the
243.217 ms maximum; the others are 102.089, 102.169 and 105.070 ms. No calls
are removed. The eight-thread run follows a complete one-thread replay and
therefore is not a fresh-process cold-start experiment. The 150 ms simulated
delay is exceeded by one matching call; 300 ms is not exceeded by these
matching calls, but neither observation proves a future whole-pipeline bound.

Rejections comprise 203 class-conflict cases, one degenerate geometry case,
one insufficient-overlap case and two nonconvergence cases. The continuous
vehicle-twist-only baseline ends 410.79 m from the reference (RMSE 215.32 m);
it is not substituted for accepted D2D poses and is not an observer result.

| Actual causal observer run | Position RMSE | Yaw RMSE | Outcome |
|---|---:|---:|---|
| GNSS XY + recorded CAN, no LiDAR | 0.23835 m | 1.08427 deg | 11690 samples completed |
| GNSS + D2D, 150 ms fixed delay | 0.23473 m | 0.59382 deg | 11690 samples completed |
| GNSS + D2D, 300 ms fixed delay | 0.23663 m | 0.60445 deg | 11690 samples completed |
| GNSS + D2D, dynamics scaled 0.7 | 0.23473 m | 0.59194 deg | Completed |
| GNSS + D2D, dynamics scaled 1.3 | 0.23473 m | 0.59485 deg | Completed |
| GNSS XY removed on [40,60), with D2D | Not a successful-run metric | — | Nonfinite at 49.57 s |
| GNSS XY removed on [40,60), without D2D | Not a successful-run metric | — | Nonfinite at 55.92 s |

The nominal 150 ms fusion applies 962 LiDAR events; the final accepted event
arrives after the recorded sensor horizon. Its maximum position/yaw error is
1.5970 m / 3.1060 degrees. Lateral plus global-observer computation, including
replay, is 8.822 s, or 0.755 ms amortized per 100 Hz sample. At 300 ms it is
11.760 s (1.006 ms/sample). These averages do not measure the worst individual
observer step, and they must not be added to the D2D maximum as a claimed
end-to-end maximum. The complete data stream is processed offline in this
experiment; disk read time is separate.

The limited nominal-parameter sweep barely changes the GNSS-present position
result. This does not identify the physical inertia or tire parameters and
does not establish robustness during a GNSS outage. The nominal full run has
98 velocity-envelope violations and 377 acceleration-envelope violations in
the causal online state (the revised history gives 78 and 372);
track-rate violations are zero. The trajectory is not entirely inside the
continuous certificate's operating box.

## Failure and timing interpretation

The with-D2D outage fails **9.57 s after removal of GNSS XY**, earlier than the
no-LiDAR case's 15.92 s. In `[40,49.57)` the matcher still processes 95 scans,
accepts 87, and its all-frame position error stays below 0.398 m. Accepted
events contain positive definite information and the observer's LiDAR XY
weight is nonzero. Thus failure cannot be described as an absent-information
interface or a complete loss of matching outputs. These observations do not
uniquely isolate every source of model mismatch.

There is a concrete certificate gap. `buildImprovedObserverCertificateData`
constructs only `diag(1,1,wPsi)` with `wPsi` between 0.15 and 1. It never
certifies GPS XY weights equal to zero or the no-pose part of the runtime's
short correction pulses. An audit of the **same** nominal matrices, using
all 65,536 combinations in each case, gives these worst Schur margins:

| Certificate mode | Worst margin |
|---|---:|
| Original GNSS-present family | -0.056051 |
| GNSS absent, strongest allowed LiDAR XY weight `W=I` | +1.0312 |
| All pose correction pulses inactive | +1.9052 |

The LiDAR term contributes `-2*Cl'*W*Cl` in the normalized certificate. `W=I`
is its strongest allowed negative-semidefinite contribution. The positive
margin therefore cannot be repaired in this certificate simply by increasing
information and saturating the existing bounded weight. This rejects the
current certificate for those modes; it is not a proof that no other observer
or certificate can work. No fictitious high-information matrix or alternative
fallback policy is inserted into the main experiments.

The failure wrapper preserves the exception and locates the finite prefix by
deterministic prefix reruns to 10 ms resolution. It does not clip, reset,
substitute truth for an estimate, or continue after a nonfinite state. Prefix
errors become enormous before numerical failure; the plot explicitly limits
its outage view to 300 m and marks both failures. Full values remain in the
local CSV/MAT files. Failure cases have no reported per-sample timing average.

A separate post hoc timing diagnostic compares the GPS-only output with
reference positions delayed by 0, 10, 20, 30 and 40 ms. At 0/20 ms the median
discrepancy is 0.1952/0.0424 m. This is consistent with lag from the held-pose
correction window, but does not uniquely establish its cause. **No reference
shift is used in any primary metric, model calibration or online output.**

The next observer design must explicitly cover absolute-position information
availability, sampled correction intervals and fixed delay, with a stated
outage-duration/observability assumption. The current full run demonstrates
GNSS-present operation and exposes an outage failure; it does not validate
the intended GNSS-loss robustness.

## Artifacts and validation

Compact public exports are in
[`results/mississippi_full_sequence_20260907/`](results/mississippi_full_sequence_20260907/).
They contain per-call errors, information and timing, full-rate summary metrics,
10 Hz error-trace exports, 87 unique passing tests, design verification,
vehicle provenance, and hashes of the original/local generated artifacts.
Full 100 Hz states, raw sensor CSVs, maps and plotted PNG/PDF/SVG remain under
`output/mississippi_20240607_120931_20260907/`; these large/raw artifacts are
not committed. The eight-thread result is in `recursive_8threads`.

Seven new information-contract tests and five motion-integration tests are
included in the 87 passing cases. Four initially filtered solver/data tests
were rerun with the available solver paths and dataset root; all passed.
Additional executed checks include the full recorded replay, independent input
calibration, three re-synthesized lateral/global designs, and the omitted-mode
certificate audit. A CSV acceptance-flag type mismatch and the reference's
28.680 ms final-edge gap were diagnosed and handled explicitly in the replay
adapter. Factory MATLAB Code Analyzer reports no findings in the final changed
MATLAB files; Python entry points compile successfully.

The top-level experiment driver reproduces the one/eight-thread runs, nominal
and sensitivity designs, all seven observer cases, the certificate audit and
overview plots. The archived measurements were executed through these stage
functions while the driver was assembled, rather than through a second full
map rebuild after completion. The original one-thread IO diagnostic used the
older individual-frame loader; new reproductions use the explicit block loader.

Material passport: empirical, same-sequence replay; no preregistered acceptance
thresholds, human subjects or statistical significance tests. Raw data and
runtime artifacts are local; small derived metrics and reproducible final
source are versioned. Nominal dynamics are partly sourced and partly explicit
unidentified priors. All complete and failed cases above are retained. This
report supports engineering diagnosis, not independent accuracy, continuous
GNSS-loss certification or a hard real-time claim.
