# Vehicle localization observer

The maintained estimator is the eight-state invariant observer with
**overlapGradient** LiDAR corrections. See [the current entry points](CURRENT_PIPELINE.md).
The retired pose-registration and seven-state implementations were removed;
[the retirement record](REGISTRATION_ARCHIVE.md) identifies their Git recovery
revision. Historical studies retain their original configurations and results.

## File layout

The module contains the following entry points and support classes. Entry points are files; shared helpers are
static methods of the `*Support` classes and are called with the class prefix,
for example `registrationSupport.selectLocalProbabilityCloud(map,seed,radius)`.
Stages with a single caller are local functions of that entry point.

| File | Contents |
| --- | --- |
| `updateLocalizationSourceWindow.m` | Causal multi-scan source horizon on recorded odometry |
| `registrationSupport.m` | Shared projection, overlap, map selection/view conditioning, canonicalization and finite-residual algebra |
| `prepareSemanticOverlapGeometry.m` | Mixture support axes and evidence weights for landmark overlap |
| `runFullLocalizationObserver.m` | Current intrinsic eight-state observer; seven-column public state interface |
| `invariantObserverSupport.m` | Intrinsic prediction, invariant innovations, common gain synthesis and solver-free verification |
| `sensorRedundancySupport.m` | Shared common L/G/M matrix construction, exact GNSS lever enclosure and certificate verification |
| `fullObserverSupport.m` | Continuous ISS gain design and certificate, input synchronization, GNSS output point |
| `lidarInjectionSupport.m` | LiDAR information filter, matched correction, residual reevaluation, correspondence-free overlap gradient, calibrated error moments, line measurements |
| `lidarCalibrationSupport.m` | Causal LiDAR IMU tilt and its provenance check, offline pitch and translation calibration |
| `poseSupport.m` | Planar pose and rigid transform of recorded pose-table rows |
| `estimateWheelLongitudinalSpeed.m` | Four-wheel longitudinal speed estimate |
| `lateralObserver/designLateralObserverGains.m` | LPV lateral-observer gain synthesis |
| `lateralObserver/runLateralVelocityObserver.m` | Hybrid lateral-velocity observer runtime |
| `lateralObserver/simulateLateralObserverScenario.m` | Lateral-observer scenario simulation |
| `lateralObserver/lateralObserverSupport.m` | Bicycle model, scheduling coordinates and polytope, gain blending, vehicle check |


## Current continuous design and IMU placement

`mncavFullObserverConfig()` selects the intrinsic eight-state route. Its
predictor uses `[X,VX,AX,Y,VY,AY,psi,r]`, with internal course rate
`r_hat + estimatedBetaDot`, `psidot=r_hat`, and nominal `rdot=0`. Raw IMU
samples enter auxiliary innovations and the lateral observer. The intrinsic
predictor has no IMU argument and no speed denominator. Public `.z` and
`.onlineZ` keep their seven-column interface; `.internalState` and `.yawRate`
expose the additional estimated rate.

The scalar outputs include squared speed, velocity/acceleration dot and cross
products, estimated rate, kinematic/course-direction constraints and acceleration
projections onto estimated body directions. Each is invariant to a simultaneous
rotation of the global states and heading. The two acceleration projections
remain available without wheel speed and preserve acceleration direction.
Squared speed has a strictly positive matched-output weight synthesized with
all other global gains. There is no projected-speed substitute or independent
scalar speed gain. Gyro retains its separate LMI-designed scalar gain and
corrects the rate state; heading prediction uses the posterior state.

The global runtime consumes `LateralInputs.sideSlipAngle` and
`LateralInputs.sideSlipAngleRate` together with lateral velocity. Synchronization
retains both series on the same frame clock; it never reconstructs an angle
from velocity. Missing angles are rejected instead of selecting a fallback.
The standard lateral observer exports both from one persistent second-order
interface. Its angle can lag the velocity direction, so lateral-angle error is
an auxiliary disturbance, not an exact direction identity. Downtown retains
its existing output-point motion adapter and now exports that adapter's angle
and derivative together; KITTI withdraws velocity-direction outputs explicitly.

`mncavFullObserverConfig` returns parameters only. Immediately before a test or
experiment, solve temporary gains with YALMIP and SeDuMi:

```matlab
cfg=mncavFullObserverConfig();
% Apply this experiment's sensor, output-point and domain overrides here.
[design,cfg]=synthesizeMncavInvariantDesign(cfg);
estimate=runFullLocalizationObserver(data,struct(),cfg,LateralInputs=lateral);
```

The solver returns an in-memory design and ready-to-run configuration. It
always re-solves, even when the supplied configuration already has gains; it
reads/writes no gain file and has no persistent gain cache. The entry point
solves missing gains before the first update. Gains may be shared within the
current experiment, while a later experiment solves again. Solver failure is an
error, with no old-gain fallback. Runtime independently verifies the common
L/G/M matrices, scalar gyro LMI and feedback-authority bounds without a solver.
The ordinary geometry and partition settings give 48 L, 8640 G and 1152 Mixed
controls (9840 total). A zero GNSS lever needs fewer G/M controls: the minimum
GNSS weight dominates all larger weights and coincident lever vertices are
omitted. A changed fixed high-gain scale, common domain, architecture or
sensor/vehicle input rejects stale temporary gains.
The nominal objective includes all eight normalized errors at four pose-anchored
cases; it is a noise proxy, not measured covariance.

The active synthesis uses the exact moving-course Jacobian restriction and
retains the GNSS lever coupling in a finite outer polytope. Feedback limits are
included as LMIs; innovations are not clipped at runtime. The fixed normalized
L/G position self-feedback limits are 8, velocity-direction is 2, and the
L/direction heading limits are 8. The optional angular/translation metric-balance
constraint is withdrawn; rotation invariance and the diagonal angular metric
remain. These choices reduce the excessive authority of the first conservative
common design.

`jointSensorLmiVerified` and `conditionalNonlinearGainConditionsVerified` mean
that these common numerical gain conditions passed. The complete nonlinear
regional ISS theorem still requires the declared actual-overlap geometry/bias,
motion, auxiliary-error and local-region contracts. A conditional finite-error
radius is computed from the synthesized gains, but actual map bounds and sampled
stability remain unverified. Physical full-state and sampled certificate flags
therefore remain false. See [the current speed-output and domain proof](../improved_observer_derivation.md#08a-current-squared-speed-feedback-and-positive-speed-iss-domain).

## Unified sensor-redundancy theory

[Section 0.6](../improved_observer_derivation.md#06-one-complete-observer-with-complementary-gnsslidar-guarantees)
now proves local nonlinear ISS for one complete continuous observer when either
GNSS or LiDAR supplies sufficient information. The two adequacy sets share one
constant Lyapunov matrix and one constant gain set; they do not select different
observers. Informative LiDAR permits GNSS weight zero. Sufficient position-only
GNSS permits LiDAR information of any rank, including zero, provided motion and
the velocity-direction/body-acceleration auxiliary channels supply heading
information. A parked vehicle with no LiDAR heading remains unobservable from
position and gyro rate alone. Every current L/G/M certificate is restricted to
`minimumCourseSpeed <= true course speed <= maximumSpeed`, with a strictly
positive lower bound. The default interval is [10,15] m/s; the explicit
small-offset scenario uses [1,30] m/s. Historical finite-stop extensions in
Section 0.7 are not certified for the current revision.

The proof bounds the actual overlap operator as `F*Hbar = S + Delta`, including
its support projection, rather than substituting Gauss--Newton information for
the true gradient derivative. Both adequate and degraded geometry require the
stated mismatch/bias bounds. Joint LMIs retain the direction channel for the
GNSS guarantee; Bernstein controls certify its whole speed interval. Uniform
higher-order bounds then close all nonlinear predictor and invariant innovations.

Run `verifySensorRedundancyTheory` to solve and verify a separate temporary
theoretical witness. It uses the same L/G/M matrix construction and declared
positive speed interval as production synthesis. This is a
conservative sufficient domain, not a necessary physical speed threshold.
This witness does not replace the runtime design or verify recorded-map
geometry. A supplied `Witness` and freshly solved `Config` can be reverified
without an SDP solve. Documented numerical examples are historical records and
are never parsed as live gains. The runtime design has its own refined common
certificate, checked by `invariantObserverSupport.verify`.

Observer test classes share `freshObserverGainsFixture`: within its lifetime,
each distinct configuration is solved once and reused, then discarded at
teardown. A second `runtests` call in the same MATLAB process creates a fresh
fixture and solves again. The lateral observer already synthesizes its current design at
test/experiment setup; no saved lateral gains are loaded.

## Zero-speed scope

The intrinsic predictor no longer divides by speed or speed squared: its
estimated yaw-rate state remains defined when velocity is zero, including
stopped rotation. This removes that model singularity and is covered by the
stopped-rotation runtime test. Zero speed is outside every current L/G/M ISS
claim, even with adequate LiDAR. All verification reports mark
`zeroSpeedCertified=false`; numerical finite execution does not imply bounded
error for an arbitrary stop duration. Runtime never clamps speed into the
certified interval. `observedSpeedWithinIssDomain` reports only a measured-speed
proxy, not verification of the true-speed assumption.

The global auxiliary channel also no longer evaluates `atan2(vy,vx)`. It uses
the upstream angle/rate interface, which remains defined through stops in the
standard lateral observer. This is an interface state, not a physical motion
direction at a zero velocity vector. The lateral runtime uses division-free master,
stationary/crawl and shadow branches outside the LPV domain. Its reciprocal-speed
LPV model and LMI remain restricted to the positive interval [1,30] m/s. Safe
zero-speed execution is distinct from a unified zero-speed hybrid stability
certificate; neither the current surrogate nor actual-overlap discussion claims
that stronger result.


Restoring squared-speed feedback changes the transient response. The low-speed
sampled tests do not preserve the old projected-speed settling rates: curb and
residual tests retain a 2 mm target at 5 s, and the landmark test retains its
1 mm target at 20 s. The stationary overlap example retains its 1 cm target
at 30 s instead of 8 s, without a zero-speed ISS claim. The 2 m/s frame-overlap
test checks finite execution and
90% initial-error reduction at 20 s only. Longer fixed-map stress cases still
show drift; these checks do not establish actual-map or sampled ISS or an
empirical accuracy improvement.

## Removed main-observer implementations

The direct-gyro seven-state implementations, their comparison-matrix and
measured-course theta synthesis, saved gains and dedicated calibration replay
were deleted on 2026-10-13. `mncavFullObserverConfig()` is the only main-observer
factory. Old replay configurations are rejected; rebuild them with the current
entry points. Historical implementation/results remain recoverable at Git
commit `5cf9fae8103e87b519a679ea236ed5fda2d6a08e`. Historical dated studies describe
their executed version, not an alternative live algorithm.

## Current synchronous runtime

The intrinsic eight-state architecture is the sole main-observer runtime.
Seven-column initialization/public state arrays remain an interface convenience;
they do not select a seven-state prediction branch.

The continuous bias and local-region arguments remain conditional on a uniform
inequality for the actual overlap operator `W=F*Hbar`. The finite invariant
secants are handled by the conditional local remainder argument. These theoretical assumptions need no per-frame enforcement in the
sampled implementation.

Aligned inputs may declare `highRate.bodyVelocityValid`,
`highRate.bodyAccelerationValid` and `highRate.yawRateValid`, one Boolean per
frame. Missing masks retain the all-valid contract. Invalid payloads, including
NaNs, are ignored and their innovations withdrawn. Missing speed is never
replaced by a predicted velocity. Gyro availability is required by the stored
continuous surrogate certificate; runtime can still withdraw it without claiming
that certificate covers the interval. Frontend tilt/deskew use of measured motion
is separate from intrinsic global prediction.

The KITTI Raw transfer entry is `runKittiOverlapTrial(datasetFolder, outputFolder)`
under `scripts/kitti/`; add YALMIP and SeDuMi before running it. The dataset
folder is a synchronized drive with its date-level calibration in the parent.
Odd zero-based scans build the OXTS-reference map and even scans are queries.
Queries use body inertial outputs, gyro-integrated tilt after one initial
tilt, and one initial reference pose with a declared offset. Initial velocity
is zero and velocity-output feedback is withdrawn. No later OXTS navigation
pose, orientation or velocity enters the observer. Gains are freshly synthesized
for declared transfer assumptions. The weak-LiDAR objective and rate model
were revised during this development trial; its scores are development results,
not an untouched validation set.
This is a global-branch trial with single-frame sources and no scan deskew;
it does not validate the vehicle-specific lateral observer or constitute the
official KITTI odometry benchmark. OXTS body inertial outputs are processed
device outputs, not a claim of independently recorded raw analog IMU signals.
Only metrics, a summary, logs and a figure persist; no numerical caches are kept.

### Dataset reference boundary

Frozen maps built using reference or simulator-truth poses are an explicit
design input, not reference leakage. Local map selection and view weighting use
the observer prediction. The current Mississippi entry loads
`output/receiver_reference/map/view_conditioned_cloud.mat`; the base
`probability_cloud.mat` is a distinct map artifact.

The current dataset protocols disclose these additional uses:

* Mississippi initializes once from a reference pose plus `(0.5, -0.4 m, 2 deg)`.
  Subsequent motion, tilt and source windows use measured wheel/IMU/lateral
  signals. Only epoch/clock fields and the first initialization pose are consumed
  from the synchronized pose CSV. Scoring-reference values do not enter the
  observer data structure.
* Downtown initializes once from the exact first-query GLIM pseudo-reference
  pose. Velocity is measured; later reference poses and GNSS do not enter
  tracking. Reported errors measure agreement with that pseudo-reference.
* CARLA initializes from the preceding three seconds of noisy GNSS and measured
  motion. The runtime consumes exported measurements and ray-cast XYZ, with no
  true pose, true velocity, semantic labels or realized sensor-bias channel.
  Truth generates the simulated sensors upstream; that is part of the declared
  sensor model, not an additional observer input. LiDAR-only operation starts
  after this measured-GNSS bootstrap.

Mississippi BESTPOS is an INS-assisted receiver solution (types 54/55/56), as
defined in the [NovAtel BESTPOS specification](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm).
Its coordinates are projected directly from the raw log, but it shares the
receiver navigation solution with the INSPVA-derived scoring reference. These
real-data discrepancies therefore do not establish independent absolute accuracy.
The fixed Mississippi LiDAR-frame translation was fitted/selected on that drive;
theta was selected on frames 1--700 and checked on 701--1169. This is fixed
calibration/model selection, with no per-frame truth correction, and whole-drive
scores include development data.

The October 12 audit paired real-data prefixes across all 14 sequences: seven
32-frame Mississippi modes, eight 24-frame Downtown runs and three 24-frame
modes on each of five CARLA routes. Removing/changing reference artifacts while
keeping initialization, measurements and maps fixed produced exactly identical
seven-state estimates in all 30 comparisons; Mississippi source components also
matched. All 21 targeted isolation tests passed, and all five complete CARLA
sensor exports reproduced their declared seeded generation model. These are
prefix checks plus code/input-provenance review, not a full-length poisoned-data
rerun or a formal exhaustive information-flow proof. The small audit summary is
kept beside the performance tables in
`output/current_dataset_replay_20261012/truth_audit.json`; no separate temporary
research report is required.

`runFullLocalizationObserver` always uses a baseline backward-Euler step and
one implicit LiDAR correction per common localization frame, after the
continuous ISS-LMI check. The historical transported runtime and its fallback
for configurations without `timing` have been deleted. Requesting
`timing="historical_transport"` is an error. Omitting `timing` cannot select
another implementation or bypass the continuous design.
`fullObserverSupport.synchronizeLocalizationInputs` aligns motion/lateral
estimates and BESTPOS positions on native LiDAR times, approximately 10 Hz.
No GNSS/LiDAR pose is propagated between frames and no 100 Hz localization
trajectory is generated. `highRate` remains the compatibility field name, but
contains frame-rate samples after synchronization. The wheel and lateral
estimators retain their separate upstream preprocessing; this is not a claim
that raw sensor acquisition and every estimator run at 10 Hz.

```bash
uv run --offline --with numpy --with pandas --with pyproj python scripts/prepareMncavBestpos.py
uv run --offline --with rosbags --with numpy --with pandas --with pyproj python scripts/calibrateMncavBestposOutputPoint.py
```

```matlab
setupVehicleLocalization;
report = runMncavFullObserverExperiment;
validation = validateFullLocalizationObserver;
```

The complete replay builds raw-scan whole-pillar coarse horizons using
measured wheel/gyro/lateral odometry, then injects their overlap gradients at
the observer prediction. The existing offline map stays fixed. No optimized
LiDAR pose or registration acceptance enters the current dataset workflow.
A historical `MatchingFolder` can supply acquisition/motion metadata only;
new runs construct the horizons directly without that cache.

The default experiment reads `/novatel/oem7/bestpos` in EPSG:32615 instead of
ODOM XY. The recorded BESTPOS types 54, 55 and 56 are INS-assisted solutions;
this channel must not be called pure GNSS. No ODOM position is read by the
current entry point. INSPVA timestamps locate the common epoch; INSPVA state
is used for evaluation and the existing reference-assisted map, not the
BESTPOS export. `mncavFullObserverConfig` loads a relative output-point offset
calibrated using seconds 1--40 of the separate 12-11-24 drive. Runtime subtracts
the rotated offset using its own estimated heading and adds calibration/heading
uncertainty to the receiver covariance. This empirical planar correction is
not a surveyed physical installation transform.

GNSS alignment uses bounded linear brackets of actual samples, not motion
extrapolation. It records both endpoint timestamps and the required future
sample wait. No interpolation crosses an invalid endpoint or a bracket longer
than 0.15 s. Overlap sources retain their native frame times. Motion and
lateral values use the existing 100 Hz preparation, interpolated at the common
frame time. Thus this experiment is offline synchronization; zero LiDAR
processing delay does not mean zero alignment latency.

Each available source must have exactly the same time array as `highRate`.
Invalid or missing packets withdraw that frame's channel. Both sources missing
leaves the motion-aided prediction. GNSS corrects position only; without LiDAR,
heading is uncorrected gyro integration. Synchronized lateral estimates are
required. The baseline uses gyro-predicted heading for motion rotation and
receiver-point correction, then LiDAR corrects the predicted pose jointly.
The corrected heading enters the next baseline step. There are no integration
substeps or `maximumIntegrationStep` setting. The MnCAV gains are
solved from the ISS LMI (see "Gains solved from the ISS LMI" below). The implementation before
Route A and its earlier measurements are described in the
[current calibration, gain and accuracy report](../research/mncav_bestpos_alignment_20260917/README.md)
and the [discrete equations](../research/mncav_synchronous_bestpos_20260917/README.md).

### Information-aware LiDAR injection

The current eight-state model retains heading and estimated rate at standstill.
Its matched metric and fixed scaling come from the intrinsic design described
above. The injection helper accepts general state dimensions for standalone
geometry tests; that general algebra is not a second observer implementation.

For `D=diag(cfg.lidar.poseScales)`, `I=D'*information*D` and
`C=D\(G*T)`, `lidarInjectionSupport.filterLidarPoseInformation` constructs
`F=U*diag(rho./(lambda+lambdaStar))*U'` and
`S=U*diag(rho.*lambda./(lambda+lambdaStar))*U'`.
Here `lambdaStar=cfg.lidar.gainInformationScale>0`; it is excluded from the
reported information spectrum. The matched rate is
`kappa*T*(P\(C'*xi))`, with current `kappa=kappaL/theta`. Eigenvalues, eigenvectors, rank,
trace, effective dimension and retained directional strengths are reported.
Rank-deficient and zero information are valid. A curb's longitudinal nullspace
is preserved. There is no minimum-eigenvalue information floor, yaw-diagonal
gate or extra multiplication by point count.

The generic `data.lidar` interface supports synthetic pose and finite-residual
validation of the same observer. `data.lidar.residualModels` can carry frozen
geometry for reevaluation by `lidarInjectionSupport.evaluateLidarRegistrationResidual`.
Pose-only inputs use the local approximation
`xi=S*(D\(qL-predictedPose))`, with wrapped heading difference. These interfaces
require a consistent local angle chart and stated residual assumptions.
Information is a geometric surrogate, not automatically calibrated inverse
pose covariance or independent measurement noise. Dataset entries instead
evaluate the overlap gradient directly, as described below.

For an external residual model, `data.lidar.evaluateResidual(k,predictedPose)` can supply
`residual`, physical `jacobian`, `weights`, and `linearizationPose` at the
post-baseline prediction on an admitted frame. `lidarInjectionSupport.buildLidarLineMeasurement`
constructs frozen point-to-line geometry from calibrated, deskewed vehicle
points and associated unit map normals. The correction uses `xi=-F*D'*J'*W*r`.
Weights may be diagonal or full PSD precision for correlated residuals. Do not
use the optimizer's final near-zero gradient. Nuisance variables are not
estimated here: an upstream nuisance elimination must consistently reduce both
information and innovation, with any prior stated explicitly.

Optional source arrays `frameReliability` (N-by-1) and `directionReliability`
(N-by-3) range from zero to one. Directions follow ascending normalized
information eigenvalues; repeated eigenspaces use their minimum reliability.
Residual providers can also supply reliability, multiplied by source reliability.
These gates require upstream association/support/noise evidence; eigenvalues
alone do not establish reliability.

At a frame, set `Q=kappa*C'*S*C`. The implicit LiDAR jump has error map
`(I+h*(P\Q))\e`, whose energy-coordinate eigenvalues are `1/(1+h*lambdaZ)`.
It is nonexpansive for any nonnegative step under the frozen affine noiseless
model. The reusable correction helper also offers explicit updates capped at
`h*max(lambdaZ)<=1.8<2`. Neither result certifies nonlinear association changes,
map errors or the entire baseline-plus-jump system. The diagnostics retain
`sampledSystemCertified=false` and `baselineFullStateCertified=false`.
The continuous certificate requires the sensor and finite-error assumptions
stated above, including the actual overlap operator. Its conditional local
region does not certify arbitrary map geometry or sampled switching. An
arbitrary longitudinal outage with curb-only geometry cannot guarantee full
position convergence.

This extension follows the fixed quadratic metric and two-chain coordinates
in Bessafa et al., *Generalized multi-output high-gain observer with application
to ego vehicle trajectory and orientation estimation*, Sections 2.2 and 4.2,
Theorem 5, [DOI: 10.1016/j.automatica.2026.112915](https://doi.org/10.1016/j.automatica.2026.112915).
The new LiDAR jump is not certified by that paper's original continuous LMI.

Longitudinal speed remains exclusively wheel derived. The lateral observer
exports its velocity and side slip at the INSPVA output point that the pose
state, map and GNSS correction share: its master state belongs to the IMU
location, and `lateralObserverConfig("mncav").outputPoint.forwardOffsetM`
(2.36 m, `config/mncavMotionOutputPoint.json`, fitted on the separate
12-11-24 drive) transports it by `vyOutput = vyObserver - d*r`. The same
transported velocity drives the source-window odometry. The IMU accelerations
still enter the global observer at the IMU location; their lever-arm terms
(`rDot*d`, `r^2*d`) are not compensated.
`prepareWheelMotionInputs` reads wheel rates, steering and IMU with no alternate
speed fallback. Missing/expired wheel aiding is an error, except the declared
short stationary startup. The pre-Route-A output directory is
`output/mncav_coarse_localization_20260924b`. On 1170 raw scans, the 1169
observer outputs have fused position RMSE 5.96 cm (5.42 cm after the 2 s
initialization transient) and heading RMSE 0.39 degrees; GNSS-only is 6.33 cm
and LiDAR-only 14.83 cm (14.24 cm after the transient). See
[the canonical map pyramid record](../research/canonical_pyramid_20260924/README.md),
[the output-point transport record](../research/output_point_transport_20260924/README.md)
and, for the earlier 10.3445 cm result without GNSS aiding or the transport,
the [coarse-only replay report](../research/mncav_coarse_localization_20260918/README.md).
The earlier 7.8474 cm experiment used stored fine features and per-frame
reference matching seeds. It is a historical comparison, not an isolated
coarse-versus-fine test. Both experiments use a same-drive map and shared
INSPVA reference, rather than independent absolute ground truth.

### Correspondence-free overlap-gradient channel

`data.lidarOverlap=struct('map',map,'sources',{sources},'valid',valid,'config',overlapGradientConfig())`
is mutually exclusive with `data.lidar`. Retired `data.lidarMatcher` callbacks
are rejected. `sources{k}` is the coarse source horizon of frame
`k` in its current body axes; the optional `valid` defaults to the nonempty
sources. At every valid frame the runtime calls
`lidarInjectionSupport.evaluateOverlapGradient(map,sources{k},prediction,config)`
at the post-baseline prediction. For every bandwidth `sigma` of
`config.scaleLadder` both mixtures are smoothed horizontally by
`N(0,sigma^2/2*I)`, which adds `sigma^2*I` to every pair covariance. The
class-balanced cross energy `E_sigma` is then the expected overlap under an
isotropic horizontal prediction error of that size; `sigma=0` is the exact
score of `registrationSupport.scoreSemanticProbabilityCloudAlignment`. Balanced
self energies are one, so `E_sigma` is the normalized similarity. The channel
injects

- the gradient `g=sum_sigma -grad(E_sigma)/E_sigma`, the analytic SE(2)
  derivative of `registrationSupport.semanticGaussianOverlap`, and
- the information `M=sum_sigma sum_ij pi_ij*J_ij'*inv(Sigma_ij)*J_ij`, the
  Gauss-Newton (EM) metric of each `-log E_sigma`. Here `pi_ij` are the pair
  responsibilities at that scale and `J_ij` is the pair residual Jacobian in
  `[X,Y,psi]`, whose yaw column carries the lever arm.
  `semanticGaussianOverlap` returns this metric as its third output.

The metric is positive semidefinite and exact for one pair at any translation.
The regularized observer step therefore stays bounded and points to the optimum
even where the exact Hessian is indefinite. Summing the scales fuses them in
information form: the sharp scale dominates near alignment, and the wide scales
keep a restoring force where the sharp kernels have decayed. With
`evidenceWeights` (default true), the map view reliability and the source
temporal stability scale the mixture masses before class balancing, as
registration weights its correspondences. The channel optimizes no pose,
selects or freezes no correspondence, and applies no acceptance test. Zero
overlap at every scale withdraws only that frame's LiDAR correction.

`computeLidarMatchedCorrection` accepts `gradient`, `information` and a
`linearizationPose` equal to the prediction in place of a residual, and uses
`xi=-F*D'*g`; for the same geometry this equals the residual form
`-F*D'*J'*W*r`. Gradient components outside the positive-information directions
are not injected and are reported as `unsupportedGradientNorm`. The calibrated
gain profile is rejected because its error calibration describes registration
poses. Diagnostics report `lidarChannel="overlapGradient"` and, per frame, the
`lidarOverlap` evaluation, availability, similarity (first ladder entry),
minimum information eigenvalue, time and gradient.
`runMncavFullObserverExperiment(LidarChannel="overlapGradient",OverlapScaleLadder=ladder)`
runs the recorded scenarios with this channel; an empty ladder keeps the
configured one.

The results below come from the 2024-06-07 MnCAV replay with the fixed Route A
inputs: 1169 frames, intensity-responsibility sign moments and evidence weights
on. The local study `research/channel_comparison_20261008` is not versioned.
Each cell gives position RMSE over all frames and heading RMSE after the first
2 s:

| LiDAR channel | Nominal, LiDAR only | Nominal, GNSS and LiDAR | Fixed scale, LiDAR only | Fixed scale, GNSS and LiDAR |
| --- | --- | --- | --- | --- |
| Registration (Route A) | 10.20 cm / 0.138 deg | 7.84 cm / 0.123 deg | 5.66 cm / 0.114 deg | 5.99 cm / 0.112 deg |
| Overlap, ladder `[0 0.5 1 2]` m (default) | 9.72 cm / 0.228 deg | 7.28 cm / 0.160 deg | 6.35 cm / 0.161 deg | 5.90 cm / 0.150 deg |
| Overlap, exact scale only | 16.60 cm / 0.859 deg | 7.51 cm / 0.164 deg | 6.45 cm / 0.163 deg | 5.86 cm / 0.152 deg |
| Overlap, scale-normalized ladder | 8.20 cm / 0.249 deg | 6.79 cm / 0.197 deg | 7.66 cm / 0.187 deg | 7.19 cm / 0.182 deg |

The default ladder has lower position RMSE than registration in three settings
and is 0.7 cm worse in the fixed-scale LiDAR-only run. Its heading RMSE is
0.04--0.09 deg worse in every setting. The heading gap is not a weaker yaw gain:
both channels inject nearly the full yaw innovation, with median strength 0.997
for the overlap and 0.994 for registration. Its median body-axis information is
48 along track, 50 across track and 6.7e3 in yaw, against 17, 21 and 3.2e3 for
registration. So the gap comes from the yaw that the overlap gradient implies.

Ladder sensitivity:

- `[0 0.5 1]` and `[0 0.5 1 2 4]` m agree with the default within 1 mm.
- `[0 1 2]` is worse without GNSS: 12.0 cm with nominal gains.
- Scale-space normalization weights each scale by `1+sigma^2/sigmaBar^2`. This
  gives every scale equal precision and overstates the information about
  fourfold. With the fixed-scale gains the normalized ladder loses lock near
  frame 598. The default ladder is therefore an unweighted sum.

The following options were removed:

- The exact Hessian as information, with absolute or positive-part
  eigenvalues. A Hessian ladder diverged with the fixed-scale gains (123 m).
- A single smoothing bandwidth.
- Similarity and curvature frame gates. The curvature gates removed the only
  along-track information in weakly observable stretches. A similarity gate
  blocked acquisition, because the overlap at the 0.9 m initial offset is 0.002.

An overlap evaluation takes 3.6 ms median per frame, against 11--15 ms for
registration; a replay takes 5 s against 15--19 s. The ladder, the weights and
the information scale were all chosen on this evaluation recording, with no
held-out drive. The overlap is a similarity, not a likelihood: its metric is not
a calibrated inverse covariance, and overlap optima still follow view-dependent
sampling density along curbs.

The unit-test scene shows where the ladder's reach ends. The ladder converges
from 6 and 9 m across the curbs, where the exact scale diverges. At those
offsets the exact responsibilities collapse onto a single pole pair: the
effective pair count is 1.0 and the metric has rank two. The minimum-norm step
then turns part of the translation error into a rotation, 8.9 deg in the first
update. The wider scales keep about five effective pairs and a full-rank
metric. From 12 m the ladder also diverges, while the exact channel finds no
overlap and stays silent. The ladder therefore widens the basin but does not
make it global. Registration remains the default channel.

#### Per-landmark likelihood with oriented components (opt-in)

Three `overlapGradientConfig` fields replace the frame-level cost at every
ladder scale: `aggregation="landmark"`, `outlierDensity` and `orientation`. The
new cost is

`c_sigma(p) = -sum_j v_j log(outlierDensity + sum_i u_i N_ij K_ij)`

from `registrationSupport.semanticLandmarkLikelihood`. It is the mixture
likelihood of every source landmark under the map's class density, with a
uniform outlier floor. The terms are:

- `N_ij`: the same Gaussian overlap integral as the frame-level cost.
- `u_i`: map masses, normalized to one per class.
- `v_j`: source masses, with a share of 1/C per class.
- `K_ij`: with `orientation`, every component carries registration's support
  axis (from `prepareSemanticOverlapGeometry`), and each pair is weighted
  by `exp(-kappa_ij*sin(dtheta_ij)^2/2)`, with
  `kappa_ij=c_i*c_j/(angularFloor^2+a_i+a_j)`.

The evidence factors multiply after the class normalization, as in
registration. The injected gradient and Gauss-Newton metric are analytic; the
metric adds `kappa_ij*(d dtheta_ij/d psi)^2` to yaw.

The local study `research/heading_gap_20261008` (not versioned) traced why the
frame-level overlap has worse heading. Started at the INSPVA reference, the
overlap cost's own fixed point has a yaw RMS of 0.212 deg, against 0.151 deg
for registration on the same frames. Three mechanisms were identified:

- The frame-level `-log sum` gives the responsibilities to the sharpest pairs,
  about 3 effective pairs at the sharp scale. A few poles and signs then set
  yaw through their lever arms.
- Evidence factors applied before class balancing are normalized away, so a
  weakly supported singleton sign keeps its full class share.
- The L2 overlap barely measures orientation. For two elongated Gaussians with
  variances `a` along and `b` across, the curvature of `-log N` in relative
  rotation is `(a-b)^2/(4ab)`, about 20 rad^-2 for a curb pair. Registration's
  angular residual, with its 1 deg floor, contributes up to `1/floor^2`, about
  3300 rad^-2 per pair at full axis confidence, and about 1400 rad^-2 for the
  test scene's curbs. Without it registration's yaw RMS rises from 0.152 to
  0.174 deg, and without its robust weights to 0.200 deg.

Per-landmark aggregation lowers the overlap fixed point's yaw RMS to 0.186 deg,
and orientation lowers it further to 0.162 deg. Registration's sliding
covariance made the overlap worse (0.287 deg), so it is not used. Among the
tested settings, an outlier floor of 1e-4 m^-2 (from 1e-6 to 1e-3) and an
angular floor of 1 deg (from 0.5 to 2 deg) were best; all reasonable settings
gave 0.154--0.172 deg.

Observer replays on the same inputs, run in one session; position RMSE /
heading RMSE after the first 2 s:

| Variant | Nominal, LiDAR only | Nominal, GNSS and LiDAR | Fixed scale, LiDAR only | Fixed scale, GNSS and LiDAR |
| --- | --- | --- | --- | --- |
| Overlap, frame (default) | 9.72 cm / 0.228 deg | 7.28 cm / 0.160 deg | 6.35 cm / 0.161 deg | 5.90 cm / 0.150 deg |
| Landmark | 9.62 cm / 0.196 deg | 7.12 cm / 0.142 deg | 6.10 cm / 0.145 deg | 5.76 cm / 0.136 deg |
| Landmark with orientation | 9.27 cm / 0.131 deg | 7.08 cm / 0.127 deg | 5.91 cm / 0.115 deg | 5.67 cm / 0.115 deg |
| Registration (Route A) | 10.20 cm / 0.138 deg | 7.84 cm / 0.123 deg | 5.66 cm / 0.114 deg | 5.99 cm / 0.112 deg |

With orientation, heading RMSE is within 0.004 deg of registration in every
setting, and the mean heading error is smaller: -0.040 to -0.052 deg, against
-0.050 to -0.076 deg. Position is better than registration in three settings
and 0.25 cm worse in the fixed-scale LiDAR-only run, where the 0.35 m excursion
near frame 894 remains. The support geometry raises the per-frame cost by about
15--40%, to 4--10 ms in these runs.

In the unit-test scene, the landmark likelihood with the ladder converges from
1 to 9 m across the curbs and does not diverge from 12 m. With the exact scale
alone it stays silent beyond about 3 m, because of the outlier floor. Unlike the
frame overlap, the mixture likelihood is not exactly unbiased when same-class
components overlap: the noise-free scene settles 2e-5 m and 4e-5 deg from the
truth. The option stays off until it is adopted.

Earlier versions of this channel used the exact score with a central-difference
curvature, and they lost lock twice. Both losses started at a coarse sign
pillar flagged by one or two marginally bright returns; all of that pillar's
nonground returns (the pole) entered the sign Gaussian. Coarse sign moments are
now weighted by intensity responsibility, with a minimum soft sign mass (see the
top-level README). Source horizons now hold 1.86 signs instead of 2.90, and the
burst frames publish no sign.

## Maintained inputs and validation

`prepareMncavMotionInputs` prepares wheel, IMU and steering inputs with the
current vehicle and receiver-clock metadata. GNSS positions and overlap
sources are prepared by their own stages. Use
`fullObserverSupport.synchronizeLocalizationInputs` to align motion and lateral
outputs before calling `runFullLocalizationObserver`.

`validateSyntheticLocalizationCascade` exercises the current observer with
synthetic measurements, availability changes and lateral-model mismatch.
`validateFullLocalizationObserver` covers the maintained MnCAV entry.
The tests under `tests/` synthesize fresh gains; they include invariant-output,
Mixed-coupling, source-window, overlap-gradient and pipeline-reference checks.
See [the pipeline guide](CURRENT_PIPELINE.md) for the supported dataset entries
and [the vehicle configuration](../config/MNCAV_CONFIGURATION.md) for parameter
and small-offset scenario requirements.
