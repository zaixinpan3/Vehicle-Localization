# Vehicle localization observer

The current dataset localization entries use **overlapGradient only**.
The pose-registration branch was archived on 2026-10-12; see
[the scope, historical version and recovery instructions](REGISTRATION_ARCHIVE.md).
The registration sections below document historical experiments and shared
geometry. They do not select an alternative current localization channel.
New Mississippi runs use `runMncavFullObserverExperiment` without a matching
replay prerequisite; Downtown and CARLA use their existing entries with the
same overlap-only default.


## File layout

The module contains the following entry points and support classes. Entry points are files; shared helpers are
static methods of the `*Support` classes and are called with the class prefix,
for example `registrationSupport.selectLocalProbabilityCloud(map,seed,radius)`.
Stages with a single caller are local functions of that entry point.

| File | Contents |
| --- | --- |
| `localizeLidarFrame.m` | Archived standalone frame-registration diagnostic |
| `updateLocalizationSourceWindow.m` | Causal multi-scan source horizon on recorded odometry |
| `registerSemanticProbabilityCloud.m` | Archived standalone pose-registration solver |
| `prepareSemanticRegistrationGeometry.m` | Geometric, support and anisotropic D2D models: association, residuals, linearization |
| `registrationSupport.m` | Registration utilities: projection, preparation, overlap, pose events, map selection and view conditioning, canonical pyramid, position-aided selection, soft association, line directions, relative height, residual primitives |
| `runFullLocalizationObserver.m` | Current intrinsic eight-state observer; seven-column public state interface |
| `invariantObserverSupport.m` | Intrinsic prediction, invariant innovations, fresh synthesis and solver-free verification |
| `fullObserverSupport.m` | Continuous ISS gain design and certificate, input synchronization, GNSS output point |
| `lidarInjectionSupport.m` | LiDAR information filter, matched correction, residual reevaluation, correspondence-free overlap gradient, calibrated error moments, line measurements |
| `lidarCalibrationSupport.m` | Causal LiDAR IMU tilt and its provenance check, offline pitch and translation calibration |
| `poseSupport.m` | Planar pose and rigid transform of recorded pose-table rows |
| `estimateWheelLongitudinalSpeed.m` | Four-wheel longitudinal speed estimate |
| `runImprovedVehicleObserver.m` | Continuous MO-HGO analysis runner for GNSS or delayed LiDAR |
| `runMotionAidedVehicleObserver.m` | Historical LiDAR-only motion-aided observer |
| `observerAnalysisSupport.m` | Design, certificates, channel evaluation, scenarios and signal reconstruction of the two analysis runners |
| `poseGraphSupport.m` | Experimental robust sliding-window pose graph |
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

The scalar outputs include speed norm, velocity/acceleration dot and cross
products, estimated rate, kinematic/course-direction constraints and acceleration
projections onto estimated body directions. Each is invariant to a simultaneous
rotation of the global states and heading. The two acceleration projections
remain available without wheel speed and preserve acceleration direction.
Gyro and projected-speed innovations have separate LMI-designed scalar gains.
Gyro corrects the rate state; heading prediction uses the posterior state.

The global runtime consumes `LateralInputs.sideSlipAngle` and
`LateralInputs.sideSlipAngleRate` together with lateral velocity. Synchronization
retains both series on the same frame clock; it never reconstructs an angle
from velocity. Missing angles are rejected instead of selecting a fallback.
The standard lateral observer exports both from one persistent second-order
interface. Its angle can lag the velocity direction, so lateral-angle error is
an auxiliary disturbance, not an exact direction identity. Downtown retains
its existing output-point motion adapter and now exports that adapter's angle
and derivative together; KITTI withdraws velocity-direction outputs explicitly.

Run `synthesizeMncavInvariantDesign` with YALMIP and SeDuMi to regenerate
`config/mncavInvariantDesign.json`. Runtime independently verifies its 256
main/gyro/aligned-speed vertices and scalar LMIs without a solver; changed design
inputs reject stale gains. The nominal objective includes all eight normalized
errors. The four default noise cases have a pose anchor; dead reckoning has no
absolute-pose steady-state objective. The diagonal angular metric prevents
LiDAR heading noise from directly updating the internal rate.

The verified inequalities concern a continuous affine/sector enclosure.
Finite invariant innovations admit a local cubic-remainder bound, conditional
on a strict base margin. Establishing that margin for the actual overlap
operator remains open; `S=F*M` alone does not certify it. No numerical local
radius is claimed. The precise equations, assumptions and
remaining tasks are in [the existing theory, Section 0.5](../improved_observer_derivation.md#05-implemented-architecture-intrinsic-prediction-and-invariant-imu-outputs).
There is no discrete-time or arbitrary-outage guarantee. Feature deficiency
and actual missing outputs are retained in runtime diagnostics.

## Unified sensor-redundancy theory

[Section 0.6](../improved_observer_derivation.md#06-one-complete-observer-with-complementary-gnsslidar-guarantees)
now proves local nonlinear ISS for one complete continuous observer when either
GNSS or LiDAR supplies sufficient information. The two adequacy sets share one
constant Lyapunov matrix and one constant gain set; they do not select different
observers. Informative LiDAR permits GNSS weight zero. Sufficient position-only
GNSS permits LiDAR information of any rank, including zero, provided motion and
the velocity-direction/body-acceleration auxiliary channels supply heading
information. A parked vehicle with no LiDAR heading remains unobservable from
position and gyro rate alone. Section 0.7 permits finite stops under an explicit
recurring-information condition.

The proof bounds the actual overlap operator as `F*Hbar = S + Delta`, including
its support projection, rather than substituting Gauss--Newton information for
the true gradient derivative. Both adequate and degraded geometry require the
stated mismatch/bias bounds. Joint LMIs retain the direction channel for the
GNSS guarantee; Bernstein controls certify its whole speed interval. Uniform
higher-order bounds then close all nonlinear predictor and invariant innovations.

Run `verifySensorRedundancyTheory` to check the separate documented theoretical
witness without an SDP solver. It verifies 128 LiDAR vertices and 480 GNSS
Bernstein controls on the declared 10--15 m/s GNSS motion domain. This is a
conservative sufficient domain, not a necessary physical speed threshold.
The witness does not replace or certify `mncavInvariantDesign.json`, nor verify
the geometry contracts for a recorded map. `Synthesize=true` re-solves only
the theory; it never installs gains. Default runtime certificate flags remain
unchanged.

## Zero-speed scope

The intrinsic predictor no longer divides by speed or speed squared: its
estimated yaw-rate state remains defined when velocity is zero, including
stopped rotation. This removes that model singularity and is covered by the
stopped-rotation runtime test.

The global auxiliary channel also no longer evaluates `atan2(vy,vx)`. It uses
the upstream angle/rate interface, which remains defined through stops in the
standard lateral observer. This is an interface state, not a physical motion
direction at a zero velocity vector. The lateral runtime uses division-free master,
stationary/crawl and shadow branches outside the LPV domain. Its reciprocal-speed
LPV model and LMI remain restricted to the positive interval [5,30] m/s. Safe
zero-speed execution is distinct from a unified zero-speed hybrid stability
certificate; neither the current surrogate nor actual-overlap discussion claims
that stronger result.

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

Accepted D2D registration now exports `lidarResidualModel`, a serializable
model of the actual accepted coarse or fine solution. It retains source/map
Gaussian geometry, normals or shape factors, frozen robust/class/temporal/view
influence, and the admitted pose subspace. The online matcher passes it directly
to `lidarInjectionSupport.evaluateLidarRegistrationResidual`, which reevaluates the residual and its
physical Jacobian at the post-baseline prediction. The source Gaussian scatter
rotates using the same analytic derivative as the matcher. The support model's
latent sliding covariance and any soft-association scatter are retained.

Partial geometry evaluates `anchor+projector*(prediction-anchor)` and applies
the same projector to its Jacobian. This preserves rejected directions in both
innovation and information. Between-hypothesis pose spread from GNSS-assisted
selection is applied as a low-rank residual whitening to both quantities; it
can only reduce information, and adds no GNSS curvature. New replay MAT files
retain models in `report.lidarResidualModels`; aligned observer packets use
`data.lidar.residualModels`. Existing files without this geometry use the local
approximation `xi=S*(D\(qL-predictedPose))`, with wrapped heading difference.
Validated directional events use filtered `directionalInformation`. Rejected
registrations remain rejected, and a directional match cannot initialize a full
absolute pose. Both interfaces assume a consistent local angle chart and
associations. Initializer, GNSS candidate selection and map correlations remain
possible: information is a geometric surrogate, not calibrated inverse pose
covariance or an assertion of independent measurement noise.

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
The current continuous surrogate has a separately verified LMI enclosure;
its actual overlap operator margin remains unproved. The finite invariant
secants have a conditional local remainder bound, without a numerical radius. Thus
`fullContinuousStateCertified=false`, while `surrogateContinuousLmiVerified`
reports the numerical inequalities. An arbitrary longitudinal outage with
curb-only geometry cannot guarantee full position convergence. Historical
seven-state ISS certificates retain their own declared assumptions.

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
replaces the registration channel and is mutually exclusive with `data.lidar`
and `data.lidarMatcher`. `sources{k}` is the coarse source horizon of frame
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
  axis (from `prepareSemanticRegistrationGeometry`), and each pair is weighted
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

## Historical LiDAR-only motion-aided baseline

The independent historical comparison entry points below are not selectable
design or runtime branches of `runFullLocalizationObserver`. Its configuration
no longer inherits `motionAidedObserverConfig`. Earlier transported-runtime
results remain in their dated studies and Git history; reproducing that removed
implementation requires the corresponding historical revision.

For the historical MnCAV experiment with precomputed, zero-delay LiDAR, use
`runMncavMotionAidedExperiment`. Its seven-state motion-aided observer directly
corrects velocity and acceleration with body-frame motion measurements.
LiDAR position residuals correct position without feeding the derivative
states. The nominal physical gains `[kp,kv,ka,kpsi]=[4,4,12,4]` were selected
on the first 60 s, subject to a new common quadratic certificate and four
accuracy constraints. The same frozen gains improve position RMSE, peak,
P95 and heading RMSE against INSPVA over the full and reserved intervals.
The all-reference/all-metric gate remains false because mixed-ODOM full-run
P95 rises by 5.18 mm; this exception is retained in the report.

```matlab
cfg = motionAidedObserverConfig;
design = observerAnalysisSupport.designMotionAidedObserverGains(cfg);
estimate = runMotionAidedVehicleObserver(data, lateralInputs, cfg);
% Recorded comparison using all existing precomputed frames:
report = runMncavMotionAidedExperiment;
```

This path accepts aligned piecewise-linear LiDAR pose with `delay=0` and
`headingConvention="unwrapped"`. Motion inputs and lateral outputs use the
same grid as the prior zero-delay experiment. Initialize from the first
measurement and measured motion, or supply `cfg.initialState` on that yaw
lift. The runner never receives evaluation references. Geometric weights
come from the full information matrix; its XY block and yaw diagonal are
applied in separate cascade stages, without XY-yaw feedback cross terms.
Qualified knot weights are linearly reconstructed between knots.

See the [derivation and gain design](../research/mncav_motion_aided_20260914/design.md)
and [recorded/synthetic validation](../research/mncav_motion_aided_20260914/validation.md).
This design has its own conditional ISS certificate. The old delayed-HGO
certificate is not reused for it. Positive-delay and GNSS behavior below
continue to use `runImprovedVehicleObserver`.

The global observer integrates the seven states
`[X,Vx,Ax,Y,Vy,Ay,psi]`. It implements the two continuous measurement contracts
in [the ISS derivation](../improved_observer_derivation.md): position-only
GNSS, or uniformly informative LiDAR pose with a known fixed delay. The mode is
fixed for a run. The independent lateral observer supplies `vy`, `beta`, and
`betaDot`; its output can also be supplied explicitly for isolated testing.

The vehicle and observer are continuous systems with constant matrices within
each measurement mode. The integration grid is a numerical approximation of
their equations.

## Continuous input contract

Call `setupVehicleLocalization` from the repository root. `sensorData.highRate`
contains aligned finite column vectors `time`, `steeringAngle`,
`longitudinalSpeed`, `longitudinalAcceleration`, `lateralAcceleration`, and
`yawRate`. Time increases strictly. Body acceleration is inertial acceleration
after gravity/lever-arm compensation. These arrays define linear numerical
input reconstructions; they are not measurement-arrival events.

The selected source is exactly one of:

* `sensorData.gnss.evaluate(t)`: a pure function returning the current physical
  position `[X;Y]`. No continuous heading output is used.
* `sensorData.lidar.evaluate(t)`: a pure function returning a struct with
  `pose=[X;Y;psi]` and a symmetric physical `information` matrix. At evaluation
  time `t`, the pose describes time `t-d`, where `d=source.delay` equals
  `cfg.measurement.fixedLidarDelay`. Declare `headingConvention="unwrapped"`.
  The runner subtracts its own estimate at **that same past time** and does
  not delay the measurement again.

Providers must meet the continuous-signal contract over the whole interval.
They are queried only at current integration/evaluation times; missing or
nonfinite measurements raise an error. Information bounds are checked at every
evaluated LiDAR point. These finite evaluations cannot establish a bound
between evaluations for an arbitrary function provider.

For an explicitly reconstructed numerical signal, replace `evaluate` with
`representation="piecewiseLinear"`, `time` equal to the high-rate grid, and
`position` (N-by-2) or `pose` (N-by-3). LiDAR also needs a 3-by-3-by-N
`information` array, its fixed `delay`, and the lifted-heading declaration.
Finite aligned arrays have no missing-sample fallback. Interpolation of positive
information matrices preserves their common lower matrix bound. Linear
reconstruction from stored samples can use future endpoints and is an offline
input model, not a claim of physical continuous delivery or online causality.

`observerAnalysisSupport.reconstructContinuousObserverSignals` is the separate recorded-data adapter.
It takes physical pose timestamps, applies the fixed LiDAR offset once, trims
to covered input times, and explicitly constructs aligned continuous arrays.
It rejects original gaps exceeding its declared interpolation limit (default
0.12 s) and rejects inadequate LiDAR directions. That limit is a reconstruction
quality setting; it is not a timer or a measurement-switching theorem.
Registration records from `localizeLidarFrame` are data products and are not
direct continuous-observer inputs.

## GNSS position mode

```matlab
cfg = improvedObserverConfig("gnss");
design = observerAnalysisSupport.improvedObserverReferenceDesign(cfg);
data.highRate = highRateInputs;
data.gnss = struct('evaluate', @(t) [8*t;0]);
cfg.observer.initialState = [0;8;0;0;0;0;0];
estimate = runImprovedVehicleObserver(data, lateralDesign, design, cfg);
```

The first six states use the position and three rotational-invariant outputs.
The yaw row uses raw estimated velocity in
`psiHatDot = r_m + kPsi*(VyHat*cos(psiHat+betaHat)-VxHat*sin(psiHat+betaHat))`.
It does not feed heading errors back into the first six states. State and
prediction are never clipped; only the first three auxiliary maps are
extended outside their velocity/acceleration box.

The theorem requires sustained motion and an invariant local heading-error
sector. Standstill runs remain numerically possible, but their diagnostics
report that the motion condition is unmet; the observer cannot infer heading
at rest from position alone. A measured speed check does not prove the true
speed bound or the initial heading-error bound. `initialHeading` is an initial
estimate used if `initialState` is omitted, not another measurement channel.

## LiDAR fixed-delay mode

```matlab
cfg = improvedObserverConfig("lidar");
design = observerAnalysisSupport.improvedObserverReferenceDesign(cfg);
d = cfg.measurement.fixedLidarDelay;
data = struct('highRate', highRateInputs);
data.lidar = struct('delay', d, 'headingConvention', "unwrapped", ...
    'evaluate', @(t) struct('pose', [8*(t-d);0;0], ...
                           'information', 1e6*eye(3)));
cfg.observer.initialState = [0;8;0;0;0;0;0];
history = @(t) [8*t;8;0;0;0;0;0];
estimate = runImprovedVehicleObserver(data, lateralDesign, design, cfg, ...
    InitialHistory=history);
```

For positive delay, `InitialHistory(t)` must return seven finite states on
`[t0-d,t0]` and match the initial current estimate at `t0`. An old LiDAR pose
cannot silently initialize the current state. Arbitrary continuous estimated
histories are permitted by the theorem; their true error is an initial
condition whose magnitude is not known from sensor data alone.

RK4 uses the method of steps: a numerical step is no longer than the delay,
so every stage can read an already available past estimate. Cubic Hermite
interpolation uses accepted endpoint states and their actual ODE/DDE
slopes. Numerical cuts also respect propagated initial-history derivative
breaks at multiples of the delay. Only a delay-length history bracket is kept;
old outputs are never changed. A zero delay is supported as the current-output
ODE limit, with its matrices reverified for that delay.

Information is normalized with the fixed pose scales `S`:
`J=S'*information*S`, `W=J/(gainInformationScale*I+J)`. The injection is
`T*K*W*(S\measuredPose-S\pastEstimatedPose)`. It retains all translation/yaw
cross terms. No inverse information, artificial eigenvalue floor, or partial
pose substitution is used. Insufficient information raises an explicit error.
All four extended auxiliary outputs remain continuously active in this mode.

## Design and certification

For reduced GNSS startup peaking, select the optional constant-gain preset:

```matlab
cfg = improvedObserverConfig("gnss", "lowPeaking");
design = observerAnalysisSupport.improvedObserverReferenceDesign(cfg);
```

This uses `theta=8`, chain coefficients `[6;8;3]` and `yawGain=0.1`.
The metric is recomputed and verified at the original operating bounds.
The reference profile remains the default. In the same synthetic noisy
sedan trial, the acceleration-error peak falls from 134.17 to 34.60 m/s²,
while sampled settling increases from 0.89 to 3.51 s. Peaks remain substantial;
this preset has a smaller certificate margin and is not an optimum or a
general physical validation. Run `tuneSyntheticObserverPeaking` for the
[parameter comparison](../research/observer_peaking_tuning_20260913/validation.md).

`observerAnalysisSupport.improvedObserverReferenceDesign(cfg)` loads
[the constant-matrix artifact](../config/continuousObserverCertificate.json)
and recomputes its numerical certificate. `observerAnalysisSupport.verifyImprovedObserverDesign`
checks actual matrices and gains; it ignores saved `certified` flags.
`observerAnalysisSupport.designImprovedObserverGains` constructs the GNSS chain gains or synthesizes
constant LiDAR `P,Q,R` for fixed gains using either a norm enclosure or
a four-vertex course-rate enclosure with residual norm bounds. There is one
runtime and one current certificate implementation.

| Reference mode | Scaling | Course-rate bound | Additional requirement |
|---|---:|---:|---|
| GNSS position | 20 | 0.6 rad/s | Positive speed and admitted local yaw sector |
| LiDAR pose | 1 | 0.00215032 rad/s | 150 ms delay, `0.99852005 I <= W <= I` |

Both use true velocity/acceleration component bounds of 16 m/s and 5 m/s².
The LiDAR artifact is a conservative existence example, not a newly certified
0.6 rad/s vehicle design. Increasing delay or widening its sector requires
successful reverification or synthesis. Actual rate excursions are evaluated
without clamping and are reported as outside the certificate envelope.

`observer.certificateVerified` means the continuous matrix inequalities passed.
`observer.certified` remains false because true-state/noise/heading assumptions
and numerical integration error are not established by the runner.
`diagnostics.certificateConditions` separates coefficient observations from
these unverified hypotheses. A successful simulation is not a full ISS
certificate for a physical sensor implementation.

## Outputs and validation

`z` and `onlineZ` contain the same immutable states with **lifted yaw**.
`pose` and `heading` wrap yaw to `[-pi,pi]`; `headingUnwrapped` retains the lift.
Other outputs include position, velocity, acceleration, speed, lateral inputs,
pose/auxiliary innovations, normalized information weights, and the delayed
states actually used. No residual is wrapped across angle branches internally.

`LateralInputs` optionally supplies aligned `time`, `lateralVelocity`,
`sideSlipAngle`, and `sideSlipAngleRate`; pass `struct()` for the unused
`lateralDesign` when isolating the global stage. If omitted, the real lateral
observer runs once and supplies its interface as before.

```matlab
results = runtests('tests/improvedObserverTest.m');
report = validateStandaloneObserver;
demo = demoSyntheticVehicleObserver;
```

`demoSyntheticVehicleObserver` provides a self-contained sedan experiment
using the actual lateral/global cascade, analytic steady-turn truth, incorrect
initial states, and clean/noisy GNSS and delayed LiDAR signals. It displays
all state traces and saves figures and metrics. See the
[synthetic demonstration results](../research/synthetic_vehicle_observer_20260913/validation.md),
including GNSS startup peaking and the LiDAR startup certificate excursion.

The tests compare GNSS yaw with its nonlinear analytic solution and delayed
yaw with the independent MATLAB `dde23` solver, as well as checking the delay
history, information directions, missing measurements, source clocks, unit
normalization, bounded memory, and immutable prefix outputs. The standalone
validation exercises all seven states under analytic motion and continuous
bounded noise. See [the runtime validation](../research/continuous_observer_runtime_20260913/validation.md).

The recorded-data entry offers separate `gnss` and `lidar` reconstructions
and retains contract failures as failures.

`validateSyntheticLocalizationCascade` can exercise the current synchronous
production chain end to end with synthetic signals and exact truth: lateral
observer, frame alignment, MnCAV synchronous observer, source outages, the
online matcher callback and lateral-model mismatch. See the
[synthetic cascade record](../research/synthetic_localization_cascade_20260929/README.md),
including its historical GNSS-only bias limitation and the numerically zero
LPV gain. That September record predates the intrinsic eight-state revision
and does not validate its new equations.

## Registration helpers

The archived standalone registration entry points are `localizeLidarFrame` and
`registerSemanticProbabilityCloud`; `prepareSemanticRegistrationGeometry`
builds the geometric, support and anisotropic models. Shared methods are static
methods of `registrationSupport`: cloud projection and preparation, Gaussian
overlap and its scoring, pose-event export, local map selection and matching,
view conditioning, the canonical pyramid, position-aided hypothesis selection,
soft point association, line directions, relative-height association and the
Gaussian/support residual primitives.

## LiDAR tracking preset

`improvedObserverConfig("lidar","tracking")` selects a verified constant-gain
alternative for the current continuous delayed LiDAR observer. Each Cartesian
chain changes from `[3;3;1]` to `[3;3;1.5]`; theta, N, yaw gain, delay and
operating/information bounds are unchanged. Load it through
`observerAnalysisSupport.improvedObserverReferenceDesign(cfg)`; stored matrices are reverified.
The default stays `reference`. The preset reduces settled position, velocity
and acceleration error by about 20% on a synthetic variable-speed gentle turn,
with a 36% larger acceleration-error peak. Its narrow course-rate certificate
is unchanged. See the [experiment and tradeoffs](../research/lidar_tracking_tuning_20260914/validation.md)
and `tuneContinuousLidarTracking` for reproduction.

## Target vehicle: UMN MnCAV

Use `mncavVehicleConfig` and `lateralObserverConfig("mncav")` for new
vehicle-dependent work. The central source identifies the 2021 Pacifica Hybrid,
stock mass/geometry/steering ratio, and explicitly unmeasured inertia/stiffness
priors. Re-synthesize lateral gains for this vehicle; the archived reference
sedan gains are not its identified design. `validateMncavObserverParameters`
checks the global LiDAR profiles with motion generated by the nominal MnCAV
bicycle plant and separate dynamic-parameter sensitivities. The global
kinematic equations themselves contain no mass or stiffness parameter.
See [MnCAV parameter provenance and limits](../research/mncav_parameter_validation_20260914/validation.md).
The current `tracking` profile remains provisional: recorded MnCAV course rates
extend far outside its narrow LiDAR certificate. Stock parameter matching
alone does not establish deployment suitability.


## MnCAV LiDAR operating-range preset

Select `improvedObserverConfig("lidar","mncav")` for the new data-informed
course-rate design. It uses theta=2 and a common four-vertex delay certificate
covering |yawRate+sideSlipAngleRate| <= 0.4 rad/s at 150 ms delay. This covers
the existing recorded audit maximum of 0.354975 rad/s, with unchanged stated
information and true-state bounds. The 0.4 bound is a design envelope, not a
physical vehicle limit or a guarantee for every future drive.

The verifier retains q and q^2 explicitly instead of replacing both by one
unstructured disturbance bound. It recomputes all four matrix inequalities;
no runtime turn-rate clamping is introduced. The reference/tracking profiles
remain historical alternatives. Fourteen wider-turn nominal MnCAV simulations
show improved position/velocity/acceleration tracking with larger initial
peaks and slightly worse heading noise response. Use
`validateMncavObserverParameters(folder,SteeringScale=220,Profiles=["tracking","mncav"])`
to reproduce. See the [proof, results and limitations](../research/mncav_lidar_envelope_20260914/validation.md).

## Archived recorded registration cascade

`runMncavFullLocalizationExperiment` freshly synthesizes MnCAV lateral and
global gains, runs the actual lateral observer, uses its velocity with
corrected gyro/speed for recursive raw-scan map matching, and executes the
global observer. `auditMncavFullLocalizationExperiment` compares both outputs
at identical scan timestamps and exports actual gains and error traces.

This historical experiment was explicitly offline: accepted pose gaps up to
one second are linearly reconstructed, and information gain-shaping lambda
is set to .001 with fresh verification. Original matching information is
preserved. Recorded gaps require future scans for some reconstructed inputs;
the 150 ms DDE setting is not demonstrated real-time latency. Same-drive map
results show improved heading and worse position RMSE after global fusion.
See the [complete experiment and limitations](../research/mncav_full_localization_20260914/validation.md).

## Archived precomputed registration poses

The archived entry `runMncavZeroDelayExperiment` now refuses execution. Its
pre-retirement implementation is recoverable at the commit in REGISTRATION_ARCHIVE.md.
It computes and saves all 1,170 raw-frame matching results before starting
the global observer. A separate cache records frame IDs, capture times,
acceptance, original poses and information. Rejected frames have no valid
cached measurement pose; their prediction output is not relabeled as LiDAR.

`observerAnalysisSupport.reconstructFrameAlignedLidarSignals` merges the original LiDAR timestamps
into the numerical input grid. With `fixedLidarDelay=0`, each accepted frame
supplies its unchanged pose/information at exactly its own capture time.
The existing continuous observer integrates between frames using the declared
offline linear reconstruction of adjacent accepted poses. There is no
instantaneous state reset or 150 ms source shift. All frame times, including
the final fractional motion-grid interval, have explicit output rows.

The entry retains the MnCAV feedback gains and independently verifies the
existing certificate matrices at zero delay. It freshly designs and runs
the lateral observer. Whole-sequence metrics use the original 100 Hz grid
and native scan grid separately, avoiding extra weight for inserted knots.
Both the mixed ODOM reference and the same-receiver INSPVA diagnostic
reference are reported. See the [zero-delay experiment](../research/mncav_zero_delay_20260914/validation.md).
