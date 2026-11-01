# Vehicle localization

The maintained estimator is a one-way cascade: the lateral LPV observer
supplies motion outputs to the intrinsic eight-state global observer, whose
LiDAR channel evaluates semantic overlap gradients. There is one production
global-observer runtime, `runFullLocalizationObserver`.

This is the module's only Markdown guide. Mathematical derivations belong in
[the observer theory](../improved_observer_derivation.md), including
[the coupled gain construction](../improved_observer_derivation.md#08b-coupled-invariant-output-gain-synthesis).
Experiment protocols and results belong with their dataset/baseline entries
and the manuscript, rather than in this runtime guide.

## Entry points

| Stage | Entry point |
| --- | --- |
| Vehicle and replay parameters | `mncavVehicleConfig`, `mncavReplayConfig` |
| Wheel/IMU/steering preparation | `prepareMncavMotionInputs` |
| Lateral gain synthesis | `designLateralObserverGains` |
| Lateral execution | `runLateralVelocityObserver` |
| Global configuration | `mncavFullObserverConfig`, `mncavSmallOffsetObserverConfig` |
| Fresh global gain synthesis | `synthesizeMncavInvariantDesign` |
| Frame-clock alignment | `fullObserverSupport.synchronizeLocalizationInputs` |
| Global execution | `runFullLocalizationObserver` |
| Current LiDAR measurement | `lidarInjectionSupport.evaluateOverlapGradient` |

Current dataset runners are `runMncavFullObserverExperiment`,
`replayDowntownGnssFreeLocalization`, `runNuscenesOverlapTrial`,
`runKittiOverlapTrial`, and `runMncavVdbLocalization`. Their initialization,
sensor availability and mapping/reference protocols are experiment-specific.
Covariance-filter, source-observer and FAST-LIO2 comparisons live separately
under [scripts/baselines](../scripts/baselines/README.md).

## State and sensor interfaces

The internal global state is `[X,VX,AX,Y,VY,AY,psi,r]`. Public `.z` and
`.onlineZ` retain seven columns for position, velocity, acceleration and
heading; `.internalState` and `.yawRate` expose the estimated rate. The
seven-column output is not a seven-state observer branch.

Intrinsic prediction uses the estimated rate `r`, upstream sideslip-rate
estimate and internal states. Raw IMU measurements enter the auxiliary
outputs and upstream lateral observer, not global prediction. The five
auxiliary outputs are squared speed, velocity-acceleration dot product,
velocity-acceleration cross product, course direction and yaw rate.

The lateral module directly exports its LPV velocity/rate state, with
algebraic output-point transport and a separate angle/rate filter. It has no
inertial master, bias, stationary/crawl correction, filtered LPV injection or
shadow velocity estimator. Its positive-speed design interval is [1,30] m/s.
Outside that interval, or on invalid dynamic input, the LPV state is held
and `velocityValid=false`; modes are `dynamic` and `unavailable`.

The global observer reads the supplied `lateralVelocity`, `sideSlipAngle`
and `sideSlipAngleRate`; it does not reconstruct the angle with
`atan2(vy,vx)`. On unavailable velocity it withdraws auxiliary rows 1--4 and
the upstream sideslip-rate prediction term. Missing acceleration withdraws
rows 2--3. GNSS, LiDAR and gyro corrections retain their own availability.
Do not interpret held lateral states as valid low-speed estimates.

Use the output-point conventions and current vehicle/replay configuration in
[config/MNCAV_CONFIGURATION.md](../config/MNCAV_CONFIGURATION.md). Rebuild
inputs if their vehicle, clock or calibration metadata is stale.

## Fresh gains and stability scope

```matlab
setupVehicleLocalization;
cfg = mncavSmallOffsetObserverConfig();
% Apply this experiment's sensor, geometry and domain overrides before solving.
[design, cfg] = synthesizeMncavInvariantDesign(cfg);
lc = lateralObserverConfig();
ld = designLateralObserverGains(lc);
lateral = runLateralVelocityObserver(data.highRate, ld, lc);
[data, lateral] = fullObserverSupport.synchronizeLocalizationInputs(data, lateral, cfg);
estimate = runFullLocalizationObserver(data, ld, cfg, LateralInputs=lateral);
```

Gain synthesis requires YALMIP and SeDuMi on the MATLAB path. Each test or
experiment solves fresh global and vehicle-specific lateral gains; there is
no saved-gain fallback. Fresh in-memory designs may be shared within one
paired experiment. The runtime independently verifies its common L/G/M
certificate and configured response/feedback conditions.

The base certificate uses one metric across informative-LiDAR, adequate-GNSS
and mixed sensor conditions. The current base factory declares [10,15] m/s;
the explicit small-offset scenario declares [1,30] m/s and a hypothetical
GNSS output-point offset. These are different geometry/configuration
assumptions, not interchangeable defaults. The lateral interval and the
global true-course interval must both hold for a cascade claim.

Production gain design also uses the configured nominal motion-response
construction. The small-offset factory uses engineering objective selection
within the original LMI feasible set; it does not impose the removed hard
motion-to-position coupling constraint. Read the configuration factories and
[the coupled derivation](../improved_observer_derivation.md#08b-coupled-invariant-output-gain-synthesis)
for the mathematical construction rather than historical solved numbers.

The nonlinear ISS conclusion is conditional on the declared motion,
auxiliary-error, actual-overlap information/bias and local-region contracts.
It is not an actual-map, arbitrary-initial-error or sampled-system
certificate. Prediction has no speed denominator at standstill, and LiDAR
may still correct supported pose directions; zero speed and arbitrary LPV
exit/re-entry remain outside the complete in-domain ISS claim. GNSS position
and gyro rate alone do not uniformly determine heading at standstill.

## LiDAR runtime

Current dataset inputs use `data.lidarOverlap`. The runner predicts and
applies GNSS/invariant corrections, evaluates the overlap gradient once at
that state, then applies an implicit Lyapunov-matched LiDAR correction.
There is no online pose optimizer or registration callback. The current
runtime uses the common localization clock; it does not propagate delayed
registered poses or select a historical transported implementation.

`overlapGradientConfig` selects the scale ladder and aggregation; its default
is the class-balanced frame overlap at bandwidths `[0,0.5,1,2]` m.
`semanticGeometryConfig` supplies geometry settings. Landmark likelihood and
orientation options are alternative measurements of this same observer,
not alternate observer runtimes. Information availability, supported pose
rank and independent heading strength are distinct diagnostics.

`updateLocalizationSourceWindow` forms a causal five-scan feature horizon
using uninterrupted valid wheel/lateral/gyro motion. When motion is
unavailable it discards history transport and emits the current scan.
Reference-built frozen maps and one declared initial reference pose are
permitted design inputs. Later reference states must remain outside online
prediction, motion preparation and corrections. Same-pass map/reference
scores do not establish independent survey accuracy.

## Support files

| File | Current role |
| --- | --- |
| `invariantObserverSupport.m` | Intrinsic prediction, five outputs, correction and design verification |
| `invariantIssSupport.m` | Local nonlinear remainder and conditional ISS analysis |
| `sensorRedundancySupport.m` | Common L/G/M sensor certificate construction and verification |
| `coupledMotionSupport.m` | Coupled physical-motion polynomial/Bernstein construction |
| `solveCommonObserverLmis.m` | Common LMI solver |
| `motionResponseSupport.m` | Bounded motion-gain schedule and nominal response design |
| `observerGainSelectionSupport.m` | Engineering objective selection over verified designs |
| `fullObserverSupport.m` | Synchronization, shared observer design and GNSS output-point handling |
| `lidarInjectionSupport.m` | Overlap measurement, information filtering and matched correction |
| `registrationSupport.m` | Shared mixture projection, map crop/view conditioning, overlap and residual algebra |
| `prepareSemanticOverlapGeometry.m` | Support-axis and mixture geometry preparation |
| `lidarCalibrationSupport.m` | Sensor-only tilt provenance and offline calibration |
| `poseSupport.m` | Mapping/reference coordinate transforms |
| `estimateWheelLongitudinalSpeed.m` | Wheel-derived longitudinal velocity |
| `lateralObserver/` | LPV design, runtime, model helpers and scenario simulation |

`registrationSupport` retains its historical name but is actively used by
perception, mapping, source windows and overlap. Its residual algebra and the
global observer's generic pose/residual interface also support current
synthetic validation. They do not run the retired pose-registration solver.
Removing them would break maintained callers.

## Validation

`validateFullLocalizationObserver` covers the current MnCAV entry;
`validateSyntheticLocalizationCascade` exercises synthetic measurements,
availability and lateral-model mismatch. Tests under `tests/` cover fresh
gains, outputs, coupling, source windows and overlap. In particular,
`activeLocalizationChannelTest` checks that registration requests and matcher
callbacks are rejected before input files or observer data are read.

## Retired implementations

The pose-registration route, direct-gyro seven-state observers, delayed-pose
replay, experimental pose graph and persisted gain certificates were removed
from the active tree. Their pre-cleanup source remains recoverable from Git
revision `067df04`; reproduce historical experiments in a separate checkout.
Historical study results are not alternative live algorithms. Old
registration selection, matcher callbacks and historical timing modes are
rejected rather than routed to a compatibility implementation.
