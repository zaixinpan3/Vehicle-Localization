# vehicleLocalization

MATLAB research implementation of vehicle localization using road curbs
and pole-like features. Perception now has a shared XY-pillar
front end: online localization consumes coarse Gaussian distributions, while
offline mapping independently reconstructs detailed structural candidates and
validates their individual points.
The runtime contains only current implementations. Regression comparisons read
frozen output data; they never select another executable implementation.

## Pipeline

```text
LiDAR XYZ points, optionally organized (vehicle coordinates)
  -> pillarizePointCloud: XY membership + whole-pillar XYZ moments and bounds
  -> segmentGround: common slope-grid terrain preprocessing
  -> analyzeGroundPillars: curb geometry, road topology, reflectivity statistics
  -> analyzeStructuralPillars: vertical support, compactness, neighbor contrast
  -> buildPerceptionCandidates: semantic XY pillar IDs
       |
       +-> online (default)
       |    buildCoarseSemanticProbabilityCloud: empirical means/covariances
       |    localizeLidarFrame: semantic D2D -> accepted observer pose event
       |
       +-> offline
            refinePerceptionCandidates: independent detailed geometry + point decisions
            collectFeatureObservations: fine points -> global coordinates
            buildSlidingWindowMap: canonical repeated-observation Gaussian field
            temporalMapToProbabilityCloud: normalized field with retained mass
```

### Perception (`perception/`)

`perceiveFrame(frame, perceptionConfig())` returns `probabilityCloud`,
`candidates`, and compact source counts. The analysis unit is a whole XY pillar
on a fixed lattice owned by the execution mode
(`pillarGridConfig(executionMode)`): the online coarse product runs on
100 x 100 pillars of 0.6 m covering [-29.9, 30.1) m, and the offline product
keeps 334 x 334 pillars of 0.3 m covering [-50, 50.2) m. Both share the
`latticeOffset = [0.1, 0.1]` m phase, so cell boundaries stay at -50 + k*0.3 m
and every coarse pillar is the union of four offline pillars. There is no
separate ROI or data fit: `pillarizePointCloud` has one lattice per mode,
returns outside it are ignored, ground segmentation rasters on the same
lattice, and the coarse output grid (1.2 m online, 0.9 m offline) shares its
origin. Build a configuration with `perceptionConfig(dataset, executionMode)`;
every pillar-stage parameter is derived for that lattice (`latticeTunedValue`),
and `perceiveFrame` rejects a mode switched after construction.
Every pillar stores its point count, XYZ mean, full XYZ covariance, bounds,
and available intensity/reflectivity maxima with finite sample counts.
The coarse path has no Z index, height bins, occupancy runs or finer cells.
Reading returns, rejecting invalid measurements, separating terrain and
accumulating whole-pillar statistics are common preprocessing.
There is no online point-level feature refinement.

Useful original ideas are retained: terrain-relative curb geometry, road-edge
continuity and adjacency, road reflectivity contrast, and vertical pole support
relative to neighboring pillars. Redundant final curb-energy and independent
pole-column post-gates were removed from the defaults. In particular, a pole
crossing an XY boundary retains its complete candidate footprint.

Set `cfg.executionMode="offline"` to additionally return `featureMasks`,
`fineCandidates` and `refinement`. `candidates` remains the online product;
`fineCandidates` records the independent offline search support. Each semantic audit contains candidate indices, evaluated indices,
and an acceptance decision for every member. Curbs use metric neighborhoods
of unorganized XYZ, local height-step evidence, and spatial boundary support.
The fine detector retains original returns near narrow boundaries and samples
them at metric spacing. Neither organized rows nor scan order are used. Nearby
curb pillars are searched only offline; unsupported geometry emits no points.
Markings use the road-derived reflectivity threshold;
poles retain detailed support and robust vertical-line residual tests, built
exclusively inside the offline branch. Those tests do not run online.
The default fine pole recovery adds missed candidates only with strong
whole-pillar geometry (at least 12 returns, 3 m height, at most 2 degrees tilt
and 0.10 m transverse standard deviation), relaxed detailed seeding and the
existing per-point validation. Original accepted pole points are preserved.
See [the frame-260 pole audit](research/fine_pole_recovery.md) and
[the thin-curb audit](research/curb_geometry_refinement.md).
There is no fallback that republishes every candidate when fine validation
rejects all points. The invocation's **only semantic selector** is
`cfg.featureNames`. Dataset profiles select these channels:

| Configuration | Requested channels |
| --- | --- |
| `perceptionConfig("Mississippi")` (default) | curb, pole, trafficSign |
| `perceptionConfig("Downtown")` | curb, pole, facade, trafficSign |

Road-marking detection has been removed from both coarse and fine perception.
See [the removal and regression record](research/road_marking_removal.md).

```matlab
cfg = perceptionConfig("Downtown");
cfg.featureNames = ["facade", "pole"]; % Optional per-call subset, in output order.
result = perceiveFrame(frame, cfg);
% cfg.featureNames = strings(1,0); explicitly requests no semantic channels.
```

The list controls coarse detectors, candidate/cloud channels, and offline
point refinement. Unrequested fine-mask fields remain false;
`refinement` contains only requested channels. Shared ground segmentation and
terrain preparation remain preprocessing. Marking-only perception retains
curb/road-boundary evidence needed to delimit its road region, but publishes
no curb candidates or curb point decisions. Sign-only calls skip ground-feature,
pole and facade detection. Unknown/duplicate names are rejected.

Modern calls reject the former nested `coarseProbabilityCloud.semanticNames`
and `offGroundFeatures.facadeDetectionEnabled` selectors; use `featureNames`
instead. The low-level standalone Gaussian builder still receives its internal
`semanticNames` list from the caller.

Offline facades require robust vertical planes and per-point distance tests.
Sign pillars use maximum intensity evidence; fine sign points must individually
exceed `trafficSignIntensityThreshold` (1600 in the recorded sensor's raw units).
Coarse sign XYZ moments include all nonground members of each candidate pillar.
These are reflective sign candidates, not sign-type recognition.

`featureMapBuildConfig().featureNames` is passed directly to perception and map
collection, with no additional facade switch. To select all Downtown channels,
copy `perceptionConfig("Downtown").featureNames` into the map configuration and
supply that route's MAT file and matched pose CSV. The viewer
`showMississippiPerception(frameIndex,matPath,mode,cfg)` colors the selected
channels; without `cfg`, a Downtown filename selects the Downtown profile.
Facades contribute normal-distance constraints to planar registration; compact
sign Gaussians contribute XY landmark constraints. The estimated pose remains
`[X,Y,psi]`. See [the restoration measurements](research/structural_perception_restoration.md)
and [the invocation-selection validation](research/perception_feature_selection.md).

The coarse product contains normalized mixture weights and empirical XYZ
means/covariances, aggregated into whole-pillar XY output cells (1.2 m on the
online 0.6 m lattice, 0.9 m on the offline 0.3 m lattice) with covariance safeguards.
`components.meanXYZ`, `covarianceXYZ`, and `heightAvailable` retain height and
its xz/yz coupling. Existing `mean` and `covariance` fields remain the exact XY
marginal. `registrationSupport.projectSemanticProbabilityCloud(cloud,3)` returns
standard XYZ component arrays; dimension 2 selects the marginal without changing
weights.
`semanticProbability` and `occupancyProbability` name
for **uncalibrated evidence and hit support**, not Bayesian semantic or free-space
occupancy posteriors. Known IMU tilt can be supplied through
`cfg.coarseProbabilityCloud.projectionRotation` before moment accumulation.

See [the whole-pillar boundary and baseline audit](research/whole_pillar_perception.md)
for the current representation, and [the prior design](research/pillar_perception_and_d2d.md)
for historical measurements.
The [runtime optimization study](research/coarse_perception_runtime_optimization.md)
measured median coarse latency of 56.5 ms versus 113.9 ms before optimization
on 19 Mississippi frames, with identical ground labels, semantic candidates,
fine feature points, and probability weights. Empty ground margins are trimmed
internally while public pillar IDs retain the original lattice.

The subsequent [complete-pipeline optimization](research/localization_pipeline_optimization.md)
measured a 70.3 ms median and 97.4 ms maximum from coarse perception through
D2D pose output over 420 warmed default-map calls, with unchanged probability
clouds. This is a measured workload result; an additional smaller-map run
retained a 108.1 ms outlier.

Run `buildPerceptionKernels` once to compile the optional C++ CPU kernels.
Rebuild after native interface changes; automatic mode falls back to MATLAB
when an older binary is detected.
`cfg.executionBackend="auto"` uses them when available; `"matlab"` forces the
MATLAB implementation and `"native"` requires a successful build. The source
and build script are versioned; generated platform binaries are ignored.

### Mapping (`mapping/`)

`buildFeatureMap(dataRoot)` runs the offline chain and saves a schema-2
repeated-observation Gaussian field. The builder uses deterministic per-block
spatial representatives, structured variational inference with an explicit
background, and unique spatial ownership. It separates within-block geometry,
block displacement, repeatability, and reference area. Frame windows schedule
observations; their union contributes evidence once to `canonicalMap`.

`runPerceptionVideoMap(outputFolder, figureHandle)` records all frames using
a `showMississippiPerception` figure's fixed camera and display settings while
collecting the same fine-perception outputs for mapping. It writes an AVI
master, registered observations, per-frame counts and timestamps, and the
canonical map to a new output folder. The recording uses one image per input
frame at the mean LiDAR cadence; processing speed does not set playback speed.
Keep the recording window size fixed. A configuration from
`featureMapBuildConfig` may be supplied as the third argument.
Each finite original point is drawn once with a semantic RGB color and the same
marker size as the gray background. Datatips print the original index and
current class in the MATLAB Command Window. The upper-left counter displays
the current source frame and total frame count.
The scatter keeps pcshow's native tag so Rotate 3D can find a point as its
rotation center. In Rotate 3D mode, right-click and select **Rotate Around a
Point**, then drag from the point of interest. After `openfig`, call
`restorePerceptionFigure(fig)` to rebuild the live point-cloud interactions
while preserving the saved view and colors. Double-click reset is unchanged.
See [the point-rotation validation](research/perception_viewer_interaction.md).
See [the direct-color rendering checks](research/direct_point_coloring.md).
See [the full-sequence video/map record](research/mississippi_perception_video_map.md).

Queries evaluate `Lambda/(Lambda+kappa)` and return coverage/status metadata.
Gaussian-cloud exports normalize the same field and retain total and per-class
mass, so queries can be reconstructed exactly before bounded index truncation.
Conditional height preserves the XY marginal and mass; missing or inadequate
height remains unavailable. Defaults use one frame per observation block;
explicit IDs and fixed frame groups are supported. These probabilities describe
model-conditional repeatability, not occupancy or permanent existence.

See [the formulation and migration contract](research/repeated_observation_map_design.md)
for priors, the ELBO, publication, mass allocation, held-out model selection,
query error bounds, and current validation limits. Query and export accept only
schema-version-2 maps. Rebuild other artifacts from observations.

The directory contains only five map algorithms and one shared support file:

| File | Purpose |
| --- | --- |
| `buildSemanticNdtGridMap.m` | Aggregate semantic observations into NDT cells |
| `buildTemporalStabilityGmmMap.m` | Fit repeated-observation geometry and conditional height |
| `buildSlidingWindowMap.m` | Ingest scheduled frames once and build canonical owned tiles |
| `queryTemporalStabilityGmmMap.m` | Evaluate the Gaussian field against clutter with coverage and error bounds |
| `temporalMapToProbabilityCloud.m` | Export the canonical normalized field with retained mass |
| `mappingSupport.m` | Share logging, statistics, and numerical/schema validation |

All six files are directly under `mapping/`, with no subdirectories. Internal
algorithm stages are local functions in the corresponding entry-point file.
Common utilities have one implementation as static `mappingSupport` methods;
for example, cloud validation is `mappingSupport.validateSemanticProbabilityCloud(cloud)`.
This replaces the former standalone validation helper.

Dataset loading, frame/pose matching, global coordinate transforms, feature
collection, and the `buildFeatureMap` offline workflow live in `scripts/`.
Cloud registration, registration preparation, projection, scoring, and pitch
calibration live in `localization/`, alongside `poseRowToPlanarPose`. Their
existing entry-point names are retained. Run `setupVehicleLocalization` to add
all three modules. Existing map formats, configuration fields, numerical
algorithms, and builder/query error identifiers are preserved.

### Localization (`localization/`)

Two estimator stages live here and form the current one-way cascade.

The **lateral-velocity observer** in `localization/lateralObserver/` has a
division-free master state `[v_y, b_ay]`. It propagates
`vyDot = ay_m - b_ay - r_m*vx` at every speed. A hidden LPV observer retains the
body-frame bicycle state `[v_y, r]`, with steering as input and `[a_y, r]` from
the IMU as output, but is evaluated only inside its certified positive-speed
interval. Both LPV matrices are affine in `rho = [Vx; 1/Vx]` and are represented
exactly on a triangle around the scheduling arc. `designLateralObserverGains`
synthesizes one gain per triangle vertex from a quadratic Lyapunov function,
`scheduleLateralObserverGain` blends the three with the barycentric coordinates
of the current speed, and the synthesis re-checks that blend against the
original certificate at every design speed.

Stationary zero-velocity information, a soft crawl-speed kinematic constraint,
and the hidden LPV lateral-velocity estimate enter the master through separate
persistent correction-injection states. Discrete mode labels never select an
output or reset a state. Ordinary participation changes use quintic `C2`
weights; abrupt invalidation sets only the corresponding correction target to
zero, so the stored injection fades without evaluating the invalid dynamic
model. This makes the exported master state and its derivative bumpless for
continuous physical inputs.

This block is block 1 of the improved ego-state observer. It runs open of the
global observer, taking only wheel speed, steering angle, and IMU signals. It
hands over the side-slip angle and its rate as exogenous known signals through
the track-angle rate `q = r_m + betaDot`, so the two stages cascade without a
loop. `runLateralVelocityObserver` supplies exactly this interface:

* `lateralVelocity`, `sideSlipAngle` and `sideSlipAngleRate` **at the declared
  output point**. The observer integrates the vehicle IMU, so its master state
  is the lateral velocity at the IMU location. `cfg.outputPoint.forwardOffsetM`
  transports it by rigid-body kinematics, `vyOutput = vyObserver - d*r`, to the
  point that the pose state, map and GNSS correction use. The MnCAV profile
  loads `config/mncavMotionOutputPoint.json` (2.36 m, fitted on seconds 1--40
  of the separate 12-11-24 drive by `calibrateMncavMotionOutputPoint`); the
  reference profile exports at the observer point. The persistent second-order
  side-slip interface tracks the direction of that output-point velocity.
  `observerPointLateralVelocity` retains the untransported state;
* `longitudinalSpeedRate`, rebuilt as `ax + vy*r` because the IMU reports a
  specific force and not the speed derivative;
* `mode`, correction-channel histories, and `dynamicModelEvaluated`, which make
  the hybrid information flow auditable;
* `diagnostics.sideSlipCommandValid`, which indicates whether the raw side-slip
  direction is valid.

The global stage has seven continuous states
`[X,Vx,Ax,Y,Vy,Ay,psi]` and planar pose output `[X,Y,psi]`.
The [continuous-time ISS derivation](improved_observer_derivation.md)
analyzes two independent measurement cases:

* Continuous GNSS position: a triangular MO-HGO has seven-state local ISS
  during sustained motion, with an explicit invariant heading-error region.
  Position-only GNSS cannot observe heading at standstill.
* Continuous LiDAR pose with a known fixed delay: uniformly positive pose
  information supports a delayed-residual MO-HGO, subject to a verified
  delay-dependent LMI using constant Lyapunov--Krasovskii matrices.

The continuous theory retains LiDAR delay and the full matrix information,
including cross terms.
The [reproducible certificate checks](research/continuous_observer_iss_20260913/validation.md)
record a constructive GNSS block certificate and a conservative 150 ms LiDAR
example with all four auxiliary channels. These are new theoretical designs.

The analysis runner `runImprovedVehicleObserver` integrates these continuous
ODE/DDE equations. Select `improvedObserverConfig("gnss")` or
`improvedObserverConfig("lidar")`; each mode uses independently verified,
constant certificate matrices. Delayed LiDAR requires an explicit estimate
history, and the innovation compares measurements with that past estimate.
See [the runtime contract](localization/README.md) and
[implementation validation](research/continuous_observer_runtime_20260913/validation.md).
Recorded data require an explicitly declared continuous reconstruction; they do
not establish physical sensor continuity.

The complete runtime is `runFullLocalizationObserver`: independent GNSS and
LiDAR streams simultaneously correct one seven-state motion-aided observer.
Missing/invalid/expired channels are withdrawn independently. Separate ISS
analyses do not imply mutually exclusive sensor operation. Its current
zero-delay, sampled implementation has a
[separate conditional common certificate and outage analysis](research/full_observer_20260916/design.md).

## Configuration (`config/`)

One config per stage. The geometric baseline is retained where experiments
support it; coarse distribution, offline validation, and registration controls
are explicit:

| config | consumed by |
| --- | --- |
| `perceptionConfig` | `perceiveFrame` (aggregates the four below) |
| `coarseSemanticProbabilityCloudConfig` | `perceiveCoarseProbabilityCloud`, `buildCoarseSemanticProbabilityCloud` |
| `pillarGridConfig(executionMode)` | fixed pillar lattice per execution mode (`gridDims`, `voxelSize`, `latticeOffset`) shared by pillarization, ground segmentation, the coarse cloud and the NDT map extent |
| `latticeTunedValue` | selects the 0.3 m or 0.6 m value of a pillar-stage parameter inside the stage configs |
| `structuralPillarConfig` | whole-pillar pole/facade/sign detection |
| `finePerceptionConfig` | `refinePerceptionCandidates` (offline only) |
| `distributionRegistrationConfig` | `registerSemanticProbabilityCloud` (`.pyramid` canonical coarse-to-fine levels) |
| `groundSegmentationConfig` | `segmentGround` |
| `groundFeatureConfig` (`.curb`, `.road`) | `extractGroundFeatures` |
| `offGroundFeatureConfig` | `extractOffGroundFeatures` (historical facade switch; modern calls use `featureNames`) |
| `temporalStabilityMapConfig` | `buildTemporalStabilityGmmMap` |
| `featureMapBuildConfig` | `buildFeatureMap` |
| `semanticNdtGridMapConfig` | `buildSemanticNdtGridMap` |
| `lateralObserverConfig` | `designLateralObserverGains`, `runLateralVelocityObserver` (`.outputPoint` from `mncavMotionOutputPoint.json` for MnCAV) |
| `wheelSpeedObserverConfig` | `estimateWheelLongitudinalSpeed`, `prepareWheelMotionInputs` |
| `improvedObserverConfig` | `designImprovedObserverGains`, `runImprovedVehicleObserver` |
| `fullObserverConfig` | `designFullObserverGains`, `runFullLocalizationObserver` |

## Quick start

For the complete MnCAV localization experiment, first export BESTPOS with
`scripts/prepareMncavBestpos.py`, calibrate the relative output point with
`scripts/calibrateMncavBestposOutputPoint.py` on the separate 12-11-24 drive,
then run `runMncavCoarseLocalizationExperiment` after
setup. BESTPOS and LiDAR both supply approximately 10 Hz measurements. The
current observer updates once per synchronized LiDAR frame, without inter-frame
measurement-pose transport. The recorded BESTPOS types are INS assisted, not
GNSS-only. Frame alignment interpolates real samples and reports its wait.
Longitudinal velocity is derived exclusively from four wheel rates,
with steering/IMU compensation. No vehicle-speed or Twist alternative is
available. Export `wheel_speed_report.csv`, `imu.csv` and `steering.csv` using
`scripts/extractVehicleReplaySensors.py`; the current full experiment reads
`output/mncav_wheel_only_20260916/sensors` and writes
`output/mncav_coarse_localization_20260924b`. All 1170 scans use fresh whole-pillar
coarse perception and recursive D2D on the canonical map pyramid; online
localization never runs point-level fine refinement. The offline map remains
fixed. Missing or expired wheel input is an explicit
preparation error. Use `validateFullLocalizationObserver` for the associated
checks. The [current production record](research/canonical_pyramid_20260924/README.md)
gives fused position RMSE 5.96 cm over 1169 outputs (5.42 cm after the
initialization transient, heading 0.39 deg), GNSS-only 6.33 cm and LiDAR-only
14.83 cm (14.24 cm after the transient), with closed-loop GNSS-aided hypothesis
selection and the lateral velocity transported to the INSPVA output point
([record](research/output_point_transport_20260924/README.md)). The earlier
[coarse localization report](research/mncav_coarse_localization_20260918/README.md)
(10.3445 cm fused, 8.8252 cm GNSS-only) predates GNSS aiding and the output-point
transport. The [aligned BESTPOS report](research/mncav_bestpos_alignment_20260917/README.md)
documents the retained independent-drive point correction and gains. Its
historical 7.8474 cm result used fine features and per-frame reference seeds,
so this is not an isolated comparison of perception modes. The same-drive map
and shared INSPVA reference do not establish independent absolute accuracy.

The [updated localization video](research/localization_video_20260917/README.md)
displays the September 17 result on aerial imagery with perception and the paired
LiDAR comparison. Run `exportLocalizationVideoData`, then the cloud preparation
and rendering commands in that record. The 1080p/30 fps video repeats actual
10 Hz outputs and is saved as
`output/localization_video_20260917/mncav_localization.mp4`.

The historical LiDAR-only offline experiment is `runMncavMotionAidedExperiment`.
The [motion-aided seven-state observer](research/mncav_motion_aided_20260914/validation.md)
retains the turning model and uses direct velocity/acceleration correction.
Its recorded comparison includes the actual continuous LiDAR input, every
native matching frame, both diagnostic references and a reserved interval.
Use `designMncavMotionAidedGains` to reproduce the training-only gain search.

```matlab
setupVehicleLocalization();                       % add modules to the path
buildPerceptionKernels();                         % optional; requires a C++ compiler
frame = loadPointCloudFrame("data/raw/MissisipiPointClouds.mat", 260);
coarse = perceiveFrame(frame, perceptionConfig());               % 0.6 m pillars, no point masks
fine = perceiveFrame(frame, perceptionConfig("Mississippi", "offline")); % 0.3 m pillars
nnz(fine.featureMasks.curb)                       % accepted offline curb points

localCloud = perceiveCoarseProbabilityCloud(frame, perceptionConfig());
selfScore = scoreSemanticProbabilityCloudAlignment( ...
    localCloud, localCloud, [0, 0, 0]);           % semantic D2D-NDT score

[probabilityCloudMap, featureData] = buildFeatureMap("data");   % offline map
[scores, queryInfo] = queryTemporalStabilityGmmMap(probabilityCloudMap, queryXY, "pole");
% queryInfo.valid identifies coverage; invalid scores are NaN.

design = designLateralObserverGains(lateralObserverConfig("mncav")); % nominal MnCAV LPV H2 synthesis
estimate = runLateralVelocityObserver(measurements, design);    % v_y, r, side slip

lateral = load("tests/reference/lateralObserverDesign.mat");
improved = improvedObserverReferenceDesign();
result = simulateImprovedObserverScenario( ...
    improved, lateral.design, improvedObserverConfig());        % complete cascade
```

`scripts/` holds the runnable entry points: `extractPointCloudsFromBag.m` and
`extractGnssFromBag.py` (ROS bag → MAT frames and GNSS/INS CSV tables),
`buildMississippiFeatureMap.m`, `runLateralObserverDesign.m`, and
`runImprovedObserverDesign.m`. `benchmarkCoarseProbabilityCloud` measures current
coarse/fine calls; `evaluateWholePillarPerception` compares frozen output data.
`evaluateRepeatedObservationMap` records actual construction, convergence and
query/export consistency on Mississippi frames 260--289.
`showMississippiPerception(260,"","coarseProbabilityCloud")` displays the complete
cloud with candidate pillar members colored; `"offline"` displays accepted fine points.

For D2D, cache the canonical field as a Gaussian cloud:

```matlab
mapCloud = temporalMapToProbabilityCloud(probabilityCloudMap);
[measurement, diagnostic] = localizeLidarFrame( ...
    frame, mapCloud, predictedPoseXYTheta, acquisitionTimestamp);
% Empty measurement means rejected alignment. Set measurement.arrivalTime
% to its actual delivery time before adding it to the observer lidar stream.
```

The localization state and every accepted measurement are **[X, Y, psi]** in
meters/radians. The caller supplies a local prediction, a map cloud, and known
IMU tilt. This is local SE(2) alignment. Online perception remains pillar-only;
fine point verification is confined to offline mapping.

`distributionRegistrationConfig` now defaults to `method="geometricD2D"`.
It matches same-class Gaussian components using both covariances. Elongated
curbs and markings constrain their normal directions; poles constrain XY.
Positive mixture masses do not attract the solution toward sampling-density
peaks. Full-pose acceptance requires three observable directions and class
consistency. A rejected result produces no observer event; partial geometry
is available only as a diagnostic. Its normal matrix is not calibrated sensor
information. The separate `scoreSemanticProbabilityCloudAlignment` function
evaluates a Gaussian-overlap diagnostic; it does not select another pose solver.

Registration is coarse to fine on a **canonical map pyramid**
(`cfg.pyramid`). `canonicalizeSemanticCloud` merges same-class point
components closer than `mapMergeRadius` (1.5 m) in the local map and
`sourceMergeRadius` (0.5 m) in the source window into one moment-matched
Gaussian each, which removes the sub-metre association aliases of split or
tile-duplicated landmarks; the same solver runs on those canonical clouds
first and its pose seeds the solve on the original clouds. Position aid and
additional seeds act on the fine level. Without aid or relative-height
association, a refinement that moves more than `trustRadius` (0.15 m) from
the coarse pose is a new search rather than a refinement, and the coarse pose
is kept (`reason="coarseRetainedByTrustRadius"`). Every result carries a
`pyramid` diagnostic. See [the mode-ambiguity study](research/alias_hypotheses_20260924/README.md)
and [the production record](research/canonical_pyramid_20260924/README.md).

Full XYZ means/covariances, including xz and yz, remain in every supported
probability-cloud component. The default `heightMode="xy"` uses the XY
marginal. Opt-in `"xyz"` makes geometric D2D check conditional height
compatibility; it never estimates Z, roll, or pitch. Supply `heightTranslation`
as the moving origin's map Z (the third output of `poseRowToPlanarPose`).
`"auto"` falls back to XY when height or its reference is absent. Small
compatible height residuals do not exert a planar pose force.

Optional `perceptionConfig().frameCalibration` and
`featureMapBuildConfig().frameCalibration` specify the same rigid increment
from stored points to the recorded body frame. Their default is identity.
`fitLidarPitchCalibration` produces an **offline pitch-only candidate** from
static feature observations; it does not add a localization state or claim a
complete sensor extrinsic calibration. Rebuild a map with the chosen transform
before using it online. Missing or mismatched calibration provenance is rejected.
Fixed map components must include repeatability; it is never synthesized from
mixture mass or substituted with unit weights.

See [algorithm, equations, configuration, and recorded validation](research/geometric_d2d_registration.md).

### Data layout expected under `dataRoot`

```
raw/MissisipiPointClouds.mat                      struct array pointClouds (H x W fields)
raw/downTownPointClouds.mat
raw/Missisipi/gnss/<bag>_front_lidar_points.csv   from extractGnssFromBag.py
raw/Missisipi/gnss/<bag>_odom.csv
raw/Missisipi/gnss/<bag>_inspva.csv
raw/Missisipi/gnss/<bag>_front_lidar_inspva_pose_1_1170.csv  from prepareInspvaMappingPoses.py
```

New Mississippi maps use INSPVA-only poses interpolated at LiDAR timestamps.
`buildMississippiFeatureMap` prepares the pose table with cached `uv`, NumPy
and pyproj dependencies when absent. Explicit `pose_*` XYZ/quaternion fields
carry EPSG:32615 positions, ellipsoidal height and grid-north orientation;
they take precedence over any historical ODOM fields. Original ODOM pose
tables remain readable to reproduce stored experiments.

Saved perception already contains globally transformed points. Use
`reprojectSavedFeatureObservations` to undo each old SE(3) transform before
applying the new pose. `rebuildInspvaSavedFeatureMap` performs that conversion
and rebuilds the field without rerunning feature extraction. A pose-table
replacement alone does not update a cached map. The existing LiDAR calibration
is retained; INSPVA remains a quality-labeled navigation reference, not an
independently validated ground-truth trajectory.

## Verification

`currentPerceptionReferenceTest` verifies all point masks on 30 frozen frames
from revision `6bf426c`, stored as point-index JSON without recorded clouds.
`pipelineRegressionTest` verifies current fine outputs and map field contracts
on the recorded scenarios. Original reference evidence remains unchanged.
`temporalStabilityGmmMapTest` checks hierarchical inference, mass conservation,
query/export consistency, coverage, height and canonical ownership.
`pillarPerceptionTest` tests the new execution boundary, sparse storage,
known-tilt moments, rejection behavior, and recorded point fidelity.
`distributionRegistrationTest` checks analytic derivatives, anisotropic covariance
rotation, semantic mass invariance, known-pose recovery, map support units, empty
inputs, and degeneracy. Existing observer and temporal-map tests remain in the suite.
`coarsePerceptionPerformanceTest` checks native/MATLAB equivalence, boundary
cases, conservative ground propagation, compact-raster indexing, and omitted
inverse lookups. Build the native kernels to execute its native-specific cases.

Recorded fidelity uses the old detector as a behavioral baseline, not labeled
truth. Current metrics and experiment identifiers are in the
[redesign report](research/pillar_perception_and_d2d.md) and its CSV artifacts.
Dataset-dependent tests require `VEHICLE_LOCALIZATION_DATA_ROOT`. Observer tests
use the stored gains; optional synthesis tests require their solver dependencies.

```matlab
setenv("VEHICLE_LOCALIZATION_DATA_ROOT", "/path/to/data");
runtests("tests");
```

## Current implementation policy

Only current algorithms and their validation tools belong in the source tree.
Obsolete execution modes, format readers, compatibility wrappers and experiment
scripts that require deleted algorithms are removed. Git history and frozen
reference data preserve comparison evidence without providing runtime fallbacks.

All XY geometry comes from `pillarGridConfig`; a `roiLimits` field is
rejected. Coarse spacing must contain exactly two XY values. Only the offline
fine stage constructs a sparse 3D index, `voxelizePillars`, which splits the
off-ground pillars into `finePerceptionConfig().poleSupportHeightResolution`
layers whose reference height is the minimum of all retained returns, including
ground. Preserve this histogram phase when using the calibrated structural gates.
It has no separate XY geometry, and no dense voxel-statistic or
inverse-lookup alternative remains. Detailed candidate settings come from
`fineStructuralConfig`.

Lateral-observer runtime configurations must include the current `hybrid` fields.
Use `lateralObserverConfig` and `estimate.diagnostics.sideSlipCommandValid` for
the current side-slip interface. Tests reuse stored gain matrices with explicit
current runtime settings; stored references are not rewritten.

## Requirements

MATLAB R2021a or later with the Image Processing Toolbox (connected components,
Hough peaks, distance transform) and the Statistics and Machine Learning Toolbox
(`KDTreeSearcher` in the map builder). ROS Toolbox only for the bag extraction
script; YALMIP and SeDuMi only for the observer gain design.
