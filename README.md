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
`candidates`, and compact source counts. The analysis unit is a 0.3 m XY pillar.
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
means/covariances, aggregated into 0.9 m XY output cells with covariance safeguards.
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
exactly on a triangle around the scheduling arc. Its gain is synthesized from a
parameter-dependent Lyapunov function by `designLateralObserverGains`, which
also re-checks the recovered gains against the original certificate.

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

* `sideSlipAngle` and `sideSlipAngleRate`, supplied by one persistent
  second-order interface state whose command blends smoothly between zero and
  the valid `atan2(vy,vx)` value;
* `longitudinalSpeedRate`, rebuilt as `ax + vy*r` because the IMU reports a
  specific force and not the speed derivative;
* `mode`, correction-channel histories, and `dynamicModelEvaluated`, which make
  the hybrid information flow auditable;
* `diagnostics.sideSlipCommandValid`, which indicates whether the raw side-slip
  direction is valid.

The global stage is in `localization/`. Its internal state is
`[X,Vx,Ax,Y,Vy,Ay,psi]`, with causal output `[X,Y,psi]`. LiDAR poses
use a full information-dependent anisotropic gain, retaining XY/yaw cross terms
and partial-rank directions. GPS and LiDAR residuals are combined in information
form within the same pose pulse. Both the invariant gain and the acceleration rows of the base gain are
reduced by a factor of ten; only
the nonlinear invariant prediction is extended outside the physical state
box. The state and its linear prediction are not clipped.

The runtime assumes a fixed LiDAR delay, default 150 ms. It integrates forward
once and transports each delayed residual and gain through the nominal motion
flow. A bounded buffer stores input-derived maps; past estimates are never
recomputed. All state outputs, including `z` and `onlineZ`, are causal.
Delivery and pulse boundaries are handled by event-split RK4.

The stored aperiodic certificate verifies the preceding current-pose pulse
model, not this transported feedback. The runtime preserves the reference
gain checks and explicitly reports that the new stability certificate is
unverified. See [localization/README.md](localization/README.md) for the
measurement contract and [the fixed-delay study](research/fixed_delay_transport_observer.md)
for the equations, tests, and recorded performance limitations.

## Configuration (`config/`)

One config per stage. The geometric baseline is retained where experiments
support it; coarse distribution, offline validation, and registration controls
are explicit:

| config | consumed by |
| --- | --- |
| `perceptionConfig` | `perceiveFrame` (aggregates the four below) |
| `coarseSemanticProbabilityCloudConfig` | `perceiveCoarseProbabilityCloud`, `buildCoarseSemanticProbabilityCloud` |
| `frameVoxelizationConfig` | ROI defaults, offline and historical voxel geometry; online uses XY spacing only |
| `structuralPillarConfig` | whole-pillar pole/facade/sign detection |
| `finePerceptionConfig` | `refinePerceptionCandidates` (offline only) |
| `distributionRegistrationConfig` | `registerSemanticProbabilityCloud` |
| `groundSegmentationConfig` | `segmentGround` |
| `groundFeatureConfig` (`.curb`, `.road`) | `extractGroundFeatures` |
| `offGroundFeatureConfig` | `extractOffGroundFeatures` (historical facade switch; modern calls use `featureNames`) |
| `temporalStabilityMapConfig` | `buildTemporalStabilityGmmMap` |
| `featureMapBuildConfig` | `buildFeatureMap` |
| `semanticNdtGridMapConfig` | `buildSemanticNdtGridMap` |
| `lateralObserverConfig` | `designLateralObserverGains`, `runLateralVelocityObserver` |
| `improvedObserverConfig` | `designImprovedObserverGains`, `runImprovedVehicleObserver` |

## Quick start

```matlab
setupVehicleLocalization();                       % add modules to the path
buildPerceptionKernels();                         % optional; requires a C++ compiler
frame = loadPointCloudFrame("data/raw/MissisipiPointClouds.mat", 260);
cfg = perceptionConfig();
coarse = perceiveFrame(frame, cfg);               % no point feature masks
cfg.executionMode = "offline";
fine = perceiveFrame(frame, cfg);
nnz(fine.featureMasks.curb)                       % accepted offline curb points

localCloud = perceiveCoarseProbabilityCloud(frame, perceptionConfig());
selfScore = scoreSemanticProbabilityCloudAlignment( ...
    localCloud, localCloud, [0, 0, 0]);           % semantic D2D-NDT score

[probabilityCloudMap, featureData] = buildFeatureMap("data");   % offline map
[scores, queryInfo] = queryTemporalStabilityGmmMap(probabilityCloudMap, queryXY, "pole");
% queryInfo.valid identifies coverage; invalid scores are NaN.

design = designLateralObserverGains(lateralObserverConfig());   % LPV H2 synthesis
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
raw/Missisipi/gnss/<bag>_front_lidar_pose_match_1_1170.csv   from matchFramePoses
```

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

Coarse geometry comes from `pillarGridConfig`, detailed offline indexing from
`fineVoxelizationConfig`, and detailed candidate settings from
`fineStructuralConfig`. Coarse spacing must contain exactly two XY values.
Only the fine stage constructs a sparse 3D index; no dense voxel-statistic or
inverse-lookup alternative remains.

Lateral-observer runtime configurations must include the current `hybrid` fields.
Use `lateralObserverConfig` and `estimate.diagnostics.sideSlipCommandValid` for
the current side-slip interface. Tests reuse stored gain matrices with explicit
current runtime settings; stored references are not rewritten.

## Requirements

MATLAB R2021a or later with the Image Processing Toolbox (connected components,
Hough peaks, distance transform) and the Statistics and Machine Learning Toolbox
(`KDTreeSearcher` in the map builder). ROS Toolbox only for the bag extraction
script; YALMIP and SeDuMi only for the observer gain design.
