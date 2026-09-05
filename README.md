# vehicleLocalization

MATLAB research implementation of vehicle localization using road curbs,
road markings, and pole-like features. Perception now has a shared XY-pillar
front end: online localization consumes coarse Gaussian distributions, while
offline mapping validates individual points only inside candidate pillars.
Historical detectors remain available through `executionMode="legacyFull"`
for comparisons with the stored reference.

## Pipeline

```text
organized LiDAR frame (vehicle coordinates)
  -> pillarizePointCloud: XY membership + sparse height histograms, no 3D volume
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
            refinePerceptionCandidates: evaluate candidate members individually
            collectFeatureObservations: fine points -> global coordinates
            buildSlidingWindowMap: temporal-stability Gaussian support map
            temporalMapToProbabilityCloud: one window's D2D representation
```

### Perception (`perception/`)

`perceiveFrame(frame, perceptionConfig())` returns `probabilityCloud`,
`candidates`, and compact source counts. The analysis unit is a 0.3 m XY pillar.
Vertical structure uses sparse 0.5 m height-bin counts; no semantic label is
assigned to a height bin. Reading returns, rejecting invalid measurements,
separating terrain, and accumulating statistics are common preprocessing.
There is no online point-level curb, pole, or marking refinement.

Useful original ideas are retained: terrain-relative curb geometry, road-edge
continuity and adjacency, road reflectivity contrast, and vertical pole support
relative to neighboring pillars. Redundant final curb-energy and independent
pole-column post-gates were removed from the defaults. In particular, a pole
crossing an XY boundary retains its complete candidate footprint.

Set `cfg.executionMode="offline"` to additionally return `featureMasks` and
`refinement`. Each semantic audit contains candidate indices, evaluated indices,
and an acceptance decision for every member. Curbs use the established residual
and boundary filters; markings use the road-derived reflectivity threshold;
poles use sparse height support and a robust vertical-line residual test.
There is no fallback that republishes every candidate when fine validation
rejects all points. The modern pipeline supports curb, roadMarking, and pole.
Facade and traffic-sign detectors and the dense semantic product remain in the
explicit historical path.

The coarse product contains normalized mixture weights and empirical XYZ
means/covariances, aggregated into 0.9 m XY output cells with covariance safeguards.
`components.meanXYZ`, `covarianceXYZ`, and `heightAvailable` retain height and
its xz/yz coupling. Existing `mean` and `covariance` fields remain the exact XY
marginal. `projectSemanticProbabilityCloud(cloud,3)` returns standard XYZ
component arrays; dimension 2 selects the marginal without changing weights.
`semanticProbability` and `occupancyProbability` are compatibility field names
for **uncalibrated evidence and hit support**, not Bayesian semantic or free-space
occupancy posteriors. Known IMU tilt can be supplied through
`cfg.coarseProbabilityCloud.projectionRotation` before moment accumulation.

See [the design and measured limitations](research/pillar_perception_and_d2d.md).
The [runtime optimization study](research/coarse_perception_runtime_optimization.md)
measured median coarse latency of 56.5 ms versus 113.9 ms before optimization
on 19 Mississippi frames, with identical ground labels, semantic candidates,
fine feature points, and probability weights. Empty ground margins are trimmed
internally while public pillar IDs retain the original lattice.

Run `buildPerceptionKernels` once to compile the optional C++ CPU kernels.
`cfg.executionBackend="auto"` uses them when available; `"matlab"` forces the
MATLAB implementation and `"native"` requires a successful build. The source
and build script are versioned; generated platform binaries are ignored.

### Mapping (`mapping/`)

`buildFeatureMap(dataRoot)` runs the whole offline chain and saves the
sliding-window probability-cloud map. The temporal-stability GMM in
`mapping/temporalStabilityGmm/` keeps the design rule of the original code:
temporal information decides *which points are sampled, how the mixture is
seeded, and which components survive*, never the EM updates themselves.
New maps preserve XYZ observations and fit a conditional Gaussian height model
after the XY mixture is finalized. The XY map and original noisy-OR/max query
remain unchanged. Existing saved XY maps have no recoverable height; rebuild
from XYZ observations to obtain it. The conditional height density integrates
to one, so adding it does not reweight landmarks by their vertical extent.

The mapping entry points are grouped by task:

| Task | Functions |
| --- | --- |
| Build an offline map | `buildFeatureMap`, `buildSlidingWindowMap` |
| Read and align observations | `loadPointCloudFrame`, `matchFramePoses`, `readFramePoseTable`, `collectFeatureObservations`, `registerPointsToGlobalFrame` |
| Build and query the temporal GMM | `buildTemporalStabilityGmmMap`, `queryTemporalStabilityGmmMap` |
| Build an NDT grid | `buildSemanticNdtGridMap` |
| Export, project, and validate distributions | `temporalMapToProbabilityCloud`, `projectSemanticProbabilityCloud`, `validateSemanticProbabilityCloud` |
| Register and score distributions | `registerSemanticProbabilityCloud`, `scoreSemanticProbabilityCloudAlignment` |
| Calibrate and prepare registration | `fitLidarPitchCalibration`, `poseRowToPlanarPose`, `prepareSemanticRegistration`, `balanceSemanticDistributions`, `semanticGaussianOverlap` |

Helpers used by one caller live as local functions in that caller's file.
Larger internal stages and shared helpers live in `mapping/private/` or
`mapping/temporalStabilityGmm/private/`; these directories must not be added
to the MATLAB path. `registerSemanticProbabilityCloud` selects the private
geometric registration backend through its existing configuration. Existing
map formats, configuration fields, and numerical algorithms are preserved.

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
* `lowSpeedHold`, retained as a compatibility flag indicating that the raw
  side-slip direction is invalid; it no longer hard-zeroes the master state.

The completed global stage is in `localization/improvedObserver/`. Its state is
`[X,Vx,Ax,Y,Vy,Ay,phi]`; it uses the nonsingular known-input model
`z3Dot=q^2*z2-2*q*z6`, `z6Dot=q^2*z5+2*q*z3`, and `phiDot=r_m`. GPS position and
lidar heading form the base output, lidar position receives the
`T*P^-1*Cl'*W(t)` information-shaped correction, and four invariant outputs
couple velocity, acceleration, heading, and side slip. The robust certificate
enumerates all 65,536 combinations of the 13-coefficient output box, heading
weight endpoints, and exact known-input vertices. Delayed or out-of-order poses
are replayed at physical timestamps. See
`localization/improvedObserver/README.md` for equations, data interfaces, and
the precise certificate boundary.

## Configuration (`config/`)

One config per stage. The geometric baseline is retained where experiments
support it; coarse distribution, offline validation, and registration controls
are explicit:

| config | consumed by |
| --- | --- |
| `perceptionConfig` | `perceiveFrame` (aggregates the four below) |
| `coarseSemanticProbabilityCloudConfig` | `perceiveCoarseProbabilityCloud`, `buildCoarseSemanticProbabilityCloud` |
| `frameVoxelizationConfig` | `pillarizePointCloud`, historical `voxelizePointCloud` |
| `finePerceptionConfig` | `refinePerceptionCandidates` (offline only) |
| `distributionRegistrationConfig` | `registerSemanticProbabilityCloud` |
| `groundSegmentationConfig` | `segmentGround` |
| `groundFeatureConfig` (`.curb`, `.road`, `.roadMarking`) | `extractGroundFeatures` |
| `offGroundFeatureConfig` | `extractOffGroundFeatures` (`facadeDetectionEnabled` is a dataset policy) |
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
scores = queryTemporalStabilityGmmMap(probabilityCloudMap, queryXY, "pole");

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
`runImprovedObserverDesign.m`. `evaluatePillarPerception` and
`evaluatePillarRegistration` write reproducible comparison tables under `output/`.
`showMississippiPerception(260,"","coarseProbabilityCloud")` displays the complete
cloud with candidate pillar members colored; `"offline"` displays accepted fine points.

For D2D, select one local map window explicitly and cache its converted cloud:

```matlab
mapCloud = temporalMapToProbabilityCloud(probabilityCloudMap, batchIndex);
[measurement, diagnostic] = localizeLidarFrame( ...
    frame, mapCloud, predictedPoseXYTheta, acquisitionTimestamp);
% Empty measurement means rejected alignment. Set measurement.arrivalTime
% to its actual delivery time before adding it to the observer lidar stream.
```

The localization state and every accepted measurement are **[X, Y, psi]** in
meters/radians. The caller supplies a local prediction, a map window, and known
IMU tilt. This is local SE(2) alignment. Online perception remains pillar-only;
fine point verification is confined to offline mapping.

`distributionRegistrationConfig` now defaults to `method="geometricD2D"`.
It matches same-class Gaussian components using both covariances. Elongated
curbs and markings constrain their normal directions; poles constrain XY.
Positive mixture masses do not attract the solution toward sampling-density
peaks. Full-pose acceptance requires three observable directions and class
consistency. A rejected result produces no observer event; partial geometry
is available only as a diagnostic. Its normal matrix is not calibrated sensor
information. Explicit `method="densityOverlap"` reproduces the old normalized
L2 objective. The separate `scoreSemanticProbabilityCloudAlignment` function
continues to evaluate that legacy objective, so its score is not comparable
to the new solver's residual compatibility.

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
before using it online. Known mismatched transforms are rejected; legacy
identity maps with missing provenance are labeled `unverifiedLegacy`.

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

`pipelineRegressionTest` explicitly selects `legacyFull` and preserves the
original reference file unchanged. It verifies historical perception and map
outputs. `pillarPerceptionTest` tests the new execution boundary, sparse storage,
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

## Relation to the original repository

| original | here |
| --- | --- |
| `VoxelizePointCloud` | `voxelizePointCloud` |
| `groundSeg` (slopeGrid mode) | `segmentGround` |
| `groundPointProcessing` + `curbExtraction` | `groundFeatures/extractGroundFeatures` and its stage files |
| `offGroundFeatureExtraction` | `offGroundFeatures/extractOffGroundFeatures` and its stage files |
| `buildSemanticVoxelGridProduct`, `coarseGridValidation` | `semanticProduct/buildSemanticVoxelGrid` |
| `finePointValidation`, `perceptionValidationPipeline` | `semanticProduct/refineSemanticPoints`, `perceiveFrame` |
| `buildSemanticTemporalStabilityGMMFeatureMap` | `temporalStabilityGmm/buildTemporalStabilityGmmMap` and stage files |
| `querySemanticTemporalStabilityGMMFeatureMap` | `temporalStabilityGmm/queryTemporalStabilityGmmMap` |
| `buildSemanticNDTGridMap` | `buildSemanticNdtGridMap` |
| glue inside `runMissisipiSemanticTemporalStabilityGMMFeatureMapTest.m` | `mapping/*.m` functions |
`localization/lateralObserver/` has no counterpart in the original repository. It
is new code, so it is verified against its own mathematics rather than against a
recorded baseline: the polytopic representation is checked for exactness and for
nonnegative barycentric coordinates across the speed range, the scheduling rate
against a finite difference, and every synthesized gain against the original
certificate of the proposition (negative definite Lyapunov derivative,
`trace(L' P L) < mu`, stable error matrix) at each grid point. The archived
`legacy/config/hgoLateralObserverConfig.m` of the original repository supplied
the vehicle parameters; its implementation no longer existed there.

`localization/improvedObserver/` is likewise new code. Its tests independently
check the nonsingular model and invariant equations, directional lidar weights,
zero-speed and wrapped-angle behavior, bounded replay rejection, all 65,536
robust-LMI combinations, and a deterministic end-to-end case containing pose
delay, out-of-order delivery, GPS dropout, and lidar degeneracy.

Deliberately left behind: profiling and visualization scripts, the `legacy/`
folder, the unused `seedOnly` and `iterativePca` ground modes, the
representative-point curb selection and diagnostic tables that the tuned
configuration never enabled, forty unreachable pole/facade helper functions, and
the bundled YALMIP/SeDuMi/SDPT3 copies (install them separately for the gain
design). Configuration fields that no code read were dropped as well.

Result structs changed shape where the old ones carried compatibility fallbacks:
`extractOffGroundFeatures` returns `columnMaps`, `trafficSign`, `facade`, and
`pole` structs (the old flat `debug` fields live under these names), and the
ground-branch `curb` config fields lost their `groundPointProcessing` prefix.

## Requirements

MATLAB R2021a or later with the Image Processing Toolbox (connected components,
Hough peaks, distance transform) and the Statistics and Machine Learning Toolbox
(`KDTreeSearcher` in the map builder). ROS Toolbox only for the bag extraction
script; YALMIP and SeDuMi only for the observer gain design.
