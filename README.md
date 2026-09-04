# vehicleLocalization

MATLAB implementation of *Robust Vehicle Localization Fusing Road Curb, Pole-like
Feature, and Building Facade*, refactored from `RobustVehicleLocalization` so that
the code reads like the research: three modules (perception, mapping,
localization), each a short sequence of named stages, each stage one file whose
local functions are its sub-steps. Every algorithm is the one from the original
codebase, verified output-for-output against it (see *Verification*).

## The pipeline as the paper tells it

```
organized LiDAR frame
   │
   ├─ perception/perceiveFrame ─────────────────────────────────────────── §III.A
   │    voxelizePointCloud            canonical 0.3 m fine voxel grid
   │    groundSegmentation/segmentGround           P → P_g, P_ng   (slope-grid propagation)
   │    groundFeatures/extractGroundFeatures       P_g  → curbs, road surface, road markings
   │    offGroundFeatures/extractOffGroundFeatures P_ng → poles, facades, traffic signs
   │    buildFeaturePointMasks         one full-frame mask per feature class
   │    semanticProduct/               optional fused 3D semantic voxel grid and refined points
   │
   ├─ mapping/buildFeatureMap ──────────────────────────────────────────── §III.B
   │    readFramePoseTable, matchFramePoses      high-precision GNSS/INS pose per frame
   │    collectFeatureObservations                perceive frames, register points globally
   │    assembleMapInput                          per-class BEV points + frame ids
   │    buildSlidingWindowMap                     one map per overlapping frame window
   │       temporalStabilityGmm/buildTemporalStabilityGmmMap
   │         buildSpatialSupportPatches           mutual-kNN patches (candidates, not constraints)
   │         estimateTemporalReliability          leave-one-bin-out cross-frame support
   │         resampleByTemporalReliability        Bernoulli resampling of the EM training set
   │         fitGaussianMixture                   ordinary full-covariance GMM EM
   │         computePosthocTemporalSupport        support amplitude, pruning, refit
   │       temporalStabilityGmm/queryTemporalStabilityGmmMap   online support lookup
   │    buildSemanticNdtGridMap                   per-frame semantic NDT cells (local frame)
   │
   └─ localization/ ────────────────────────────────────────────────────── §III.C
        runReplayHighGainObserver     100 Hz high-gain observer with timestamped pose replay
        designReplayObserverGains     offline LMI synthesis of the pose jump gain (YALMIP + SeDuMi)
        simulateReplayObserverScenario synthetic drive with delayed, out-of-order poses
        lateralObserver/              LPV lateral-velocity observer (v_y, r)
          lateralBicycleModel           2-DOF model, affine in rho = [Vx; 1/Vx]
          buildSchedulingPolytope       triangle covering the scheduling arc
          schedulingCoordinates         barycentric coordinates alpha and their rate
          designLateralObserverGains    gridded H2 LMI synthesis of L(rho)
          scheduleLateralObserverGain   gain lookup along the trajectory
          runLateralVelocityObserver    online estimation of v_y, r, and side slip
          simulateLateralObserverScenario  synthetic drive with noisy IMU
        improvedObserver/             complete cascaded seven-state observer
          runImprovedVehicleObserver    delayed GPS/lidar fusion and replay
          designImprovedObserverGains   robust LMI synthesis over 65,536 vertices
          verifyImprovedObserverDesign  exhaustive numerical certificate check
          simulateImprovedObserverScenario  delay, dropout, and degeneracy scenario
```

### Perception (`perception/`)

* `voxelizePointCloud` bins the frame into the canonical grid shared by every stage
  (`config/frameVoxelizationConfig`).
* `groundSegmentation/segmentGround` is the slope-grid ground segmentation: robust
  per-cell low height, sensor-height seed near the vehicle, slope-limited outward
  propagation, smooth-component promotion and bridging, hole filling, residual
  labeling.
* `groundFeatures/` is the ground branch, in the order the orchestrator
  `extractGroundFeatures` calls it: `buildCurbEnergyMaps` (height step, residual
  slope, curvature, roughness, relative height, relief, line-shape gating),
  `extractRoadSurface` (seeded growth with curb cells as barriers),
  `refineCurbCellsByRoadAdjacency` (continuation, gap completion, shadow and
  duplicate suppression, shoulder recovery), `selectCurbPointsFromCells`,
  `thinCurbPointsToDominantBoundary`, `extractRoadMarkings`.
* `offGroundFeatures/` is the non-ground branch, orchestrated by
  `extractOffGroundFeatures`: `buildFineColumnFeatureMaps` and
  `buildFineColumnShapeScores` (column occupancy, vertical run length, point-versus-
  line shape), `extractTrafficSignChannel`, `extractFacadeFeatures`
  (`detectFacadeLines` Hough → `assignFacadeColumnsToLines` → `refineFacadeWithFineGrid`
  planarity), `detectPoleCandidates` (global column analysis) and
  `refinePolesWithFineGrid` (local slice-density validation).
* `semanticProduct/` fuses both branches into semantic tags on the 3D voxel grid
  (`buildSemanticVoxelGrid`) and recovers refined per-class points
  (`refineSemanticPoints`); `mapping/buildSemanticNdtGridMap` consumes it.

### Mapping (`mapping/`)

`buildFeatureMap(dataRoot)` runs the whole offline chain and saves the
sliding-window probability-cloud map. The temporal-stability GMM in
`mapping/temporalStabilityGmm/` keeps the design rule of the original code:
temporal information decides *which points are sampled, how the mixture is
seeded, and which components survive*, never the EM updates themselves.

### Localization (`localization/`)

Three estimator components live here. The improved observer composes the two
state estimators as a one-way cascade.

The **global ego-state estimator** is the retrodictive-replay high-gain observer
(Bessafa et al., 2026) on the transformed state `[X, Vx, Ax, Y, Vy, Ay]` in the
map frame: delayed pose measurements are inserted as discrete jumps at their
physical timestamp and the buffered history is replayed to the present. The jump
gain comes from `designReplayObserverGains`, which needs YALMIP and SeDuMi on the
path; the online observer itself has no external dependency.

The **lateral-velocity observer** in `localization/lateralObserver/` works on the
body-frame state `x = [v_y, r]` of the 2-DOF bicycle model, with the steering
angle as input and `y = [a_y, r]` from the IMU as output. Both `A` and `C` are
affine in the scheduling parameter `rho = [Vx; 1/Vx]`, so they are represented
exactly on a triangle that covers the scheduling arc (the arc is convex, so the
harmonic-mean third vertex closes a triangle around it). The gain `L(rho)` is
synthesized by semidefinite programming from a parameter-dependent Lyapunov
function `V = e' P(rho) e`: the Lipschitz nonlinearity is absorbed by Young's
inequality with a fixed `tau`, `Y = P L` removes the bilinearity, a slack matrix
`X` decouples `P` from `A` through a second Young step with `eta`, and two Schur
complements give LMIs imposed on a grid of speeds and longitudinal
accelerations. Minimizing `mu` subject to `trace(W_k) <= mu` bounds the weighted
H2 gain from measurement noise to the estimation error `z = Q^(1/2) e`.
`designLateralObserverGains` then re-checks the recovered gains against the
original, non-convexified certificate.

This block is block 1 of the improved ego-state observer. It runs open of the
global observer, taking only wheel speed, steering angle, and IMU signals. It
hands over the side-slip angle and its rate as exogenous known signals through
the track-angle rate `q = r_m + betaDot`, so the two stages cascade without a
loop. `runLateralVelocityObserver` supplies exactly this interface:

* `sideSlipAngle` and `sideSlipAngleRate`, the rate taken analytically from the
  first row of the observer right-hand side, never by differentiating the
  estimate;
* `longitudinalSpeedRate`, rebuilt as `ax + vy*r` because the IMU reports a
  specific force and not the speed derivative, and it is that rate which both
  schedules the gain and enters the side-slip rate;
* `lowSpeedHold`, which zeroes the exported lateral velocity and both side-slip
  outputs below the minimum scheduling speed where they lose meaning.

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

One config per stage, named after the stage, values identical to the tuned
original setup (including the overrides the original map-building script applied
on top of its config files):

| config | consumed by |
| --- | --- |
| `perceptionConfig` | `perceiveFrame` (aggregates the four below) |
| `frameVoxelizationConfig` | `voxelizePointCloud` |
| `groundSegmentationConfig` | `segmentGround` |
| `groundFeatureConfig` (`.curb`, `.road`, `.roadMarking`) | `extractGroundFeatures` |
| `offGroundFeatureConfig` | `extractOffGroundFeatures` (`facadeDetectionEnabled` is a dataset policy) |
| `temporalStabilityMapConfig` | `buildTemporalStabilityGmmMap` |
| `featureMapBuildConfig` | `buildFeatureMap` |
| `semanticNdtGridMapConfig` | `buildSemanticNdtGridMap` |
| `replayObserverConfig` | `runReplayHighGainObserver`, `designReplayObserverGains` |
| `lateralObserverConfig` | `designLateralObserverGains`, `runLateralVelocityObserver` |
| `improvedObserverConfig` | `designImprovedObserverGains`, `runImprovedVehicleObserver` |

## Quick start

```matlab
setupVehicleLocalization();                       % add modules to the path
frame = loadPointCloudFrame("data/raw/MissisipiPointClouds.mat", 260);
perception = perceiveFrame(frame, perceptionConfig());
nnz(perception.featureMasks.curb)                 % curb points of this frame

[probabilityCloudMap, featureData] = buildFeatureMap("data");   % offline map
scores = queryTemporalStabilityGmmMap(probabilityCloudMap, queryXY, "pole");

result = simulateReplayObserverScenario(designedCfg);           % observer demo

design = designLateralObserverGains(lateralObserverConfig());   % LPV H2 synthesis
estimate = runLateralVelocityObserver(measurements, design);    % v_y, r, side slip

lateral = load("tests/reference/lateralObserverDesign.mat");
improved = improvedObserverReferenceDesign();
result = simulateImprovedObserverScenario( ...
    improved, lateral.design, improvedObserverConfig());        % complete cascade
```

`scripts/` holds the runnable entry points: `extractPointCloudsFromBag.m` and
`extractGnssFromBag.py` (ROS bag → MAT frames and GNSS/INS CSV tables),
`buildMississippiFeatureMap.m`, `runReplayObserverDesign.m`, and
`runLateralObserverDesign.m`, and `runImprovedObserverDesign.m`.

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

The refactor was checked against the original code on real data: frames 260,
300, and 326 of the Mississippi route and frame 400 of the Downtown route (with
facades enabled), a 30-frame temporal-stability map with query scores, and the
replay observer on a recorded 24 s scenario. All 206 comparisons (ground labels,
every feature channel, energy maps, column maps, facade lines, semantic voxel
tags, refined points, NDT components, registered observations, map input, GMM
parameters and components, query scores, observer states) are equal to the
original outputs, exactly for integer and logical products and to 1e-8 or better
for floating-point ones.

`tests/` contains the unit tests carried over from the original repository and
`pipelineRegressionTest`, which reproduces `tests/reference/pipelineReference.mat`
(captured from the original code). The observer check always runs; the
perception and mapping checks run when `VEHICLE_LOCALIZATION_DATA_ROOT` points
at the data root above.

**Known pre-existing failure.** `designReplayObserverGains` (the offline LMI
that synthesizes the replay pose-jump gain) is infeasible for every configured
`rhoCandidates` entry. This is inherited, not introduced: the original
`solveHGOgain` fails identically, with the same message and the same
net-contraction limit, on both the reduced test configuration and the full one.
The flow condition permits growth at `theta*flowAh = 6` 1/s, so across one 0.2 s
pose interval the Lyapunov function may grow by `exp(1.2)`, and net contraction
then requires a jump factor `rho < 0.301` from a correction that observes only
x, y, and yaw out of six states. The design artifact the online observer
actually uses predates the joint flow-and-jump LMI: its saved struct has a
`flow` field but no `main`, and its `rhoCandidates` all lie above the limit the
current code enforces. The online observer itself is unaffected and is verified
against the reference; `replayObserverTest` marks the corresponding test as a
known failure rather than hiding it.

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
| `HGO`, `solveHGOgain` | `runReplayHighGainObserver`, `designReplayObserverGains` |
| `runBessafa2026ReplayObserverSimulation` | `simulateReplayObserverScenario` (no figure export) |

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
