# Current implementation cleanup

Date: 2026-09-08. Output reference revision:
`6bf426cd5b14d89f8e138ede2ae78dcfdb8fb747`.

## Source policy and implementation

The maintained source tree contains current algorithms and current validation
entry points. Historical behavior is preserved as immutable output data and Git
history, not as a selectable executable implementation.

Removed the `legacyFull` perception branch and the `full` mode alias, its separate
branch-grid construction, old ground/off-ground orchestration, full-frame mask
assembly, dense semantic voxel products, and the column-score wrapper. Removed
unreferenced road-marking, pole/facade voxel refiners and their fine-grid helpers.
The active fine detector keeps its detailed sparse support and point validation;
this is the current offline algorithm, not a preserved alternate pipeline.

`pillarGridConfig` supplies exactly two XY spacings. A third spacing now raises
`perception:InvalidPillarSpacing`. `fineVoxelizationConfig` supplies the separate
3D offline index. The index no longer builds dense moment tensors or inverse
voxel lookup arrays. `fineStructuralConfig` contains detailed candidate settings;
unused voxel-refinement configuration and alternate parameter-name fallbacks
were removed with their consumers.

Map query and export now require schema version 2. Deleted the noisy-OR/max
reader and summed-support exporter. Gaussian registration has one pose solver,
`geometricD2D`; the density-overlap pose optimizer was deleted. The current
Gaussian-overlap diagnostic remains independently tested and is not a solver
selection. Registration requires explicit calibration on both inputs and
repeatability on the fixed map. Current map and coarse-cloud producers attach
the calibration selected by configuration. Missing metadata is rejected rather
than assigned an unverified transform or synthetic repeatability.

The lateral observer requires current hybrid configuration; automatic augmentation
of old saved runtime configurations and the unused `lowSpeedHold` output were
removed. Tests explicitly combine stored gain matrices with current hybrid
settings without changing the stored designs. The raw side-slip validity flag
is `estimate.diagnostics.sideSlipCommandValid`.

Removed experiment scripts tied to deleted solvers or source-checkout comparisons.
`benchmarkCoarseProbabilityCloud` measures current coarse and offline calls.
`evaluateWholePillarPerception` compares frozen output snapshots. Historical
research reports remain dated records; they are not runnable source entry points.

## Regression evidence

Before changing the source, captured masks and coarse candidate IDs from 30
frames into `tests/reference/perceptionMasks.json`. The fixture contains original
point indices and metadata, not raw point clouds or executable baseline code.
The original MAT reference artifacts were not regenerated.

- Mississippi: 75, 100, 150, 200, 225, 260, 300, 326, 370, 400, 450, 475, 500,
  550, 600, 700, 775, 800, 850, 900, 1000, 1050, 1100, 1125, 1150.
- Downtown: 100, 200, 300, 400, 500.

All 30 current fine outputs match every frozen mask exactly, including ground,
curb, road marking, pole, facade, and traffic sign. Coarse candidate comparison
also reports zero added and zero removed pillars on every selected channel.
This establishes preservation relative to the immediately preceding version;
it does not independently measure ground-truth accuracy or undo the earlier
whole-pillar redesign's documented difference from its older coarse baseline.

The full MATLAB suite completed with **327 passed, zero failed, two filtered**.
The filtered synthesis cases require unavailable YALMIP/SDP dependencies. The
suite exercises current perception, native/MATLAB equivalence, map inference and
queries, geometric registration, and observer scenarios. Retired tests that ran
an old implementation now read fixed results or test explicit rejection of old
inputs. Current NDT aggregation remains independently tested.

Initial integration checks identified fixtures that depended on implicit map
calibration or saved hybrid-configuration augmentation. Those fixtures were
migrated explicitly; no runtime compatibility fallback was reintroduced.
No full recorded trajectory replay, new accuracy study, or real-time qualification
was performed for this cleanup.

Factory Code Analyzer reported zero findings in all 45 changed/new MATLAB files.
The current coarse/fine benchmark completed on Mississippi frame 260.

Validation exports are under `research/results/current_cleanup_20260908/`.
