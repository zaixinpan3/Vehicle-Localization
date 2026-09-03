function cfg = offGroundFeatureConfig()
% offGroundFeatureConfig: Parameters of the non-ground feature branch: fine
% voxel grid and column shape scores, traffic-sign channel, facade line
% detection and refinement, and pole candidate detection and validation. The
% values reproduce the tuned RobustVehicleLocalization configuration.
% facadeDetectionEnabled is a dataset policy: the Mississippi suburban route
% runs without facades, the Downtown route with them.
%
% Input:
%   none
%
% Output:
%   cfg: flat parameter struct consumed by extractOffGroundFeatures
    cfg = struct();

    % Fine-column point-versus-line shape scores (the voxel size comes from frameVoxelizationConfig)
    cfg.fineShapeScoreNeighborhoodRadiusCells = 2;
    cfg.fineShapeScoreWeightPower = 2.0;
    cfg.fineShapeScoreLinearityPower = 1.0;
    cfg.fineShapeScoreMinSeedCount = 3;
    cfg.fineShapeScoreSaturatedSeedCount = 5;

    % Traffic-sign channel: high-intensity voxels removed before structural analysis
    cfg.trafficSignIntensityThreshold = 1600;

    % Facade line detection by oriented weighted Hough voting on the fine-column map
    cfg.facadeDetectionEnabled = false;
    cfg.thetaResolutionDeg = 5;
    cfg.thetaPriorRangeDeg = [];
    cfg.orientationToleranceDeg = 25;
    cfg.scoreGamma = 0.05;
    cfg.supportGammaMin = 0.55;
    cfg.supportQuantile = 0.85;
    cfg.supportAbs = 0.20;
    cfg.maxManifoldWeight = 0.15;
    cfg.useLogAccumulator = true;
    cfg.facadeHoughLineExponent = 1.0;
    cfg.facadeHoughEnergyExponent = 1.0;
    cfg.facadeHoughVoteWeightQuantile = 0.95;
    cfg.houghPeakThresholdRatio = 0.70;
    cfg.houghPeakQuantizationScale = 100;
    cfg.maxHoughPeaks = 8;
    cfg.houghSuppressionSize = [25, 25];
    cfg.lineDistanceThresholdVoxels = 0.5;
    cfg.facadeLineFillGapVoxels = 15;
    cfg.facadeLineMinLengthVoxels = 18;
    cfg.splitValidationEnabled = false;
    cfg.minAssignedPixels = 10;
    cfg.minPeakLengthMeters = 0.0;
    cfg.mergeThetaTolDeg = 2.0;
    cfg.mergeRhoTolMeters = 1.0;
    cfg.maxOutputLines = 0;
    cfg.thetaClusterTolDeg = 5.0;
    cfg.maxThetaClusters = 3;
    cfg.maxLinesPerThetaCluster = 4;
    cfg.fitWeightExponent = 0.0;
    cfg.facadeMaskUseKeptPeaks = true;

    % Facade refinement by local fine-grid patch planarity and normal alignment
    cfg.facadeRefineEnabled = true;
    cfg.facadeRefineHorizontalSupportRadiusVoxels = 2;
    cfg.facadeRefinePatchSizeVoxels = [4, 4, 6];
    cfg.facadeRefineMinPatchVoxels = 8;
    cfg.facadeRefineMinPatchPoints = 30;
    cfg.facadeRefinePlanarityThreshold = 0.35;
    cfg.facadeRefineMaxNormalAngleDeg = 20;
    cfg.facadeRefineKeepSeedWhenEmpty = true;

    % Pole candidate detection on the fine-column run-layer map
    cfg.poleCountQuantiles = [0.25, 0.90];
    cfg.poleRangeQuantiles = [0.25, 0.85];
    cfg.poleEnergyWeight = 0.5;
    cfg.poleSeedRunLayerThreshold = 5;
    cfg.poleSupportRunLayerThreshold = 6;
    cfg.poleCoreMinRunLayerThreshold = 4;
    cfg.poleCoreMaxLineScore = 0.90;
    cfg.poleComponentSupportMinOccupiedLayers = 2;
    cfg.poleComponentContextMinRunLayerRatio = 0.58;
    cfg.poleSplitLayerMinMemberOccupiedLayers = 3;
    cfg.poleSplitLayerMinCombinedOccupiedLayers = 7;
    cfg.poleSplitLayerMinCombinedRunLayers = 3;
    cfg.poleSplitLayerMinPointScore = 0.70;
    cfg.poleSplitLayerMaxLineScore = 0.35;
    cfg.polePatternSupportRunLayerThreshold = 6;
    cfg.polePatternNeighborRunLayerThreshold = 1;
    cfg.poleRunLayerContrastThreshold = 0.80;
    cfg.poleDominantSeedMinPointScore = 0.40;
    cfg.poleDominantSeedMinRunLayerDrop = 2;
    cfg.poleDominantSeedMinRunLayerRatio = 1.5;
    cfg.poleSeedDominanceRadiusVoxels = 2;
    cfg.poleSeedDominanceMinRunLayerDrop = 2;
    cfg.poleSeedDominanceMinRunLayerRatio = 1.5;
    cfg.poleSeedDominanceMaxNearRunLayerNeighbors = 2;
    cfg.poleCompactSeedRunLayerThreshold = 6;
    cfg.poleCompactSeedMinPointScore = 0.20;
    cfg.poleCompactSeedMaxNearRunLayerNeighbors = 3;
    cfg.poleCandidateMinRunLayerThreshold = 2;
    cfg.poleOccupiedLayerMinPoints = 3;
    cfg.poleCandidateSingletonMinColumnPoints = 15;
    cfg.poleCandidateSingletonContextRadiusVoxels = 2;
    cfg.poleCandidateSingletonContextMinRunLayerRatio = 3.0;
    cfg.poleCandidateSingletonLowSupportMaxColumnPoints = 50;
    cfg.poleCandidateSingletonLowSupportMaxContextRunSum = 30;
    cfg.poleCandidateSingletonLowSupportMinRunToContextSumRatio = 0.30;
    cfg.poleCandidatePairMaxContextRunSum = 6;
    cfg.poleCandidatePairMinRunToContextSumRatio = 2.0;
    cfg.poleCandidatePairImbalancedMaxMemberRunRatio = 0.65;
    cfg.poleCandidatePairImbalancedMinDominantRunLayerDrop = 4.0;
    cfg.poleCandidatePairContextRadiusVoxels = 2;
    cfg.poleCandidatePairContextMinRunLayerDrop = 2.0;
    cfg.poleSplitPatternCoreRunLayerThreshold = 5;
    cfg.poleSplitPatternMultiSeedCoreRunLayerThreshold = 8;
    cfg.poleSplitPatternNeighborMaxRunLayerDrop = 1;
    cfg.poleMultiCellMeanRunLayerMinRatio = 2.0;
    cfg.poleMultiCellMemberMinInternalRunRatio = 0.30;
    cfg.poleMultiCellMemberRunLayerMinDrop = 1.0;
    cfg.poleMultiCellMemberRunLayerMinRatio = 1.25;
    cfg.poleMaxFootprintSpanCells = 2;
    cfg.poleAdaptiveNeighborhoodRadiusVoxels = 0;

    % Pole validation by local fine-grid slice density (refinePolesWithFineGrid)
    cfg.poleRefineEnabled = true;
    cfg.poleRefineMinCandidateSlices = 4;
    cfg.poleRefineMinLocalRatio = 0.70;
    cfg.poleRefineMinGlobalRatio = 0.70;
    cfg.poleRefineMinMeanPointScore = 0.70;
    cfg.purpleRatioThreshold = 0.60;
    cfg.purpleRatioBlockSize = 1;
    cfg.purpleMinSliceObjectPoints = 2;
    cfg.poleRefineRelaxedMinCandidateSlicesEnabled = true;
    cfg.poleRefineRelaxedMaxBaseHeightMeters = 0.5;
    cfg.poleRefineRelaxedMinLocalRatio = 0.75;
    cfg.poleRefineRelaxedMinGlobalRatio = 0.70;
    cfg.poleVerticalBridgeRadiusVoxels = 1;
end
