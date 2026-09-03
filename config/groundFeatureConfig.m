function cfg = groundFeatureConfig()
% groundFeatureConfig: Parameters of the ground-point feature branch: curb
% energy maps, curb cell refinement, road-surface recovery, curb point
% selection, and road-marking selection. The values reproduce the tuned
% RobustVehicleLocalization configuration, including the dominant-boundary
% point filter and its bidirectional continuation that the map-building
% pipeline enabled on top of groundFeatureConfig().curb.
%
% Input:
%   none
%
% Output:
%   cfg: struct with fields
%       curb: curb energy-map, cell refinement, and point selection parameters
%       road: road-surface grid extraction parameters
%       roadMarking: relative road reflectivity threshold parameters
    cfg = struct();
    cfg.curb = curbParameters();
    cfg.road = roadSurfaceParameters();
    cfg.roadMarking = roadMarkingParameters();
end

function curb = curbParameters()
% curbParameters: Curb energy-map, cell refinement, and point selection
% parameters grouped by processing stage.
%
% Input:
%   none
%
% Output:
%   curb: flat parameter struct consumed by the curb extraction stages
    curb = struct();

    % Cell statistics and detrending
    curb.detrendBoxRadiusCells = 2;
    curb.minPointsPerCell = 1;
    curb.minValidNeighborCount = 2;

    % Per-feature energy targets: height step, residual slope, curvature, roughness, relative height, full-cell relief
    curb.heightStepMinMeters = 0.04;
    curb.heightStepMaxMeters = 0.30;
    curb.heightStepTargetMeters = 0.08;
    curb.heightStepSigmaMeters = 0.03;
    curb.residualSlopeTargetDeg = 7.0;
    curb.residualSlopeSigmaDeg = 3.0;
    curb.curvatureTarget = 1.0;
    curb.curvatureSigma = 0.35;
    curb.roughnessTargetMeters = 0.04;
    curb.roughnessSigmaMeters = 0.02;
    curb.relativeHeightRadiusCells = 6;
    curb.relativeHeightMinMeters = 0.04;
    curb.relativeHeightSaturatedMeters = 0.12;
    curb.fullCellReliefMinPoints = 2;
    curb.fullCellReliefMinMeters = 0.08;
    curb.fullCellReliefSaturatedMeters = 0.25;

    % Local linearity gate on the base energy (second-moment anisotropy of high-energy cells)
    curb.linearityEnergyGateEnabled = true;
    curb.linearityRadiusMeters = 1.20;
    curb.linearityEnergyThreshold = 0.18;
    curb.linearityWeightPower = 1.0;
    curb.linearityMinWeightSum = 0.15;
    curb.linearityAnisotropyPower = 1.0;
    curb.linearityMinLengthMeters = 0.24;
    curb.linearitySaturatedLengthMeters = 0.72;
    curb.linearityMaxWidthMeters = 0.50;
    curb.linearityRejectWidthMeters = 1.05;
    curb.linearityMinSeedCount = 5;
    curb.linearitySaturatedSeedCount = 14;
    curb.linearityMinValidRatio = 0.25;
    curb.linearitySaturatedValidRatio = 0.55;
    curb.linearityCenterEnergyMin = 0.12;
    curb.linearityCenterEnergySaturated = 0.42;

    % Directional line support and connected line-component support
    curb.directionalLinearityEnabled = true;
    curb.directionalLineEnergyThreshold = 0.65;
    curb.directionalLineRadiusCells = 5;
    curb.directionalLineMinCount = 3;
    curb.directionalLineSaturatedCount = 4;
    curb.directionalLineCenterEnergyMin = 0.40;
    curb.directionalLineCenterEnergySaturated = 0.50;
    curb.directionalLineComponentScoreMin = 0.50;
    curb.directionalLineComponentScoreSaturated = 0.80;
    curb.directionalLineThinnessMin = 0.90;
    curb.directionalLineThinnessSaturated = 0.98;
    curb.linearityEnergyFloor = 0.15;
    curb.linearityEnergyPower = 2.0;
    curb.componentLineSupportEnabled = true;
    curb.componentSeedEnergyThreshold = 0.18;
    curb.componentMinCells = 8;
    curb.componentSaturatedCells = 35;
    curb.componentMinLengthMeters = 0.80;
    curb.componentSaturatedLengthMeters = 2.40;
    curb.componentMaxWidthMeters = 0.90;
    curb.componentRejectWidthMeters = 1.20;
    curb.componentAnisotropyPower = 0.60;
    curb.componentFillSupportRadiusCells = 2;
    curb.componentFillLinearityThreshold = 0.45;
    curb.componentFillMinStrongLinearityCount = 3;
    curb.componentFillSaturatedStrongLinearityCount = 7;
    curb.centerHeightStepMinMeters = 0.06;
    curb.centerHeightStepSaturatedMeters = 0.09;
    curb.centerRoughnessMinMeters = 0.010;
    curb.centerRoughnessSaturatedMeters = 0.025;
    curb.centerEvidenceEnergyFloor = 0.25;
    curb.componentFillWeight = 0.60;
    curb.normalizeTotalEnergyToUnitMax = true;

    % Curb cell extraction thresholds on the total energy map
    curb.extractionEnergyThreshold = 0.60;
    curb.extractionBaseEnergyThreshold = 0.18;
    curb.extractionLinearityMin = 0.20;
    curb.extractionComponentScoreMin = 0.12;
    curb.extractionCenterEvidenceMin = 0.75;
    curb.extractionHeightStepMinMeters = 0.035;
    curb.extractionStandaloneBaseEnergyThreshold = 0.35;
    curb.extractionStandaloneRoughnessMinMeters = 0.003;
    curb.extractionStandaloneBridgeBaseEnergyThreshold = 0.28;
    curb.extractionStandaloneBridgeCenterEvidenceMin = 0.75;
    curb.extractionStandaloneBridgeRadiusCells = 2;
    curb.extractionStandaloneBridgeMinSeedCount = 1;
    curb.extractionFillBaseEnergyThreshold = 0.12;
    curb.extractionFillHeightStepMinMeters = 0.035;
    curb.extractionFillGateMin = 0.15;
    curb.extractionFillSupportCountMin = 4;
    curb.extractionFillSeedCountMin = 60;
    curb.extractionFillValidRatioMin = 0.45;
    curb.extractionEndpointBaseEnergyThreshold = 0.18;
    curb.extractionEndpointHeightStepMinMeters = 0.08;
    curb.extractionEndpointCenterEvidenceMin = 0.75;
    curb.extractionEndpointSeedCountMin = 25;
    curb.extractionEndpointValidRatioMin = 0.40;

    % Road-adjacency refinement of curb cells (refineCurbCellsByRoadAdjacency stages)
    curb.roadAdjacencyFilterEnabled = true;
    curb.roadAdjacencyRadiusCells = 4;
    curb.roadAdjacencyComponentMinAdjacentCells = 3;
    curb.roadAdjacencyComponentMinAdjacentFraction = 0.15;
    curb.roadAdjacencyComponentMaxFullKeepCells = 40;
    curb.roadAdjacencySmallComponentStrongEnergyThreshold = 0.60;
    curb.roadAdjacencySmallComponentMinStrongCells = 1;
    curb.roadAdjacencyLargeComponentMinCells = 180;
    curb.roadAdjacencyRoadFacingAnchorEnabled = true;
    curb.roadAdjacencyRoadFacingAnchorRadiusCells = 0;
    curb.roadAdjacencyRoadReferenceRadiusCells = 4;
    curb.roadAdjacencyFeaturePeakAnchorEnabled = true;
    curb.roadAdjacencyBoundaryContinuationEnabled = true;
    curb.roadAdjacencyBoundaryContinuationRadiusCells = 0;
    curb.roadAdjacencyBoundaryContinuationMaxGapCells = 6;
    curb.roadAdjacencyBoundaryContinuationMinAnchorCols = 8;
    curb.roadAdjacencyExcludeRoadCells = false;
    curb.boundaryGapCompletionEnabled = true;
    curb.boundaryGapCompletionMaxGapCells = 8;
    curb.boundaryGapCompletionMinBaseEnergy = 0.28;
    curb.boundaryGapCompletionMinLinearity = 0.30;
    curb.boundaryGapCompletionMinHeightStepMeters = 0.025;
    curb.boundaryGapCompletionMinRoughnessMeters = 0.003;
    curb.boundaryPathCompletionEnabled = true;
    curb.boundaryPathCompletionProjectionRadiusCells = 0;
    curb.boundaryPathCompletionMaxGapCells = 24;
    curb.boundaryPathCompletionMaxSlopeRowsPerCol = 0.40;
    curb.boundaryPathCompletionRadiusCells = 2;
    curb.boundaryPathCompletionMinBaseEnergy = 0.04;
    curb.boundaryPathCompletionMinHeightStepMeters = 0.014;
    curb.boundaryPathCompletionMinLinearity = 0.05;
    curb.sameSideDuplicateSuppressionEnabled = true;
    curb.sameSideDuplicateMinOverlapFraction = 0.25;
    curb.sameSideDuplicateMinSeparationCells = 2;
    curb.sameColumnFeaturePeakSelectionEnabled = true;
    curb.sameColumnFeaturePeakSearchRadiusCells = 2;
    curb.sameColumnFeaturePeakMinBaseEnergy = 0.25;
    curb.sameColumnFeaturePeakMinHeightStepMeters = 0.025;
    curb.sameColumnFeaturePeakMinLinearity = 0.25;
    curb.sameColumnFeaturePeakPromotionMinHeightGainMeters = 0.004;
    curb.sameColumnFeaturePeakPromotionScoreRatio = 0.95;
    curb.sameColumnFeaturePeakPromotionBaseRatio = 0.75;
    curb.sameColumnFeaturePeakPromotionMinRoughnessMeters = 0.010;
    curb.sameColumnFeaturePeakPromotionRoughnessRatio = 1.50;
    curb.sameColumnFeaturePeakRoadFacingMinBaseEnergy = 0.32;
    curb.sameColumnFeaturePeakRoadFacingMinRoughnessMeters = 0.020;
    curb.sameColumnFeaturePeakRoadFacingMinLinearity = 0.32;
    curb.sameColumnFeaturePeakRoadFacingMinCenterEvidence = 0.90;
    curb.sameColumnFeaturePeakOuterProtectionMaxHeightGainMeters = 0.012;
    curb.sameColumnFeaturePeakSuppressionMinHeightGainMeters = 0.012;
    curb.sameColumnFeaturePeakSuppressionScoreRatio = 1.25;
    curb.sameSideCellShadowSuppressionEnabled = true;
    curb.sameSideCellShadowRadiusCells = 0;
    curb.sameSideCellShadowMinSeparationCells = 1.0;
    curb.sameSideCellShadowProtectedLinearityMin = 0.47;
    curb.sameSideCellShadowProtectedRoughnessMinMeters = 0.010;
    curb.sameSideCellShadowNegativeFeatureLinearityMin = 0.30;
    curb.sameSideCellShadowFeatureRoughnessScaleMeters = 0.020;
    curb.sameSideCellShadowDominantScoreRatio = 1.25;
    curb.sameSideCellShadowDominantRoughnessGainMeters = 0.004;
    curb.sameSideCellShadowSmoothRoughnessMaxMeters = 0.010;
    curb.sameSideCellShadowReliableCloserCenterEvidenceMin = 0.90;
    curb.sameSideCellShadowReliableCloserDominantTotalMin = 0.40;
    curb.roadFacingShoulderRecoveryEnabled = true;
    curb.roadFacingShoulderRecoveryMinBaseEnergy = 0.55;
    curb.roadFacingShoulderRecoveryMinHeightStepMeters = 0.055;
    curb.roadFacingShoulderRecoveryMinLinearity = 0.45;
    curb.roadFacingShoulderRecoveryAnchorCenterEvidenceMin = 0.90;
    curb.roadFacingLineShoulderRecoveryMinBaseEnergy = 0.30;
    curb.roadFacingLineShoulderRecoveryMinHeightStepMeters = 0.055;
    curb.roadFacingLineShoulderRecoveryMinLinearity = 0.45;
    curb.roadAwayShoulderRecoveryMinBaseEnergy = 0.75;
    curb.roadAwayShoulderRecoveryMinTotalEnergy = 0.35;
    curb.roadAwayShoulderRecoveryMinHeightStepMeters = 0.050;
    curb.roadAwayShoulderRecoveryMinLinearity = 0.35;
    curb.roadAwayShoulderRecoveryAnchorRoughnessMinMeters = 0.035;
    curb.roadAwayLineShoulderRecoveryMinBaseEnergy = 0.45;
    curb.roadAwayLineShoulderRecoveryMinHeightStepMeters = 0.055;
    curb.roadAwayLineShoulderRecoveryMinLinearity = 0.45;
    curb.roadAwayLineShoulderRecoveryMinCenterEvidence = 0.90;
    curb.boundaryRunCompletionEnabled = true;
    curb.boundaryRunCompletionMaxGapCells = 8;
    curb.boundaryRunCompletionMaxEndpointCells = 4;
    curb.boundaryRunCompletionNoEvidenceMaxGapCells = 2;
    curb.boundaryRunCompletionMinCenterEvidence = 0.90;
    curb.boundaryRunCompletionMinBaseEnergy = 0.33;
    curb.boundaryRunCompletionMinHeightStepMeters = 0.016;
    curb.boundaryRunCompletionMinRoughnessMeters = 0.035;
    curb.boundaryRunCompletionMinLinearity = 0.25;
    curb.boundaryRunCompletionStepRoughMinBaseEnergy = 0.20;
    curb.boundaryRunCompletionStepRoughMinHeightStepMeters = 0.080;
    curb.boundaryRunCompletionStepRoughMinRoughnessMeters = 0.045;
    curb.boundaryRunCompletionLineStepMinBaseEnergy = 0.30;
    curb.boundaryRunCompletionLineStepMinHeightStepMeters = 0.055;
    curb.boundaryRunCompletionLineStepMinLinearity = 0.45;
    curb.boundaryRunCompletionLineStepMinCenterEvidence = 0.04;
    curb.boundaryRunCompletionStrongCenterLineStepMinBaseEnergy = 0.75;
    curb.boundaryRunCompletionStrongCenterLineStepMinHeightStepMeters = 0.055;
    curb.boundaryRunCompletionStrongCenterLineStepMinLinearity = 0.35;
    curb.boundaryRunCompletionRawLineStepMinCenterEvidence = 0.50;
    curb.boundaryRunCompletionRawLineStepMinBaseEnergy = 0.35;
    curb.boundaryRunCompletionRawLineStepMinHeightStepMeters = 0.055;
    curb.boundaryRunCompletionRawLineStepMinLinearity = 0.40;
    curb.boundaryRunCompletionEndpointLineStepMinCenterEvidence = 0.10;
    curb.boundaryRunCompletionEndpointLineStepMinBaseEnergy = 0.35;
    curb.boundaryRunCompletionEndpointLineStepMinHeightStepMeters = 0.055;
    curb.boundaryRunCompletionEndpointLineStepMinLinearity = 0.35;
    curb.sameSideRoadFacingDuplicateSuppressionEnabled = true;
    curb.sameSideRoadFacingDuplicateSearchRowsCells = 2;
    curb.sameSideRoadFacingDuplicateOuterCenterEvidenceMin = 0.90;
    curb.sameSideRoadFacingDuplicateOuterLinearityMin = 0.85;
    curb.sameSideRoadFacingDuplicateOuterStepLinearityMin = 0.35;
    curb.sameSideRoadFacingDuplicateInnerLinearityMax = 0.65;
    curb.sameSideRoadFacingDuplicateInnerStepLinearityMax = 0.48;
    curb.sameSideRoadFacingDuplicateInnerRoughnessMaxMeters = 0.040;
    curb.sameSideRoadFacingDuplicateInnerCenterEvidenceMax = 0.05;
    curb.sameSideRoadFacingDuplicateOuterBaseRatio = 0.80;
    curb.sameSideRoadFacingDuplicateOuterBaseGainRatio = 1.20;
    curb.sameSideRoadFacingDuplicateOuterHeightGainMeters = 0.015;
    curb.sameSideOutlierSuppressionEnabled = true;
    curb.sameSideOutlierReferenceMinCells = 6;
    curb.sameSideOutlierMaxComponentCells = 8;
    curb.sameSideOutlierMaxColumnSpanCells = 24;
    curb.sameSideOutlierMaxExtraDistanceCells = 6;
    curb.intrinsicComponentMinCells = Inf;
    curb.intrinsicComponentMaxCells = 0;
    curb.intrinsicComponentMinSeedCells = Inf;
    curb.continuationExpansionEnabled = false;
    curb.continuationExpansionRadiusCells = 2;
    curb.continuationBaseEnergyThreshold = 0.35;
    curb.continuationCenterEvidenceMin = 0.25;

    % Curb point selection inside accepted cells and dense near-road filtering (selectCurbPointsFromCells)
    curb.curbPointLowerQuantile = 0.20;
    curb.curbPointMinLowerResidualMeters = 0.025;
    curb.curbPointResidualWeight = 0.15;
    curb.curbPointQuantileWeight = 0.85;
    curb.denseCurbNearRoadFilterEnabled = true;
    curb.denseCurbNearRoadRadiusCells = 2;
    curb.denseCurbMinResidualMeters = 0.025;
    curb.denseCurbMinBaseEnergy = 0.75;
    curb.denseCurbMinRoughnessMeters = 0.030;

    % Dominant road-boundary thinning of curb points (thinCurbPointsToDominantBoundary)
    curb.dominantBoundaryPointFilterEnabled = true;
    curb.dominantBoundaryNeighborRows = 1;
    curb.dominantBoundaryColumnTrackingEnabled = true;
    curb.dominantBoundaryColumnSearchRadiusCells = 3;
    curb.dominantBoundaryColumnMinPoints = 2;
    curb.dominantBoundaryPointQuantile = 0.50;
    curb.dominantBoundaryYBandMeters = 0.20;
    curb.dominantBoundaryRoadFacingCellMarginMeters = 0.12;
    curb.dominantBoundaryRoadAwayNeighborMarginMeters = Inf;
    curb.dominantBoundaryRoadReferenceFilterEnabled = true;
    curb.dominantBoundaryRoadReferenceRadiusCells = 16;
    curb.dominantBoundaryWeakSupportFilterEnabled = true;
    curb.dominantBoundaryWeakSupportMinBaseEnergy = 0.12;
    curb.dominantBoundaryWeakSupportMinLinearity = 0.10;
    curb.dominantBoundaryWeakSupportMinRoughnessMeters = 0.010;
    curb.dominantBoundaryWeakSupportMinCenterEvidence = 0.75;
    curb.dominantBoundaryWeakSupportHighStepMinMeters = 0.08;
    curb.dominantBoundaryMinSidePoints = 12;
    curb.dominantBoundaryRowMinCountFraction = 0.15;
    curb.dominantBoundaryContinuationEnabled = true;
    curb.dominantBoundaryContinuationDirection = "both";
    curb.dominantBoundaryContinuationMaxGapCells = 8;
    curb.dominantBoundaryContinuationSearchRows = 1;
    curb.dominantBoundaryContinuationMinTotalEnergy = 0.0;
    curb.dominantBoundaryContinuationMinXCenterMeters = 0.0;
    curb.dominantBoundaryContinuationMaxRelativeHeightMeters = 0.18;
    curb.dominantBoundaryContinuationMinBaseEnergy = 0.25;
    curb.dominantBoundaryContinuationMinHeightStepMeters = 0.020;
    curb.dominantBoundaryContinuationMinLinearity = 0.0;

    % Weighted combination of the per-feature energy maps into the total energy
    curb.weights = struct();
    curb.weights.heightStep = 0.30;
    curb.weights.residualSlope = 0.30;
    curb.weights.curvature = 0.10;
    curb.weights.roughness = 0.30;
    curb.weights.relativeHeight = 0.18;
    curb.weights.fullCellRelief = 0.00;
end

function road = roadSurfaceParameters()
% roadSurfaceParameters: Road-surface extraction parameters: curb cells act as
% barriers, ego-near supported cells seed the road, and the road grows through
% height-continuous ground cells so curved and branching roads stay connected.
%
% Input:
%   none
%
% Output:
%   road: parameter struct consumed by extractRoadSurface
    road = struct();
    road.curbBarrierRadiusCells = 1;
    road.minPointsPerCell = 1;
    road.maxRoadEnergy = 0.95;
    road.maxRoadRoughnessMeters = 0.12;
    road.curbInteriorConstraintEnabled = true;
    road.curbInteriorClearanceCells = 1;
    road.curbInteriorMinBoundaryCells = 4;
    road.curbInteriorMaxExtrapolationMeters = 6.0;
    road.seedXLimitsMeters = [3.0, 14.0];
    road.seedAbsYMaxMeters = 4.0;
    road.fallbackSeedEnabled = true;
    road.fallbackSeedReferenceXYMeters = [4.0, 0.0];
    road.maxNeighborHeightStepMeters = 0.10;
    road.maxSeedHeightDeviationMeters = 0.60;
    road.minRoadCells = 8;
end

function roadMarking = roadMarkingParameters()
% roadMarkingParameters: Relative road-surface reflectivity rule that selects
% road-marking points: a high quantile of the road reflectivity plus a robust
% spread multiple, floored by an absolute minimum reflectivity.
%
% Input:
%   none
%
% Output:
%   roadMarking: parameter struct consumed by extractRoadMarkings
    roadMarking = struct();
    roadMarking.roadReflectivityHighlightQuantile = 0.95;
    roadMarking.roadReflectivityHighlightMadScale = 2.5;
    roadMarking.roadReflectivityHighlightMinimum = 2.3e4;
end
