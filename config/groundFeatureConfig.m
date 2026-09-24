function cfg = groundFeatureConfig(spacing)
% groundFeatureConfig: Parameters of the ground-point feature branch: curb
% energy maps, whole-pillar curb refinement, and road-surface recovery.
% Fine curb XYZ geometry is configured separately
% by finePerceptionConfig; this common stage contains no curb point thinning.
%
% Input:
%   none
%
% Output:
%   cfg: struct with fields
%       curb: curb energy-map, cell refinement, and point selection parameters
%       road: road-surface grid extraction parameters
    if nargin < 1, spacing = pillarGridConfig().voxelSize(1); end
    cfg = struct();
    cfg.curb = curbParameters(spacing);
    cfg.road = roadSurfaceParameters(spacing);
end

function curb = curbParameters(spacing)
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
    curb.minPointsPerCell = latticeTunedValue(spacing, 1, 3);
    curb.minValidNeighborCount = 2;

    % Per-feature energy targets: height step, residual slope, curvature, roughness, relative height, full-cell relief
    curb.heightStepMinMeters = latticeTunedValue(spacing, 0.04, 0.055);
    curb.heightStepMaxMeters = latticeTunedValue(spacing, 0.30, 0.35);
    curb.heightStepTargetMeters = latticeTunedValue(spacing, 0.08, 0.105);
    curb.heightStepSigmaMeters = latticeTunedValue(spacing, 0.03, 0.035);
    curb.residualSlopeTargetDeg = latticeTunedValue(spacing, 7.0, 6.0);
    curb.residualSlopeSigmaDeg = latticeTunedValue(spacing, 3.0, 2.0);
    curb.curvatureTarget = latticeTunedValue(spacing, 1.0, 0.40);
    curb.curvatureSigma = latticeTunedValue(spacing, 0.35, 0.14);
    curb.roughnessTargetMeters = latticeTunedValue(spacing, 0.04, 0.055);
    curb.roughnessSigmaMeters = latticeTunedValue(spacing, 0.02, 0.025);
    curb.relativeHeightRadiusCells = latticeTunedValue(spacing, 6, 3);
    curb.relativeHeightMinMeters = 0.04;
    curb.relativeHeightSaturatedMeters = 0.12;
    curb.fullCellReliefMinPoints = latticeTunedValue(spacing, 2, 8);
    curb.fullCellReliefMinMeters = 0.08;
    curb.fullCellReliefSaturatedMeters = 0.25;

    % Local linearity gate on the base energy (second-moment anisotropy of high-energy cells)
    curb.linearityEnergyGateEnabled = true;
    curb.linearityRadiusMeters = latticeTunedValue(spacing, 1.20, 2.40);
    curb.linearityEnergyThreshold = 0.18;
    curb.linearityWeightPower = 1.0;
    curb.linearityMinWeightSum = 0.15;
    curb.linearityAnisotropyPower = 1.0;
    curb.linearityMinLengthMeters = latticeTunedValue(spacing, 0.24, 0.48);
    curb.linearitySaturatedLengthMeters = latticeTunedValue(spacing, 0.72, 1.44);
    curb.linearityMaxWidthMeters = latticeTunedValue(spacing, 0.50, 1.00);
    curb.linearityRejectWidthMeters = latticeTunedValue(spacing, 1.05, 2.10);
    curb.linearityMinSeedCount = latticeTunedValue(spacing, 5, 5);
    curb.linearitySaturatedSeedCount = latticeTunedValue(spacing, 14, 14);
    curb.linearityMinValidRatio = 0.25;
    curb.linearitySaturatedValidRatio = 0.55;
    curb.linearityCenterEnergyMin = 0.12;
    curb.linearityCenterEnergySaturated = 0.42;

    % Directional line support and connected line-component support
    curb.directionalLinearityEnabled = true;
    curb.directionalLineEnergyThreshold = 0.65;
    curb.directionalLineRadiusCells = latticeTunedValue(spacing, 5, 5);
    curb.directionalLineMinCount = latticeTunedValue(spacing, 3, 3);
    curb.directionalLineSaturatedCount = latticeTunedValue(spacing, 4, 4);
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
    curb.componentMinCells = latticeTunedValue(spacing, 8, 8);
    curb.componentSaturatedCells = latticeTunedValue(spacing, 35, 35);
    curb.componentMinLengthMeters = latticeTunedValue(spacing, 0.80, 1.60);
    curb.componentSaturatedLengthMeters = latticeTunedValue(spacing, 2.40, 4.80);
    curb.componentMaxWidthMeters = latticeTunedValue(spacing, 0.90, 1.80);
    curb.componentRejectWidthMeters = latticeTunedValue(spacing, 1.20, 2.40);
    curb.componentAnisotropyPower = 0.60;
    curb.componentFillSupportRadiusCells = latticeTunedValue(spacing, 2, 2);
    curb.componentFillLinearityThreshold = 0.45;
    curb.componentFillMinStrongLinearityCount = latticeTunedValue(spacing, 3, 3);
    curb.componentFillSaturatedStrongLinearityCount = latticeTunedValue(spacing, 7, 7);
    curb.centerHeightStepMinMeters = latticeTunedValue(spacing, 0.06, 0.08);
    curb.centerHeightStepSaturatedMeters = latticeTunedValue(spacing, 0.09, 0.12);
    curb.centerRoughnessMinMeters = latticeTunedValue(spacing, 0.010, 0.015);
    curb.centerRoughnessSaturatedMeters = latticeTunedValue(spacing, 0.025, 0.040);
    curb.centerEvidenceEnergyFloor = 0.25;
    curb.componentFillWeight = 0.60;
    curb.normalizeTotalEnergyToUnitMax = true;

    % Curb cell extraction thresholds on the total energy map
    curb.extractionEnergyThreshold = 0.60;
    curb.extractionBaseEnergyThreshold = 0.18;
    curb.extractionLinearityMin = 0.20;
    curb.extractionComponentScoreMin = 0.12;
    curb.extractionCenterEvidenceMin = 0.75;
    curb.extractionHeightStepMinMeters = latticeTunedValue(spacing, 0.035, 0.050);
    curb.extractionStandaloneBaseEnergyThreshold = 0.35;
    curb.extractionStandaloneRoughnessMinMeters = latticeTunedValue(spacing, 0.003, 0.005);
    curb.extractionStandaloneBridgeBaseEnergyThreshold = 0.28;
    curb.extractionStandaloneBridgeCenterEvidenceMin = 0.75;
    curb.extractionStandaloneBridgeRadiusCells = latticeTunedValue(spacing, 2, 2);
    curb.extractionStandaloneBridgeMinSeedCount = 1;
    curb.extractionFillBaseEnergyThreshold = 0.12;
    curb.extractionFillHeightStepMinMeters = latticeTunedValue(spacing, 0.035, 0.050);
    curb.extractionFillGateMin = 0.15;
    curb.extractionFillSupportCountMin = latticeTunedValue(spacing, 4, 4);
    curb.extractionFillSeedCountMin = latticeTunedValue(spacing, 60, 60);
    curb.extractionFillValidRatioMin = 0.45;
    curb.extractionEndpointBaseEnergyThreshold = 0.18;
    curb.extractionEndpointHeightStepMinMeters = latticeTunedValue(spacing, 0.08, 0.10);
    curb.extractionEndpointCenterEvidenceMin = 0.75;
    curb.extractionEndpointSeedCountMin = latticeTunedValue(spacing, 25, 25);
    curb.extractionEndpointValidRatioMin = 0.40;

    % Road-adjacency refinement of curb cells (refineCurbCellsByRoadAdjacency stages)
    curb.roadAdjacencyFilterEnabled = true;
    curb.roadAdjacencyRadiusCells = latticeTunedValue(spacing, 4, 2);
    curb.roadAdjacencyComponentMinAdjacentCells = latticeTunedValue(spacing, 3, 2);
    curb.roadAdjacencyComponentMinAdjacentFraction = 0.15;
    curb.roadAdjacencyComponentMaxFullKeepCells = latticeTunedValue(spacing, 40, 14);
    curb.roadAdjacencySmallComponentStrongEnergyThreshold = 0.60;
    curb.roadAdjacencySmallComponentMinStrongCells = 1;
    curb.roadAdjacencyLargeComponentMinCells = latticeTunedValue(spacing, 180, 60);
    curb.roadAdjacencyRoadFacingAnchorEnabled = true;
    curb.roadAdjacencyRoadFacingAnchorRadiusCells = 0;
    curb.roadAdjacencyRoadReferenceRadiusCells = latticeTunedValue(spacing, 4, 2);
    curb.roadAdjacencyFeaturePeakAnchorEnabled = true;
    curb.roadAdjacencyBoundaryContinuationEnabled = true;
    curb.roadAdjacencyBoundaryContinuationRadiusCells = 0;
    curb.roadAdjacencyBoundaryContinuationMaxGapCells = latticeTunedValue(spacing, 6, 3);
    curb.roadAdjacencyBoundaryContinuationMinAnchorCols = latticeTunedValue(spacing, 8, 4);
    curb.roadAdjacencyExcludeRoadCells = false;
    curb.boundaryGapCompletionEnabled = true;
    curb.boundaryGapCompletionMaxGapCells = latticeTunedValue(spacing, 8, 4);
    curb.boundaryGapCompletionMinBaseEnergy = 0.28;
    curb.boundaryGapCompletionMinLinearity = 0.30;
    curb.boundaryGapCompletionMinHeightStepMeters = latticeTunedValue(spacing, 0.025, 0.033);
    curb.boundaryGapCompletionMinRoughnessMeters = latticeTunedValue(spacing, 0.003, 0.005);
    curb.boundaryPathCompletionEnabled = true;
    curb.boundaryPathCompletionProjectionRadiusCells = 0;
    curb.boundaryPathCompletionMaxGapCells = latticeTunedValue(spacing, 24, 12);
    curb.boundaryPathCompletionMaxSlopeRowsPerCol = 0.40;
    curb.boundaryPathCompletionRadiusCells = latticeTunedValue(spacing, 2, 1);
    curb.boundaryPathCompletionMinBaseEnergy = 0.04;
    curb.boundaryPathCompletionMinHeightStepMeters = latticeTunedValue(spacing, 0.014, 0.020);
    curb.boundaryPathCompletionMinLinearity = 0.05;
    curb.sameSideDuplicateSuppressionEnabled = true;
    curb.sameSideDuplicateMinOverlapFraction = 0.25;
    curb.sameSideDuplicateMinSeparationCells = latticeTunedValue(spacing, 2, 1);
    curb.sameColumnFeaturePeakSelectionEnabled = true;
    curb.sameColumnFeaturePeakSearchRadiusCells = latticeTunedValue(spacing, 2, 1);
    curb.sameColumnFeaturePeakMinBaseEnergy = 0.25;
    curb.sameColumnFeaturePeakMinHeightStepMeters = latticeTunedValue(spacing, 0.025, 0.033);
    curb.sameColumnFeaturePeakMinLinearity = 0.25;
    curb.sameColumnFeaturePeakPromotionMinHeightGainMeters = latticeTunedValue(spacing, 0.004, 0.005);
    curb.sameColumnFeaturePeakPromotionScoreRatio = 0.95;
    curb.sameColumnFeaturePeakPromotionBaseRatio = 0.75;
    curb.sameColumnFeaturePeakPromotionMinRoughnessMeters = latticeTunedValue(spacing, 0.010, 0.015);
    curb.sameColumnFeaturePeakPromotionRoughnessRatio = 1.50;
    curb.sameColumnFeaturePeakRoadFacingMinBaseEnergy = 0.32;
    curb.sameColumnFeaturePeakRoadFacingMinRoughnessMeters = latticeTunedValue(spacing, 0.020, 0.030);
    curb.sameColumnFeaturePeakRoadFacingMinLinearity = 0.32;
    curb.sameColumnFeaturePeakRoadFacingMinCenterEvidence = 0.90;
    curb.sameColumnFeaturePeakOuterProtectionMaxHeightGainMeters = latticeTunedValue(spacing, 0.012, 0.016);
    curb.sameColumnFeaturePeakSuppressionMinHeightGainMeters = latticeTunedValue(spacing, 0.012, 0.016);
    curb.sameColumnFeaturePeakSuppressionScoreRatio = 1.25;
    curb.sameSideCellShadowSuppressionEnabled = true;
    curb.sameSideCellShadowRadiusCells = 0;
    curb.sameSideCellShadowMinSeparationCells = 1.0;
    curb.sameSideCellShadowProtectedLinearityMin = 0.47;
    curb.sameSideCellShadowProtectedRoughnessMinMeters = latticeTunedValue(spacing, 0.010, 0.015);
    curb.sameSideCellShadowNegativeFeatureLinearityMin = 0.30;
    curb.sameSideCellShadowFeatureRoughnessScaleMeters = latticeTunedValue(spacing, 0.020, 0.030);
    curb.sameSideCellShadowDominantScoreRatio = 1.25;
    curb.sameSideCellShadowDominantRoughnessGainMeters = latticeTunedValue(spacing, 0.004, 0.006);
    curb.sameSideCellShadowSmoothRoughnessMaxMeters = latticeTunedValue(spacing, 0.010, 0.015);
    curb.sameSideCellShadowReliableCloserCenterEvidenceMin = 0.90;
    curb.sameSideCellShadowReliableCloserDominantTotalMin = 0.40;
    curb.roadFacingShoulderRecoveryEnabled = latticeTunedValue(spacing, true, false);
    curb.roadFacingShoulderRecoveryMinBaseEnergy = 0.55;
    curb.roadFacingShoulderRecoveryMinHeightStepMeters = latticeTunedValue(spacing, 0.055, 0.07);
    curb.roadFacingShoulderRecoveryMinLinearity = 0.45;
    curb.roadFacingShoulderRecoveryAnchorCenterEvidenceMin = 0.90;
    curb.roadFacingLineShoulderRecoveryMinBaseEnergy = 0.30;
    curb.roadFacingLineShoulderRecoveryMinHeightStepMeters = latticeTunedValue(spacing, 0.055, 0.07);
    curb.roadFacingLineShoulderRecoveryMinLinearity = 0.45;
    curb.roadAwayShoulderRecoveryMinBaseEnergy = 0.75;
    curb.roadAwayShoulderRecoveryMinTotalEnergy = 0.35;
    curb.roadAwayShoulderRecoveryMinHeightStepMeters = latticeTunedValue(spacing, 0.050, 0.065);
    curb.roadAwayShoulderRecoveryMinLinearity = 0.35;
    curb.roadAwayShoulderRecoveryAnchorRoughnessMinMeters = latticeTunedValue(spacing, 0.035, 0.05);
    curb.roadAwayLineShoulderRecoveryMinBaseEnergy = 0.45;
    curb.roadAwayLineShoulderRecoveryMinHeightStepMeters = latticeTunedValue(spacing, 0.055, 0.07);
    curb.roadAwayLineShoulderRecoveryMinLinearity = 0.45;
    curb.roadAwayLineShoulderRecoveryMinCenterEvidence = 0.90;
    curb.boundaryRunCompletionEnabled = true;
    curb.boundaryRunCompletionMaxGapCells = latticeTunedValue(spacing, 8, 4);
    curb.boundaryRunCompletionMaxEndpointCells = latticeTunedValue(spacing, 4, 2);
    curb.boundaryRunCompletionNoEvidenceMaxGapCells = latticeTunedValue(spacing, 2, 1);
    curb.boundaryRunCompletionMinCenterEvidence = 0.90;
    curb.boundaryRunCompletionMinBaseEnergy = 0.33;
    curb.boundaryRunCompletionMinHeightStepMeters = latticeTunedValue(spacing, 0.016, 0.021);
    curb.boundaryRunCompletionMinRoughnessMeters = latticeTunedValue(spacing, 0.035, 0.05);
    curb.boundaryRunCompletionMinLinearity = 0.25;
    curb.boundaryRunCompletionStepRoughMinBaseEnergy = 0.20;
    curb.boundaryRunCompletionStepRoughMinHeightStepMeters = latticeTunedValue(spacing, 0.080, 0.10);
    curb.boundaryRunCompletionStepRoughMinRoughnessMeters = latticeTunedValue(spacing, 0.045, 0.065);
    curb.boundaryRunCompletionLineStepMinBaseEnergy = 0.30;
    curb.boundaryRunCompletionLineStepMinHeightStepMeters = latticeTunedValue(spacing, 0.055, 0.07);
    curb.boundaryRunCompletionLineStepMinLinearity = 0.45;
    curb.boundaryRunCompletionLineStepMinCenterEvidence = 0.04;
    curb.boundaryRunCompletionStrongCenterLineStepMinBaseEnergy = 0.75;
    curb.boundaryRunCompletionStrongCenterLineStepMinHeightStepMeters = latticeTunedValue(spacing, 0.055, 0.07);
    curb.boundaryRunCompletionStrongCenterLineStepMinLinearity = 0.35;
    curb.boundaryRunCompletionRawLineStepMinCenterEvidence = 0.50;
    curb.boundaryRunCompletionRawLineStepMinBaseEnergy = 0.35;
    curb.boundaryRunCompletionRawLineStepMinHeightStepMeters = latticeTunedValue(spacing, 0.055, 0.07);
    curb.boundaryRunCompletionRawLineStepMinLinearity = 0.40;
    curb.boundaryRunCompletionEndpointLineStepMinCenterEvidence = 0.10;
    curb.boundaryRunCompletionEndpointLineStepMinBaseEnergy = 0.35;
    curb.boundaryRunCompletionEndpointLineStepMinHeightStepMeters = latticeTunedValue(spacing, 0.055, 0.07);
    curb.boundaryRunCompletionEndpointLineStepMinLinearity = 0.35;
    curb.sameSideRoadFacingDuplicateSuppressionEnabled = true;
    curb.sameSideRoadFacingDuplicateSearchRowsCells = latticeTunedValue(spacing, 2, 1);
    curb.sameSideRoadFacingDuplicateOuterCenterEvidenceMin = 0.90;
    curb.sameSideRoadFacingDuplicateOuterLinearityMin = 0.85;
    curb.sameSideRoadFacingDuplicateOuterStepLinearityMin = 0.35;
    curb.sameSideRoadFacingDuplicateInnerLinearityMax = 0.65;
    curb.sameSideRoadFacingDuplicateInnerStepLinearityMax = 0.48;
    curb.sameSideRoadFacingDuplicateInnerRoughnessMaxMeters = latticeTunedValue(spacing, 0.040, 0.06);
    curb.sameSideRoadFacingDuplicateInnerCenterEvidenceMax = 0.05;
    curb.sameSideRoadFacingDuplicateOuterBaseRatio = 0.80;
    curb.sameSideRoadFacingDuplicateOuterBaseGainRatio = 1.20;
    curb.sameSideRoadFacingDuplicateOuterHeightGainMeters = latticeTunedValue(spacing, 0.015, 0.02);
    curb.sameSideOutlierSuppressionEnabled = true;
    curb.sameSideOutlierReferenceMinCells = latticeTunedValue(spacing, 6, 3);
    curb.sameSideOutlierMaxComponentCells = latticeTunedValue(spacing, 8, 3);
    curb.sameSideOutlierMaxColumnSpanCells = latticeTunedValue(spacing, 24, 12);
    curb.sameSideOutlierMaxExtraDistanceCells = latticeTunedValue(spacing, 6, 3);
    curb.intrinsicComponentMinCells = Inf;
    curb.intrinsicComponentMaxCells = 0;
    curb.intrinsicComponentMinSeedCells = Inf;
    curb.continuationExpansionEnabled = false;
    curb.continuationExpansionRadiusCells = latticeTunedValue(spacing, 2, 1);
    curb.continuationBaseEnergyThreshold = 0.35;
    curb.continuationCenterEvidenceMin = 0.25;

    % Whole-pillar boundary continuation used by road-adjacency refinement.
    curb.dominantBoundaryContinuationEnabled = true;
    curb.dominantBoundaryContinuationDirection = "both";
    curb.dominantBoundaryContinuationMaxGapCells = latticeTunedValue(spacing, 8, 4);
    curb.dominantBoundaryContinuationSearchRows = 1;
    curb.dominantBoundaryContinuationMinTotalEnergy = 0.0;
    curb.dominantBoundaryContinuationMinXCenterMeters = 0.0;
    curb.dominantBoundaryContinuationMaxRelativeHeightMeters = 0.18;
    curb.dominantBoundaryContinuationMinBaseEnergy = 0.25;
    curb.dominantBoundaryContinuationMinHeightStepMeters = latticeTunedValue(spacing, 0.020, 0.026);
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

function road = roadSurfaceParameters(spacing)
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
    road.maxRoadRoughnessMeters = latticeTunedValue(spacing, 0.12, 0.04);
    road.curbInteriorConstraintEnabled = true;
    road.curbInteriorClearanceCells = 1;
    road.curbInteriorMinBoundaryCells = latticeTunedValue(spacing, 4, 2);
    road.curbInteriorMaxExtrapolationMeters = 6.0;
    road.seedXLimitsMeters = [3.0, 14.0];
    road.seedAbsYMaxMeters = 4.0;
    road.fallbackSeedEnabled = true;
    road.fallbackSeedReferenceXYMeters = [4.0, 0.0];
    road.maxNeighborHeightStepMeters = 0.10;
    road.maxSeedHeightDeviationMeters = 0.60;
    road.minRoadCells = latticeTunedValue(spacing, 8, 2);
end
