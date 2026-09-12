function cfg = finePerceptionConfig()
% finePerceptionConfig: Offline point validation in metric coordinates.
% Local 3D geometry produces thin curbs from unorganized points. Pole/facade
% distance rejection about fitted lines or planes. No online dependency.
    cfg = struct("poleMinimumHeight", 1.0, "poleMaximumVerticalGapMeters", 0.75, "poleMaximumRadius", 0.35, ...
        "poleMaximumTiltDegrees", 20, "poleMinimumPoints", 6, ...
        "poleWideSurfaceMinimumAxisStd", 0.12, "poleWideSurfaceMinimumAspectRatio", 3.0, ...
        "robustScale", 3.0, "minimumResidualScale", 0.02, ...
        "poleMinimumSliceRatio", 0.60, "poleMinimumSupportRatio", 0.70, ...
        "poleMinimumSupportedHeight", 1.5, ...
        "poleShortSupportHeight", 2.0, "poleShortSupportMaximumTiltDegrees", 6, "poleShortSupportMaximumRadialRms", 0.10, ...
        "poleShortSupportWideSurfaceMinimumAxisStd", 0.09, ...
        "poleLowContrastSupportRatio", 0.80, "poleLowContrastMaximumRadius", 0.10, ...
        "poleIsolationCoreRadius", 0.25, "poleIsolationNeighborhoodRadius", 0.75, ...
        "poleIsolationMinimumCoreFraction", 0.80, ...
        "poleDenseCoreMinimumPoints", 100, "poleDenseCoreMinimumDensityContrast", 10, ...
        "poleSupportHeightResolution", 0.5, "poleSupportMinimumPoints", 3, ...
        "poleRecoveryEnabled", true, "poleRecoveryMinimumPoints", 12, "poleRecoveryMinimumHeight", 3.0, ...
        "poleRecoveryMaximumTiltDegrees", 3, "poleRecoveryMaximumRadialStd", 0.10, ...
        "poleRecoveryMaximumNeighborAxisRms", 0.15, ...
        "poleIndependentMaximumNeighborAxisRms", 0.25, ...
        "poleIndependentMinimumCoreFraction", 0.95, "poleIndependentMaximumRadialRms", 0.12, ...
        "poleSparseRecoveryMinimumHeight", 4.0, "poleSparseRecoveryMaximumTiltDegrees", 1.0, ...
        "poleSparseRecoveryMaximumRadialRms", 0.06, "poleBoundaryMinimumCountRatio", 3.0, ...
        "facadeMinimumPoints", 12, "facadeMinimumHeight", 1.0, ...
        "facadeMinimumLength", 1.0, "facadeMaximumDistance", 0.20, ...
        "facadeMaximumNormalAngleDegrees", 20);
    % Unorganized curb geometry and metric boundary sampling.
    cfg.curbCandidateRadiusCells = 1;
    cfg.curbNeighborhoodRadiusMeters = 0.65;
    cfg.curbMinimumNeighbors = 8;
    cfg.curbMinimumReliefMeters = 0.07;
    cfg.curbMaximumReliefMeters = 0.30;
    cfg.curbMinimumPlaneResidualMeters = 0.018;
    cfg.curbMinimumOutputPlaneResidualMeters = 0.020;
    cfg.curbMinimumSurfaceNeighbors = 6;
    cfg.curbMaximumSurfaceResidualMeters = 0.025;
    cfg.curbMaximumSurfaceSlope = 0.35;
    cfg.curbMinimumSlope = 0.08;
    cfg.curbLocalStripHalfWidthMeters = 0.12;
    cfg.curbMinimumStripNeighbors = 4;
    cfg.curbSeedMidHeightBandMeters = 0.06;
    cfg.curbMidHeightBandMeters = 0.07;
    cfg.curbMinimumSeedScore = 0.1;
    cfg.curbProposalCellSizeMeters = 0.30;
    cfg.curbMinimumSupportCells = 6;
    cfg.curbMinimumOutputSupportCells = 3;
    cfg.curbGradientReversalFraction = 0.80;
    cfg.curbBoundaryTangentRadiusMeters = 2.0;
    cfg.curbMaximumBoundaryNormalAngleDegrees = 45;
    cfg.curbMaximumEndpointNormalAngleDegrees = 20;
    cfg.curbMinimumBoundaryNormalFraction = 0.60;
    cfg.curbMaximumProposalAnchors = 128;
    cfg.curbMinimumBoundaryLengthMeters = 1.5;
    cfg.curbMaximumProposalLengthMeters = 25.0;
    cfg.curbMaximumSupportGapMeters = 3.0;
    cfg.curbConsensusBandMeters = 0.10;
    cfg.curbDuplicateBandMeters = 1.2;
    cfg.curbCompetingEdgeRadiusMeters = 1.5;
    cfg.curbCompetingEdgeMinimumSeparationMeters = 0.2;
    cfg.curbCompetingEdgeGradientRatio = 1.4;
    cfg.curbCompetingEdgeScoreWeight = 2.0;
    cfg.curbAlternativeEdgeGradientRatio = 1.0;
    cfg.curbCompetingBoundaryMinimumFraction = 0.60;
    cfg.curbRidgeMaximumGapMeters = 0.75;
    cfg.curbRidgeMaximumNormalAngleDegrees = 40;
    cfg.curbDuplicateEndpointMarginMeters = 3.0;
    cfg.curbSupportWeight = 0.2;
    cfg.curbProposalBatchSize = 256;
    cfg.curbMaximumCurvature = 0.05;
    cfg.curbCurveResidualRatio = 0.8;
    cfg.curbCurveExtensionMeters = 1.0;
    cfg.curbOutputBandMeters = 0.06;
    cfg.curbOutputSpacingMeters = 0.15;
    % Fine-only search beyond supported boundary endpoints, in metric XYZ.
    cfg.curbContinuationLengthMeters = 12.0;
    cfg.curbContinuationHalfWidthMeters = 1.0;
    cfg.curbContinuationMaximumGapMeters = 0.75;
    cfg.curbContinuationMaximumAngleDegrees = 30;
    cfg.curbContinuationTangentLengthMeters = 2.0;
    cfg.curbContinuationAlongRadiusMeters = 2.0;
end
