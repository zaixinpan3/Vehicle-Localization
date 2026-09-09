function cfg = finePerceptionConfig()
% finePerceptionConfig: Offline point validation in metric coordinates.
% Local 3D geometry produces thin curbs from unorganized points. Pole/facade
% distance rejection about fitted lines or planes. No online dependency.
    cfg = struct("poleMinimumHeight", 1.0, "poleMaximumRadius", 0.35, ...
        "poleMaximumTiltDegrees", 20, "poleMinimumPoints", 6, ...
        "robustScale", 3.0, "minimumResidualScale", 0.02, ...
        "poleMinimumSliceRatio", 0.60, "poleMinimumSupportRatio", 0.70, ...
        "poleMinimumSupportedHeight", 1.5, ...
        "poleSupportHeightResolution", 0.5, "poleSupportMinimumPoints", 3, ...
        "poleRecoveryEnabled", true, "poleRecoveryMinimumPoints", 12, "poleRecoveryMinimumHeight", 3.0, ...
        "poleRecoveryMaximumTiltDegrees", 2, "poleRecoveryMaximumRadialStd", 0.10, ...
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
    cfg.curbMaximumProposalAnchors = 128;
    cfg.curbMinimumBoundaryLengthMeters = 1.5;
    cfg.curbMaximumProposalLengthMeters = 25.0;
    cfg.curbMaximumSupportGapMeters = 3.0;
    cfg.curbConsensusBandMeters = 0.10;
    cfg.curbDuplicateBandMeters = 1.2;
    cfg.curbDuplicateEndpointMarginMeters = 3.0;
    cfg.curbSupportWeight = 0.2;
    cfg.curbProposalBatchSize = 256;
    cfg.curbMaximumCurvature = 0.05;
    cfg.curbCurveResidualRatio = 0.8;
    cfg.curbCurveExtensionMeters = 1.0;
    cfg.curbOutputBandMeters = 0.06;
    cfg.curbOutputSpacingMeters = 0.15;
end
