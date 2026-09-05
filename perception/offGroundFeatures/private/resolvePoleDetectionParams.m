function params = resolvePoleDetectionParams(cfg)
% resolvePoleDetectionParams: Read and sanitize the unified 2D pole
% candidate parameters with compact defaults and optional compatibility
% fallbacks.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   params: struct with validated pole-detection parameters and kernels
    persistent cachedConfig cachedParams
    if ~isempty(cachedParams) && isequaln(cachedConfig,cfg)
        params = cachedParams;
        return;
    end
    params = struct();
    params.baseConfidenceThreshold = 0.55;
    if isfield(cfg, "poleBaseConfidenceThreshold") && isscalar(cfg.poleBaseConfidenceThreshold) && isfinite(cfg.poleBaseConfidenceThreshold)
        params.baseConfidenceThreshold = double(cfg.poleBaseConfidenceThreshold);
    elseif isfield(cfg, "poleThresholdAbs") && isscalar(cfg.poleThresholdAbs) && isfinite(cfg.poleThresholdAbs)
        params.baseConfidenceThreshold = double(cfg.poleThresholdAbs);
    end
    params.baseConfidenceThreshold = min(max(params.baseConfidenceThreshold, 0), 1);

    params.supportConfidenceThreshold = 0.35;
    if isfield(cfg, "poleSupportConfidenceThreshold") && isscalar(cfg.poleSupportConfidenceThreshold) && isfinite(cfg.poleSupportConfidenceThreshold)
        params.supportConfidenceThreshold = double(cfg.poleSupportConfidenceThreshold);
    end
    params.supportConfidenceThreshold = min(max(params.supportConfidenceThreshold, 0), params.baseConfidenceThreshold);

    params.seedRunLayerThreshold = 2;
    if isfield(cfg, "poleSeedRunLayerThreshold") && isscalar(cfg.poleSeedRunLayerThreshold) && isfinite(cfg.poleSeedRunLayerThreshold)
        params.seedRunLayerThreshold = max(1, round(double(cfg.poleSeedRunLayerThreshold)));
    end

    params.supportRunLayerThreshold = params.seedRunLayerThreshold;
    if isfield(cfg, "poleSupportRunLayerThreshold") && isscalar(cfg.poleSupportRunLayerThreshold) && isfinite(cfg.poleSupportRunLayerThreshold)
        params.supportRunLayerThreshold = max(1, round(double(cfg.poleSupportRunLayerThreshold)));
    end
    params.supportRunLayerThreshold = min(params.supportRunLayerThreshold, params.seedRunLayerThreshold);

    params.patternSupportRunLayerThreshold = params.seedRunLayerThreshold;
    if isfield(cfg, "polePatternSupportRunLayerThreshold") && isscalar(cfg.polePatternSupportRunLayerThreshold) && isfinite(cfg.polePatternSupportRunLayerThreshold)
        params.patternSupportRunLayerThreshold = max(1, round(double(cfg.polePatternSupportRunLayerThreshold)));
    end

    params.patternNeighborRunLayerThreshold = 1;
    if isfield(cfg, "polePatternNeighborRunLayerThreshold") && isscalar(cfg.polePatternNeighborRunLayerThreshold) && isfinite(cfg.polePatternNeighborRunLayerThreshold)
        params.patternNeighborRunLayerThreshold = max(1, round(double(cfg.polePatternNeighborRunLayerThreshold)));
    end

    params.candidateMinRunLayerThreshold = 1;
    if isfield(cfg, "poleCandidateMinRunLayerThreshold") && isscalar(cfg.poleCandidateMinRunLayerThreshold) && isfinite(cfg.poleCandidateMinRunLayerThreshold)
        params.candidateMinRunLayerThreshold = max(1, round(double(cfg.poleCandidateMinRunLayerThreshold)));
    end

    params.coreMinRunLayerThreshold = 4;
    if isfield(cfg, "poleCoreMinRunLayerThreshold") && isscalar(cfg.poleCoreMinRunLayerThreshold) && ...
            isfinite(cfg.poleCoreMinRunLayerThreshold)
        params.coreMinRunLayerThreshold = max(1, round(double(cfg.poleCoreMinRunLayerThreshold)));
    end
    params.coreMaxLineScore = 0.05;
    if isfield(cfg, "poleCoreMaxLineScore") && isscalar(cfg.poleCoreMaxLineScore) && ...
            isfinite(cfg.poleCoreMaxLineScore)
        params.coreMaxLineScore = min(max(double(cfg.poleCoreMaxLineScore), 0), 1);
    end
    params.componentSupportMinOccupiedLayers = 2;
    if isfield(cfg, "poleComponentSupportMinOccupiedLayers") && isscalar(cfg.poleComponentSupportMinOccupiedLayers) && ...
            isfinite(cfg.poleComponentSupportMinOccupiedLayers)
        params.componentSupportMinOccupiedLayers = max(1, round(double(cfg.poleComponentSupportMinOccupiedLayers)));
    end
    params.componentContextMinRunLayerRatio = 0.70;
    if isfield(cfg, "poleComponentContextMinRunLayerRatio") && isscalar(cfg.poleComponentContextMinRunLayerRatio) && ...
            isfinite(cfg.poleComponentContextMinRunLayerRatio)
        params.componentContextMinRunLayerRatio = max(0, double(cfg.poleComponentContextMinRunLayerRatio));
    end
    params.splitLayerMinMemberOccupiedLayers = 3;
    if isfield(cfg, "poleSplitLayerMinMemberOccupiedLayers") && isscalar(cfg.poleSplitLayerMinMemberOccupiedLayers) && ...
            isfinite(cfg.poleSplitLayerMinMemberOccupiedLayers)
        params.splitLayerMinMemberOccupiedLayers = max(1, round(double(cfg.poleSplitLayerMinMemberOccupiedLayers)));
    end
    params.splitLayerMinCombinedOccupiedLayers = 7;
    if isfield(cfg, "poleSplitLayerMinCombinedOccupiedLayers") && isscalar(cfg.poleSplitLayerMinCombinedOccupiedLayers) && ...
            isfinite(cfg.poleSplitLayerMinCombinedOccupiedLayers)
        params.splitLayerMinCombinedOccupiedLayers = max(1, round(double(cfg.poleSplitLayerMinCombinedOccupiedLayers)));
    end
    params.splitLayerMinCombinedRunLayers = 3;
    if isfield(cfg, "poleSplitLayerMinCombinedRunLayers") && isscalar(cfg.poleSplitLayerMinCombinedRunLayers) && ...
            isfinite(cfg.poleSplitLayerMinCombinedRunLayers)
        params.splitLayerMinCombinedRunLayers = max(1, round(double(cfg.poleSplitLayerMinCombinedRunLayers)));
    end
    params.splitLayerMinPointScore = 0.70;
    if isfield(cfg, "poleSplitLayerMinPointScore") && isscalar(cfg.poleSplitLayerMinPointScore) && ...
            isfinite(cfg.poleSplitLayerMinPointScore)
        params.splitLayerMinPointScore = min(max(double(cfg.poleSplitLayerMinPointScore), 0), 1);
    end
    params.splitLayerMaxLineScore = 0.35;
    if isfield(cfg, "poleSplitLayerMaxLineScore") && isscalar(cfg.poleSplitLayerMaxLineScore) && ...
            isfinite(cfg.poleSplitLayerMaxLineScore)
        params.splitLayerMaxLineScore = min(max(double(cfg.poleSplitLayerMaxLineScore), 0), 1);
    end
    params.runLayerContrastThreshold = 0.5;
    if isfield(cfg, "poleRunLayerContrastThreshold") && isscalar(cfg.poleRunLayerContrastThreshold) && isfinite(cfg.poleRunLayerContrastThreshold)
        params.runLayerContrastThreshold = min(max(double(cfg.poleRunLayerContrastThreshold), 0), 1);
    end
    params.dominantSeedMinPointScore = 0.40;
    if isfield(cfg, "poleDominantSeedMinPointScore") && isscalar(cfg.poleDominantSeedMinPointScore) && isfinite(cfg.poleDominantSeedMinPointScore)
        params.dominantSeedMinPointScore = min(max(double(cfg.poleDominantSeedMinPointScore), 0), 1);
    end
    params.dominantSeedMinRunLayerDrop = 2;
    if isfield(cfg, "poleDominantSeedMinRunLayerDrop") && isscalar(cfg.poleDominantSeedMinRunLayerDrop) && isfinite(cfg.poleDominantSeedMinRunLayerDrop)
        params.dominantSeedMinRunLayerDrop = max(0, double(cfg.poleDominantSeedMinRunLayerDrop));
    end
    params.dominantSeedMinRunLayerRatio = 1.5;
    if isfield(cfg, "poleDominantSeedMinRunLayerRatio") && isscalar(cfg.poleDominantSeedMinRunLayerRatio) && isfinite(cfg.poleDominantSeedMinRunLayerRatio)
        params.dominantSeedMinRunLayerRatio = max(1, double(cfg.poleDominantSeedMinRunLayerRatio));
    end
    params.seedDominanceRadiusVoxels = 2;
    if isfield(cfg, "poleSeedDominanceRadiusVoxels") && isscalar(cfg.poleSeedDominanceRadiusVoxels) && isfinite(cfg.poleSeedDominanceRadiusVoxels)
        params.seedDominanceRadiusVoxels = max(0, round(double(cfg.poleSeedDominanceRadiusVoxels)));
    end
    params.seedDominanceMinRunLayerDrop = params.dominantSeedMinRunLayerDrop;
    if isfield(cfg, "poleSeedDominanceMinRunLayerDrop") && isscalar(cfg.poleSeedDominanceMinRunLayerDrop) && isfinite(cfg.poleSeedDominanceMinRunLayerDrop)
        params.seedDominanceMinRunLayerDrop = max(0, double(cfg.poleSeedDominanceMinRunLayerDrop));
    end
    params.seedDominanceMinRunLayerRatio = params.dominantSeedMinRunLayerRatio;
    if isfield(cfg, "poleSeedDominanceMinRunLayerRatio") && isscalar(cfg.poleSeedDominanceMinRunLayerRatio) && isfinite(cfg.poleSeedDominanceMinRunLayerRatio)
        params.seedDominanceMinRunLayerRatio = max(1, double(cfg.poleSeedDominanceMinRunLayerRatio));
    end
    params.seedDominanceMaxNearRunLayerNeighborCount = 2;
    if isfield(cfg, "poleSeedDominanceMaxNearRunLayerNeighbors") && isscalar(cfg.poleSeedDominanceMaxNearRunLayerNeighbors) && ...
            isfinite(cfg.poleSeedDominanceMaxNearRunLayerNeighbors)
        params.seedDominanceMaxNearRunLayerNeighborCount = max(0, round(double(cfg.poleSeedDominanceMaxNearRunLayerNeighbors)));
    end
    params.compactSeedRunLayerThreshold = max(params.seedRunLayerThreshold, 6);
    if isfield(cfg, "poleCompactSeedRunLayerThreshold") && isscalar(cfg.poleCompactSeedRunLayerThreshold) && ...
            isfinite(cfg.poleCompactSeedRunLayerThreshold)
        params.compactSeedRunLayerThreshold = max(1, round(double(cfg.poleCompactSeedRunLayerThreshold)));
    end
    params.compactSeedMinPointScore = 0.20;
    if isfield(cfg, "poleCompactSeedMinPointScore") && isscalar(cfg.poleCompactSeedMinPointScore) && ...
            isfinite(cfg.poleCompactSeedMinPointScore)
        params.compactSeedMinPointScore = min(max(double(cfg.poleCompactSeedMinPointScore), 0), 1);
    end
    params.compactSeedMaxNearRunLayerNeighborCount = 3;
    if isfield(cfg, "poleCompactSeedMaxNearRunLayerNeighbors") && isscalar(cfg.poleCompactSeedMaxNearRunLayerNeighbors) && ...
            isfinite(cfg.poleCompactSeedMaxNearRunLayerNeighbors)
        params.compactSeedMaxNearRunLayerNeighborCount = max(0, round(double(cfg.poleCompactSeedMaxNearRunLayerNeighbors)));
    end
    params.candidateSingletonMinColumnPoints = 15;
    if isfield(cfg, "poleCandidateSingletonMinColumnPoints") && isscalar(cfg.poleCandidateSingletonMinColumnPoints) && ...
            isfinite(cfg.poleCandidateSingletonMinColumnPoints)
        params.candidateSingletonMinColumnPoints = max(1, round(double(cfg.poleCandidateSingletonMinColumnPoints)));
    end
    params.candidateSingletonContextRadiusVoxels = 4;
    if isfield(cfg, "poleCandidateSingletonContextRadiusVoxels") && isscalar(cfg.poleCandidateSingletonContextRadiusVoxels) && ...
            isfinite(cfg.poleCandidateSingletonContextRadiusVoxels)
        params.candidateSingletonContextRadiusVoxels = max(0, round(double(cfg.poleCandidateSingletonContextRadiusVoxels)));
    end
    params.candidateSingletonContextMinRunLayerRatio = 2.0;
    if isfield(cfg, "poleCandidateSingletonContextMinRunLayerRatio") && isscalar(cfg.poleCandidateSingletonContextMinRunLayerRatio) && ...
            isfinite(cfg.poleCandidateSingletonContextMinRunLayerRatio)
        params.candidateSingletonContextMinRunLayerRatio = max(1, double(cfg.poleCandidateSingletonContextMinRunLayerRatio));
    end
    params.candidateSingletonLowSupportMaxColumnPoints = 50;
    if isfield(cfg, "poleCandidateSingletonLowSupportMaxColumnPoints") && isscalar(cfg.poleCandidateSingletonLowSupportMaxColumnPoints) && ...
            isfinite(cfg.poleCandidateSingletonLowSupportMaxColumnPoints)
        params.candidateSingletonLowSupportMaxColumnPoints = max(1, round(double(cfg.poleCandidateSingletonLowSupportMaxColumnPoints)));
    end
    params.candidateSingletonLowSupportMaxContextRunSum = 30;
    if isfield(cfg, "poleCandidateSingletonLowSupportMaxContextRunSum") && isscalar(cfg.poleCandidateSingletonLowSupportMaxContextRunSum) && ...
            isfinite(cfg.poleCandidateSingletonLowSupportMaxContextRunSum)
        params.candidateSingletonLowSupportMaxContextRunSum = max(0, double(cfg.poleCandidateSingletonLowSupportMaxContextRunSum));
    end
    params.candidateSingletonLowSupportMinRunToContextSumRatio = 0.30;
    if isfield(cfg, "poleCandidateSingletonLowSupportMinRunToContextSumRatio") && isscalar(cfg.poleCandidateSingletonLowSupportMinRunToContextSumRatio) && ...
            isfinite(cfg.poleCandidateSingletonLowSupportMinRunToContextSumRatio)
        params.candidateSingletonLowSupportMinRunToContextSumRatio = max(0, double(cfg.poleCandidateSingletonLowSupportMinRunToContextSumRatio));
    end
    params.candidatePairMaxContextRunSum = 6;
    if isfield(cfg, "poleCandidatePairMaxContextRunSum") && isscalar(cfg.poleCandidatePairMaxContextRunSum) && ...
            isfinite(cfg.poleCandidatePairMaxContextRunSum)
        params.candidatePairMaxContextRunSum = max(0, double(cfg.poleCandidatePairMaxContextRunSum));
    end
    params.candidatePairMinRunToContextSumRatio = 2.0;
    if isfield(cfg, "poleCandidatePairMinRunToContextSumRatio") && isscalar(cfg.poleCandidatePairMinRunToContextSumRatio) && ...
            isfinite(cfg.poleCandidatePairMinRunToContextSumRatio)
        params.candidatePairMinRunToContextSumRatio = max(0, double(cfg.poleCandidatePairMinRunToContextSumRatio));
    end
    params.candidatePairImbalancedMaxMemberRunRatio = 0.65;
    if isfield(cfg, "poleCandidatePairImbalancedMaxMemberRunRatio") && isscalar(cfg.poleCandidatePairImbalancedMaxMemberRunRatio) && ...
            isfinite(cfg.poleCandidatePairImbalancedMaxMemberRunRatio)
        params.candidatePairImbalancedMaxMemberRunRatio = min(max(double(cfg.poleCandidatePairImbalancedMaxMemberRunRatio), 0), 1);
    end
    params.candidatePairImbalancedMinDominantRunLayerDrop = 4.0;
    if isfield(cfg, "poleCandidatePairImbalancedMinDominantRunLayerDrop") && isscalar(cfg.poleCandidatePairImbalancedMinDominantRunLayerDrop) && ...
            isfinite(cfg.poleCandidatePairImbalancedMinDominantRunLayerDrop)
        params.candidatePairImbalancedMinDominantRunLayerDrop = max(0, double(cfg.poleCandidatePairImbalancedMinDominantRunLayerDrop));
    end
    params.candidatePairContextRadiusVoxels = 2;
    if isfield(cfg, "poleCandidatePairContextRadiusVoxels") && isscalar(cfg.poleCandidatePairContextRadiusVoxels) && ...
            isfinite(cfg.poleCandidatePairContextRadiusVoxels)
        params.candidatePairContextRadiusVoxels = max(1, round(double(cfg.poleCandidatePairContextRadiusVoxels)));
    end
    params.candidatePairContextMinRunLayerDrop = 2.0;
    if isfield(cfg, "poleCandidatePairContextMinRunLayerDrop") && isscalar(cfg.poleCandidatePairContextMinRunLayerDrop) && ...
            isfinite(cfg.poleCandidatePairContextMinRunLayerDrop)
        params.candidatePairContextMinRunLayerDrop = max(0, double(cfg.poleCandidatePairContextMinRunLayerDrop));
    end
    params.splitPatternCoreRunLayerThreshold = 8;
    if isfield(cfg, "poleSplitPatternCoreRunLayerThreshold") && isscalar(cfg.poleSplitPatternCoreRunLayerThreshold) && ...
            isfinite(cfg.poleSplitPatternCoreRunLayerThreshold)
        params.splitPatternCoreRunLayerThreshold = max(1, round(double(cfg.poleSplitPatternCoreRunLayerThreshold)));
    end
    params.splitPatternMultiSeedCoreRunLayerThreshold = 8;
    if isfield(cfg, "poleSplitPatternMultiSeedCoreRunLayerThreshold") && isscalar(cfg.poleSplitPatternMultiSeedCoreRunLayerThreshold) && ...
            isfinite(cfg.poleSplitPatternMultiSeedCoreRunLayerThreshold)
        params.splitPatternMultiSeedCoreRunLayerThreshold = max(1, round(double(cfg.poleSplitPatternMultiSeedCoreRunLayerThreshold)));
    end
    params.splitPatternNeighborMaxRunLayerDrop = 1;
    if isfield(cfg, "poleSplitPatternNeighborMaxRunLayerDrop") && isscalar(cfg.poleSplitPatternNeighborMaxRunLayerDrop) && ...
            isfinite(cfg.poleSplitPatternNeighborMaxRunLayerDrop)
        params.splitPatternNeighborMaxRunLayerDrop = max(0, double(cfg.poleSplitPatternNeighborMaxRunLayerDrop));
    end
    params.multiCellMeanRunLayerMinRatio = 2.0;
    if isfield(cfg, "poleMultiCellMeanRunLayerMinRatio") && isscalar(cfg.poleMultiCellMeanRunLayerMinRatio) && ...
            isfinite(cfg.poleMultiCellMeanRunLayerMinRatio)
        params.multiCellMeanRunLayerMinRatio = max(1, double(cfg.poleMultiCellMeanRunLayerMinRatio));
    end
    params.multiCellMemberMinInternalRunRatio = 0.30;
    if isfield(cfg, "poleMultiCellMemberMinInternalRunRatio") && isscalar(cfg.poleMultiCellMemberMinInternalRunRatio) && ...
            isfinite(cfg.poleMultiCellMemberMinInternalRunRatio)
        params.multiCellMemberMinInternalRunRatio = min(max(double(cfg.poleMultiCellMemberMinInternalRunRatio), 0), 1);
    end
    params.multiCellMemberRunLayerMinDrop = 1.0;
    if isfield(cfg, "poleMultiCellMemberRunLayerMinDrop") && isscalar(cfg.poleMultiCellMemberRunLayerMinDrop) && ...
            isfinite(cfg.poleMultiCellMemberRunLayerMinDrop)
        params.multiCellMemberRunLayerMinDrop = max(0, double(cfg.poleMultiCellMemberRunLayerMinDrop));
    end
    params.multiCellMemberRunLayerMinRatio = 1.25;
    if isfield(cfg, "poleMultiCellMemberRunLayerMinRatio") && isscalar(cfg.poleMultiCellMemberRunLayerMinRatio) && ...
            isfinite(cfg.poleMultiCellMemberRunLayerMinRatio)
        params.multiCellMemberRunLayerMinRatio = max(1, double(cfg.poleMultiCellMemberRunLayerMinRatio));
    end
    params.pairMinRunLayerDrop = params.dominantSeedMinRunLayerDrop;
    params.pairMinRunLayerRatio = params.dominantSeedMinRunLayerRatio;

    params.neighborhoodRadius = 2;
    if isfield(cfg, "poleAdaptiveNeighborhoodRadiusVoxels") && isscalar(cfg.poleAdaptiveNeighborhoodRadiusVoxels) && isfinite(cfg.poleAdaptiveNeighborhoodRadiusVoxels)
        params.neighborhoodRadius = max(0, round(double(cfg.poleAdaptiveNeighborhoodRadiusVoxels)));
    elseif isfield(cfg, "poleRefineHorizontalStitchRadiusVoxels") && isscalar(cfg.poleRefineHorizontalStitchRadiusVoxels) && isfinite(cfg.poleRefineHorizontalStitchRadiusVoxels)
        params.neighborhoodRadius = max(0, round(double(cfg.poleRefineHorizontalStitchRadiusVoxels)));
    end

    params.countQuantiles = [0.25, 0.90];
    if isfield(cfg, "poleCountQuantiles") && numel(cfg.poleCountQuantiles) == 2 && all(isfinite(cfg.poleCountQuantiles(:)))
        params.countQuantiles = double(cfg.poleCountQuantiles(:).');
    end
    params.rangeQuantiles = [0.25, 0.85];
    if isfield(cfg, "poleRangeQuantiles") && numel(cfg.poleRangeQuantiles) == 2 && all(isfinite(cfg.poleRangeQuantiles(:)))
        params.rangeQuantiles = double(cfg.poleRangeQuantiles(:).');
    end
    params.energyExponent = 0.5;
    if isfield(cfg, "poleEnergyAlpha") && isscalar(cfg.poleEnergyAlpha) && isfinite(cfg.poleEnergyAlpha)
        params.energyExponent = min(max(double(cfg.poleEnergyAlpha), 0), 1);
    elseif isfield(cfg, "poleEnergyExponent") && isscalar(cfg.poleEnergyExponent) && isfinite(cfg.poleEnergyExponent)
        params.energyExponent = min(max(double(cfg.poleEnergyExponent), 0), 1);
    end
    params.energyWeight = 0.5;
    if isfield(cfg, "poleEnergyWeight") && isscalar(cfg.poleEnergyWeight) && isfinite(cfg.poleEnergyWeight)
        params.energyWeight = min(max(double(cfg.poleEnergyWeight), 0), 1);
    end
    params.maxFootprintSpanCells = 2;
    if isfield(cfg, "poleMaxFootprintSpanCells") && isscalar(cfg.poleMaxFootprintSpanCells) && isfinite(cfg.poleMaxFootprintSpanCells)
        params.maxFootprintSpanCells = max(1, round(double(cfg.poleMaxFootprintSpanCells)));
    end
    params.neighborhoodKernel = true((2 * params.neighborhoodRadius) + 1, (2 * params.neighborhoodRadius) + 1);
    cachedConfig = cfg;
    cachedParams = params;
end
