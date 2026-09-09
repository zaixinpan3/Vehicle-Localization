function candidates = detectPoleCandidates(columnMaps, facadeMask, fineVoxelGrid, params)
% detectPoleCandidates: OFFLINE ONLY detailed analysis of pole-like
% structures. Columns not claimed by facades are eligible. Core columns
% combine a long contiguous occupied z-run with low local line-likeness;
% support columns are the occupied neighbors with enough occupied layers;
% cores whose vertical support is split across two adjacent columns are
% recovered from combined layer evidence. Each core is grown into the
% smallest footprint (singleton, then edge-adjacent pair) that dominates the
% run-layer evidence of its local context, footprints are kept only when
% they are compact and their run-layer share exceeds the context contrast
% threshold, and finally each component must be supported by enough z
% layers with a sufficient component-wide point count.
%
% Input:
%   columnMaps: fine-analysis column statistics extended with
%       runLayerMap, pointScore, and lineScore
%   facadeMask: [Ny x Nx] logical columns already assigned to facades
%   fineVoxelGrid: sparse support from analyzeFineStructuralCandidates
%   params: struct from resolvePoleDetectionParams
%
% Output:
%   candidates: struct with eligibleMask, coreMask, supportMask,
%       splitLayerCoreMask, splitLayerCandidateMask, coreSeedMask,
%       footprintCandidateMask, contextCandidateMask, contextRunLayerRatio,
%       componentRunLayerSum, contextRunLayerSum, compactFootprintMask, and
%       candidateMask (the raw pole mask passed to 3D refinement)
    runLayerMap = columnMaps.runLayerMap;
    eligibleMask = columnMaps.occupiedMask & ~facadeMask;
    layerCountMap = single(columnMaps.occupiedLayerCount);

    coreMask = buildPoleCoreMask(eligibleMask, runLayerMap, columnMaps.lineScore, params);
    supportMask = buildPoleSupportMask(eligibleMask, layerCountMap, params);
    [splitLayerCoreMask, splitLayerCandidateMask] = buildSplitLayerPoleCandidates( ...
        eligibleMask, supportMask, layerCountMap, runLayerMap, columnMaps.lineScore, columnMaps.pointScore, params);
    footprintCfg=struct('minimumContextFraction',params.componentContextMinRunLayerRatio, ...
        'maximumFootprintSpanCells',params.maxFootprintSpanCells);
    [footprintCandidateMask,contextCandidateMask,compactFootprintMask, ...
        contextRunLayerRatio,componentRunLayerSum,contextRunLayerSum]=selectPillarFootprints( ...
        coreMask,supportMask,layerCountMap,runLayerMap,footprintCfg,splitLayerCandidateMask);
    candidateMask = filterPoleCandidatesByObjectLayerSupport(compactFootprintMask, fineVoxelGrid, params);

    candidates = struct();
    candidates.eligibleMask = eligibleMask;
    candidates.coreMask = coreMask;
    candidates.supportMask = supportMask;
    candidates.splitLayerCoreMask = splitLayerCoreMask;
    candidates.splitLayerCandidateMask = splitLayerCandidateMask;
    candidates.coreSeedMask = coreMask | splitLayerCoreMask;
    candidates.footprintCandidateMask = footprintCandidateMask;
    candidates.contextCandidateMask = contextCandidateMask;
    candidates.contextRunLayerRatio = single(contextRunLayerRatio);
    candidates.componentRunLayerSum = single(componentRunLayerSum);
    candidates.contextRunLayerSum = single(contextRunLayerSum);
    candidates.compactFootprintMask = compactFootprintMask;
    candidates.candidateMask = candidateMask;
end

function coreMask = buildPoleCoreMask(eligibleMask, coreRunLayerMap, lineScore, params)
% buildPoleCoreMask: Select pole core cells directly from
% high vertical run-layer evidence and low local second-moment line score,
% avoiding the older outer-ring layer-count comparison used before local
% line-likeness was available.
%
% Input:
%   eligibleMask: [Ny x Nx] logical cells allowed to seed pole candidates
%   coreRunLayerMap: [Ny x Nx] numeric maximum contiguous qualified
%       occupied-layer count map
%   lineScore: [Ny x Nx] numeric local weighted second-moment line score
%   params: struct from resolvePoleDetectionParams with
%       coreMinRunLayerThreshold and coreMaxLineScore
%
% Output:
%   coreMask: [Ny x Nx] logical low-linearity high-run pole-core mask
    coreMask = false(size(eligibleMask));
    if isempty(eligibleMask) || isempty(coreRunLayerMap) || isempty(lineScore) || ...
            ~isequal(size(eligibleMask), size(coreRunLayerMap)) || ~isequal(size(eligibleMask), size(lineScore))
        return;
    end

    minCoreRunLayers = 4;
    if isfield(params, "coreMinRunLayerThreshold") && isscalar(params.coreMinRunLayerThreshold) && ...
            isfinite(params.coreMinRunLayerThreshold)
        minCoreRunLayers = max(0, double(params.coreMinRunLayerThreshold));
    end
    maxCoreLineScore = 0.05;
    if isfield(params, "coreMaxLineScore") && isscalar(params.coreMaxLineScore) && isfinite(params.coreMaxLineScore)
        maxCoreLineScore = min(max(double(params.coreMaxLineScore), 0), 1);
    end

    coreRunLayerMap = double(coreRunLayerMap);
    coreRunLayerMap(~isfinite(coreRunLayerMap)) = 0;
    lineScore = double(lineScore);
    lineScore(~isfinite(lineScore)) = 1;
    coreMask = logical(eligibleMask) & (coreRunLayerMap >= minCoreRunLayers) & (lineScore < maxCoreLineScore);
end

function supportMask = buildPoleSupportMask(eligibleMask, layerCountMap, params)
% buildPoleSupportMask: Select cells that can participate as
% immediate support around a unified pole core using the qualified
% occupied-layer count rather than a separate seed-source category.
%
% Input:
%   eligibleMask: [Ny x Nx] logical cells allowed inside pole footprints
%   layerCountMap: [Ny x Nx] numeric qualified occupied-layer count map
%   params: struct from resolvePoleDetectionParams with
%       componentSupportMinOccupiedLayers
%
% Output:
%   supportMask: [Ny x Nx] logical support-cell mask
    supportMask = false(size(eligibleMask));
    if isempty(eligibleMask) || isempty(layerCountMap) || ~isequal(size(eligibleMask), size(layerCountMap))
        return;
    end

    minSupportLayers = 2;
    if isfield(params, "componentSupportMinOccupiedLayers") && isscalar(params.componentSupportMinOccupiedLayers) && ...
            isfinite(params.componentSupportMinOccupiedLayers)
        minSupportLayers = max(1, round(double(params.componentSupportMinOccupiedLayers)));
    end

    layerCountMap = double(layerCountMap);
    layerCountMap(~isfinite(layerCountMap)) = 0;
    supportMask = logical(eligibleMask) & (layerCountMap >= minSupportLayers);
end

function [coreMask, candidateMask] = buildSplitLayerPoleCandidates(eligibleMask, supportMask, layerCountMap, runLayerMap, lineScore, pointScore, params)
% buildSplitLayerPoleCandidates: Recover compact pole
% cores whose vertical support is split across two edge-adjacent fine
% columns, using combined occupied-layer evidence with point-like and
% low-line local shape gates before the normal candidate filters run.
%
% Input:
%   eligibleMask: [Ny x Nx] logical cells allowed to participate
%   supportMask: [Ny x Nx] logical occupied-layer support cells
%   layerCountMap: [Ny x Nx] numeric qualified occupied-layer counts
%   runLayerMap: [Ny x Nx] numeric contiguous occupied z-run counts
%   lineScore: [Ny x Nx] numeric local line-likeness score
%   pointScore: [Ny x Nx] numeric local point-likeness score
%   params: struct from resolvePoleDetectionParams with split-layer
%       core thresholds
%
% Output:
%   coreMask: [Ny x Nx] logical cells promoted as split-layer pole cores
%   candidateMask: [Ny x Nx] logical edge-adjacent split-layer footprints
    coreMask = false(size(eligibleMask));
    candidateMask = false(size(eligibleMask));
    if isempty(eligibleMask) || isempty(supportMask) || isempty(layerCountMap) || isempty(runLayerMap) || ...
            isempty(lineScore) || isempty(pointScore) || ~isequal(size(eligibleMask), size(supportMask)) || ...
            ~isequal(size(eligibleMask), size(layerCountMap)) || ~isequal(size(eligibleMask), size(runLayerMap)) || ...
            ~isequal(size(eligibleMask), size(lineScore)) || ~isequal(size(eligibleMask), size(pointScore))
        return;
    end

    minMemberLayers = 3;
    if isfield(params, "splitLayerMinMemberOccupiedLayers") && isscalar(params.splitLayerMinMemberOccupiedLayers) && ...
            isfinite(params.splitLayerMinMemberOccupiedLayers)
        minMemberLayers = max(1, round(double(params.splitLayerMinMemberOccupiedLayers)));
    end
    minCombinedLayers = 7;
    if isfield(params, "splitLayerMinCombinedOccupiedLayers") && isscalar(params.splitLayerMinCombinedOccupiedLayers) && ...
            isfinite(params.splitLayerMinCombinedOccupiedLayers)
        minCombinedLayers = max(1, round(double(params.splitLayerMinCombinedOccupiedLayers)));
    end
    minCombinedRunLayers = 3;
    if isfield(params, "splitLayerMinCombinedRunLayers") && isscalar(params.splitLayerMinCombinedRunLayers) && ...
            isfinite(params.splitLayerMinCombinedRunLayers)
        minCombinedRunLayers = max(1, round(double(params.splitLayerMinCombinedRunLayers)));
    end
    minPointScore = 0.70;
    if isfield(params, "splitLayerMinPointScore") && isscalar(params.splitLayerMinPointScore) && isfinite(params.splitLayerMinPointScore)
        minPointScore = min(max(double(params.splitLayerMinPointScore), 0), 1);
    end
    maxLineScore = 0.35;
    if isfield(params, "splitLayerMaxLineScore") && isscalar(params.splitLayerMaxLineScore) && isfinite(params.splitLayerMaxLineScore)
        maxLineScore = min(max(double(params.splitLayerMaxLineScore), 0), 1);
    end

    layerCountMap = double(layerCountMap);
    layerCountMap(~isfinite(layerCountMap)) = 0;
    runLayerMap = double(runLayerMap);
    runLayerMap(~isfinite(runLayerMap)) = 0;
    pointScore = double(pointScore);
    pointScore(~isfinite(pointScore)) = 0;
    lineScore = double(lineScore);
    lineScore(~isfinite(lineScore)) = 1;

    memberMask = logical(eligibleMask) & logical(supportMask) & ...
        (layerCountMap >= minMemberLayers) & (pointScore >= minPointScore) & (lineScore <= maxLineScore);
    if ~any(memberMask(:))
        return;
    end

    minCoreLayers = max(1, round(double(params.coreMinRunLayerThreshold)));
    horizontalPairMask = memberMask(:, 1:end-1) & memberMask(:, 2:end) & ...
        ((layerCountMap(:, 1:end-1) + layerCountMap(:, 2:end)) >= minCombinedLayers) & ...
        ((runLayerMap(:, 1:end-1) + runLayerMap(:, 2:end)) >= minCombinedRunLayers) & ...
        (max(layerCountMap(:, 1:end-1), layerCountMap(:, 2:end)) >= minCoreLayers);
    verticalPairMask = memberMask(1:end-1, :) & memberMask(2:end, :) & ...
        ((layerCountMap(1:end-1, :) + layerCountMap(2:end, :)) >= minCombinedLayers) & ...
        ((runLayerMap(1:end-1, :) + runLayerMap(2:end, :)) >= minCombinedRunLayers) & ...
        (max(layerCountMap(1:end-1, :), layerCountMap(2:end, :)) >= minCoreLayers);

    candidateMask(:, 1:end-1) = candidateMask(:, 1:end-1) | horizontalPairMask;
    candidateMask(:, 2:end) = candidateMask(:, 2:end) | horizontalPairMask;
    candidateMask(1:end-1, :) = candidateMask(1:end-1, :) | verticalPairMask;
    candidateMask(2:end, :) = candidateMask(2:end, :) | verticalPairMask;
    coreMask = candidateMask;
end

function filteredMask = filterPoleCandidatesByObjectLayerSupport(componentMask, fineVoxelGrid, params)
% filterPoleCandidatesByObjectLayerSupport: Keep raw pole-candidate
% connected components only when the entire component has enough z layers
% whose component-wide point count meets the occupied-layer point threshold.
%
% Input:
%   componentMask: [Ny x Nx] logical raw pole-candidate mask
%   fineVoxelGrid: sparse voxel column/z/count arrays, plus
%       occupiedLayerMinPoints metadata
%   params: struct from resolvePoleDetectionParams with
%       minCandidateSliceCount and occupiedLayerMinPoints
%
% Output:
%   filteredMask: [Ny x Nx] logical raw candidate mask after component-wide
%       qualified-layer support filtering
    filteredMask = false(size(componentMask));
    if ~any(componentMask(:))
        return;
    end
    hasSparseGrid = isstruct(fineVoxelGrid) && ...
        all(isfield(fineVoxelGrid, ["sparseVoxelColumnLinIdx", ...
        "sparseVoxelZBin", "sparseVoxelCount", "sparseMapSize", ...
        "sparseNumZLayers"])) && ...
        isequal(double(fineVoxelGrid.sparseMapSize(:).'), double(size(componentMask)));
    assert(hasSparseGrid,'perception:MissingFineSupport','Fine detection requires sparse voxel support.');

    minCandidateSliceCount = 3;
    if isfield(params, "minCandidateSliceCount") && isscalar(params.minCandidateSliceCount) && isfinite(params.minCandidateSliceCount)
        minCandidateSliceCount = max(1, round(double(params.minCandidateSliceCount)));
    end
    occupiedLayerMinPoints = 3;
    if isfield(params, "occupiedLayerMinPoints") && isscalar(params.occupiedLayerMinPoints) && isfinite(params.occupiedLayerMinPoints)
        occupiedLayerMinPoints = max(1, round(double(params.occupiedLayerMinPoints)));
    elseif isfield(fineVoxelGrid, "occupiedLayerMinPoints") && isscalar(fineVoxelGrid.occupiedLayerMinPoints) && isfinite(fineVoxelGrid.occupiedLayerMinPoints)
        occupiedLayerMinPoints = max(1, round(double(fineVoxelGrid.occupiedLayerMinPoints)));
    end

    cc = bwconncomp(logical(componentMask), 8);
    if cc.NumObjects < 1
        return;
    end

    componentIdMap = zeros(size(componentMask), "uint32");
    for iComp = 1:cc.NumObjects
        componentIdMap(cc.PixelIdxList{iComp}) = uint32(iComp);
    end

    xyLinIdx = double(fineVoxelGrid.sparseVoxelColumnLinIdx(:));
    zBin = double(fineVoxelGrid.sparseVoxelZBin(:));
    voxelCount = double(fineVoxelGrid.sparseVoxelCount(:));
    numZLayers = double(fineVoxelGrid.sparseNumZLayers);
    validSparseVoxel = isfinite(xyLinIdx) & xyLinIdx >= 1 & ...
        xyLinIdx <= numel(componentMask) & xyLinIdx == floor(xyLinIdx) & ...
        isfinite(zBin) & zBin >= 1 & zBin <= numZLayers & ...
        zBin == floor(zBin) & isfinite(voxelCount) & voxelCount > 0;
    xyLinIdx = xyLinIdx(validSparseVoxel);
    zBin = zBin(validSparseVoxel);
    voxelCount = voxelCount(validSparseVoxel);
    componentId = double(componentIdMap(xyLinIdx));
    validVoxel = componentId > 0;
    if ~any(validVoxel)
        return;
    end
    layerCounts = accumarray([componentId(validVoxel), double(zBin(validVoxel))], ...
        voxelCount(validVoxel), [cc.NumObjects, numZLayers], @sum, 0);
    keepComponent = sum(layerCounts >= occupiedLayerMinPoints, 2) >= minCandidateSliceCount;
    if any(keepComponent)
        filteredMask = ismember(componentIdMap, uint32(find(keepComponent)));
    end
end
