function candidates = detectPoleCandidates(columnMaps, facadeMask, fineVoxelGrid, params)
% detectPoleCandidates: Global (column-level) analysis of pole-like
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
%   columnMaps: struct from buildFineColumnFeatureMaps extended with
%       runLayerMap, pointScore, and lineScore
%   facadeMask: [Ny x Nx] logical columns already assigned to facades
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
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
    footprintCandidateMask = buildPoleFootprintCandidates( ...
        coreMask, supportMask, layerCountMap, runLayerMap, params);
    footprintCandidateMask = footprintCandidateMask | splitLayerCandidateMask;
    [contextCandidateMask, contextRunLayerRatio, componentRunLayerSum, contextRunLayerSum] = ...
        filterPoleCandidatesByContextContrast(footprintCandidateMask, runLayerMap, params);
    compactFootprintMask = filterPoleCandidateFootprints(contextCandidateMask, params);
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

function candidateMask = buildPoleFootprintCandidates(coreMask, supportMask, layerCountMap, runLayerMap, params)
% buildPoleFootprintCandidates: Build unified pole
% footprint candidates by selecting the smallest candidate footprint that
% already satisfies local pole-candidate context support: singleton first
% and edge-adjacent pair second.
%
% Input:
%   coreMask: [Ny x Nx] logical unified pole-core mask
%   supportMask: [Ny x Nx] logical cells allowed to join a core footprint
%   layerCountMap: [Ny x Nx] numeric qualified occupied-layer count map
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   params: struct from resolvePoleDetectionParams with context
%       window parameters
%
% Output:
%   candidateMask: [Ny x Nx] logical raw footprint candidate mask
    candidateMask = false(size(coreMask));
    if isempty(coreMask) || isempty(supportMask) || isempty(layerCountMap) || isempty(runLayerMap) || ...
            ~isequal(size(coreMask), size(supportMask)) || ~isequal(size(coreMask), size(layerCountMap)) || ...
            ~isequal(size(coreMask), size(runLayerMap)) || ~any(coreMask(:))
        return;
    end

    mapSize = size(coreMask);
    layerCountMap = double(layerCountMap);
    layerCountMap(~isfinite(layerCountMap)) = 0;
    runLayerMap = double(runLayerMap);
    runLayerMap(~isfinite(runLayerMap)) = 0;
    supportMask = logical(supportMask);
    [coreRows, coreCols] = find(logical(coreMask));
    for iCore = 1:numel(coreRows)
        coreRow = coreRows(iCore);
        coreCol = coreCols(iCore);
        coreLinIdx = sub2ind(mapSize, coreRow, coreCol);
        selectedLinIdx = selectMinimalPoleSupportFootprint( ...
            coreLinIdx, supportMask, layerCountMap, runLayerMap, mapSize, params);
        candidateMask(selectedLinIdx) = true;
    end
    candidateMask = filterMinimalPoleCandidateFootprints(candidateMask, coreMask, runLayerMap, params);
end

function selectedLinIdx = selectMinimalPoleSupportFootprint(coreLinIdx, supportMask, layerCountMap, runLayerMap, mapSize, params)
% selectMinimalPoleSupportFootprint: Choose the smallest valid
% footprint subset containing one core cell that satisfies pole-candidate
% local context support, using vertical run-layer evidence as the primary
% tie breaker so stronger z-layer cells are absorbed first.
%
% Input:
%   coreLinIdx: scalar linear index of the unified core cell
%   supportMask: [Ny x Nx] logical cells allowed to join a core footprint
%   layerCountMap: [Ny x Nx] numeric qualified occupied-layer count map
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   mapSize: [1 x 2] map size [Ny Nx]
%   params: struct from resolvePoleDetectionParams with context
%       window parameters
%
% Output:
%   selectedLinIdx: [M x 1] selected valid footprint linear indices
    selectedLinIdx = double(coreLinIdx);
    if isempty(coreLinIdx) || isempty(supportMask)
        return;
    end

    if passesPoleCandidateContext(selectedLinIdx, runLayerMap, mapSize, params)
        return;
    end

    [coreRow, coreCol] = ind2sub(mapSize, double(coreLinIdx));
    neighborOffsets = [-1 0; 1 0; 0 -1; 0 1];
    bestScore = -inf;
    bestLinIdx = zeros(0, 1);
    for iNbr = 1:size(neighborOffsets, 1)
        rowIdx = coreRow + neighborOffsets(iNbr, 1);
        colIdx = coreCol + neighborOffsets(iNbr, 2);
        if rowIdx < 1 || rowIdx > mapSize(1) || colIdx < 1 || colIdx > mapSize(2) || ~supportMask(rowIdx, colIdx)
            continue;
        end
        neighborLinIdx = sub2ind(mapSize, rowIdx, colIdx);
        candidateLinIdx = [double(coreLinIdx); double(neighborLinIdx)];
        if ~passesPoleCandidateContext(candidateLinIdx, runLayerMap, mapSize, params)
            continue;
        end
        candidateScore = sum(double(runLayerMap(candidateLinIdx))) + ...
            (0.01 .* sum(double(layerCountMap(candidateLinIdx))));
        if candidateScore > (bestScore + eps)
            bestScore = candidateScore;
            bestLinIdx = candidateLinIdx;
        end
    end
    if ~isempty(bestLinIdx)
        selectedLinIdx = bestLinIdx;
    end
end

function filteredMask = filterMinimalPoleCandidateFootprints(componentMask, coreMask, runLayerMap, params)
% filterMinimalPoleCandidateFootprints: Reduce each connected raw pole
% candidate to the smallest valid singleton or edge-adjacent pair footprint
% containing at least one core cell that already satisfies local context
% support.
%
% Input:
%   componentMask: [Ny x Nx] logical raw footprint mask
%   coreMask: [Ny x Nx] logical pole-core cells
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   params: struct from resolvePoleDetectionParams with context
%       window parameters
%
% Output:
%   filteredMask: [Ny x Nx] logical smallest feasible footprint mask
    filteredMask = false(size(componentMask));
    if isempty(componentMask) || ~any(componentMask(:)) || ~isequal(size(componentMask), size(coreMask))
        return;
    end

    mapSize = size(componentMask);
    runLayerMap = double(runLayerMap);
    runLayerMap(~isfinite(runLayerMap)) = 0;
    cc = bwconncomp(logical(componentMask), 8);
    for iComp = 1:cc.NumObjects
        compLinIdx = double(cc.PixelIdxList{iComp}(:));
        if isempty(compLinIdx)
            continue;
        end

        selectedLinIdx = selectPreferredMinimalPoleFootprint(compLinIdx, coreMask, runLayerMap, mapSize, params);
        if isempty(selectedLinIdx)
            coreLinIdx = compLinIdx(logical(coreMask(compLinIdx)));
            selectedLinIdx = selectBestValidPoleFootprintSubset(coreLinIdx, runLayerMap, mapSize);
            if isempty(selectedLinIdx)
                selectedLinIdx = selectBestValidPoleFootprintSubset(compLinIdx, runLayerMap, mapSize);
            end
        end
        filteredMask(selectedLinIdx) = true;
    end
end

function selectedLinIdx = selectPreferredMinimalPoleFootprint(compLinIdx, coreMask, runLayerMap, mapSize, params)
% selectPreferredMinimalPoleFootprint: Select the smallest pole
% footprint that passes context support using an explicit singleton then
% edge-pair order, with the strongest run-layer sum used only to break ties
% within the same footprint size.
%
% Input:
%   compLinIdx: [N x 1] candidate component linear indices
%   coreMask: [Ny x Nx] logical pole-core cells
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   mapSize: [1 x 2] map size [Ny Nx]
%   params: struct from resolvePoleDetectionParams with context
%       window parameters
%
% Output:
%   selectedLinIdx: [M x 1] selected footprint, or empty
    selectedLinIdx = zeros(0, 1);
    compLinIdx = double(compLinIdx(:));
    if isempty(compLinIdx)
        return;
    end

    coreLinIdx = compLinIdx(logical(coreMask(compLinIdx)));
    selectedLinIdx = selectBestPassingFootprintOfSize(coreLinIdx, compLinIdx, 1, runLayerMap, mapSize, params);
    if ~isempty(selectedLinIdx)
        return;
    end

    selectedLinIdx = selectBestPassingFootprintOfSize(coreLinIdx, compLinIdx, 2, runLayerMap, mapSize, params);
    if ~isempty(selectedLinIdx)
        return;
    end

end

function selectedLinIdx = selectBestPassingFootprintOfSize(coreLinIdx, compLinIdx, subsetSize, runLayerMap, mapSize, params)
% selectBestPassingFootprintOfSize: Find the strongest valid
% footprint of one requested size that contains a core cell and passes the
% component context-ratio candidate test.
%
% Input:
%   coreLinIdx: [K x 1] core-cell linear indices inside the component
%   compLinIdx: [N x 1] component linear indices
%   subsetSize: scalar footprint size to test
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   mapSize: [1 x 2] map size [Ny Nx]
%   params: struct from resolvePoleDetectionParams
%
% Output:
%   selectedLinIdx: [subsetSize x 1] best passing footprint, or empty
    selectedLinIdx = zeros(0, 1);
    if isempty(coreLinIdx) || numel(compLinIdx) < subsetSize
        return;
    end

    bestScore = -inf;
    if subsetSize == 1
        for iCore = 1:numel(coreLinIdx)
            candidateLinIdx = double(coreLinIdx(iCore));
            if ~passesPoleCandidateContext(candidateLinIdx, runLayerMap, mapSize, params)
                continue;
            end
            candidateScore = double(runLayerMap(candidateLinIdx));
            if candidateScore > (bestScore + eps)
                bestScore = candidateScore;
                selectedLinIdx = candidateLinIdx;
            end
        end
        return;
    end

    if subsetSize ~= 2
        return;
    end

    compMemberMask = false(mapSize);
    compMemberMask(compLinIdx) = true;
    neighborOffsets = [-1 0; 1 0; 0 -1; 0 1];
    for iCore = 1:numel(coreLinIdx)
        [coreRow, coreCol] = ind2sub(mapSize, double(coreLinIdx(iCore)));
        for iNbr = 1:size(neighborOffsets, 1)
            rowIdx = coreRow + neighborOffsets(iNbr, 1);
            colIdx = coreCol + neighborOffsets(iNbr, 2);
            if rowIdx < 1 || rowIdx > mapSize(1) || colIdx < 1 || colIdx > mapSize(2) || ~compMemberMask(rowIdx, colIdx)
                continue;
            end
            neighborLinIdx = sub2ind(mapSize, rowIdx, colIdx);
            candidateLinIdx = [double(coreLinIdx(iCore)); double(neighborLinIdx)];
            if ~passesPoleCandidateContext(candidateLinIdx, runLayerMap, mapSize, params)
                continue;
            end
            candidateScore = sum(double(runLayerMap(candidateLinIdx)));
            if candidateScore > (bestScore + eps)
                bestScore = candidateScore;
                selectedLinIdx = candidateLinIdx(:);
            end
        end
    end
end

function passesContext = passesPoleCandidateContext(candidateLinIdx, runLayerMap, mapSize, params)
% passesPoleCandidateContext: Apply the same component/context
% run-layer ratio used by pole candidate filtering to one proposed
% singleton or edge-adjacent pair footprint before allowing it to absorb
% more cells.
%
% Input:
%   candidateLinIdx: [N x 1] proposed footprint linear indices
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   mapSize: [1 x 2] map size [Ny Nx]
%   params: struct from resolvePoleDetectionParams with context
%       window size and minimum ratio
%
% Output:
%   passesContext: logical scalar true when the footprint passes context
    candidateLinIdx = double(candidateLinIdx(:));
    if isempty(candidateLinIdx)
        passesContext = false;
        return;
    end

    minContextRatio = max(0, double(params.componentContextMinRunLayerRatio));
    [~, componentRunSum, contextRunSum] = computePoleCandidateContextRatio( ...
        candidateLinIdx, runLayerMap, mapSize);
    if contextRunSum <= eps
        passesContext = true;
    else
        passesContext = (componentRunSum ./ contextRunSum) > minContextRatio;
    end
end

function [filteredMask, ratioMap, componentRunSumMap, contextRunSumMap] = filterPoleCandidatesByContextContrast(componentMask, runLayerMap, params)
% filterPoleCandidatesByContextContrast: Keep unified pole
% candidate components only when their summed run-layer evidence is more
% than a configured fraction of the summed run-layer evidence inside a
% fixed local context window that includes the component.
%
% Input:
%   componentMask: [Ny x Nx] logical raw unified pole-candidate mask
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   params: struct from resolvePoleDetectionParams with context
%       window size and minimum run-layer ratio
%
% Output:
%   filteredMask: [Ny x Nx] logical candidate mask after context contrast
%   ratioMap: [Ny x Nx] numeric component/context run-layer ratio
%   componentRunSumMap: [Ny x Nx] numeric component run-layer sums
%   contextRunSumMap: [Ny x Nx] numeric local context run-layer sums
    filteredMask = false(size(componentMask));
    ratioMap = zeros(size(componentMask), "single");
    componentRunSumMap = zeros(size(componentMask), "single");
    contextRunSumMap = zeros(size(componentMask), "single");
    if isempty(componentMask) || ~any(componentMask(:))
        return;
    end

    runLayerMap = double(runLayerMap);
    if ~isequal(size(componentMask), size(runLayerMap))
        return;
    end
    runLayerMap(~isfinite(runLayerMap)) = 0;
    runLayerMap = max(runLayerMap, 0);

    minContextRatio = max(0, double(params.componentContextMinRunLayerRatio));

    mapSize = size(componentMask);
    cc = bwconncomp(logical(componentMask), 8);
    for iComp = 1:cc.NumObjects
        compLinIdx = double(cc.PixelIdxList{iComp}(:));
        if isempty(compLinIdx)
            continue;
        end

        [contextRatio, componentRunSum, contextRunSum] = computePoleCandidateContextRatio( ...
            compLinIdx, runLayerMap, mapSize);
        ratioMap(compLinIdx) = single(contextRatio);
        componentRunSumMap(compLinIdx) = single(componentRunSum);
        contextRunSumMap(compLinIdx) = single(contextRunSum);
        if contextRunSum <= eps || (contextRatio > minContextRatio)
            filteredMask(compLinIdx) = true;
        end
    end
end

function [contextRatio, componentRunSum, contextRunSum] = computePoleCandidateContextRatio(candidateLinIdx, runLayerMap, mapSize)
% computePoleCandidateContextRatio: Compute a compact pole footprint's
% run-layer share inside its bbox-plus-one local context without allocating
% temporary full-size logical masks.
%
% Input:
%   candidateLinIdx: [N x 1] proposed footprint linear indices
%   runLayerMap: [Ny x Nx] numeric maximum contiguous z-run layer map
%   mapSize: [1 x 2] map size [Ny Nx]
%
% Output:
%   contextRatio: scalar component/context run-layer ratio
%   componentRunSum: scalar summed run-layer evidence in candidate cells
%   contextRunSum: scalar summed run-layer evidence in local context window
    candidateLinIdx = double(candidateLinIdx(:));
    [rows, cols] = ind2sub(mapSize, candidateLinIdx);
    [rowMin, rowMax, colMin, colMax] = computePoleFootprintContextBounds(rows, cols, mapSize);
    componentRunSum = sum(double(runLayerMap(candidateLinIdx)));
    contextRunSum = sum(double(runLayerMap(rowMin:rowMax, colMin:colMax)), "all");
    contextRatio = 0;
    if contextRunSum > eps
        contextRatio = componentRunSum ./ contextRunSum;
    end
end

function [rowMin, rowMax, colMin, colMax] = computePoleFootprintContextBounds(rows, cols, mapSize)
% computePoleFootprintContextBounds: Build the local context window
% around one compact pole footprint by expanding the footprint bounding box
% by one cell. A singleton uses a 3-by-3 neighborhood, and an edge-adjacent
% pair uses a 3-by-4 or 4-by-3 neighborhood depending on orientation.
%
% Input:
%   rows: [K x 1] component row indices
%   cols: [K x 1] component column indices
%   mapSize: [1 x 2] map size [Ny Nx]
%
% Output:
%   rowMin, rowMax, colMin, colMax: scalar inclusive window bounds
    rows = double(rows(:));
    cols = double(cols(:));
    rowMin = max(1, min(rows) - 1);
    rowMax = min(mapSize(1), max(rows) + 1);
    colMin = max(1, min(cols) - 1);
    colMax = min(mapSize(2), max(cols) + 1);
end

function filteredMask = filterPoleCandidateFootprints(componentMask, params)
% filterPoleCandidateFootprints: Keep only connected energy
% components whose coarse-grid footprint fits within one compact square
% block so a single pole cannot span more than the intended local support.
%
% Input:
%   componentMask: [Ny x Nx] logical connected-component mask
%   params: struct from resolvePoleDetectionParams with
%       maxFootprintSpanCells
%
% Output:
%   filteredMask: [Ny x Nx] logical mask after footprint filtering
    filteredMask = false(size(componentMask));
    if ~any(componentMask(:))
        return;
    end

    maxSpanCells = 2;
    if isfield(params, "maxFootprintSpanCells") && isscalar(params.maxFootprintSpanCells) && isfinite(params.maxFootprintSpanCells)
        maxSpanCells = max(1, round(double(params.maxFootprintSpanCells)));
    end

    cc = bwconncomp(logical(componentMask), 8);
    for iComp = 1:cc.NumObjects
        [rows, cols] = ind2sub(size(componentMask), cc.PixelIdxList{iComp});
        if isempty(rows)
            continue;
        end
        rowSpanCells = (max(rows) - min(rows)) + 1;
        colSpanCells = (max(cols) - min(cols)) + 1;
        if (rowSpanCells <= maxSpanCells) && (colSpanCells <= maxSpanCells) && isValidPoleFootprintShape(rows, cols)
            filteredMask(cc.PixelIdxList{iComp}) = true;
        elseif (rowSpanCells <= maxSpanCells) && (colSpanCells <= maxSpanCells)
            subsetLinIdx = selectBestValidPoleFootprintSubset(cc.PixelIdxList{iComp}, componentMask, size(componentMask));
            filteredMask(subsetLinIdx) = true;
        end
    end
end

function subsetLinIdx = selectBestValidPoleFootprintSubset(compLinIdx, runLayerMap, mapSize)
% selectBestValidPoleFootprintSubset: Choose the highest-run valid
% singleton or edge-adjacent pair subset from an invalid compact footprint
% component.
%
% Input:
%   compLinIdx: [N x 1] linear indices from one invalid candidate component
%   runLayerMap: [Ny x Nx] numeric maximum contiguous occupied z-layer count
%   mapSize: [1 x 2] map size [Ny Nx]
%
% Output:
%   subsetLinIdx: [M x 1] linear indices for the selected valid subset
    compLinIdx = double(compLinIdx(:));
    subsetLinIdx = zeros(0, 1);
    if isempty(compLinIdx)
        return;
    end

    bestScore = -inf;
    bestCount = 0;
    compMemberMask = false(mapSize);
    compMemberMask(compLinIdx) = true;
    [rows, cols] = ind2sub(mapSize, compLinIdx);
    forwardOffsets = [1 0; 0 1];
    for iCell = 1:numel(compLinIdx)
        for iNbr = 1:size(forwardOffsets, 1)
            rowIdx = rows(iCell) + forwardOffsets(iNbr, 1);
            colIdx = cols(iCell) + forwardOffsets(iNbr, 2);
            if rowIdx > mapSize(1) || colIdx > mapSize(2) || ~compMemberMask(rowIdx, colIdx)
                continue;
            end
            candidateLinIdx = [compLinIdx(iCell); double(sub2ind(mapSize, rowIdx, colIdx))];
            candidateScore = sum(runLayerMap(candidateLinIdx));
            if (candidateScore > bestScore) || ((candidateScore == bestScore) && (2 > bestCount))
                bestScore = candidateScore;
                bestCount = 2;
                subsetLinIdx = candidateLinIdx(:);
            end
        end
    end

    for iCell = 1:numel(compLinIdx)
        candidateLinIdx = compLinIdx(iCell);
        candidateScore = runLayerMap(candidateLinIdx);
        if (candidateScore > bestScore) || ((candidateScore == bestScore) && (1 > bestCount))
            bestScore = candidateScore;
            bestCount = 1;
            subsetLinIdx = candidateLinIdx;
        end
    end
end

function isValid = isValidPoleFootprintShape(rows, cols)
% isValidPoleFootprintShape: Check whether one candidate footprint is
% a valid pole pattern after local contrast trimming.
%
% Input:
%   rows: [N x 1] row subscripts for the candidate footprint
%   cols: [N x 1] column subscripts for the candidate footprint
%
% Output:
%   isValid: logical scalar true for singleton or edge-adjacent pair
    rows = double(rows(:));
    cols = double(cols(:));
    trueCount = numel(rows);
    if trueCount == 1
        isValid = true;
        return;
    end
    if trueCount == 2
        isValid = (abs(rows(1) - rows(2)) + abs(cols(1) - cols(2))) == 1;
        return;
    end
    isValid = false;
end

function filteredMask = filterPoleCandidatesByObjectLayerSupport(componentMask, fineVoxelGrid, params)
% filterPoleCandidatesByObjectLayerSupport: Keep raw pole-candidate
% connected components only when the entire component has enough z layers
% whose component-wide point count meets the occupied-layer point threshold.
%
% Input:
%   componentMask: [Ny x Nx] logical raw pole-candidate mask
%   fineVoxelGrid: struct with either count3D in the same [Ny x Nx x Nz]
%       layout as componentMask or sparse voxel column/z/count arrays, plus
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
    hasDenseGrid = isstruct(fineVoxelGrid) && isfield(fineVoxelGrid, "count3D") && ...
        ~isempty(fineVoxelGrid.count3D) && ndims(fineVoxelGrid.count3D) == 3 && ...
        isequal(size(componentMask), ...
        [size(fineVoxelGrid.count3D, 1), size(fineVoxelGrid.count3D, 2)]);
    hasSparseGrid = isstruct(fineVoxelGrid) && ...
        all(isfield(fineVoxelGrid, ["sparseVoxelColumnLinIdx", ...
        "sparseVoxelZBin", "sparseVoxelCount", "sparseMapSize", ...
        "sparseNumZLayers"])) && ...
        isequal(double(fineVoxelGrid.sparseMapSize(:).'), double(size(componentMask)));
    if ~hasDenseGrid && ~hasSparseGrid
        filteredMask = logical(componentMask);
        return;
    end

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

    if hasDenseGrid
        count3D = fineVoxelGrid.count3D;
        nonzeroVoxelIdx = find(count3D > 0);
        if isempty(nonzeroVoxelIdx)
            return;
        end
        [yBin, xBin, zBin] = ind2sub(size(count3D), nonzeroVoxelIdx);
        xyLinIdx = sub2ind(size(componentMask), yBin, xBin);
        voxelCount = double(count3D(nonzeroVoxelIdx));
        numZLayers = size(count3D, 3);
    else
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
    end
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
