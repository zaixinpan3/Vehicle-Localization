function [footprint,context,compact,ratio,componentSum,contextSum]= ...
        selectPillarFootprints(core,support,tieBreaker,evidence,cfg,additionalFootprint)
% selectPillarFootprints: Select compact whole-pillar supports from XY maps.
% Inputs are one scalar per pillar. The same XY geometry serves the modern
% statistical detector and the explicitly offline historical detector.
    footprint=buildPoleFootprintCandidates(core,support,tieBreaker,evidence,cfg);
    footprint=footprint | additionalFootprint;
    [context,ratio,componentSum,contextSum]=filterPoleCandidatesByContextContrast(footprint,evidence,cfg);
    compact=filterPoleCandidateFootprints(context,cfg);
end

function candidateMask = buildPoleFootprintCandidates(coreMask, supportMask, pointCountMap, evidenceMap, params)
% buildPoleFootprintCandidates: Build unified pole
% footprint candidates by selecting the smallest candidate footprint that
% already satisfies local pole-candidate context support: singleton first
% and edge-adjacent pair second.
%
% Input:
%   coreMask: [Ny x Nx] logical unified pole-core mask
%   supportMask: [Ny x Nx] logical cells allowed to join a core footprint
%   pointCountMap: [Ny x Nx] numeric qualified point count map
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
%   params: struct from resolvePoleDetectionParams with context
%       window parameters
%
% Output:
%   candidateMask: [Ny x Nx] logical raw footprint candidate mask
    candidateMask = false(size(coreMask));
    if isempty(coreMask) || isempty(supportMask) || isempty(pointCountMap) || isempty(evidenceMap) || ...
            ~isequal(size(coreMask), size(supportMask)) || ~isequal(size(coreMask), size(pointCountMap)) || ...
            ~isequal(size(coreMask), size(evidenceMap)) || ~any(coreMask(:))
        return;
    end

    mapSize = size(coreMask);
    pointCountMap = double(pointCountMap);
    pointCountMap(~isfinite(pointCountMap)) = 0;
    evidenceMap = double(evidenceMap);
    evidenceMap(~isfinite(evidenceMap)) = 0;
    supportMask = logical(supportMask);
    [coreRows, coreCols] = find(logical(coreMask));
    for iCore = 1:numel(coreRows)
        coreRow = coreRows(iCore);
        coreCol = coreCols(iCore);
        coreLinIdx = sub2ind(mapSize, coreRow, coreCol);
        selectedLinIdx = selectMinimalPoleSupportFootprint( ...
            coreLinIdx, supportMask, pointCountMap, evidenceMap, mapSize, params);
        candidateMask(selectedLinIdx) = true;
    end
    candidateMask = filterMinimalPoleCandidateFootprints(candidateMask, coreMask, evidenceMap, params);
end

function selectedLinIdx = selectMinimalPoleSupportFootprint(coreLinIdx, supportMask, pointCountMap, evidenceMap, mapSize, params)
% selectMinimalPoleSupportFootprint: Choose the smallest valid
% footprint subset containing one core cell that satisfies pole-candidate
% local context support, using vertical whole-pillar height evidence as the primary
% tie breaker so stronger pillar cells are absorbed first.
%
% Input:
%   coreLinIdx: scalar linear index of the unified core cell
%   supportMask: [Ny x Nx] logical cells allowed to join a core footprint
%   pointCountMap: [Ny x Nx] numeric qualified point count map
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
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

    if passesPoleCandidateContext(selectedLinIdx, evidenceMap, mapSize, params)
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
        if ~passesPoleCandidateContext(candidateLinIdx, evidenceMap, mapSize, params)
            continue;
        end
        candidateScore = sum(double(evidenceMap(candidateLinIdx))) + ...
            (0.01 .* sum(double(pointCountMap(candidateLinIdx))));
        if candidateScore > (bestScore + eps)
            bestScore = candidateScore;
            bestLinIdx = candidateLinIdx;
        end
    end
    if ~isempty(bestLinIdx)
        selectedLinIdx = bestLinIdx;
    end
end

function filteredMask = filterMinimalPoleCandidateFootprints(componentMask, coreMask, evidenceMap, params)
% filterMinimalPoleCandidateFootprints: Reduce each connected raw pole
% candidate to the smallest valid singleton or edge-adjacent pair footprint
% containing at least one core cell that already satisfies local context
% support.
%
% Input:
%   componentMask: [Ny x Nx] logical raw footprint mask
%   coreMask: [Ny x Nx] logical pole-core cells
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
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
    evidenceMap = double(evidenceMap);
    evidenceMap(~isfinite(evidenceMap)) = 0;
    cc = bwconncomp(logical(componentMask), 8);
    for iComp = 1:cc.NumObjects
        compLinIdx = double(cc.PixelIdxList{iComp}(:));
        if isempty(compLinIdx)
            continue;
        end

        selectedLinIdx = selectPreferredMinimalPoleFootprint(compLinIdx, coreMask, evidenceMap, mapSize, params);
        if isempty(selectedLinIdx)
            coreLinIdx = compLinIdx(logical(coreMask(compLinIdx)));
            selectedLinIdx = selectBestValidPoleFootprintSubset(coreLinIdx, evidenceMap, mapSize);
            if isempty(selectedLinIdx)
                selectedLinIdx = selectBestValidPoleFootprintSubset(compLinIdx, evidenceMap, mapSize);
            end
        end
        filteredMask(selectedLinIdx) = true;
    end
end

function selectedLinIdx = selectPreferredMinimalPoleFootprint(compLinIdx, coreMask, evidenceMap, mapSize, params)
% selectPreferredMinimalPoleFootprint: Select the smallest pole
% footprint that passes context support using an explicit singleton then
% edge-pair order, with the strongest whole-pillar height sum used only to break ties
% within the same footprint size.
%
% Input:
%   compLinIdx: [N x 1] candidate component linear indices
%   coreMask: [Ny x Nx] logical pole-core cells
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
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
    selectedLinIdx = selectBestPassingFootprintOfSize(coreLinIdx, compLinIdx, 1, evidenceMap, mapSize, params);
    if ~isempty(selectedLinIdx)
        return;
    end

    selectedLinIdx = selectBestPassingFootprintOfSize(coreLinIdx, compLinIdx, 2, evidenceMap, mapSize, params);
    if ~isempty(selectedLinIdx)
        return;
    end

end

function selectedLinIdx = selectBestPassingFootprintOfSize(coreLinIdx, compLinIdx, subsetSize, evidenceMap, mapSize, params)
% selectBestPassingFootprintOfSize: Find the strongest valid
% footprint of one requested size that contains a core cell and passes the
% component context-ratio candidate test.
%
% Input:
%   coreLinIdx: [K x 1] core-cell linear indices inside the component
%   compLinIdx: [N x 1] component linear indices
%   subsetSize: scalar footprint size to test
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
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
            if ~passesPoleCandidateContext(candidateLinIdx, evidenceMap, mapSize, params)
                continue;
            end
            candidateScore = double(evidenceMap(candidateLinIdx));
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
            if ~passesPoleCandidateContext(candidateLinIdx, evidenceMap, mapSize, params)
                continue;
            end
            candidateScore = sum(double(evidenceMap(candidateLinIdx)));
            if candidateScore > (bestScore + eps)
                bestScore = candidateScore;
                selectedLinIdx = candidateLinIdx(:);
            end
        end
    end
end

function passesContext = passesPoleCandidateContext(candidateLinIdx, evidenceMap, mapSize, params)
% passesPoleCandidateContext: Apply the same component/context
% whole-pillar height ratio used by pole candidate filtering to one proposed
% singleton or edge-adjacent pair footprint before allowing it to absorb
% more cells.
%
% Input:
%   candidateLinIdx: [N x 1] proposed footprint linear indices
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
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

    minContextRatio = max(0, double(params.minimumContextFraction));
    [~, componentEvidenceSum, contextEvidenceSum] = computePoleCandidateContextRatio( ...
        candidateLinIdx, evidenceMap, mapSize);
    if contextEvidenceSum <= eps
        passesContext = true;
    else
        passesContext = (componentEvidenceSum ./ contextEvidenceSum) > minContextRatio;
    end
end

function [filteredMask, ratioMap, componentEvidenceSumMap, contextEvidenceSumMap] = filterPoleCandidatesByContextContrast(componentMask, evidenceMap, params)
% filterPoleCandidatesByContextContrast: Keep unified pole
% candidate components only when their summed whole-pillar height evidence is more
% than a configured fraction of the summed whole-pillar height evidence inside a
% fixed local context window that includes the component.
%
% Input:
%   componentMask: [Ny x Nx] logical raw unified pole-candidate mask
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
%   params: struct from resolvePoleDetectionParams with context
%       window size and minimum whole-pillar height ratio
%
% Output:
%   filteredMask: [Ny x Nx] logical candidate mask after context contrast
%   ratioMap: [Ny x Nx] numeric component/context geometric evidence ratio
%   componentEvidenceSumMap: [Ny x Nx] numeric component whole-pillar height sums
%   contextEvidenceSumMap: [Ny x Nx] numeric local context whole-pillar height sums
    filteredMask = false(size(componentMask));
    ratioMap = zeros(size(componentMask), "single");
    componentEvidenceSumMap = zeros(size(componentMask), "single");
    contextEvidenceSumMap = zeros(size(componentMask), "single");
    if isempty(componentMask) || ~any(componentMask(:))
        return;
    end

    evidenceMap = double(evidenceMap);
    if ~isequal(size(componentMask), size(evidenceMap))
        return;
    end
    evidenceMap(~isfinite(evidenceMap)) = 0;
    evidenceMap = max(evidenceMap, 0);

    minContextRatio = max(0, double(params.minimumContextFraction));

    mapSize = size(componentMask);
    cc = bwconncomp(logical(componentMask), 8);
    for iComp = 1:cc.NumObjects
        compLinIdx = double(cc.PixelIdxList{iComp}(:));
        if isempty(compLinIdx)
            continue;
        end

        [contextRatio, componentEvidenceSum, contextEvidenceSum] = computePoleCandidateContextRatio( ...
            compLinIdx, evidenceMap, mapSize);
        ratioMap(compLinIdx) = single(contextRatio);
        componentEvidenceSumMap(compLinIdx) = single(componentEvidenceSum);
        contextEvidenceSumMap(compLinIdx) = single(contextEvidenceSum);
        if contextEvidenceSum <= eps || (contextRatio > minContextRatio)
            filteredMask(compLinIdx) = true;
        end
    end
end

function [contextRatio, componentEvidenceSum, contextEvidenceSum] = computePoleCandidateContextRatio(candidateLinIdx, evidenceMap, mapSize)
% computePoleCandidateContextRatio: Compute a compact pole footprint's
% whole-pillar height share inside its bbox-plus-one local context without allocating
% temporary full-size logical masks.
%
% Input:
%   candidateLinIdx: [N x 1] proposed footprint linear indices
%   evidenceMap: [Ny x Nx] numeric whole-pillar geometric evidence map
%   mapSize: [1 x 2] map size [Ny Nx]
%
% Output:
%   contextRatio: scalar component/context geometric evidence ratio
%   componentEvidenceSum: scalar summed whole-pillar height evidence in candidate cells
%   contextEvidenceSum: scalar summed whole-pillar height evidence in local context window
    candidateLinIdx = double(candidateLinIdx(:));
    [rows, cols] = ind2sub(mapSize, candidateLinIdx);
    [rowMin, rowMax, colMin, colMax] = computePoleFootprintContextBounds(rows, cols, mapSize);
    componentEvidenceSum = sum(double(evidenceMap(candidateLinIdx)));
    contextEvidenceSum = sum(double(evidenceMap(rowMin:rowMax, colMin:colMax)), "all");
    contextRatio = 0;
    if contextEvidenceSum > eps
        contextRatio = componentEvidenceSum ./ contextEvidenceSum;
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
%       maximumFootprintSpanCells
%
% Output:
%   filteredMask: [Ny x Nx] logical mask after footprint filtering
    filteredMask = false(size(componentMask));
    if ~any(componentMask(:))
        return;
    end

    maxSpanCells = 2;
    if isfield(params, "maximumFootprintSpanCells") && isscalar(params.maximumFootprintSpanCells) && isfinite(params.maximumFootprintSpanCells)
        maxSpanCells = max(1, round(double(params.maximumFootprintSpanCells)));
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

function subsetLinIdx = selectBestValidPoleFootprintSubset(compLinIdx, evidenceMap, mapSize)
% selectBestValidPoleFootprintSubset: Choose the highest-run valid
% singleton or edge-adjacent pair subset from an invalid compact footprint
% component.
%
% Input:
%   compLinIdx: [N x 1] linear indices from one invalid candidate component
%   evidenceMap: [Ny x Nx] numeric maximum contiguous occupied pillar count
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
            candidateScore = sum(evidenceMap(candidateLinIdx));
            if (candidateScore > bestScore) || ((candidateScore == bestScore) && (2 > bestCount))
                bestScore = candidateScore;
                bestCount = 2;
                subsetLinIdx = candidateLinIdx(:);
            end
        end
    end

    for iCell = 1:numel(compLinIdx)
        candidateLinIdx = compLinIdx(iCell);
        candidateScore = evidenceMap(candidateLinIdx);
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
