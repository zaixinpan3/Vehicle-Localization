function [footprint,context,compact,ratio,componentSum,contextSum]= ...
        selectPillarFootprints(core,support,tieBreaker,evidence,cfg,additionalFootprint)
% selectPillarFootprints: Select compact whole-pillar supports from XY maps.
% Inputs are one scalar per pillar. The same XY geometry serves the modern
% statistical detector and the offline fine detector.
    footprint=buildPoleFootprintCandidates(core,support,tieBreaker,evidence,cfg);
    footprint=footprint | additionalFootprint;
    [context,ratio,componentSum,contextSum]=filterPoleCandidatesByContextContrast(footprint,evidence,cfg);
    compact=filterPoleCandidateFootprints(context,cfg);
end

function candidateMask = buildPoleFootprintCandidates(coreMask, supportMask, pointCountMap, evidenceMap, params)
% Batch singleton and edge-pair evidence tests once per core. Reuse the
% tests during component reduction; preserve up/down/left/right tie order.
    candidateMask = false(size(coreMask));
    if isempty(coreMask) || isempty(supportMask) || isempty(pointCountMap) || isempty(evidenceMap) || ...
            ~isequal(size(coreMask), size(supportMask), size(pointCountMap), size(evidenceMap)) || ~any(coreMask(:))
        return;
    end
    pointCountMap = double(pointCountMap);
    pointCountMap(~isfinite(pointCountMap)) = 0;
    evidenceMap = double(evidenceMap);
    evidenceMap(~isfinite(evidenceMap)) = 0;
    coreIds = find(coreMask); coreIds = coreIds(:);
    evidenceValues = evidenceMap(:); pointCountMap = pointCountMap(:);
    supportMask = supportMask(:);
    [rows, cols] = ind2sub(size(coreMask), coreIds);
    tests = buildContextTests(coreIds, rows, cols, evidenceMap, params);
    candidateMask(coreIds) = true;
    bestScore = -inf(size(coreIds));
    bestNeighbor = zeros(size(coreIds));
    for direction = 1:4
        neighbor = tests.neighbors(:,direction);
        eligible = ~tests.singlePass & tests.pairPass(:,direction) & ...
            logical(supportMask(max(neighbor,1)));
        score = evidenceValues(coreIds) + evidenceValues(max(neighbor,1)) + ...
            0.01 .* (pointCountMap(coreIds) + pointCountMap(max(neighbor,1)));
        better = eligible & score > bestScore + eps;
        bestScore(better) = score(better);
        bestNeighbor(better) = neighbor(better);
    end
    candidateMask(bestNeighbor(bestNeighbor > 0)) = true;

    % Connected components use the same column-major core traversal as the
    % scalar selector. Only passing proposals enter the sequential tie test.
    components = bwconncomp(candidateMask,8);
    labels = labelmatrix(components); labels = labels(:);
    coreRows = zeros(numel(coreMask),1);
    coreRows(coreIds) = 1:numel(coreIds);
    candidateMask(:) = false;
    for component = 1:components.NumObjects
        members = components.PixelIdxList{component}(:);
        core = coreRows(members); core = core(core > 0);
        passed = core(tests.singlePass(core));
        if ~isempty(passed)
            chosen = firstBestScore(evidenceValues(coreIds(passed)));
            selected = coreIds(passed(chosen));
        else
            neighbor = tests.neighbors(core,:);
            neighborLabels = reshape(labels(max(neighbor,1)),size(neighbor));
            eligible = tests.pairPass(core,:) & neighborLabels == component;
            % Transpose before find: core first, then up/down/left/right.
            [direction, localCore] = find(eligible.');
            if isempty(localCore)
                selected = selectBestValidPoleFootprintSubset(coreIds(core),evidenceMap,size(coreMask));
                if isempty(selected)
                    selected = selectBestValidPoleFootprintSubset(members,evidenceMap,size(coreMask));
                end
            else
                a = coreIds(core(localCore));
                b = tests.neighbors(core(localCore) + (direction-1).*numel(coreIds));
                chosen = firstBestScore(evidenceValues(a) + evidenceValues(b));
                selected = [a(chosen);b(chosen)];
            end
        end
        candidateMask(selected) = true;
    end
end

function chosen = firstBestScore(scores)
% Retain strict score > best + eps, including the original first-win order.
    best = -inf; chosen = 1;
    for k = 1:numel(scores)
        if scores(k) > best + eps
            best = scores(k); chosen = k;
        end
    end
end

function tests = buildContextTests(ids, rows, cols, evidence, params)
% Query only core-centered windows, never all pairs in the whole raster.
% Each column of neighbors represents up, down, left, then right.
    mapSize = size(evidence);
    minRatio = max(0,double(params.minimumContextFraction));
    rowOffset = [-1 1 0 0]; colOffset = [0 0 -1 1];
    nr = rows + rowOffset; nc = cols + colOffset;
    valid = nr >= 1 & nr <= mapSize(1) & nc >= 1 & nc <= mapSize(2);
    neighbors = nr + (nc-1).*mapSize(1);
    neighbors(~valid) = 0;
    tests = struct('neighbors',neighbors,'singlePass',false(size(ids)), ...
        'pairPass',false(size(neighbors)));
    tests.singlePass = contextPassBatch(ids,zeros(size(ids)),rows,cols, ...
        rows,cols,evidence,minRatio);
    for direction = 1:4
        use = valid(:,direction);
        tests.pairPass(use,direction) = contextPassBatch(ids(use),neighbors(use,direction), ...
            rows(use),cols(use),nr(use,direction),nc(use,direction),evidence,minRatio);
    end
end

function passed = contextPassBatch(a,b,ar,ac,br,bc,evidence,minRatio)
% Sum each small rectangle in column-major order. Unlike an integral image,
% this avoids subtracting large distant sums for weak local pole evidence.
    if isempty(a), passed = false(size(a)); return; end
    mapSize = size(evidence);
    loR = max(min(ar,br)-1,1); hiR = min(max(ar,br)+1,mapSize(1));
    loC = max(min(ac,bc)-1,1); hiC = min(max(ac,bc)+1,mapSize(2));
    height = hiR-loR+1; width = hiC-loC+1;
    count = height.*width;
    offset = 0:max(count)-1;
    queryR = loR + mod(offset,height);
    queryC = loC + floor(offset./height);
    valid = offset < count;
    idx = queryR + (queryC-1).*mapSize(1); idx(~valid) = 1;
    flatEvidence = evidence(:);
    values = flatEvidence(idx); values = reshape(values,size(idx)); values(~valid) = 0;
    context = sum(values,2);
    component = flatEvidence(a);
    pair = b > 0; component(pair) = component(pair) + flatEvidence(b(pair));
    ratio = component./context;
    passed = context <= eps | ratio > minRatio;
    % SIMD reduction order can differ from sum(rectangle,"all"). Resolve
    % floating-point threshold ties with the original local reduction. This
    % also handles cancellation when a caller supplies signed evidence.
    errorBound = 32 .* eps .* sum(abs(values),2);
    ambiguous = abs(context-eps) <= errorBound | ...
        abs(component-minRatio.*context) <= (abs(minRatio)+1).*errorBound | ...
        ~isfinite(ratio);
    for k = find(ambiguous).'
        exactContext = sum(evidence(loR(k):hiR(k),loC(k):hiC(k)),"all");
        passed(k) = exactContext <= eps || component(k)./exactContext > minRatio;
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
