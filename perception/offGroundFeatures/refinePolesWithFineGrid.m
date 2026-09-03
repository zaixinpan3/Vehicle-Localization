function [poleMaskRefined, refineDebug] = refinePolesWithFineGrid(poleMask, facadeMask, fineVoxelGrid, pointScore, lineScore, cfg)
% refinePolesWithFineGrid: Validate each pole-support component
% by reconstructing a local 3D grid for every 2D pole candidate and then
% applying a paper-aligned local-density and global-height verification.
%
% Input:
%   poleMask: [Ny x Nx] logical pole support mask after 2D reconstruction
%   facadeMask: [Ny x Nx] logical facade mask
%   fineVoxelGrid: struct with fields valid, count3D, zCenters
%   pointScore: [Ny x Nx] single point-likeness map
%   lineScore: [Ny x Nx] single line-likeness map
%   cfg: off-ground processing configuration struct
%
% Output:
%   poleMaskRefined: [Ny x Nx] logical refined pole mask
%   refineDebug: struct with per-component validation metrics
    poleMaskRefined = logical(poleMask);
    params = resolvePoleRefineParams(cfg);
    refineDebug = struct("enabled", params.enabled, "applied", false, "usedFineGrid", false, ...
        "reason", "", "fallbackReason", "", "componentCount", 0, "keptCount", 0, ...
        "componentPillarCount", zeros(0, 1), "componentLocalRatio", zeros(0, 1), ...
        "componentGlobalRatio", zeros(0, 1), "componentVerticalContinuityRatio", zeros(0, 1), ...
        "componentHeightMeters", zeros(0, 1), "componentBaseHeightMeters", zeros(0, 1), ...
        "componentTopHeightMeters", zeros(0, 1), "componentVerticalRunVoxels", zeros(0, 1), ...
        "componentMaxAreaVoxels", zeros(0, 1), "componentMaxSpanVoxels", zeros(0, 1), ...
        "componentCandidateSliceCount", zeros(0, 1), "componentObjectPointCount", zeros(0, 1), ...
        "componentNeighborhoodPointCount", zeros(0, 1), "componentKeptPillarCount", zeros(0, 1), ...
        "componentMeanPointScore", zeros(0, 1), ...
        "componentMeanLineScore", zeros(0, 1), "componentConfidence", zeros(0, 1), ...
        "componentKeepMask", false(0, 1), "componentRejectReason", {cell(0, 1)}, ...
        "preFinalCoarseMask", false(0, 0), "finalProjectionBypassMask", false(0, 0), ...
        "finalFineVoxelMask", false(0, 0, 0), "finalFineOrigin", [0, 0, 0], ...
        "finalFineVoxelSize", [1, 1, 1], "finalFineZCenters", zeros(0, 1));

    if ~params.enabled
        refineDebug.reason = "disabled";
        return;
    end
    if ~any(poleMaskRefined(:))
        refineDebug.reason = "emptyPoleMask";
        return;
    end

    count3D = zeros(0, 0, 0, "single");
    zCenters = zeros(0, 1);
    fineGridReason = "missingFineGrid";
    fineGridMeta = struct();
    if isstruct(fineVoxelGrid) && isfield(fineVoxelGrid, "valid") && logical(fineVoxelGrid.valid)
        [count3D, zCenters, fineGridReason, fineGridMeta] = resolveFineGridForRefine(fineVoxelGrid, poleMaskRefined, params);
    end
    useFineGrid = ~isempty(count3D) && (size(count3D, 3) >= 1);
    refineDebug.usedFineGrid = useFineGrid;
    if ~useFineGrid
        refineDebug.fallbackReason = fineGridReason;
    end

    ccPole = bwconncomp(poleMaskRefined, 8);
    refineDebug.componentCount = ccPole.NumObjects;
    if ccPole.NumObjects < 1
        refineDebug.reason = "noComponents";
        return;
    end

    keepFlags = false(ccPole.NumObjects, 1);
    finalProjectionBypassMask = false(size(poleMaskRefined));
    pillarCountVec = zeros(ccPole.NumObjects, 1);
    localRatioVec = zeros(ccPole.NumObjects, 1);
    globalRatioVec = zeros(ccPole.NumObjects, 1);
    continuityVec = zeros(ccPole.NumObjects, 1);
    heightVec = zeros(ccPole.NumObjects, 1);
    baseHeightVec = zeros(ccPole.NumObjects, 1);
    topHeightVec = zeros(ccPole.NumObjects, 1);
    runVec = zeros(ccPole.NumObjects, 1);
    areaVec = zeros(ccPole.NumObjects, 1);
    spanVec = zeros(ccPole.NumObjects, 1);
    candidateSliceVec = zeros(ccPole.NumObjects, 1);
    objectPointVec = zeros(ccPole.NumObjects, 1);
    neighborhoodPointVec = zeros(ccPole.NumObjects, 1);
    keptPillarCountVec = zeros(ccPole.NumObjects, 1);
    confidenceVec = zeros(ccPole.NumObjects, 1);
    meanPointVec = zeros(ccPole.NumObjects, 1);
    meanLineVec = zeros(ccPole.NumObjects, 1);
    rejectReasonVec = repmat({''}, ccPole.NumObjects, 1);
    maskKept = false(size(poleMaskRefined));
    mapSize = size(poleMaskRefined);
    keptFineVoxelMask = false(size(count3D));
    if useFineGrid
        refineDebug.finalFineOrigin(1:2) = fineGridMeta.originXY(1:2);
        refineDebug.finalFineVoxelSize(1:2) = fineGridMeta.voxelSizeXY(1:2);
        refineDebug.finalFineZCenters = double(zCenters(:));
        voxelStep = estimateVoxelStep(zCenters);
        refineDebug.finalFineOrigin(3) = zCenters(1) - (0.5 * voxelStep);
        refineDebug.finalFineVoxelSize(3) = voxelStep;
    end

    for iComp = 1:ccPole.NumObjects
        compLinIdx = ccPole.PixelIdxList{iComp};
        if useFineGrid
            [keepComp, keptCompLinIdx, metrics, keptCompFineVoxelMask] = validatePoleComponentFineGrid( ...
                compLinIdx, mapSize, facadeMask, count3D, zCenters, fineGridMeta, pointScore, lineScore, params);
        else
            [keepComp, keptCompLinIdx, metrics] = validatePoleComponent2d( ...
                compLinIdx, mapSize, pointScore, lineScore, params);
            keptCompFineVoxelMask = false(size(count3D));
        end

        keepFlags(iComp) = keepComp;
        pillarCountVec(iComp) = metrics.pillarCount;
        localRatioVec(iComp) = metrics.localRatio;
        globalRatioVec(iComp) = metrics.globalRatio;
        continuityVec(iComp) = metrics.verticalContinuityRatio;
        heightVec(iComp) = metrics.heightMeters;
        baseHeightVec(iComp) = metrics.baseHeightMeters;
        topHeightVec(iComp) = metrics.topHeightMeters;
        runVec(iComp) = metrics.verticalRun;
        areaVec(iComp) = metrics.maxArea;
        spanVec(iComp) = metrics.maxSpan;
        candidateSliceVec(iComp) = metrics.candidateSliceCount;
        objectPointVec(iComp) = metrics.objectPointCount;
        neighborhoodPointVec(iComp) = metrics.neighborhoodPointCount;
        keptPillarCountVec(iComp) = numel(keptCompLinIdx);
        meanPointVec(iComp) = metrics.meanPointScore;
        meanLineVec(iComp) = metrics.meanLineScore;
        confidenceVec(iComp) = metrics.componentConfidence;
        rejectReasonVec{iComp} = metrics.rejectReason;

        if keepComp && ~isempty(keptCompLinIdx)
            maskKept(keptCompLinIdx) = true;
            if useFineGrid && ~isempty(keptCompFineVoxelMask)
                keptFineVoxelMask = keptFineVoxelMask | logical(keptCompFineVoxelMask);
            end
        end
    end

    poleMaskRefined = maskKept;
    if useFineGrid && any(keptFineVoxelMask(:))
        projectedFineMask = mapFineMaskToComponentCoarseMask( ...
            keptFineVoxelMask, maskKept, [size(count3D, 1), size(count3D, 2)], fineGridMeta);
        poleMaskRefined = maskKept & projectedFineMask;
    end
    refineDebug.preFinalCoarseMask = poleMaskRefined;
    refineDebug.preFinalFineVoxelMask = keptFineVoxelMask;
    refineDebug.finalProjectionBypassMask = finalProjectionBypassMask;
    refineDebug.applied = true;
    if useFineGrid
        refineDebug.reason = "ok";
    else
        refineDebug.reason = "fallback2D";
    end
    refineDebug.keptCount = nnz(keepFlags);
    refineDebug.componentPillarCount = pillarCountVec;
    refineDebug.componentLocalRatio = localRatioVec;
    refineDebug.componentGlobalRatio = globalRatioVec;
    refineDebug.componentVerticalContinuityRatio = continuityVec;
    refineDebug.componentHeightMeters = heightVec;
    refineDebug.componentBaseHeightMeters = baseHeightVec;
    refineDebug.componentTopHeightMeters = topHeightVec;
    refineDebug.componentVerticalRunVoxels = runVec;
    refineDebug.componentMaxAreaVoxels = areaVec;
    refineDebug.componentMaxSpanVoxels = spanVec;
    refineDebug.componentCandidateSliceCount = candidateSliceVec;
    refineDebug.componentObjectPointCount = objectPointVec;
    refineDebug.componentNeighborhoodPointCount = neighborhoodPointVec;
    refineDebug.componentKeptPillarCount = keptPillarCountVec;
    refineDebug.componentMeanPointScore = meanPointVec;
    refineDebug.componentMeanLineScore = meanLineVec;
    refineDebug.componentConfidence = confidenceVec;
    refineDebug.componentKeepMask = keepFlags;
    refineDebug.componentRejectReason = rejectReasonVec;
    if useFineGrid
        refineDebug.finalFineVoxelMask = keptFineVoxelMask;
    end
end

function [count3D, zCenters, reason, supportMeta] = resolveFineGridForRefine(fineVoxelGrid, poleMask, ~)
% resolveFineGridForRefine: Materialize a fine 3D voxel grid for pole
% refinement using the current 2D pole support only.
%
% Input:
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
%   poleMask: [Ny x Nx] logical pole support mask
% Output:
%   count3D: [NyFine x NxFine x Nz] single voxel-count tensor
%   zCenters: [Nz x 1] double z-center coordinates
%   reason: string reason when output is empty
%   supportMeta: struct with fine-grid and coarse-grid mapping metadata
    supportMask = logical(poleMask);
    [count3D, zCenters, reason, supportMeta] = resolveFineGridForSupportMask(fineVoxelGrid, supportMask);
end

function coarseMask = mapFineMaskToComponentCoarseMask(fineVoxelMask, coarseComponentMask, fineMaskSize, supportMeta)
% mapFineMaskToComponentCoarseMask: Keep only the original coarse
% component cells whose support windows overlap accepted fine-grid voxels.
%
% Input:
%   fineVoxelMask: [NyFine x NxFine x Nz] logical global fine voxel mask
%   coarseComponentMask: [Ny x Nx] logical coarse component mask
%   fineMaskSize: [1 x 2] fine-grid size [NyFine NxFine]
%   supportMeta: struct from buildFineGridSupportMeta
%
% Output:
%   coarseMask: [Ny x Nx] logical retained coarse support mask
    coarseMask = false(size(coarseComponentMask));
    if isempty(fineVoxelMask) || isempty(coarseComponentMask) || ~any(coarseComponentMask(:))
        return;
    end

    fineXYMask = any(logical(fineVoxelMask), 3);
    if ~any(fineXYMask(:))
        return;
    end

    [coarseRows, coarseCols] = find(logical(coarseComponentMask));
    for iCell = 1:numel(coarseRows)
        [fineRowMin, fineRowMax, fineColMin, fineColMax] = mapCoarseCellToFineBounds( ...
            coarseRows(iCell), coarseCols(iCell), fineMaskSize, supportMeta);
        if fineRowMin > fineRowMax || fineColMin > fineColMax
            continue;
        end
        if any(fineXYMask(fineRowMin:fineRowMax, fineColMin:fineColMax), 'all')
            coarseMask(coarseRows(iCell), coarseCols(iCell)) = true;
        end
    end
end

function [rowMin, rowMax, colMin, colMax, localFineMask, hasSupport] = buildLocalFineSupportMaskForCoarseComponent(compLinIdx, mapSize, fineMaskSize, supportMeta)
% buildLocalFineSupportMaskForCoarseComponent: Convert one coarse
% connected component into a padded fine-grid crop and a local logical
% support mask without materializing a full-frame fine XY mask.
%
% Input:
%   compLinIdx: [K x 1] linear indices of one coarse component
%   mapSize: [1 x 2] coarse map size [Ny Nx]
%   fineMaskSize: [1 x 2] fine map size [NyFine NxFine]
%   supportMeta: struct from buildFineGridSupportMeta
%
% Output:
%   rowMin, rowMax, colMin, colMax: inclusive fine-grid crop bounds
%   localFineMask: logical support mask in the padded crop
%   hasSupport: logical scalar indicating that at least one fine cell maps
%       to the coarse component
    rowMin = 1;
    rowMax = 0;
    colMin = 1;
    colMax = 0;
    localFineMask = false(0, 0);
    hasSupport = false;

    if isempty(compLinIdx) || numel(mapSize) < 2 || numel(fineMaskSize) < 2 || any(fineMaskSize(1:2) < 1)
        return;
    end

    [coarseRows, coarseCols] = ind2sub(mapSize, double(compLinIdx(:)));
    coarseRows = coarseRows(:);
    coarseCols = coarseCols(:);
    numCells = numel(coarseRows);
    fineRowMinVec = zeros(numCells, 1);
    fineRowMaxVec = zeros(numCells, 1);
    fineColMinVec = zeros(numCells, 1);
    fineColMaxVec = zeros(numCells, 1);
    validCell = false(numCells, 1);
    for iCell = 1:numCells
        [fineRowMin, fineRowMax, fineColMin, fineColMax] = mapCoarseCellToFineBounds( ...
            coarseRows(iCell), coarseCols(iCell), fineMaskSize, supportMeta);
        if fineRowMin <= fineRowMax && fineColMin <= fineColMax
            fineRowMinVec(iCell) = fineRowMin;
            fineRowMaxVec(iCell) = fineRowMax;
            fineColMinVec(iCell) = fineColMin;
            fineColMaxVec(iCell) = fineColMax;
            validCell(iCell) = true;
        end
    end
    if ~any(validCell)
        return;
    end

    supportRowMin = min(fineRowMinVec(validCell));
    supportRowMax = max(fineRowMaxVec(validCell));
    supportColMin = min(fineColMinVec(validCell));
    supportColMax = max(fineColMaxVec(validCell));
    rowMin = max(1, supportRowMin - 1);
    rowMax = min(fineMaskSize(1), supportRowMax + 1);
    colMin = max(1, supportColMin - 1);
    colMax = min(fineMaskSize(2), supportColMax + 1);
    localFineMask = false(rowMax - rowMin + 1, colMax - colMin + 1);
    validIdx = find(validCell);
    for iValid = 1:numel(validIdx)
        iCell = validIdx(iValid);
        localRows = (fineRowMinVec(iCell):fineRowMaxVec(iCell)) - rowMin + 1;
        localCols = (fineColMinVec(iCell):fineColMaxVec(iCell)) - colMin + 1;
        localFineMask(localRows, localCols) = true;
    end
    hasSupport = any(localFineMask(:));
end

function metrics = initializePoleComponentMetrics(compLinIdx, mapSize, pointScore, lineScore)
% initializePoleComponentMetrics: Build a reusable metrics template
% with 2D footprint statistics and upstream shape-score summaries for one
% pole component before optional 3D refinement updates.
%
% Input:
%   compLinIdx: [K x 1] linear indices of one pole component in [Ny x Nx]
%   mapSize: [1 x 2] size of the [Ny x Nx] map
%   pointScore: [Ny x Nx] single point-likeness map
%   lineScore: [Ny x Nx] single line-likeness map
%
% Output:
%   metrics: struct initialized with footprint, score, and placeholder
%       fields for later 3D verification results
    metrics = struct("pillarCount", 0, "localRatio", 0, "globalRatio", 0, ...
        "verticalContinuityRatio", 0, "heightMeters", 0, "baseHeightMeters", 0, ...
        "topHeightMeters", 0, "verticalRun", 0, "maxArea", 0, "maxSpan", 0, ...
        "candidateSliceCount", 0, "objectPointCount", 0, "neighborhoodPointCount", 0, ...
        "meanPointScore", 0, "meanLineScore", 0, ...
        "componentConfidence", 0, "rejectReason", "");

    compLinIdx = double(compLinIdx(:));
    if isempty(compLinIdx)
        metrics.rejectReason = "emptyComponent";
        return;
    end

    [rowIdx, colIdx] = ind2sub(mapSize, compLinIdx);
    spanRow = (max(rowIdx) - min(rowIdx)) + 1;
    spanCol = (max(colIdx) - min(colIdx)) + 1;

    metrics.pillarCount = numel(compLinIdx);
    metrics.maxArea = metrics.pillarCount;
    metrics.maxSpan = max(spanRow, spanCol);
    metrics.meanPointScore = mean(double(pointScore(compLinIdx)));
    metrics.meanLineScore = mean(double(lineScore(compLinIdx)));
end

function [keepComp, keptLinIdx, metrics, keptFineVoxelMask] = validatePoleComponentFineGrid(compLinIdx, mapSize, ~, count3D, zCenters, fineGridMeta, pointScore, lineScore, params)
% validatePoleComponentFineGrid: Verify one 2D pole component by
% treating its full fine-column footprint as one object, slicing that
% object horizontally through the local 3D fine grid, and retaining the
% component when z slices with sufficient component-to-neighborhood density
% ratio form a valid vertical run.
%
% Input:
%   compLinIdx: [K x 1] linear indices of one pole component in [Ny x Nx]
%   mapSize: [1 x 2] size of the [Ny x Nx] map
%   count3D: [NyFine x NxFine x Nz] single voxel-count tensor
%   zCenters: [Nz x 1] double z-center coordinates
%   fineGridMeta: struct with fine-grid and coarse-grid mapping metadata
%   pointScore: [Ny x Nx] single point-likeness map
%   lineScore: [Ny x Nx] single line-likeness map
%   params: struct from resolvePoleRefineParams
%
% Output:
%   keepComp: logical scalar keep/remove decision
%   keptLinIdx: [M x 1] refined subset of component pillars kept after 3D verification
%   metrics: struct of measured criteria values
%   keptFineVoxelMask: [NyFine x NxFine x Nz] logical final validated fine
%       voxel mask for this component in the global fine-grid frame
    keepComp = false;
    keptLinIdx = zeros(0, 1);
    metrics = initializePoleComponentMetrics(compLinIdx, mapSize, pointScore, lineScore);
    keptFineVoxelMask = [];

    compLinIdx = double(compLinIdx(:));
    if isempty(compLinIdx) || isempty(count3D) || size(count3D, 3) < 1
        metrics.rejectReason = "missingFineGrid";
        return;
    end

    voxelStep = estimateVoxelStep(zCenters);

    fineRows = size(count3D, 1);
    fineCols = size(count3D, 2);
    usesFineXY = (fineRows ~= mapSize(1)) || (fineCols ~= mapSize(2));

    if usesFineXY
        compMaskCoarse = false(mapSize);
        compMaskCoarse(compLinIdx) = true;
        [rowMin, rowMax, colMin, colMax, localCompMask, hasLocalFineSupport] = ...
            buildLocalFineSupportMaskForCoarseComponent(compLinIdx, mapSize, [fineRows, fineCols], fineGridMeta);
        if ~hasLocalFineSupport
            metrics.rejectReason = "missingFineGrid";
            return;
        end
        subVol = double(count3D(rowMin:rowMax, colMin:colMax, :));
    else
        [rowIdx, colIdx] = ind2sub(mapSize, compLinIdx);
        rowMin = max(1, min(rowIdx) - 1);
        rowMax = min(mapSize(1), max(rowIdx) + 1);
        colMin = max(1, min(colIdx) - 1);
        colMax = min(mapSize(2), max(colIdx) + 1);
        subVol = double(count3D(rowMin:rowMax, colMin:colMax, :));
        localRows = rowIdx - rowMin + 1;
        localCols = colIdx - colMin + 1;
        localCompMask = false(size(subVol, 1), size(subVol, 2));
        localCompMask(sub2ind(size(localCompMask), localRows, localCols)) = true;
    end

    occupiedColumnMask = localCompMask & any(subVol > 0, 3);
    if ~any(occupiedColumnMask(:))
        metrics.rejectReason = "noSupportMask";
        return;
    end

    [componentRatioZ, componentNeighborhoodCount, componentObjectCount, componentRedZMask] = ...
        computePoleCoreSliceMetrics(subVol, localCompMask, params.purpleRatioThreshold, ...
        params.purpleRatioBlockSize, params.purpleMinSliceObjectPoints, params.occupiedLayerMinPoints);
    objectLayerMask = componentObjectCount >= params.occupiedLayerMinPoints;
    objectLayerCount = nnz(objectLayerMask);
    minObjectLayerCount = params.minCandidateSliceCount;
    if params.relaxedMinCandidateSlicesEnabled
        minObjectLayerCount = max(1, minObjectLayerCount - 1);
    end
    if objectLayerCount < minObjectLayerCount
        metrics.candidateSliceCount = objectLayerCount;
        metrics.objectPointCount = sum(componentObjectCount(objectLayerMask));
        metrics.neighborhoodPointCount = sum(componentNeighborhoodCount(objectLayerMask));
        metrics.rejectReason = "objectLayerCount";
        return;
    end

    bestCoreMetrics = measurePoleCoreSlices( ...
        componentRedZMask, componentRatioZ, componentNeighborhoodCount, componentObjectCount, zCenters, voxelStep, params.minCandidateSliceCount);
    if ~bestCoreMetrics.valid && params.relaxedMinCandidateSlicesEnabled && params.minCandidateSliceCount > 1
        relaxedCoreMetrics = measurePoleCoreSlices( ...
            componentRedZMask, componentRatioZ, componentNeighborhoodCount, componentObjectCount, zCenters, voxelStep, params.minCandidateSliceCount - 1);
        if relaxedCoreMetrics.valid && relaxedCoreMetrics.baseHeightMeters <= params.relaxedMaxBaseHeightMeters ...
                && relaxedCoreMetrics.localRatio >= params.relaxedMinLocalRatio ...
                && relaxedCoreMetrics.globalRatio >= params.relaxedMinGlobalRatio
            bestCoreMetrics = relaxedCoreMetrics;
        end
    end
    if ~bestCoreMetrics.valid
        metrics.rejectReason = "qualifiedSliceCount";
        return;
    end
    if bestCoreMetrics.localRatio < params.minLocalRatio || bestCoreMetrics.globalRatio < params.minGlobalRatio
        metrics.candidateSliceCount = bestCoreMetrics.candidateSliceCount;
        metrics.objectPointCount = bestCoreMetrics.objectPointCount;
        metrics.neighborhoodPointCount = bestCoreMetrics.neighborhoodPointCount;
        metrics.localRatio = bestCoreMetrics.localRatio;
        metrics.globalRatio = bestCoreMetrics.globalRatio;
        metrics.rejectReason = "support";
        return;
    end
    finalRedZMask = false(size(componentRedZMask));
    finalRedZMask(bestCoreMetrics.validSliceIdx) = true;
    finalRedVoxelMask = reshape(localCompMask, size(localCompMask, 1), size(localCompMask, 2), 1) & ...
        (subVol > 0) & reshape(finalRedZMask, 1, 1, []);
    finalProjection = any(finalRedVoxelMask, 3);
    if ~any(finalProjection(:)) || ~bestCoreMetrics.valid
        metrics.rejectReason = "qualifiedSliceCount";
        return;
    end

    metrics.verticalRun = bestCoreMetrics.verticalRun;
    metrics.candidateSliceCount = bestCoreMetrics.candidateSliceCount;
    metrics.objectPointCount = bestCoreMetrics.objectPointCount;
    metrics.neighborhoodPointCount = bestCoreMetrics.neighborhoodPointCount;
    metrics.localRatio = bestCoreMetrics.localRatio;
    metrics.globalRatio = bestCoreMetrics.globalRatio;
    metrics.verticalContinuityRatio = bestCoreMetrics.verticalContinuityRatio;
    metrics.baseHeightMeters = bestCoreMetrics.baseHeightMeters;
    metrics.topHeightMeters = bestCoreMetrics.topHeightMeters;
    metrics.heightMeters = bestCoreMetrics.heightMeters;
    metrics.componentConfidence = clamp01(metrics.localRatio);
    metrics.maxArea = nnz(finalProjection);
    [finalRows, finalCols] = find(finalProjection);
    if ~isempty(finalRows)
        metrics.maxSpan = max((max(finalRows) - min(finalRows)) + 1, (max(finalCols) - min(finalCols)) + 1);
    end

    keptFineVoxelMask = false(size(count3D));
    if usesFineXY
        keptFineVoxelMask(rowMin:rowMax, colMin:colMax, :) = finalRedVoxelMask;
        validatedCoarseMask = mapFineMaskToComponentCoarseMask(keptFineVoxelMask, compMaskCoarse, [fineRows, fineCols], fineGridMeta);
        keptLinIdx = find(validatedCoarseMask);
    else
        keptFineVoxelMask(rowMin:rowMax, colMin:colMax, :) = finalRedVoxelMask;
        [projRows, projCols] = find(finalProjection);
        keptLinIdx = sub2ind(mapSize, projRows + rowMin - 1, projCols + colMin - 1);
        keptLinIdx = unique(double(keptLinIdx(:)));
    end
    if isempty(keptLinIdx)
        metrics.rejectReason = "qualifiedSliceCount";
        return;
    end

    refinedMetrics = initializePoleComponentMetrics(keptLinIdx, mapSize, pointScore, lineScore);
    metrics.pillarCount = refinedMetrics.pillarCount;
    metrics.meanPointScore = refinedMetrics.meanPointScore;
    metrics.meanLineScore = refinedMetrics.meanLineScore;
    if metrics.meanPointScore < params.minMeanPointScore
        metrics.rejectReason = "pointScore";
        return;
    end
    keepComp = true;
    metrics.rejectReason = "";
end

function coreMetrics = measurePoleCoreSlices(redZMask, ratioZ, neighborhoodCount, objectCount, zCenters, voxelStep, minQualifiedSlices)
% measurePoleCoreSlices: Reduce a validated red-slice mask to support
% ratios and height statistics by retaining every z-slice that passes the
% slice-level component-to-neighborhood density test.
%
% Input:
%   redZMask: [Nz x 1] logical qualifying z-slice mask
%   ratioZ: [Nz x 1] double per-slice or expanded blockwise ratios
%   neighborhoodCount: [Nz x 1] double neighborhood point counts
%   objectCount: [Nz x 1] double component point counts
%   zCenters: [Nz x 1] double z-center coordinates
%   voxelStep: optional scalar positive z-step override
%   minQualifiedSlices: scalar minimum number of qualified slices required
%       for the component to pass fine validation
%
% Output:
%   coreMetrics: struct with validity, support, continuity, and height
%       statistics plus the retained slice indices and voxel spacing
    if nargin < 6 || ~(isfinite(voxelStep) && (voxelStep > 0))
        voxelStep = estimateVoxelStep(zCenters);
    end
    if nargin < 7 || ~isscalar(minQualifiedSlices) || ~isfinite(minQualifiedSlices)
        minQualifiedSlices = 1;
    end
    minQualifiedSlices = max(1, round(double(minQualifiedSlices)));

    coreMetrics = struct("valid", false, "validSliceIdx", zeros(0, 1), ...
        "voxelStep", voxelStep, "verticalRun", 0, ...
        "candidateSliceCount", 0, "objectPointCount", 0, ...
        "neighborhoodPointCount", 0, "localRatio", 0, "globalRatio", 0, ...
        "verticalContinuityRatio", 0, "baseHeightMeters", 0, ...
        "topHeightMeters", 0, "heightMeters", 0);

    redZMask = logical(redZMask(:));
    if nnz(redZMask) < minQualifiedSlices
        return;
    end
    runMask = redZMask;
    validSliceIdx = find(runMask);

    coreMetrics.valid = true;
    coreMetrics.validSliceIdx = double(validSliceIdx(:));
    coreMetrics.verticalRun = numel(validSliceIdx);
    coreMetrics.candidateSliceCount = numel(validSliceIdx);
    coreMetrics.objectPointCount = sum(objectCount(runMask));
    coreMetrics.neighborhoodPointCount = sum(neighborhoodCount(runMask));
    coreMetrics.localRatio = mean(ratioZ(runMask));
    if coreMetrics.neighborhoodPointCount > 0
        coreMetrics.globalRatio = coreMetrics.objectPointCount ./ coreMetrics.neighborhoodPointCount;
    end

    runStarts = find(diff([false; runMask; false]) == 1);
    runEnds = find(diff([false; runMask; false]) == -1) - 1;
    if isempty(runStarts)
        coreMetrics.verticalContinuityRatio = 0;
    else
        coreMetrics.verticalContinuityRatio = max(runEnds - runStarts + 1) ./ numel(validSliceIdx);
    end
    coreMetrics.baseHeightMeters = zCenters(validSliceIdx(1)) - (0.5 * coreMetrics.voxelStep);
    coreMetrics.topHeightMeters = zCenters(validSliceIdx(end)) + (0.5 * coreMetrics.voxelStep);
    coreMetrics.heightMeters = max(0, coreMetrics.topHeightMeters - coreMetrics.baseHeightMeters);
end

function [rowMinRect, rowMaxRect, colMinRect, colMaxRect] = computePoleNeighborhoodBounds(rowIdx, colIdx, mapSize)
% computePoleNeighborhoodBounds: Expand one fine 2D component to a
% one-cell padded rectangular neighborhood, clamping the bounds to the
% local map before the per-slice object-to-neighborhood ratio is measured.
%
% Input:
%   rowIdx: [K x 1] row indices of the fine component inside a local map
%   colIdx: [K x 1] column indices of the fine component inside a local map
%   mapSize: [1 x 2] size of the local [rows x cols] map
%
% Output:
%   rowMinRect: scalar lower row bound of the padded rectangle
%   rowMaxRect: scalar upper row bound of the padded rectangle
%   colMinRect: scalar lower column bound of the padded rectangle
%   colMaxRect: scalar upper column bound of the padded rectangle
    rowMinRect = max(1, min(rowIdx) - 1);
    rowMaxRect = min(mapSize(1), max(rowIdx) + 1);
    colMinRect = max(1, min(colIdx) - 1);
    colMaxRect = min(mapSize(2), max(colIdx) + 1);
end

function redZMask = buildQualifiedRedSliceMask(redCandidateZ, sliceCount)
% buildQualifiedRedSliceMask: Mark every ratio-qualified z-slice as
% red without applying any contiguous run-length filter.
%
% Input:
%   redCandidateZ: [K x 1] double indices of candidate z-slices
%   sliceCount: scalar total number of z-slices in the local subvolume
%
% Output:
%   redZMask: [sliceCount x 1] logical qualifying z-slice mask
    redZMask = false(sliceCount, 1);
    if isempty(redCandidateZ)
        return;
    end

    redZMask(redCandidateZ) = true;
end

function [keepComp, keptLinIdx, metrics] = validatePoleComponent2d(compLinIdx, mapSize, pointScore, lineScore, params)
% validatePoleComponent2d: Score one pole-support component from 2D
% maps only, providing a graceful fallback when fine 3D data is absent by
% applying direct threshold checks instead of the 3D slice-density score.
%
% Input:
%   compLinIdx: [K x 1] linear indices of one pole component in [Ny x Nx]
%   mapSize: [1 x 2] size of the [Ny x Nx] map
%   pointScore: [Ny x Nx] single point-likeness map
%   lineScore: [Ny x Nx] single line-likeness map
%   params: struct from resolvePoleRefineParams
%
% Output:
%   keepComp: logical scalar keep/remove decision
%   keptLinIdx: [K x 1] linear indices of the 2D component kept in fallback mode
%   metrics: struct of measured criteria values
    keepComp = false;
    keptLinIdx = double(compLinIdx(:));
    metrics = initializePoleComponentMetrics(compLinIdx, mapSize, pointScore, lineScore);
    compLinIdx = double(compLinIdx(:));
    if isempty(compLinIdx)
        return;
    end

    compPoint = double(pointScore(compLinIdx));
    compLine = double(lineScore(compLinIdx));
    pointDominance = compPoint ./ max(compPoint + compLine, eps);
    metrics.localRatio = mean(pointDominance);
    metrics.globalRatio = mean(pointDominance);
    metrics.objectPointCount = metrics.pillarCount;
    metrics.neighborhoodPointCount = metrics.pillarCount;
    metrics.candidateSliceCount = metrics.pillarCount;

    metrics.componentConfidence = clamp01(metrics.localRatio);

    minLocalRatio = params.minSupportRatio;
    minGlobalRatio = max(0.5 * params.minSupportRatio, 0.05);
    if metrics.pillarCount > 1
        minLocalRatio = min(minLocalRatio, params.wideMinLocalRatio);
        minGlobalRatio = max(minGlobalRatio, params.wideMinGlobalRatio);
    end

    keepComp = (metrics.localRatio >= minLocalRatio) && ...
        (metrics.globalRatio >= minGlobalRatio) && ...
        (metrics.maxSpan <= params.maxFootprintWidthVoxels) && ...
        (metrics.meanLineScore <= params.maxMeanLineScore);
    if keepComp
        metrics.rejectReason = "";
        return;
    end
    if metrics.localRatio < minLocalRatio
        metrics.rejectReason = "support";
        return;
    end
    if metrics.globalRatio < minGlobalRatio
        metrics.rejectReason = "shape";
        return;
    end
    if metrics.maxSpan > params.maxFootprintWidthVoxels
        metrics.rejectReason = "footprint";
        return;
    end
    if metrics.meanLineScore > params.maxMeanLineScore
        metrics.rejectReason = "lineScore";
        return;
    end
    metrics.rejectReason = "support";
end

function [ratioZ, neighborhoodCount, objectCount, redZMask] = computePoleCoreSliceMetrics(subVol, componentMask, purpleRatioThreshold, purpleRatioBlockSize, purpleMinSliceObjectPoints, occupiedLayerMinPoints)
% computePoleCoreSliceMetrics: Reduce one local fine-grid component
% to per-slice object and neighborhood counts, qualify z slices using the
% component-to-neighborhood point-density ratio, and optionally aggregate
% adjacent slices with a short sliding block.
%
% Input:
%   subVol: [Hr x Wr x Nz] double local voxel-count tensor
%   componentMask: [Hr x Wr] logical local fine-column component mask
%   purpleRatioThreshold: scalar minimum component-to-neighborhood density
%       ratio required for one slice or slice block to qualify
%   purpleRatioBlockSize: scalar number of adjacent slices to aggregate for
%       density-ratio qualification
%   purpleMinSliceObjectPoints: scalar minimum component points required
%       for one z-slice or block to qualify as density-supported
%   occupiedLayerMinPoints: scalar minimum component points required for
%       an individual layer to count as occupied for candidate validation
%
% Output:
%   ratioZ: [Nz x 1] double per-slice ratio values
%   neighborhoodCount: [Nz x 1] double neighborhood point counts
%   objectCount: [Nz x 1] double component point counts
%   redZMask: [Nz x 1] logical qualified red-slice mask over slices where
%       the full component has nonzero object points
    sliceCount = size(subVol, 3);
    ratioZ = zeros(sliceCount, 1);
    neighborhoodCount = zeros(sliceCount, 1);
    objectCount = zeros(sliceCount, 1);
    redZMask = false(sliceCount, 1);
    if isempty(subVol) || isempty(componentMask) || ~any(componentMask(:))
        return;
    end
    if nargin < 6 || ~isscalar(occupiedLayerMinPoints) || ~isfinite(occupiedLayerMinPoints)
        occupiedLayerMinPoints = purpleMinSliceObjectPoints;
    end
    occupiedLayerMinPoints = max(1, round(double(occupiedLayerMinPoints)));

    [compRows, compCols] = find(componentMask);
    [rowMinRect, rowMaxRect, colMinRect, colMaxRect] = computePoleNeighborhoodBounds(compRows, compCols, size(componentMask));
    compSubVol = subVol(rowMinRect:rowMaxRect, colMinRect:colMaxRect, :);
    localComponentMask = componentMask(rowMinRect:rowMaxRect, colMinRect:colMaxRect);
    componentSubVol = compSubVol .* reshape(localComponentMask, size(localComponentMask, 1), size(localComponentMask, 2), 1);
    neighborhoodCount = permute(sum(compSubVol, [1, 2]), [3, 1, 2]);
    neighborhoodCount = double(neighborhoodCount(:));
    objectCount = permute(sum(componentSubVol, [1, 2]), [3, 1, 2]);
    objectCount = double(objectCount(:));

    validRatio = neighborhoodCount > 0;
    ratioZ(validRatio) = objectCount(validRatio) ./ neighborhoodCount(validRatio);
    if purpleRatioBlockSize == 1
        redCandidateZ = find((objectCount >= max(purpleMinSliceObjectPoints, occupiedLayerMinPoints)) & ...
            (ratioZ > purpleRatioThreshold));
        redZMask = buildQualifiedRedSliceMask(redCandidateZ, numel(objectCount));
        return;
    end

    blockSize = min(max(1, round(double(purpleRatioBlockSize))), numel(objectCount));
    blockCount = numel(objectCount) - blockSize + 1;
    blockObjectCount = conv(objectCount, ones(blockSize, 1), "valid");
    blockNeighborhoodCount = conv(neighborhoodCount, ones(blockSize, 1), "valid");
    blockRatio = zeros(blockCount, 1);
    validBlockRatio = blockNeighborhoodCount > 0;
    blockRatio(validBlockRatio) = blockObjectCount(validBlockRatio) ./ blockNeighborhoodCount(validBlockRatio);
    blockMinObjectCount = max(1, purpleMinSliceObjectPoints * blockSize);
    redCandidateBlocks = find((blockObjectCount >= blockMinObjectCount) & ...
        (blockRatio > purpleRatioThreshold));
    for iBlock = 1:numel(redCandidateBlocks)
        blockStart = redCandidateBlocks(iBlock);
        blockSlices = blockStart:(blockStart + blockSize - 1);
        occupiedBlockSlices = blockSlices(objectCount(blockSlices) >= occupiedLayerMinPoints);
        redZMask(occupiedBlockSlices) = true;
        ratioZ(occupiedBlockSlices) = max(ratioZ(occupiedBlockSlices), blockRatio(blockStart));
    end
end

function params = resolvePoleRefineParams(cfg)
% resolvePoleRefineParams: Read and sanitize the pole 3D refinement
% parameters that drive support-ratio, continuity, height, footprint, and
% final-projection validation while preserving older aliases and the 2D
% fallback thresholds.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   params: struct with validated scalar refinement parameters and kernels
    params = struct();
    params.enabled = true;
    if isfield(cfg, "poleRefineEnabled") && isscalar(cfg.poleRefineEnabled)
        params.enabled = logical(cfg.poleRefineEnabled);
    end

    params.horizontalStitchRadius = 1;
    if isfield(cfg, "poleRefineHorizontalStitchRadiusVoxels") && isscalar(cfg.poleRefineHorizontalStitchRadiusVoxels) && isfinite(cfg.poleRefineHorizontalStitchRadiusVoxels)
        params.horizontalStitchRadius = max(0, round(double(cfg.poleRefineHorizontalStitchRadiusVoxels)));
    elseif isfield(cfg, "poleAdaptiveNeighborhoodRadiusVoxels") && isscalar(cfg.poleAdaptiveNeighborhoodRadiusVoxels) && isfinite(cfg.poleAdaptiveNeighborhoodRadiusVoxels)
        params.horizontalStitchRadius = max(0, round(double(cfg.poleAdaptiveNeighborhoodRadiusVoxels)));
    end

    params.verticalStitchRadius = 1;
    if isfield(cfg, "poleRefineVerticalStitchRadiusVoxels") && isscalar(cfg.poleRefineVerticalStitchRadiusVoxels) && isfinite(cfg.poleRefineVerticalStitchRadiusVoxels)
        params.verticalStitchRadius = max(0, round(double(cfg.poleRefineVerticalStitchRadiusVoxels)));
    elseif isfield(cfg, "poleVerticalBridgeRadiusVoxels") && isscalar(cfg.poleVerticalBridgeRadiusVoxels) && isfinite(cfg.poleVerticalBridgeRadiusVoxels)
        params.verticalStitchRadius = max(0, round(double(cfg.poleVerticalBridgeRadiusVoxels)));
    end

    params.verticalBridgeRadius = 1;
    if isfield(cfg, "poleVerticalBridgeRadiusVoxels") && isscalar(cfg.poleVerticalBridgeRadiusVoxels) && isfinite(cfg.poleVerticalBridgeRadiusVoxels)
        params.verticalBridgeRadius = max(0, round(double(cfg.poleVerticalBridgeRadiusVoxels)));
    else
        params.verticalBridgeRadius = params.verticalStitchRadius;
    end

    params.energyWeight = 0.5;
    if isfield(cfg, "poleEnergyWeight") && isscalar(cfg.poleEnergyWeight) && isfinite(cfg.poleEnergyWeight)
        params.energyWeight = min(max(double(cfg.poleEnergyWeight), 0), 1);
    end

    params.minCandidateSliceCount = 2;
    if isfield(cfg, "poleRefineMinCandidateSlices") && isscalar(cfg.poleRefineMinCandidateSlices) && isfinite(cfg.poleRefineMinCandidateSlices)
        params.minCandidateSliceCount = max(1, round(double(cfg.poleRefineMinCandidateSlices)));
    end
    params.occupiedLayerMinPoints = resolvePoleOccupiedLayerMinPoints(cfg);

    params.minLocalRatio = 0.70;
    if isfield(cfg, "poleRefineMinLocalRatio") && isscalar(cfg.poleRefineMinLocalRatio) && isfinite(cfg.poleRefineMinLocalRatio)
        params.minLocalRatio = min(max(double(cfg.poleRefineMinLocalRatio), 0), 1);
    end

    params.minGlobalRatio = 0.70;
    if isfield(cfg, "poleRefineMinGlobalRatio") && isscalar(cfg.poleRefineMinGlobalRatio) && isfinite(cfg.poleRefineMinGlobalRatio)
        params.minGlobalRatio = min(max(double(cfg.poleRefineMinGlobalRatio), 0), 1);
    end

    params.minMeanPointScore = 0;
    if isfield(cfg, "poleRefineMinMeanPointScore") && isscalar(cfg.poleRefineMinMeanPointScore) && isfinite(cfg.poleRefineMinMeanPointScore)
        params.minMeanPointScore = min(max(double(cfg.poleRefineMinMeanPointScore), 0), 1);
    end

    params.purpleRatioThreshold = 0.50;
    if isfield(cfg, "purpleRatioThreshold") && isscalar(cfg.purpleRatioThreshold) && isfinite(cfg.purpleRatioThreshold)
        params.purpleRatioThreshold = min(max(double(cfg.purpleRatioThreshold), 0), 1);
    end

    params.purpleRatioBlockSize = 1;
    if isfield(cfg, "purpleRatioBlockSize") && isscalar(cfg.purpleRatioBlockSize) && isfinite(cfg.purpleRatioBlockSize)
        params.purpleRatioBlockSize = max(1, round(double(cfg.purpleRatioBlockSize)));
    elseif isfield(cfg, "poleRefinePurpleRatioBlockSize") && isscalar(cfg.poleRefinePurpleRatioBlockSize) && isfinite(cfg.poleRefinePurpleRatioBlockSize)
        params.purpleRatioBlockSize = max(1, round(double(cfg.poleRefinePurpleRatioBlockSize)));
    end

    params.purpleMinSliceObjectPoints = params.occupiedLayerMinPoints;
    if isfield(cfg, "purpleMinSliceObjectPoints") && isscalar(cfg.purpleMinSliceObjectPoints) && isfinite(cfg.purpleMinSliceObjectPoints)
        params.purpleMinSliceObjectPoints = max(1, round(double(cfg.purpleMinSliceObjectPoints)));
    end

    params.relaxedMinCandidateSlicesEnabled = false;
    if isfield(cfg, "poleRefineRelaxedMinCandidateSlicesEnabled") && isscalar(cfg.poleRefineRelaxedMinCandidateSlicesEnabled)
        params.relaxedMinCandidateSlicesEnabled = logical(cfg.poleRefineRelaxedMinCandidateSlicesEnabled);
    end

    params.relaxedMaxBaseHeightMeters = 0.5;
    if isfield(cfg, "poleRefineRelaxedMaxBaseHeightMeters") && isscalar(cfg.poleRefineRelaxedMaxBaseHeightMeters) && isfinite(cfg.poleRefineRelaxedMaxBaseHeightMeters)
        params.relaxedMaxBaseHeightMeters = double(cfg.poleRefineRelaxedMaxBaseHeightMeters);
    end

    params.relaxedMinLocalRatio = 0.75;
    if isfield(cfg, "poleRefineRelaxedMinLocalRatio") && isscalar(cfg.poleRefineRelaxedMinLocalRatio) && isfinite(cfg.poleRefineRelaxedMinLocalRatio)
        params.relaxedMinLocalRatio = min(max(double(cfg.poleRefineRelaxedMinLocalRatio), 0), 1);
    end

    params.relaxedMinGlobalRatio = 0.70;
    if isfield(cfg, "poleRefineRelaxedMinGlobalRatio") && isscalar(cfg.poleRefineRelaxedMinGlobalRatio) && isfinite(cfg.poleRefineRelaxedMinGlobalRatio)
        params.relaxedMinGlobalRatio = min(max(double(cfg.poleRefineRelaxedMinGlobalRatio), 0), 1);
    end

    params.minSupportRatio = 0.12;
    params.maxFootprintWidthVoxels = 3;
    params.maxMeanLineScore = 1;
    params.wideMinLocalRatio = 0.82;
    params.wideMinGlobalRatio = 0.75;
    params.neighborhoodRadius = params.horizontalStitchRadius;
    params.horizontalKernel = true((2 * params.horizontalStitchRadius) + 1, (2 * params.horizontalStitchRadius) + 1);
    params.neighborhoodKernel = params.horizontalKernel;
end
