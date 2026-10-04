function [ground,curb] = detectDowntownCurbs(context,ground,allPoints,cfg,candidateCellMask,supportCellMask)
% detectDowntownCurbs: Refine points strictly within existing curb candidates.
% The accepted joint reference selects a narrow dominant boundary, removes
% short fragments and elevated obstacles, and extends only measured samples
% supported by clear straight anchors. This is an offline point validation
% stage. Coarse candidates and their whole-pillar moments remain unchanged.
% No point outside the supplied candidate envelope can acquire a curb label.
%
% Input:
%   context: groundPoints, groundOriginalPointIdx, groundCellLinIdx and groundXYView.
%   ground: analyzeGroundPillars output, including initial road and energy maps.
%   allPoints: retained raw ground and off-ground XYZ coordinates in meters.
%   cfg: downtownCurbConfig for this lattice spacing.
%   candidateCellMask: optional native [Ny Nx] coarse curb candidate envelope;
%       defaults to ground.curbCellMask and remains immutable.
% Output:
%   ground: unchanged input coarse product.
%   curb: original point indices, candidate/accepted masks, native cell mask,
%       and boundary, component, obstacle, and endpoint diagnostics.
    points = double(context.groundPoints);
    cells = double(context.groundCellLinIdx(:));
    xy = context.groundXYView;
    maps = ground.energyMaps;
    initialRoad = ground.initialRoadResult;
    if nargin<5, candidateCellMask=ground.curbCellMask; end
    assert(isequal(size(candidateCellMask),size(xy.countMap)), ...
        'Curb candidates must use the existing ground raster.');
    eligible = sampleCellMapAtPoints(cells,candidateCellMask)>0;
    % Neighboring support stabilizes narrow-boundary fitting when precision
    % gating fragments the object. Only eligible points receive final labels.
    if nargin<6,supportCellMask=candidateCellMask;end
    supportEligible=sampleCellMapAtPoints(cells,supportCellMask)>0;
    candidates = supportEligible & sampleCellMapAtPoints(cells,maps.extractedMask)>0;
    accepted = filterNearRoadPoints(candidates,points,cells,ground.stats, ...
        maps,initialRoad.roadCellMask,cfg);
    accepted = filterDominantBoundaryPoints(accepted,points,cells,xy, ...
        initialRoad.roadSeedMask,initialRoad.roadCellMask,maps,cfg);
    curb = struct('pointIndices',double(context.groundOriginalPointIdx(:)), ...
        'candidateMask',eligible,'beforeSupportMask',accepted);
    mapSize = size(xy.countMap);
    curb.supportValidation = table();
    if cfg.pointSupport.enabled
        [accepted,curb.supportValidation] = filterDowntownCurbPointSupport( ...
            points,cells,mapSize,accepted,cfg.pointSupport);
    end
    curb.beforeObstacleMask = accepted;
    curb.obstacleValidation = table();
    if cfg.obstacleClearance.enabled
        [accepted,curb.obstacleValidation] = filterDowntownCurbObstacles( ...
            points,allPoints,cells,mapSize,accepted,cfg.obstacleClearance);
    end
    curb.beforeExtensionMask = accepted;
    curb.extensionValidation = table();
    curb.addedMask = false(size(accepted));
    if cfg.endpointExtension.enabled
        assert(cfg.obstacleClearance.enabled,'Curb extension requires obstacle clearance.');
        [accepted,curb.extensionValidation,curb.addedMask] = extendDowntownCurbEndpoints( ...
            points,allPoints,cells,mapSize,maps,accepted,cfg.endpointExtension,cfg.obstacleClearance,supportEligible);
    end
    curb.acceptedMask = accepted & eligible;
    cellMask = false(fliplr(mapSize));
    cellMask(cells(curb.acceptedMask)) = true;
    curb.cellMask = cellMask.';
end

function curbPointMask = filterNearRoadPoints(extractedPointMask, groundPoints, groundCellLinIdx, stats, energyMaps, roadCellMask, cfg)
    curbPointMask = logical(extractedPointMask(:));
    if ~isstruct(cfg) || ~isfield(cfg, "denseNearRoadFilterEnabled") ...
            || ~logical(cfg.denseNearRoadFilterEnabled) || ~any(curbPointMask)
        return;
    end

    radiusCells = 0;
    if isfield(cfg, "denseNearRoadRadiusCells")
        radiusCells = max(0, round(double(cfg.denseNearRoadRadiusCells)));
    end
    if radiusCells <= 0 || isempty(roadCellMask)
        return;
    end

    nearRoadCellMask = conv2(double(logical(roadCellMask)), ones((2 * radiusCells) + 1), "same") > 0;
    nearRoadPointMask = sampleCellMapAtPoints(groundCellLinIdx, nearRoadCellMask) > 0;
    if ~any(nearRoadPointMask & curbPointMask)
        return;
    end

    minResidualMeters = 0.025;
    if isfield(cfg, "denseMinResidualMeters")
        minResidualMeters = max(0, double(cfg.denseMinResidualMeters));
    end
    minBaseEnergy = 0.75;
    if isfield(cfg, "denseMinBaseEnergy")
        minBaseEnergy = max(0, double(cfg.denseMinBaseEnergy));
    end
    minRoughnessMeters = 0.030;
    if isfield(cfg, "denseMinRoughnessMeters")
        minRoughnessMeters = max(0, double(cfg.denseMinRoughnessMeters));
    end

    heightMap = double(stats.heightMap);
    localMeanMap = heightMap - double(stats.detrendedHeightMap);
    localMeanValues = double(sampleCellMapAtPointsPreserveNaN(groundCellLinIdx, localMeanMap));
    heightValues = double(sampleCellMapAtPointsPreserveNaN(groundCellLinIdx, heightMap));
    missingMean = ~isfinite(localMeanValues);
    localMeanValues(missingMean) = heightValues(missingMean);
    residualValues = max(localMeanValues - double(groundPoints(:, 3)), 0);
    residualValues(~isfinite(residualValues)) = 0;
    baseValues = double(sampleCellMapAtPoints(groundCellLinIdx, energyMaps.totalBase));
    roughnessValues = double(sampleCellMapAtPoints(groundCellLinIdx, energyMaps.roughnessMeters));
    strongNearRoadMask = residualValues >= minResidualMeters ...
        | baseValues >= minBaseEnergy ...
        | roughnessValues >= minRoughnessMeters;
    curbPointMask = curbPointMask & (~nearRoadPointMask | strongNearRoadMask);
end

function curbPointMask = filterDominantBoundaryPoints(curbPointMask, groundPoints, groundCellLinIdx, xyView, roadSeedMask, roadCellMask, energyMaps, cfg)
    curbPointMask = logical(curbPointMask(:));
    if ~isstruct(cfg) || ~isfield(cfg, "boundaryPointFilterEnabled") ...
            || ~logical(cfg.boundaryPointFilterEnabled) || ~any(curbPointMask)
        return;
    end
    if ~isstruct(xyView) || ~isfield(xyView, "countMap") || ~isfield(xyView, "yMap") || ~isfield(xyView, "yCenters")
        return;
    end

    numRows = size(xyView.countMap, 1);
    numCols = size(xyView.countMap, 2);
    yCenters = double(xyView.yCenters(:));
    if numel(yCenters) ~= numRows
        return;
    end

    cellLinIdx = double(groundCellLinIdx(:));
    validPointMask = isfinite(cellLinIdx) & cellLinIdx >= 1 & cellLinIdx <= (numRows * numCols) ...
        & cellLinIdx == floor(cellLinIdx);
    selectedPointIdx = find(curbPointMask & validPointMask);
    if isempty(selectedPointIdx)
        return;
    end

    [selectedCols, selectedRows] = ind2sub([numCols, numRows], cellLinIdx(selectedPointIdx));
    referenceY = resolveRoadSeedReferenceY(xyView, roadSeedMask);
    neighborRows = 1;
    if isfield(cfg, "boundaryNeighborRows")
        neighborRows = max(0, round(double(cfg.boundaryNeighborRows)));
    end
    minSidePoints = 12;
    if isfield(cfg, "boundaryMinSidePoints")
        minSidePoints = max(1, round(double(cfg.boundaryMinSidePoints)));
    end
    rowMinCountFraction = 0.15;
    if isfield(cfg, "boundaryRowMinCountFraction")
        rowMinCountFraction = min(max(double(cfg.boundaryRowMinCountFraction), 0), 1);
    end
    [rowKeepMask, roadAwayNeighborRowMask] = buildDominantBoundaryRowMasks( ...
        selectedRows, selectedCols, numRows, numCols, neighborRows, ...
        minSidePoints, rowMinCountFraction, cfg);

    if ~any(rowKeepMask(:))
        return;
    end

    pointCols = zeros(numel(cellLinIdx), 1);
    pointRows = zeros(numel(cellLinIdx), 1);
    [pointCols(validPointMask), pointRows(validPointMask)] = ind2sub([numCols, numRows], cellLinIdx(validPointMask));
    validCellSubMask = validPointMask & pointRows >= 1 & pointRows <= numRows & pointCols >= 1 & pointCols <= numCols;
    boundaryPointMask = false(size(curbPointMask));
    boundaryPointMask(validCellSubMask) = curbPointMask(validCellSubMask) ...
        & rowKeepMask(sub2ind([numRows, numCols], pointRows(validCellSubMask), pointCols(validCellSubMask)));
    boundaryPointIdx = find(boundaryPointMask);
    if isempty(boundaryPointIdx)
        curbPointMask(:) = false;
        return;
    end

    pointQuantile = 0.45;
    if isfield(cfg, "boundaryPointQuantile")
        pointQuantile = min(max(double(cfg.boundaryPointQuantile), 0), 1);
    end
    yBandMeters = 0.10;
    if isfield(cfg, "boundaryYBandMeters")
        yBandMeters = max(0, double(cfg.boundaryYBandMeters));
    end
    roadFacingCellMarginMeters = Inf;
    if isfield(cfg, "boundaryRoadFacingCellMarginMeters")
        roadFacingCellMarginMeters = double(cfg.boundaryRoadFacingCellMarginMeters);
        if isempty(roadFacingCellMarginMeters) || ~isfinite(roadFacingCellMarginMeters(1))
            roadFacingCellMarginMeters = Inf;
        else
            roadFacingCellMarginMeters = max(0, roadFacingCellMarginMeters(1));
        end
    end
    roadAwayNeighborMarginMeters = Inf;
    if isfield(cfg, "boundaryRoadAwayNeighborMarginMeters")
        roadAwayNeighborMarginMeters = double(cfg.boundaryRoadAwayNeighborMarginMeters);
        if isempty(roadAwayNeighborMarginMeters) || ~isfinite(roadAwayNeighborMarginMeters(1))
            roadAwayNeighborMarginMeters = Inf;
        else
            roadAwayNeighborMarginMeters = max(0, roadAwayNeighborMarginMeters(1));
        end
    end
    roadReferenceFilterEnabled = false;
    if isfield(cfg, "boundaryRoadReferenceFilterEnabled")
        roadReferenceFilterEnabled = logical(cfg.boundaryRoadReferenceFilterEnabled);
    end
    roadReferenceRadiusCells = 16;
    if isfield(cfg, "boundaryRoadReferenceRadiusCells")
        roadReferenceRadiusCells = max(0, round(double(cfg.boundaryRoadReferenceRadiusCells)));
    end
    weakSupportFilterEnabled = false;
    if isfield(cfg, "boundaryWeakSupportFilterEnabled")
        weakSupportFilterEnabled = logical(cfg.boundaryWeakSupportFilterEnabled);
    end
    if yBandMeters == 0
        curbPointMask = boundaryPointMask;
        return;
    end

    keepBoundaryPoint = false(numel(boundaryPointIdx), 1);
    pointCol = pointCols(boundaryPointIdx);
    pointRow = pointRows(boundaryPointIdx);
    pointSide = sign(double(groundPoints(boundaryPointIdx, 2)) - referenceY);
    pointY = double(groundPoints(boundaryPointIdx, 2));
    boundaryCols = unique(pointCol);
    for colListIdx = 1:numel(boundaryCols)
        colIdx = boundaryCols(colListIdx);
        columnPointMask = pointCol == colIdx;
        rowHasPoint = false(numRows, 1);
        rowHasPoint(unique(pointRow(columnPointMask))) = true;
        runEdges = diff([false; rowHasPoint; false]);
        runStarts = find(runEdges == 1);
        runEnds = find(runEdges == -1) - 1;
        for runIdx = 1:numel(runStarts)
            runPointMask = columnPointMask & pointRow >= runStarts(runIdx) & pointRow <= runEnds(runIdx);
            yValues = pointY(runPointMask);
            if isempty(yValues)
                continue;
            end
            yTarget = quantile(yValues, pointQuantile);
            keepBoundaryPoint = keepBoundaryPoint | (runPointMask & abs(pointY - yTarget) <= yBandMeters);
        end
    end
    if isfinite(roadFacingCellMarginMeters)
        roadFacingPointMask = false(size(keepBoundaryPoint));
        aboveReferenceRowMask = pointRow >= 1 & pointRow <= numel(yCenters) & pointSide > 0;
        belowReferenceRowMask = pointRow >= 1 & pointRow <= numel(yCenters) & pointSide < 0;
        roadFacingPointMask(aboveReferenceRowMask) = pointY(aboveReferenceRowMask) ...
            < yCenters(pointRow(aboveReferenceRowMask)) - roadFacingCellMarginMeters;
        roadFacingPointMask(belowReferenceRowMask) = pointY(belowReferenceRowMask) ...
            > yCenters(pointRow(belowReferenceRowMask)) + roadFacingCellMarginMeters;
        keepBoundaryPoint = keepBoundaryPoint & ~roadFacingPointMask;
    end
    if isfinite(roadAwayNeighborMarginMeters)
        validNeighborCellMask = pointRow >= 1 & pointRow <= numRows & pointCol >= 1 & pointCol <= numCols;
        validNeighborRowMask = false(size(keepBoundaryPoint));
        validNeighborRowMask(validNeighborCellMask) = roadAwayNeighborRowMask( ...
            sub2ind([numRows, numCols], pointRow(validNeighborCellMask), pointCol(validNeighborCellMask)));
        roadAwayPointMask = false(size(keepBoundaryPoint));
        aboveReferenceNeighborMask = validNeighborRowMask & pointSide > 0;
        belowReferenceNeighborMask = validNeighborRowMask & pointSide < 0;
        roadAwayPointMask(aboveReferenceNeighborMask) = pointY(aboveReferenceNeighborMask) ...
            >= yCenters(pointRow(aboveReferenceNeighborMask)) - roadAwayNeighborMarginMeters;
        roadAwayPointMask(belowReferenceNeighborMask) = pointY(belowReferenceNeighborMask) ...
            <= yCenters(pointRow(belowReferenceNeighborMask)) + roadAwayNeighborMarginMeters;
        keepBoundaryPoint = keepBoundaryPoint & ~roadAwayPointMask;
    end
    if roadReferenceFilterEnabled && ~isempty(roadCellMask) && isequal(size(roadCellMask), [numRows, numCols])
        roadReferenceMask = buildDominantBoundaryRoadReferenceMask(roadCellMask, roadReferenceRadiusCells);
        pointRoadReference = sampleCellMapAtPoints(cellLinIdx(boundaryPointIdx), roadReferenceMask) > 0;
        keepBoundaryPoint = keepBoundaryPoint & pointRoadReference;
    end
    if weakSupportFilterEnabled && isstruct(energyMaps) && isfield(energyMaps, "rawExtractedMask") ...
            && isfield(energyMaps, "totalBase") && isfield(energyMaps, "heightStepMeters") ...
            && isfield(energyMaps, "roughnessMeters") && isfield(energyMaps, "linearity") ...
            && isfield(energyMaps, "linearityComponentCenterEvidence")
        pointSupported = sampleDominantBoundarySupportAtPoints( ...
            cellLinIdx(boundaryPointIdx), energyMaps, cfg);
        keepBoundaryPoint = keepBoundaryPoint & pointSupported;
    end

    filteredPointMask = false(size(curbPointMask));
    filteredPointMask(boundaryPointIdx(keepBoundaryPoint)) = true;
    curbPointMask = filteredPointMask;
end

function [rowKeepMask, roadAwayNeighborRowMask] = buildDominantBoundaryRowMasks(selectedRows, selectedCols, numRows, numCols, neighborRows, minSidePoints, rowMinCountFraction, cfg)
    rowKeepMask = false(numRows, numCols);
    roadAwayNeighborRowMask = false(numRows, numCols);
    columnTrackingEnabled = true;
    if isfield(cfg, "boundaryColumnTrackingEnabled")
        columnTrackingEnabled = logical(cfg.boundaryColumnTrackingEnabled);
    end
    columnSearchRadiusCells = 3;
    if isfield(cfg, "boundaryColumnSearchRadiusCells")
        columnSearchRadiusCells = max(0, round(double(cfg.boundaryColumnSearchRadiusCells)));
    end
    columnMinPoints = 2;
    if isfield(cfg, "boundaryColumnMinPoints")
        columnMinPoints = max(1, round(double(cfg.boundaryColumnMinPoints)));
    end

    if columnTrackingEnabled
        columnKernel = ones(1, 2 * columnSearchRadiusCells + 1);
        validSelectedMask = selectedRows >= 1 & selectedRows <= numRows ...
            & selectedCols >= 1 & selectedCols <= numCols;
        if nnz(validSelectedMask) >= minSidePoints
            rowColumnCounts = accumarray([selectedRows(validSelectedMask), selectedCols(validSelectedMask)], ...
                1, [numRows, numCols], @sum, 0);
            supportedRowColumnCounts = conv2(rowColumnCounts, columnKernel, "same");
            for colIdx = 1:numCols
                rowCounts = supportedRowColumnCounts(:, colIdx);
                [dominantCount, ~] = max(rowCounts);
                if dominantCount < columnMinPoints
                    continue;
                end
                eligibleRowMask = rowCounts >= max(double(columnMinPoints), double(dominantCount) .* rowMinCountFraction);
                runEdges = diff([false; eligibleRowMask; false]);
                runStarts = find(runEdges == 1);
                runEnds = find(runEdges == -1) - 1;
                for runIdx = 1:numel(runStarts)
                    runRows = runStarts(runIdx):runEnds(runIdx);
                    [~, runPeakIdx] = max(rowCounts(runRows));
                    dominantRow = runRows(runPeakIdx);
                    rowStart = max(1, dominantRow - neighborRows);
                    rowEnd = min(numRows, dominantRow + neighborRows);
                    rowKeepMask(rowStart:rowEnd, colIdx) = true;
                end
            end
        end
    end

    if any(rowKeepMask(:))
        return;
    end

    rowKeepVector = false(numRows, 1);
    validSelectedMask = selectedRows >= 1 & selectedRows <= numRows;
    if nnz(validSelectedMask) < minSidePoints
        return;
    end
    rowCounts = accumarray(selectedRows(validSelectedMask), 1, [numRows, 1], @sum, 0);
    [dominantCount, ~] = max(rowCounts);
    if dominantCount < minSidePoints
        return;
    end
    eligibleRowMask = rowCounts >= max(double(minSidePoints), double(dominantCount) .* rowMinCountFraction);
    runEdges = diff([false; eligibleRowMask; false]);
    runStarts = find(runEdges == 1);
    runEnds = find(runEdges == -1) - 1;
    for runIdx = 1:numel(runStarts)
        runRows = runStarts(runIdx):runEnds(runIdx);
        [~, runPeakIdx] = max(rowCounts(runRows));
        dominantRow = runRows(runPeakIdx);
        rowStart = max(1, dominantRow - neighborRows);
        rowEnd = min(numRows, dominantRow + neighborRows);
        rowKeepVector(rowStart:rowEnd) = true;
    end
    rowKeepMask = repmat(rowKeepVector, 1, numCols);
    roadAwayNeighborRowMask = false(numRows, numCols);
end

function roadReferenceMask = buildDominantBoundaryRoadReferenceMask(roadCellMask, radiusCells)
    radiusCells = max(0, round(double(radiusCells)));
    roadReferenceMask = logical(roadCellMask);
    if radiusCells == 0 || isempty(roadReferenceMask)
        return;
    end

    supportKernel = ones((2 * radiusCells) + 1, (2 * radiusCells) + 1);
    roadReferenceMask = conv2(double(roadReferenceMask), supportKernel, "same") > 0;
end

function pointSupported = sampleDominantBoundarySupportAtPoints(cellLinIdx, energyMaps, cfg)
    rawSupportMap = logical(energyMaps.rawExtractedMask);
    rawSupportFields = ["extractionStandaloneSeedMask", "extractionStandaloneBridgeMask", ...
        "extractionFillMask", "extractionEndpointMask"];
    for fieldIdx = 1:numel(rawSupportFields)
        fieldName = rawSupportFields(fieldIdx);
        if isfield(energyMaps, fieldName)
            rawSupportMap = rawSupportMap | logical(energyMaps.(fieldName));
        end
    end

    minBaseEnergy = 0.12;
    if isfield(cfg, "boundaryWeakSupportMinBaseEnergy")
        minBaseEnergy = max(0, double(cfg.boundaryWeakSupportMinBaseEnergy));
    end
    minLinearity = 0.10;
    if isfield(cfg, "boundaryWeakSupportMinLinearity")
        minLinearity = max(0, double(cfg.boundaryWeakSupportMinLinearity));
    end
    minRoughnessMeters = 0.010;
    if isfield(cfg, "boundaryWeakSupportMinRoughnessMeters")
        minRoughnessMeters = max(0, double(cfg.boundaryWeakSupportMinRoughnessMeters));
    end
    minCenterEvidence = 0.75;
    if isfield(cfg, "boundaryWeakSupportMinCenterEvidence")
        minCenterEvidence = max(0, double(cfg.boundaryWeakSupportMinCenterEvidence));
    end
    highStepMinMeters = 0.08;
    if isfield(cfg, "boundaryWeakSupportHighStepMinMeters")
        highStepMinMeters = max(0, double(cfg.boundaryWeakSupportHighStepMinMeters));
    end

    pointRawSupport = sampleCellMapAtPoints(cellLinIdx, rawSupportMap) > 0;
    pointBaseEnergy = double(sampleCellMapAtPoints(cellLinIdx, energyMaps.totalBase));
    pointHeightStepMeters = double(sampleCellMapAtPoints(cellLinIdx, energyMaps.heightStepMeters));
    pointRoughnessMeters = double(sampleCellMapAtPoints(cellLinIdx, energyMaps.roughnessMeters));
    pointLinearity = double(sampleCellMapAtPoints(cellLinIdx, energyMaps.linearity));
    pointCenterEvidence = double(sampleCellMapAtPoints(cellLinIdx, energyMaps.linearityComponentCenterEvidence));
    pointSupported = pointRawSupport ...
        | (pointBaseEnergy >= minBaseEnergy ...
        & (pointLinearity >= minLinearity ...
        | pointRoughnessMeters >= minRoughnessMeters ...
        | pointCenterEvidence >= minCenterEvidence)) ...
        | (pointHeightStepMeters >= highStepMinMeters ...
        & pointCenterEvidence >= minCenterEvidence ...
        & pointRoughnessMeters >= minRoughnessMeters);
end

function referenceY = resolveRoadSeedReferenceY(xyView, roadSeedMask)
    yMap = double(xyView.yMap);
    referenceY = 0;
    if isempty(roadSeedMask)
        return;
    end

    seedYValues = yMap(logical(roadSeedMask) & isfinite(yMap));
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end
end

function values = sampleCellMapAtPoints(cellLinIdx, cellMap)
    values = zeros(numel(cellLinIdx), 1, "single");
    if isempty(cellLinIdx) || isempty(cellMap)
        return;
    end

    mapValues = double(cellMap.');
    mapValues = mapValues(:);
    validCell = isfinite(cellLinIdx) & cellLinIdx >= 1 & cellLinIdx <= numel(mapValues) & cellLinIdx == floor(cellLinIdx);
    values(validCell) = single(mapValues(double(cellLinIdx(validCell))));
    values(~isfinite(values)) = 0;
end

function values = sampleCellMapAtPointsPreserveNaN(cellLinIdx, cellMap)
    values = NaN(numel(cellLinIdx), 1, "single");
    if isempty(cellLinIdx) || isempty(cellMap)
        return;
    end

    mapValues = double(cellMap.');
    mapValues = mapValues(:);
    validCell = isfinite(cellLinIdx) & cellLinIdx >= 1 & cellLinIdx <= numel(mapValues) & cellLinIdx == floor(cellLinIdx);
    values(validCell) = single(mapValues(double(cellLinIdx(validCell))));
end
