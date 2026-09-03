function curbPointMask = thinCurbPointsToDominantBoundary(curbPointMask, groundPoints, groundCellLinIdx, xyView, roadSeedMask, roadCellMask, energyMaps, cfg)
% thinCurbPointsToDominantBoundary: Thin dense curb-channel points
% to the dominant road-boundary ridge on each side of the ego-near road
% seed. The filter first keeps the strongest curb-cell row band per road
% side, then applies a per-column robust y-position gate and a road-facing
% in-cell margin so parallel sidewalk shoulders and broad road-edge cells do
% not publish every point in the cell.
%
% Input:
%   curbPointMask: [N x 1] logical dense curb-point mask
%   groundPoints: [N x 3] double ground-point coordinates
%   groundCellLinIdx: [N x 1] XY-cell linear indices aligned to points
%   xyView: XY view struct with countMap, yMap, and yCenters
%   roadSeedMask: [Ny x Nx] logical road seed raster for side reference
%   roadCellMask: [Ny x Nx] logical road raster used as boundary support
%   energyMaps: struct with raw and derived curb-energy rasters
%   cfg: struct from groundFeatureConfig().curb with dominant boundary fields
%
% Output:
%   curbPointMask: [N x 1] logical curb-point mask after boundary thinning
    curbPointMask = logical(curbPointMask(:));
    if ~isstruct(cfg) || ~isfield(cfg, "dominantBoundaryPointFilterEnabled") ...
            || ~logical(cfg.dominantBoundaryPointFilterEnabled) || ~any(curbPointMask)
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
    if isfield(cfg, "dominantBoundaryNeighborRows")
        neighborRows = max(0, round(double(cfg.dominantBoundaryNeighborRows)));
    end
    minSidePoints = 12;
    if isfield(cfg, "dominantBoundaryMinSidePoints")
        minSidePoints = max(1, round(double(cfg.dominantBoundaryMinSidePoints)));
    end
    rowMinCountFraction = 0.15;
    if isfield(cfg, "dominantBoundaryRowMinCountFraction")
        rowMinCountFraction = min(max(double(cfg.dominantBoundaryRowMinCountFraction), 0), 1);
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
    if isfield(cfg, "dominantBoundaryPointQuantile")
        pointQuantile = min(max(double(cfg.dominantBoundaryPointQuantile), 0), 1);
    end
    yBandMeters = 0.10;
    if isfield(cfg, "dominantBoundaryYBandMeters")
        yBandMeters = max(0, double(cfg.dominantBoundaryYBandMeters));
    end
    roadFacingCellMarginMeters = Inf;
    if isfield(cfg, "dominantBoundaryRoadFacingCellMarginMeters")
        roadFacingCellMarginMeters = double(cfg.dominantBoundaryRoadFacingCellMarginMeters);
        if isempty(roadFacingCellMarginMeters) || ~isfinite(roadFacingCellMarginMeters(1))
            roadFacingCellMarginMeters = Inf;
        else
            roadFacingCellMarginMeters = max(0, roadFacingCellMarginMeters(1));
        end
    end
    roadAwayNeighborMarginMeters = Inf;
    if isfield(cfg, "dominantBoundaryRoadAwayNeighborMarginMeters")
        roadAwayNeighborMarginMeters = double(cfg.dominantBoundaryRoadAwayNeighborMarginMeters);
        if isempty(roadAwayNeighborMarginMeters) || ~isfinite(roadAwayNeighborMarginMeters(1))
            roadAwayNeighborMarginMeters = Inf;
        else
            roadAwayNeighborMarginMeters = max(0, roadAwayNeighborMarginMeters(1));
        end
    end
    roadReferenceFilterEnabled = false;
    if isfield(cfg, "dominantBoundaryRoadReferenceFilterEnabled")
        roadReferenceFilterEnabled = logical(cfg.dominantBoundaryRoadReferenceFilterEnabled);
    end
    roadReferenceRadiusCells = 16;
    if isfield(cfg, "dominantBoundaryRoadReferenceRadiusCells")
        roadReferenceRadiusCells = max(0, round(double(cfg.dominantBoundaryRoadReferenceRadiusCells)));
    end
    weakSupportFilterEnabled = false;
    if isfield(cfg, "dominantBoundaryWeakSupportFilterEnabled")
        weakSupportFilterEnabled = logical(cfg.dominantBoundaryWeakSupportFilterEnabled);
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
% buildDominantBoundaryRowMasks: Build per-column row gates for the
% dominant road-boundary curb point filter. The column tracker follows
% curved and branching boundaries by selecting all locally supported curb
% ridge rows independently in each x column using nearby-column support,
% with a global multi-ridge fallback for sparse frames.
%
% Input:
%   selectedRows: [M x 1] row subscripts of candidate curb points
%   selectedCols: [M x 1] column subscripts of candidate curb points
%   numRows: scalar XY raster row count
%   numCols: scalar XY raster column count
%   neighborRows: scalar number of adjacent rows retained around boundary
%   minSidePoints: scalar minimum side support count
%   rowMinCountFraction: scalar relative count threshold for eligible rows
%   cfg: struct from groundFeatureConfig().curb with column tracking fields
%
% Output:
%   rowKeepMask: [numRows x numCols] logical per-column row gate
%   roadAwayNeighborRowMask: [numRows x numCols] logical road-away neighbor
%       gate used by the optional road-away point suppression margin
    rowKeepMask = false(numRows, numCols);
    roadAwayNeighborRowMask = false(numRows, numCols);
    columnTrackingEnabled = true;
    if isfield(cfg, "dominantBoundaryColumnTrackingEnabled")
        columnTrackingEnabled = logical(cfg.dominantBoundaryColumnTrackingEnabled);
    end
    columnSearchRadiusCells = 3;
    if isfield(cfg, "dominantBoundaryColumnSearchRadiusCells")
        columnSearchRadiusCells = max(0, round(double(cfg.dominantBoundaryColumnSearchRadiusCells)));
    end
    columnMinPoints = 2;
    if isfield(cfg, "dominantBoundaryColumnMinPoints")
        columnMinPoints = max(1, round(double(cfg.dominantBoundaryColumnMinPoints)));
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
% buildDominantBoundaryRoadReferenceMask: Expand the road-surface
% raster into a local support map used by the dominant-boundary point
% filter so completed curb cells without nearby road evidence are not
% published as semantic curb points.
%
% Input:
%   roadCellMask: [Ny x Nx] logical road-surface raster
%   radiusCells: scalar neighborhood radius in XY cells
%
% Output:
%   roadReferenceMask: [Ny x Nx] logical map of cells with nearby road
%       support
    radiusCells = max(0, round(double(radiusCells)));
    roadReferenceMask = logical(roadCellMask);
    if radiusCells == 0 || isempty(roadReferenceMask)
        return;
    end

    supportKernel = ones((2 * radiusCells) + 1, (2 * radiusCells) + 1);
    roadReferenceMask = conv2(double(roadReferenceMask), supportKernel, "same") > 0;
end

function pointSupported = sampleDominantBoundarySupportAtPoints(cellLinIdx, energyMaps, cfg)
% sampleDominantBoundarySupportAtPoints: Evaluate whether dominant
% boundary points come from raw curb evidence or from a completed cell with
% enough independent geometric support to survive point-level publication.
%
% Input:
%   cellLinIdx: [N x 1] numeric XY-cell linear indices aligned to points
%   energyMaps: struct with raw and derived curb-energy rasters
%   cfg: struct from groundFeatureConfig().curb with weak-support thresholds
%
% Output:
%   pointSupported: [N x 1] logical support decision for each point
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
    if isfield(cfg, "dominantBoundaryWeakSupportMinBaseEnergy")
        minBaseEnergy = max(0, double(cfg.dominantBoundaryWeakSupportMinBaseEnergy));
    end
    minLinearity = 0.10;
    if isfield(cfg, "dominantBoundaryWeakSupportMinLinearity")
        minLinearity = max(0, double(cfg.dominantBoundaryWeakSupportMinLinearity));
    end
    minRoughnessMeters = 0.010;
    if isfield(cfg, "dominantBoundaryWeakSupportMinRoughnessMeters")
        minRoughnessMeters = max(0, double(cfg.dominantBoundaryWeakSupportMinRoughnessMeters));
    end
    minCenterEvidence = 0.75;
    if isfield(cfg, "dominantBoundaryWeakSupportMinCenterEvidence")
        minCenterEvidence = max(0, double(cfg.dominantBoundaryWeakSupportMinCenterEvidence));
    end
    highStepMinMeters = 0.08;
    if isfield(cfg, "dominantBoundaryWeakSupportHighStepMinMeters")
        highStepMinMeters = max(0, double(cfg.dominantBoundaryWeakSupportHighStepMinMeters));
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
