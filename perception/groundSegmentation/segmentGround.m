function groundPointIdx = segmentGround(voxelGrid, cfg)
% segmentGround: Split the voxelized frame into ground and non-ground points
% with deterministic slope-grid ground segmentation. Retained points are
% rasterized into XY cells whose robust low height forms a terrain profile;
% flat cells near the vehicle that agree with the sensor-height prior seed the
% ground height, which is then propagated outward under a slope limit and
% range-dependent noise tolerance. Smooth flat components that the ordered
% propagation missed are promoted and bridged to accepted ground, holes are
% filled by local medians, and finally every point is labeled as ground when
% its residual to the cell ground estimate is within tolerance.
%
% Input:
%   voxelGrid: spatial index with gridConfig, points [K x 3], and aligned
%       pointIndices [K x 1]. Dense 3D voxel statistics are not required.
%   cfg: struct from groundSegmentationConfig
%
% Output:
%   groundPointIdx: [N x 1] original point indices (into the organized frame)
%       of the points labeled as ground
    assert(nargin >= 2 && isstruct(cfg), ...
        "cfg must be provided as a struct from groundSegmentationConfig.");

    assert(isstruct(voxelGrid), "voxelGrid must be a struct.");
    assert(isfield(voxelGrid, "gridConfig") && isstruct(voxelGrid.gridConfig), ...
        "voxelGrid must contain gridConfig from voxelizePointCloud.");
    assert(isfield(voxelGrid, "points") && size(voxelGrid.points, 2) == 3, ...
        "voxelGrid must contain points [K x 3].");
    assert(isfield(voxelGrid, "pointIndices") && isvector(voxelGrid.pointIndices) && ...
        numel(voxelGrid.pointIndices) == size(voxelGrid.points, 1), ...
        "voxelGrid must contain pointIndices aligned to points.");

    seedMaxZ = readRequiredScalarConfig(cfg, "groundSeedMaxZ");
    assert(isfinite(seedMaxZ), "groundSeedMaxZ must be finite.");
    cellSizeXY = resolveSlopeGridXYCellSize(cfg, voxelGrid);
    params = resolveSlopeGridParams(cfg, seedMaxZ);
    params.useNativeKernels = isfield(cfg, "useNativeKernels") && cfg.useNativeKernels;
    groundMask = extractGroundMaskSlopeGrid(voxelGrid, cellSizeXY, params);

    if ~any(groundMask)
        groundPointIdx = zeros(0, 1);
        return;
    end

    assert(numel(voxelGrid.pointIndices) == numel(groundMask), ...
        "pointIndices length must match filtered points.");
    groundPointIdx = double(voxelGrid.pointIndices(groundMask));
end

function cellSizeXY = resolveSlopeGridXYCellSize(cfg, gridResult)
% resolveSlopeGridXYCellSize: Resolve the XY cell resolution used by
% slope-grid ground segmentation, allowing a slopeGridXYCellSize override
% so the propagation grid can be coarser than the fine 3D voxel grid.
%
% Input:
%   cfg: segmentGround config
%   gridResult: canonical voxelizePointCloud output
%
% Output:
%   cellSizeXY: [1 x 2] XY cell size
    if isfield(cfg, "slopeGridXYCellSize") && ~isempty(cfg.slopeGridXYCellSize)
        rawCellSize = double(cfg.slopeGridXYCellSize(:).');
        if isscalar(rawCellSize)
            cellSizeXY = [rawCellSize, rawCellSize];
        elseif numel(rawCellSize) >= 2
            cellSizeXY = rawCellSize(1:2);
        else
            cellSizeXY = [1, 1];
        end
        assert(all(isfinite(cellSizeXY)) && all(cellSizeXY > 0), ...
            "slopeGridXYCellSize must contain positive finite values.");
        return;
    end

    cellSizeXY = resolveGroundXYCellSize(cfg, gridResult);
end

function cellSizeXY = resolveGroundXYCellSize(cfg, gridResult)
% resolveGroundXYCellSize: Resolve the XY cell resolution used by
% ground-segmentation before points are grouped into 2D seed cells.
%
% Input:
%   cfg: segmentGround config
%   gridResult: canonical voxelizePointCloud output
%
% Output:
%   cellSizeXY: [1 x 2] XY cell size
    if isfield(cfg, "xyCellSize") && ~isempty(cfg.xyCellSize)
        rawCellSize = double(cfg.xyCellSize(:).');
    elseif isfield(cfg, "groundXYCellSize") && ~isempty(cfg.groundXYCellSize)
        rawCellSize = double(cfg.groundXYCellSize(:).');
    elseif isfield(cfg, "cellSize") && ~isempty(cfg.cellSize)
        rawCellSize = double(cfg.cellSize(:).');
    elseif isfield(gridResult, "gridConfig") && isstruct(gridResult.gridConfig) && ...
            isfield(gridResult.gridConfig, "voxelSize") && numel(gridResult.gridConfig.voxelSize) >= 2
        rawCellSize = double(gridResult.gridConfig.voxelSize(1:2));
    else
        rawCellSize = [1, 1];
    end

    if isscalar(rawCellSize)
        cellSizeXY = [rawCellSize, rawCellSize];
    elseif numel(rawCellSize) >= 2
        cellSizeXY = rawCellSize(1:2);
    else
        cellSizeXY = [1, 1];
    end
    assert(all(isfinite(cellSizeXY)) && all(cellSizeXY > 0), ...
        "XY cell size must contain positive finite values.");
end

function params = resolveSlopeGridParams(cfg, fallbackPriorHeight)
% resolveSlopeGridParams: Read deterministic grid-ground extraction
% controls from the segmentGround configuration and normalize them into one
% parameter struct used by robust low-height extraction, flat-cell
% detection, slope-limited propagation, hole filling, and point labeling.
%
% Input:
%   cfg: segmentGround config
%   fallbackPriorHeight: scalar height used if slopeGridPriorHeight is not
%       configured
%
% Output:
%   params: struct with slope-grid algorithm thresholds and window sizes
    params = struct();
    params.lowOutlierThreshold = readOptionalNonnegativeScalarConfig(cfg, "slopeGridLowOutlierThreshold", 0.35);
    params.minCellPoints = readOptionalPositiveIntegerConfig(cfg, "slopeGridMinCellPoints", 1);
    params.flatSpanBase = readOptionalNonnegativeScalarConfig(cfg, "slopeGridFlatSpanBase", 0.35);
    params.flatSpanRangeSlope = readOptionalNonnegativeScalarConfig(cfg, "slopeGridFlatSpanRangeSlope", 0.003);
    params.flatSpanMax = readOptionalNonnegativeScalarConfig(cfg, "slopeGridFlatSpanMax", 0.75);
    params.priorHeight = readOptionalScalarConfig(cfg, "slopeGridPriorHeight", fallbackPriorHeight);
    params.seedXLimits = readOptionalTwoElementConfig(cfg, "slopeGridSeedXLimits", [-8, 12]);
    params.seedYAbsMax = readOptionalNonnegativeScalarConfig(cfg, "slopeGridSeedYAbsMax", 6);
    params.priorTolerance = readOptionalNonnegativeScalarConfig(cfg, "slopeGridPriorTolerance", 0.35);
    params.priorMinSeedCells = readOptionalPositiveIntegerConfig(cfg, "slopeGridPriorMinSeedCells", 20);
    params.seedTolerance = readOptionalNonnegativeScalarConfig(cfg, "slopeGridSeedTolerance", 0.65);
    params.baseHeightTolerance = readOptionalNonnegativeScalarConfig(cfg, "slopeGridBaseHeightTolerance", 0.08);
    params.maxSlope = readOptionalNonnegativeScalarConfig(cfg, "slopeGridMaxSlope", 0.25);
    params.noiseBase = readOptionalNonnegativeScalarConfig(cfg, "slopeGridNoiseBase", 0.03);
    params.noiseRangeSlope = readOptionalNonnegativeScalarConfig(cfg, "slopeGridNoiseRangeSlope", 0.002);
    params.noiseMax = readOptionalNonnegativeScalarConfig(cfg, "slopeGridNoiseMax", 0.20);
    params.smoothComponentPromotionEnabled = readOptionalLogicalConfig(cfg, "slopeGridSmoothComponentPromotionEnabled", true);
    params.smoothComponentBridgeMinCells = readOptionalPositiveIntegerConfig(cfg, "slopeGridSmoothComponentBridgeMinCells", 8);
    params.smoothComponentBridgeMaxCells = readOptionalPositiveIntegerConfig(cfg, "slopeGridSmoothComponentBridgeMaxCells", 300);
    params.smoothComponentBridgeMaxHeightRange = readOptionalNonnegativeScalarConfig(cfg, "slopeGridSmoothComponentBridgeMaxHeightRange", 0.45);
    params.smoothComponentBridgeRadiusCells = readOptionalNonnegativeIntegerConfig(cfg, "slopeGridSmoothComponentBridgeRadiusCells", 10);
    params.smoothComponentBridgeMaxIterations = readOptionalPositiveIntegerConfig(cfg, "slopeGridSmoothComponentBridgeMaxIterations", 8);
    params.holeFillWindowSize = readOptionalPositiveIntegerConfig(cfg, "slopeGridHoleFillWindowSize", 5);
    params.groundToleranceBase = readOptionalNonnegativeScalarConfig(cfg, "slopeGridGroundToleranceBase", 0.28);
    params.groundToleranceRangeSlope = readOptionalNonnegativeScalarConfig(cfg, "slopeGridGroundToleranceRangeSlope", 0.004);
    params.groundToleranceMax = readOptionalNonnegativeScalarConfig(cfg, "slopeGridGroundToleranceMax", 0.65);
    params.belowTolerance = readOptionalNonnegativeScalarConfig(cfg, "slopeGridBelowTolerance", 0.20);
    params.interpToleranceScale = readOptionalUnitIntervalConfig(cfg, "slopeGridInterpToleranceScale", 0.60);
    assert(params.seedXLimits(1) <= params.seedXLimits(2), ...
        "slopeGridSeedXLimits must be ordered [min max].");
    assert(params.smoothComponentBridgeMinCells <= params.smoothComponentBridgeMaxCells, ...
        "slopeGridSmoothComponentBridgeMinCells must be <= slopeGridSmoothComponentBridgeMaxCells.");
end

function groundMask = extractGroundMaskSlopeGrid(gridResult, cellSizeXY, params)
% extractGroundMaskSlopeGrid: Segment ground points with deterministic
% minimum-height rasterization, distance-dependent flat-cell gating,
% near-vehicle seed-height estimation, slope-limited near-to-far
% propagation, optional interpolation-only hole filling, and final
% point-to-ground residual labeling.
%
% Input:
%   gridResult: canonical voxelizePointCloud output
%   cellSizeXY: [1 x 2] XY cell size
%   params: slope-grid ground segmentation parameters
%
% Output:
%   groundMask: [K x 1] logical mask aligned with gridResult.points
    points = double(gridResult.points);
    numPoints = size(points, 1);
    groundMask = false(numPoints, 1);
    if numPoints == 0
        return;
    end

    [pointCellLinIdx, dimsXY, minCornerXY] = mapPointsToSlopeGridCells(gridResult, cellSizeXY);
    validPointMask = pointCellLinIdx >= 1 & all(isfinite(points), 2);
    if ~any(validPointMask)
        return;
    end

    [cellStats, pointCellLinIdx] = computeSlopeGridCellStats(points, pointCellLinIdx, dimsXY, params);
    if isempty(cellStats.occupiedLinIdx)
        return;
    end

    [cellCenterX, cellCenterY, cellRange] = computeSlopeGridCellGeometry(cellStats.occupiedLinIdx, dimsXY, minCornerXY, cellSizeXY);
    flatSpanThreshold = min(params.flatSpanBase + params.flatSpanRangeSlope .* cellRange, params.flatSpanMax);
    flatCandidate = cellStats.count >= params.minCellPoints & ...
        (cellStats.zMax - cellStats.zLow) < flatSpanThreshold;

    groundHeight = nan(double(prod(dimsXY)), 1);
    state = zeros(double(prod(dimsXY)), 1, "uint8");
    seedHeight = estimateSlopeGridSeedHeight(cellStats, flatCandidate, cellCenterX, cellCenterY, params);
    initialGround = flatCandidate & abs(cellStats.zLow - seedHeight) < params.seedTolerance;
    groundHeight(double(cellStats.occupiedLinIdx(initialGround))) = cellStats.zLow(initialGround);
    state(double(cellStats.occupiedLinIdx(initialGround))) = uint8(1);

    [groundHeight, state] = propagateSlopeGridGround( ...
        cellStats, flatCandidate, cellRange, groundHeight, state, dimsXY, cellSizeXY, params);
    [groundHeight, state] = promoteSlopeGridSmoothFlatComponents( ...
        cellStats, flatCandidate, cellRange, groundHeight, state, dimsXY, cellSizeXY, params);
    [groundHeight, state] = fillSlopeGridHoles( ...
        cellStats.occupiedLinIdx, groundHeight, state, dimsXY, params.holeFillWindowSize);

    groundMask = labelSlopeGridGroundPoints( ...
        points, pointCellLinIdx, groundHeight, state, cellRange, cellStats.occupiedLinIdx, params);
end

function [pointCellLinIdx, dimsXY, minCornerXY] = mapPointsToSlopeGridCells(gridResult, cellSizeXY)
% mapPointsToSlopeGridCells: Assign retained points to the XY grid
% used by slope-grid ground segmentation and return the grid dimensions and
% lower metric corner required for cell-center range computation.
%
% Input:
%   gridResult: canonical voxelizePointCloud output
%   cellSizeXY: [1 x 2] XY cell size
%
% Output:
%   pointCellLinIdx: [K x 1] int32 linear cell indices, 0 for invalid cells
%   dimsXY: [1 x 2] grid dimensions [Nx Ny]
%   minCornerXY: [1 x 2] lower XY grid corner
    points = double(gridResult.points);
    numPoints = size(points, 1);
    pointCellLinIdx = zeros(numPoints, 1, "int32");
    gridCfg = gridResult.gridConfig;
    minCornerXY = [0, 0];
    if isfield(gridCfg, "minCorner") && numel(gridCfg.minCorner) >= 2
        minCornerXY = double(gridCfg.minCorner(1:2));
    elseif isfield(gridCfg, "origin") && numel(gridCfg.origin) >= 2 && ...
            isfield(gridCfg, "voxelSize") && numel(gridCfg.voxelSize) >= 2
        minCornerXY = double(gridCfg.origin(1:2)) - (0.5 .* double(gridCfg.voxelSize(1:2)));
    end

    maxCornerXY = minCornerXY + cellSizeXY;
    if isfield(gridCfg, "maxCorner") && numel(gridCfg.maxCorner) >= 2
        maxCornerXY = double(gridCfg.maxCorner(1:2));
    elseif isfield(gridCfg, "dims") && numel(gridCfg.dims) >= 2 && ...
            isfield(gridCfg, "voxelSize") && numel(gridCfg.voxelSize) >= 2
        maxCornerXY = minCornerXY + (double(gridCfg.dims(1:2)) .* double(gridCfg.voxelSize(1:2)));
    elseif numPoints > 0
        maxCornerXY = max(points(:, 1:2), [], 1);
    end

    dimsXY = max(ceil((maxCornerXY - minCornerXY) ./ cellSizeXY), 1);
    if numPoints == 0
        return;
    end

    xBin = floor((points(:, 1) - minCornerXY(1)) ./ cellSizeXY(1)) + 1;
    yBin = floor((points(:, 2) - minCornerXY(2)) ./ cellSizeXY(2)) + 1;
    valid = isfinite(xBin) & isfinite(yBin);
    valid = valid & xBin >= 1 & xBin <= dimsXY(1) & yBin >= 1 & yBin <= dimsXY(2);
    if any(valid)
        pointCellLinIdx(valid) = int32(sub2ind(dimsXY, double(xBin(valid)), double(yBin(valid))));
    end
end

function [cellStats, pointCellLinIdx] = computeSlopeGridCellStats(points, pointCellLinIdx, dimsXY, params)
% computeSlopeGridCellStats: Sort valid points by cell and height,
% compute count, minimum height, second-lowest height, maximum height, and
% robust low height for every occupied slope-grid cell.
%
% Input:
%   points: [K x 3] point coordinates
%   pointCellLinIdx: [K x 1] int32 cell indices
%   dimsXY: [1 x 2] grid dimensions
%   params: slope-grid parameters containing lowOutlierThreshold
%
% Output:
%   cellStats: struct with occupiedLinIdx,count,zMin,zSecond,zMax,zLow
%   pointCellLinIdx: [K x 1] int32 cell indices clipped to valid cells
    numCells = double(prod(dimsXY));
    valid = pointCellLinIdx >= 1 & double(pointCellLinIdx) <= numCells & all(isfinite(points), 2);
    pointCellLinIdx(~valid) = int32(0);
    cellStats = struct("occupiedLinIdx", zeros(0, 1, "int32"), "count", zeros(0, 1), ...
        "zMin", zeros(0, 1), "zSecond", zeros(0, 1), "zMax", zeros(0, 1), "zLow", zeros(0, 1));
    if ~any(valid)
        return;
    end

    validCell = double(pointCellLinIdx(valid));
    validZ = points(valid, 3);
    if isfield(params,"useNativeKernels") && params.useNativeKernels
        reduced = perceptionKernelsMex('groundStats',validCell,double(validZ),numCells);
        cellStats.occupiedLinIdx = int32(reduced(:,1));
        cellStats.count = reduced(:,2);
        cellStats.zMin = reduced(:,3); cellStats.zSecond = reduced(:,4); cellStats.zMax = reduced(:,5);
        cellStats.zLow = cellStats.zMin;
        useSecond = cellStats.count>=2 & (cellStats.zSecond-cellStats.zMin)>params.lowOutlierThreshold;
        cellStats.zLow(useSecond) = cellStats.zSecond(useSecond);
        return;
    end
    [~, sortOrder] = sortrows([validCell, validZ], [1, 2]);
    sortedCell = validCell(sortOrder);
    sortedZ = validZ(sortOrder);
    groupStartIdx = find([true; diff(sortedCell) ~= 0]);
    groupEndIdx = [groupStartIdx(2:end) - 1; numel(sortedCell)];
    count = groupEndIdx - groupStartIdx + 1;
    zMin = sortedZ(groupStartIdx);
    secondIdx = min(groupStartIdx + 1, groupEndIdx);
    zSecond = sortedZ(secondIdx);
    zMax = sortedZ(groupEndIdx);
    useSecond = count >= 2 & (zSecond - zMin) > params.lowOutlierThreshold;
    zLow = zMin;
    zLow(useSecond) = zSecond(useSecond);

    cellStats.occupiedLinIdx = int32(sortedCell(groupStartIdx));
    cellStats.count = double(count);
    cellStats.zMin = zMin;
    cellStats.zSecond = zSecond;
    cellStats.zMax = zMax;
    cellStats.zLow = zLow;
end

function [cellCenterX, cellCenterY, cellRange] = computeSlopeGridCellGeometry(occupiedLinIdx, dimsXY, minCornerXY, cellSizeXY)
% computeSlopeGridCellGeometry: Convert occupied linear cell indices
% into metric cell centers and horizontal ranges from the LiDAR origin.
%
% Input:
%   occupiedLinIdx: [C x 1] int32 occupied cell indices
%   dimsXY: [1 x 2] grid dimensions
%   minCornerXY: [1 x 2] lower XY grid corner
%   cellSizeXY: [1 x 2] XY cell size
%
% Output:
%   cellCenterX,cellCenterY,cellRange: [C x 1] cell geometry vectors
    [xSub, ySub] = ind2sub(dimsXY, double(occupiedLinIdx(:)));
    cellCenterX = minCornerXY(1) + ((double(xSub(:)) - 0.5) .* cellSizeXY(1));
    cellCenterY = minCornerXY(2) + ((double(ySub(:)) - 0.5) .* cellSizeXY(2));
    cellRange = hypot(cellCenterX, cellCenterY);
end

function seedHeight = estimateSlopeGridSeedHeight(cellStats, flatCandidate, cellCenterX, cellCenterY, params)
% estimateSlopeGridSeedHeight: Estimate the initial ground height from
% flat near-vehicle cells whose robust low height is close to the configured
% prior height when enough cells support that prior, otherwise falling back
% to the robust median low height of all flat near-vehicle seed cells before
% using the configured prior as the final fallback.
%
% Input:
%   cellStats: occupied cell statistics
%   flatCandidate: [C x 1] logical flat-cell candidate mask
%   cellCenterX,cellCenterY: [C x 1] metric cell centers
%   params: slope-grid seed parameters
%
% Output:
%   seedHeight: scalar initial ground height estimate
    seedRegion = cellCenterX >= params.seedXLimits(1) & cellCenterX <= params.seedXLimits(2) & ...
        abs(cellCenterY) <= params.seedYAbsMax;
    seedMask = flatCandidate & seedRegion & abs(cellStats.zLow - params.priorHeight) < params.priorTolerance;
    fallbackSeedMask = flatCandidate & seedRegion & isfinite(cellStats.zLow);
    if nnz(seedMask) >= params.priorMinSeedCells
        seedHeight = median(cellStats.zLow(seedMask), "omitnan");
    elseif any(fallbackSeedMask)
        seedHeight = median(cellStats.zLow(fallbackSeedMask), "omitnan");
    elseif any(seedMask)
        seedHeight = median(cellStats.zLow(seedMask), "omitnan");
    else
        seedHeight = params.priorHeight;
    end
    if ~isfinite(seedHeight)
        seedHeight = params.priorHeight;
    end
end

function [groundHeight, state] = propagateSlopeGridGround(cellStats, flatCandidate, cellRange, groundHeight, state, dimsXY, cellSizeXY, params)
% propagateSlopeGridGround: Visit occupied cells in near-to-far order,
% accept flat cells whose robust low height is compatible with already
% accepted 8-neighbor ground heights, and assign interpolation-only ground
% estimates to rejected cells that still have neighboring ground support.
%
% Input:
%   cellStats: occupied cell statistics
%   flatCandidate: [C x 1] logical flat-cell candidate mask
%   cellRange: [C x 1] occupied cell ranges
%   groundHeight: [Nx*Ny x 1] current ground height estimates
%   state: [Nx*Ny x 1] uint8 cell state, 0 unknown, 1 ground, 2 interp
%   dimsXY: [1 x 2] grid dimensions
%   cellSizeXY: [1 x 2] XY cell size
%   params: slope-grid propagation parameters
%
% Output:
%   groundHeight,state: updated ground estimates and states
    [~, order] = sort(cellRange, "ascend");
    occupiedLinIdx = double(cellStats.occupiedLinIdx(:));
    zLow = cellStats.zLow(:);
    if isfield(params, "useNativeKernels") && params.useNativeKernels
        [groundHeight, state] = perceptionKernelsMex('propagateGround', ...
            double(order), occupiedLinIdx, double(zLow), logical(flatCandidate), ...
            double(cellRange), groundHeight, state, double(dimsXY), double(cellSizeXY), ...
            [params.noiseBase, params.noiseRangeSlope, params.noiseMax, ...
             params.baseHeightTolerance, params.maxSlope]);
        return;
    end
    [neighborIdxByStats, neighborDistanceByStats] = buildSlopeGridOccupiedNeighborLookup( ...
        occupiedLinIdx, dimsXY, cellSizeXY);
    for orderIdx = 1:numel(order)
        statsIdx = order(orderIdx);
        linIdx = occupiedLinIdx(statsIdx);
        if state(linIdx) == uint8(1)
            continue;
        end

        neighborIdx = neighborIdxByStats(statsIdx, :);
        validNeighbor = neighborIdx > 0;
        neighborIdx = neighborIdx(validNeighbor);
        neighborDistance = neighborDistanceByStats(statsIdx, validNeighbor);
        groundNeighborMask = state(neighborIdx) == uint8(1) & isfinite(groundHeight(neighborIdx));
        if ~any(groundNeighborMask)
            continue;
        end

        supportedGroundHeights = groundHeight(neighborIdx(groundNeighborMask));
        neighHeight = medianSmallFinite(supportedGroundHeights);
        if ~isfinite(neighHeight)
            continue;
        end

        noiseMargin = min(params.noiseBase + params.noiseRangeSlope .* cellRange(statsIdx), params.noiseMax);
        compatible = false;
        if flatCandidate(statsIdx)
            % Preserve the original conservative gate explicitly. Previously
            % a column of residuals and a row of tolerances expanded to a
            % matrix; if(any(...)) required support at every tolerance. This
            % is equivalent to any residual passing the smallest tolerance.
            allowed = params.baseHeightTolerance + params.maxSlope .* min(neighborDistance(groundNeighborMask)) + noiseMargin;
            compatible = any(abs(zLow(statsIdx) - supportedGroundHeights) < allowed);
        end

        if compatible
            groundHeight(linIdx) = zLow(statsIdx);
            state(linIdx) = uint8(1);
        else
            groundHeight(linIdx) = neighHeight;
            state(linIdx) = uint8(2);
        end
    end
end

function [neighborIdxByStats, neighborDistanceByStats] = buildSlopeGridOccupiedNeighborLookup(occupiedLinIdx, dimsXY, cellSizeXY)
% buildSlopeGridOccupiedNeighborLookup: Precompute 8-neighbor linear
% grid indices and metric neighbor distances for occupied slope-grid cells
% so propagation does not repeatedly call ind2sub and sub2ind in the hot
% near-to-far loop.
%
% Input:
%   occupiedLinIdx: [C x 1] occupied linear grid indices
%   dimsXY: [1 x 2] grid dimensions
%   cellSizeXY: [1 x 2] XY cell size
%
% Output:
%   neighborIdxByStats: [C x 8] neighbor linear indices, 0 for invalid
%       border neighbors
%   neighborDistanceByStats: [C x 8] metric neighbor distances
    occupiedLinIdx = double(occupiedLinIdx(:));
    numOccupied = numel(occupiedLinIdx);
    neighborIdxByStats = zeros(numOccupied, 8);
    xOffset = [-1, 0, 1, -1, 1, -1, 0, 1];
    yOffset = [-1, -1, -1, 0, 0, 1, 1, 1];
    offsetDistance = hypot(xOffset(:).' .* cellSizeXY(1), yOffset(:).' .* cellSizeXY(2));
    neighborDistanceByStats = repmat(offsetDistance, numOccupied, 1);
    if numOccupied == 0
        return;
    end

    [xSub, ySub] = ind2sub(dimsXY, occupiedLinIdx);
    for iNeighbor = 1:numel(xOffset)
        xNeighbor = double(xSub(:)) + xOffset(iNeighbor);
        yNeighbor = double(ySub(:)) + yOffset(iNeighbor);
        validNeighbor = xNeighbor >= 1 & xNeighbor <= dimsXY(1) & yNeighbor >= 1 & yNeighbor <= dimsXY(2);
        if any(validNeighbor)
            neighborIdxByStats(validNeighbor, iNeighbor) = sub2ind(dimsXY, xNeighbor(validNeighbor), yNeighbor(validNeighbor));
        end
    end
end

function value = medianSmallFinite(values)
% medianSmallFinite: Compute the median of a small finite vector with
% minimal overhead for hot loops where inputs contain at most eight neighbor
% heights.
%
% Input:
%   values: numeric vector containing candidate finite values
%
% Output:
%   value: scalar median or NaN when no finite values exist
    values = sort(double(values(isfinite(values))));
    numValues = numel(values);
    if numValues == 0
        value = NaN;
        return;
    end

    midIdx = floor((numValues + 1) / 2);
    if mod(numValues, 2) == 1
        value = values(midIdx);
    else
        value = 0.5 * (values(midIdx) + values(midIdx + 1));
    end
end

function [groundHeight, state] = promoteSlopeGridSmoothFlatComponents(cellStats, flatCandidate, cellRange, groundHeight, state, dimsXY, cellSizeXY, params)
% promoteSlopeGridSmoothFlatComponents: Complete slope-compatible flat
% surface components after the ordered propagation pass. Components that
% already contain accepted ground are promoted as ground, and compact
% detached components can be iteratively bridged to nearby accepted ground
% when their median height is slope-compatible with local support.
%
% Input:
%   cellStats: occupied cell statistics
%   flatCandidate: [C x 1] logical flat-cell candidate mask
%   cellRange: [C x 1] occupied cell ranges
%   groundHeight: [Nx*Ny x 1] current ground height estimates
%   state: [Nx*Ny x 1] uint8 cell state
%   dimsXY: [1 x 2] grid dimensions
%   cellSizeXY: [1 x 2] XY cell size
%   params: slope-grid component promotion parameters
%
% Output:
%   groundHeight,state: updated ground estimates and states
    if ~params.smoothComponentPromotionEnabled || ~any(flatCandidate)
        return;
    end

    [componentId, flatLinIdx, flatStatsIdx] = buildSlopeGridSmoothFlatComponents( ...
        cellStats, flatCandidate, cellRange, dimsXY, cellSizeXY, params);
    if isempty(componentId)
        return;
    end

    numComponents = max(componentId);
    componentSize = accumarray(componentId, 1, [numComponents, 1], @sum, 0);
    componentMinZ = accumarray(componentId, cellStats.zLow(flatStatsIdx), [numComponents, 1], @min, NaN);
    componentMaxZ = accumarray(componentId, cellStats.zLow(flatStatsIdx), [numComponents, 1], @max, NaN);
    componentHasGround = accumarray(componentId, state(flatLinIdx) == uint8(1), [numComponents, 1], @max, false);
    [componentOrder, componentStartIdx, componentEndIdx] = buildSlopeGridComponentMemberIndex(componentId, numComponents);

    promotedComponent = componentHasGround;
    groundedFlatMask = componentHasGround(componentId);
    groundHeight(flatLinIdx(groundedFlatMask)) = cellStats.zLow(flatStatsIdx(groundedFlatMask));
    state(flatLinIdx(groundedFlatMask)) = uint8(1);

    if params.smoothComponentBridgeRadiusCells < 1 || ~any(promotedComponent)
        return;
    end

    bridgeEligible = componentSize >= params.smoothComponentBridgeMinCells & ...
        componentSize <= params.smoothComponentBridgeMaxCells & ...
        (componentMaxZ - componentMinZ) <= params.smoothComponentBridgeMaxHeightRange;
    if canUseSlopeGridDistanceTransformBridge(cellSizeXY)
        componentMedianHeight = groupedFiniteMedian(componentId, cellStats.zLow(flatStatsIdx), numComponents);
        componentMedianRange = groupedFiniteMedian(componentId, cellRange(flatStatsIdx), numComponents);
        [groundHeight, state] = bridgeSlopeGridSmoothComponentsDistanceTransform( ...
            componentId, flatLinIdx, flatStatsIdx, componentMedianHeight, componentMedianRange, bridgeEligible, ...
            promotedComponent, cellStats, groundHeight, state, dimsXY, cellSizeXY, params);
        return;
    end

    for iIteration = 1:params.smoothComponentBridgeMaxIterations
        stateMap = reshape(state == uint8(1), dimsXY);
        groundHeightMap = reshape(groundHeight, dimsXY);
        supportMap = conv2(double(stateMap), ones((2 * params.smoothComponentBridgeRadiusCells) + 1), "same") > 0;
        supportedFlatMask = supportMap(flatLinIdx);
        supportedComponent = false(numComponents, 1);
        if any(supportedFlatMask)
            supportedComponent = accumarray(componentId(supportedFlatMask), true, [numComponents, 1], @any, false);
        end
        candidateComponents = find(bridgeEligible & ~promotedComponent & supportedComponent).';
        promotedThisIteration = false;
        for iComponent = candidateComponents
            memberRows = componentOrder(componentStartIdx(iComponent):componentEndIdx(iComponent));
            memberLinIdx = flatLinIdx(memberRows);
            memberStatsIdx = flatStatsIdx(memberRows);
            bridgeCompatible = isSlopeGridSmoothComponentBridgeCompatible( ...
                memberLinIdx, memberStatsIdx, cellStats, cellRange, stateMap, groundHeightMap, dimsXY, cellSizeXY, params);
            if bridgeCompatible
                [groundHeight, state] = setSlopeGridSmoothComponentGround( ...
                    memberRows, flatLinIdx, flatStatsIdx, cellStats, groundHeight, state);
                promotedComponent(iComponent) = true;
                promotedThisIteration = true;
            end
        end

        if ~promotedThisIteration
            break;
        end
    end
end

function [componentId, flatLinIdx, flatStatsIdx] = buildSlopeGridSmoothFlatComponents(cellStats, flatCandidate, cellRange, dimsXY, cellSizeXY, params)
% buildSlopeGridSmoothFlatComponents: Build connected components over
% flat cells using only 8-neighbor edges whose robust low heights satisfy
% the same local slope compatibility rule used during ground propagation.
%
% Input:
%   cellStats: occupied cell statistics
%   flatCandidate: [C x 1] logical flat-cell candidate mask
%   cellRange: [C x 1] occupied cell ranges
%   dimsXY: [1 x 2] grid dimensions
%   cellSizeXY: [1 x 2] XY cell size
%   params: slope-grid compatibility parameters
%
% Output:
%   componentId: [F x 1] smooth component id per flat cell
%   flatLinIdx: [F x 1] linear grid indices for flat cells
%   flatStatsIdx: [F x 1] indices into cellStats for flat cells
    flatStatsIdx = find(flatCandidate(:));
    flatLinIdx = double(cellStats.occupiedLinIdx(flatStatsIdx));
    numFlatCells = numel(flatLinIdx);
    componentId = zeros(numFlatCells, 1);
    if numFlatCells == 0
        return;
    end
    if isfield(params,"useNativeKernels") && params.useNativeKernels
        componentId = perceptionKernelsMex('smoothComponents',flatLinIdx, ...
            double(cellStats.zLow(flatStatsIdx)),double(cellRange(flatStatsIdx)), ...
            double(dimsXY),double(cellSizeXY), ...
            [params.noiseBase,params.noiseRangeSlope,params.noiseMax,params.baseHeightTolerance,params.maxSlope]);
        return;
    end

    flatIndexByLin = zeros(double(prod(dimsXY)), 1);
    flatIndexByLin(flatLinIdx) = 1:numFlatCells;
    [xSub, ySub] = ind2sub(dimsXY, flatLinIdx);
    xOffset = [1, 0, 1, 1];
    yOffset = [0, 1, 1, -1];
    offsetDistance = hypot(xOffset(:) .* cellSizeXY(1), yOffset(:) .* cellSizeXY(2));
    sourceList = zeros(numel(xOffset) * numFlatCells, 1);
    targetList = zeros(numel(xOffset) * numFlatCells, 1);
    edgeCount = 0;
    for iOffset = 1:numel(xOffset)
        xNeighbor = double(xSub(:)) + xOffset(iOffset);
        yNeighbor = double(ySub(:)) + yOffset(iOffset);
        validNeighbor = xNeighbor >= 1 & xNeighbor <= dimsXY(1) & yNeighbor >= 1 & yNeighbor <= dimsXY(2);
        neighborLinIdx = zeros(numFlatCells, 1);
        neighborLinIdx(validNeighbor) = sub2ind(dimsXY, xNeighbor(validNeighbor), yNeighbor(validNeighbor));
        neighborFlatIdx = zeros(numFlatCells, 1);
        neighborFlatIdx(validNeighbor) = flatIndexByLin(neighborLinIdx(validNeighbor));
        pairMask = neighborFlatIdx > 0;
        if ~any(pairMask)
            continue;
        end

        sourceIdx = find(pairMask);
        targetIdx = neighborFlatIdx(pairMask);
        sourceStatsIdx = flatStatsIdx(sourceIdx);
        targetStatsIdx = flatStatsIdx(targetIdx);
        sourceNoise = min(params.noiseBase + params.noiseRangeSlope .* cellRange(sourceStatsIdx), params.noiseMax);
        targetNoise = min(params.noiseBase + params.noiseRangeSlope .* cellRange(targetStatsIdx), params.noiseMax);
        allowed = params.baseHeightTolerance + params.maxSlope .* offsetDistance(iOffset) + max(sourceNoise, targetNoise);
        compatible = abs(cellStats.zLow(sourceStatsIdx) - cellStats.zLow(targetStatsIdx)) < allowed;
        if any(compatible)
            newSource = sourceIdx(compatible);
            newTarget = targetIdx(compatible);
            newEdgeCount = numel(newSource);
            sourceList(edgeCount + (1:newEdgeCount)) = newSource;
            targetList(edgeCount + (1:newEdgeCount)) = newTarget;
            edgeCount = edgeCount + newEdgeCount;
        end
    end

    sourceList = sourceList(1:edgeCount);
    targetList = targetList(1:edgeCount);
    if edgeCount == 0
        componentId = (1:numFlatCells).';
        return;
    end

    componentGraph = graph(sourceList, targetList, [], numFlatCells);
    componentId = conncomp(componentGraph).';
end

function [componentOrder, componentStartIdx, componentEndIdx] = buildSlopeGridComponentMemberIndex(componentId, numComponents)
% buildSlopeGridComponentMemberIndex: Build a compact sorted lookup
% table from smooth component id to rows in the flat-cell arrays so repeated
% component member access avoids full componentId scans during bridge
% iterations.
%
% Input:
%   componentId: [F x 1] smooth component id per flat cell
%   numComponents: scalar number of smooth components
%
% Output:
%   componentOrder: [F x 1] flat-cell row indices sorted by component id
%   componentStartIdx: [numComponents x 1] start row in componentOrder
%   componentEndIdx: [numComponents x 1] end row in componentOrder
    [sortedComponentId, componentOrder] = sort(componentId(:), "ascend");
    groupStart = find([true; diff(sortedComponentId) ~= 0]);
    groupEnd = [groupStart(2:end) - 1; numel(sortedComponentId)];
    componentStartIdx = zeros(numComponents, 1);
    componentEndIdx = zeros(numComponents, 1);
    componentStartIdx(sortedComponentId(groupStart)) = groupStart;
    componentEndIdx(sortedComponentId(groupStart)) = groupEnd;
end

function useDistanceTransform = canUseSlopeGridDistanceTransformBridge(cellSizeXY)
% canUseSlopeGridDistanceTransformBridge: Decide whether the optimized
% bridge path can use bwdist. The distance transform gives exact Euclidean
% cell distances only for equal XY cell spacing, so anisotropic grids keep
% the local-search fallback.
%
% Input:
%   cellSizeXY: [1 x 2] XY cell size
%
% Output:
%   useDistanceTransform: logical scalar true when bwdist can be used
    useDistanceTransform = exist("bwdist", "file") == 2 && ...
        numel(cellSizeXY) >= 2 && isfinite(cellSizeXY(1)) && isfinite(cellSizeXY(2)) && ...
        abs(cellSizeXY(1) - cellSizeXY(2)) <= (eps(max(cellSizeXY)) * 16);
end

function [groundHeight, state] = bridgeSlopeGridSmoothComponentsDistanceTransform(componentId, flatLinIdx, flatStatsIdx, componentMedianHeight, componentMedianRange, bridgeEligible, promotedComponent, cellStats, groundHeight, state, dimsXY, cellSizeXY, params)
% bridgeSlopeGridSmoothComponentsDistanceTransform: Iteratively bridge
% compact smooth flat components to accepted ground using one global distance
% transform per iteration instead of per-component neighborhood searches.
% Component support height is estimated from the nearest accepted ground cell
% reached by each flat member inside the bridge radius.
%
% Input:
%   componentId: [F x 1] smooth component id per flat cell
%   flatLinIdx: [F x 1] linear grid indices for flat cells
%   flatStatsIdx: [F x 1] indices into cellStats for flat cells
%   componentMedianHeight: [numComponents x 1] median zLow per component
%   componentMedianRange: [numComponents x 1] median range per component
%   bridgeEligible: [numComponents x 1] logical bridge eligibility mask
%   promotedComponent: [numComponents x 1] logical already-ground component
%       mask
%   cellStats: occupied cell statistics
%   groundHeight: [Nx*Ny x 1] current ground height estimates
%   state: [Nx*Ny x 1] uint8 cell state
%   dimsXY: [1 x 2] grid dimensions
%   cellSizeXY: [1 x 2] XY cell size
%   params: slope-grid bridge parameters
%
% Output:
%   groundHeight,state: updated ground estimates and states
    numComponents = numel(promotedComponent);
    radiusMeters = params.smoothComponentBridgeRadiusCells * cellSizeXY(1);
    componentNoise = min(params.noiseBase + params.noiseRangeSlope .* componentMedianRange, params.noiseMax);
    for iIteration = 1:params.smoothComponentBridgeMaxIterations
        stateMap = reshape(state == uint8(1) & isfinite(groundHeight), dimsXY);
        if ~any(stateMap(:))
            break;
        end

        groundHeightMap = reshape(groundHeight, dimsXY);
        [distanceCells, nearestGroundLinIdx] = bwdist(stateMap);
        flatDistanceMeters = double(distanceCells(flatLinIdx)) .* cellSizeXY(1);
        nearestGroundHeight = double(groundHeightMap(nearestGroundLinIdx(flatLinIdx)));
        validSupport = flatDistanceMeters <= radiusMeters & isfinite(nearestGroundHeight);
        if ~any(validSupport)
            break;
        end

        supportedComponentId = componentId(validSupport);
        supportedDistance = flatDistanceMeters(validSupport);
        supportedHeight = nearestGroundHeight(validSupport);
        [~, supportOrder] = sortrows([supportedComponentId(:), supportedDistance(:)], [1, 2]);
        sortedComponentId = supportedComponentId(supportOrder);
        supportGroupStart = find([true; diff(sortedComponentId) ~= 0]);
        closestComponentId = sortedComponentId(supportGroupStart);
        closestSupportRows = supportOrder(supportGroupStart);
        componentMinDistance = inf(numComponents, 1);
        componentSupportHeight = nan(numComponents, 1);
        componentMinDistance(closestComponentId) = supportedDistance(closestSupportRows);
        componentSupportHeight(closestComponentId) = supportedHeight(closestSupportRows);
        allowed = params.baseHeightTolerance + (params.maxSlope .* componentMinDistance) + componentNoise;
        bridgeCompatible = bridgeEligible & ~promotedComponent & isfinite(componentSupportHeight) & ...
            componentMinDistance <= radiusMeters & abs(componentMedianHeight - componentSupportHeight) <= allowed;
        if ~any(bridgeCompatible)
            break;
        end

        promotedComponent(bridgeCompatible) = true;
        promotedFlatMask = bridgeCompatible(componentId);
        groundHeight(flatLinIdx(promotedFlatMask)) = cellStats.zLow(flatStatsIdx(promotedFlatMask));
        state(flatLinIdx(promotedFlatMask)) = uint8(1);
    end
end

function bridgeCompatible = isSlopeGridSmoothComponentBridgeCompatible(memberLinIdx, memberStatsIdx, cellStats, cellRange, stateMap, groundHeightMap, dimsXY, cellSizeXY, params)
% isSlopeGridSmoothComponentBridgeCompatible: Test whether one compact
% smooth flat component can be bridged to current ground support by checking
% local support within a radius and comparing median component height to the
% median nearby ground height under the slope-limited tolerance.
%
% Input:
%   memberLinIdx: linear grid indices belonging to one smooth component
%   memberStatsIdx: indices into cellStats for the component members
%   cellStats: occupied cell statistics
%   cellRange: [C x 1] occupied cell ranges
%   stateMap: [Nx x Ny] logical accepted-ground support map
%   groundHeightMap: [Nx x Ny] current ground height map
%   dimsXY: [1 x 2] grid dimensions
%   cellSizeXY: [1 x 2] XY cell size
%   params: slope-grid bridge parameters
%
% Output:
%   bridgeCompatible: logical scalar true when the component can be promoted
    bridgeCompatible = false;
    radiusCells = params.smoothComponentBridgeRadiusCells;
    radiusMeters = radiusCells * max(cellSizeXY);
    [memberX, memberY] = ind2sub(dimsXY, memberLinIdx(:));
    xMin = max(min(memberX) - radiusCells, 1);
    xMax = min(max(memberX) + radiusCells, dimsXY(1));
    yMin = max(min(memberY) - radiusCells, 1);
    yMax = min(max(memberY) + radiusCells, dimsXY(2));
    supportMask = stateMap(xMin:xMax, yMin:yMax) & isfinite(groundHeightMap(xMin:xMax, yMin:yMax));
    if ~any(supportMask(:))
        return;
    end

    [supportXLocal, supportYLocal] = find(supportMask);
    supportX = supportXLocal + xMin - 1;
    supportY = supportYLocal + yMin - 1;
    supportWithinRadius = false(numel(supportX), 1);
    minDistance = inf;
    for iMember = 1:numel(memberX)
        distance = hypot((double(supportX) - double(memberX(iMember))) .* cellSizeXY(1), ...
            (double(supportY) - double(memberY(iMember))) .* cellSizeXY(2));
        supportWithinRadius = supportWithinRadius | distance <= radiusMeters;
        minDistance = min(minDistance, min(distance));
    end
    if ~any(supportWithinRadius) || ~isfinite(minDistance) || minDistance > radiusMeters
        return;
    end

    supportHeights = groundHeightMap(xMin:xMax, yMin:yMax);
    supportHeights = supportHeights(supportMask);
    supportHeights = supportHeights(supportWithinRadius);
    componentMedianHeight = median(cellStats.zLow(memberStatsIdx), "omitnan");
    supportMedianHeight = median(supportHeights, "omitnan");
    componentRange = median(cellRange(memberStatsIdx), "omitnan");
    if ~isfinite(componentMedianHeight) || ~isfinite(supportMedianHeight) || ~isfinite(componentRange)
        return;
    end

    noiseMargin = min(params.noiseBase + params.noiseRangeSlope .* componentRange, params.noiseMax);
    allowed = params.baseHeightTolerance + params.maxSlope .* minDistance + noiseMargin;
    bridgeCompatible = abs(componentMedianHeight - supportMedianHeight) <= allowed;
end

function [groundHeight, state] = setSlopeGridSmoothComponentGround(memberRows, flatLinIdx, flatStatsIdx, cellStats, groundHeight, state)
% setSlopeGridSmoothComponentGround: Promote all cells of one smooth
% flat component to directly accepted ground using their robust low heights.
%
% Input:
%   memberRows: flat-cell array rows belonging to one smooth component
%   flatLinIdx: [F x 1] linear grid indices for flat cells
%   flatStatsIdx: [F x 1] indices into cellStats for flat cells
%   cellStats: occupied cell statistics
%   groundHeight: [Nx*Ny x 1] current ground height estimates
%   state: [Nx*Ny x 1] uint8 cell state
%
% Output:
%   groundHeight,state: updated ground estimates and states
    memberLinIdx = flatLinIdx(memberRows);
    memberStatsIdx = flatStatsIdx(memberRows);
    groundHeight(memberLinIdx) = cellStats.zLow(memberStatsIdx);
    state(memberLinIdx) = uint8(1);
end

function [groundHeight, state] = fillSlopeGridHoles(occupiedLinIdx, groundHeight, state, dimsXY, windowSize)
% fillSlopeGridHoles: Assign interpolation-only ground estimates to
% occupied cells that still lack a valid ground height by taking the median
% of valid ground estimates in a fixed local window.
%
% Input:
%   occupiedLinIdx: [C x 1] occupied cell indices
%   groundHeight: [Nx*Ny x 1] current ground height estimates
%   state: [Nx*Ny x 1] uint8 cell state
%   dimsXY: [1 x 2] grid dimensions
%   windowSize: scalar odd or even local window width
%
% Output:
%   groundHeight,state: updated ground estimates and states
    if windowSize <= 1
        return;
    end

    halfWindow = floor(double(windowSize) / 2);
    occupiedLinIdx = double(occupiedLinIdx(:));
    missingIdx = occupiedLinIdx(~isfinite(groundHeight(occupiedLinIdx)));
    if isempty(missingIdx)
        return;
    end

    [xSub, ySub] = ind2sub(dimsXY, missingIdx);
    offsets = -halfWindow:halfWindow;
    [xOffset, yOffset] = ndgrid(offsets, offsets);
    xOffset = xOffset(:).';
    yOffset = yOffset(:).';
    neighborValues = nan(numel(missingIdx), numel(xOffset));
    for iOffset = 1:numel(xOffset)
        xNeighbor = double(xSub(:)) + xOffset(iOffset);
        yNeighbor = double(ySub(:)) + yOffset(iOffset);
        validNeighbor = xNeighbor >= 1 & xNeighbor <= dimsXY(1) & yNeighbor >= 1 & yNeighbor <= dimsXY(2);
        if any(validNeighbor)
            neighborIdx = sub2ind(dimsXY, xNeighbor(validNeighbor), yNeighbor(validNeighbor));
            neighborValues(validNeighbor, iOffset) = groundHeight(neighborIdx);
        end
    end

    filledValues = median(neighborValues, 2, "omitnan");
    fillMask = isfinite(filledValues);
    if any(fillMask)
        fillIdx = missingIdx(fillMask);
        groundHeight(fillIdx) = filledValues(fillMask);
        state(fillIdx) = uint8(2);
    end
end

function groundMask = labelSlopeGridGroundPoints(points, pointCellLinIdx, groundHeight, state, cellRange, occupiedLinIdx, params)
% labelSlopeGridGroundPoints: Label individual points as ground when
% their vertical residual from the cell ground estimate lies within the
% configured below-ground and distance-dependent above-ground tolerances.
%
% Input:
%   points: [K x 3] point coordinates
%   pointCellLinIdx: [K x 1] int32 point cell indices
%   groundHeight: [Nx*Ny x 1] ground height estimates
%   state: [Nx*Ny x 1] uint8 cell state, 1 ground, 2 interp
%   cellRange: [C x 1] occupied cell ranges
%   occupiedLinIdx: [C x 1] occupied cell indices
%   params: slope-grid point labeling parameters
%
% Output:
%   groundMask: [K x 1] logical point ground mask
    numPoints = size(points, 1);
    groundMask = false(numPoints, 1);
    pointLinIdx = double(pointCellLinIdx(:));
    valid = pointLinIdx >= 1 & pointLinIdx <= numel(groundHeight) & isfinite(points(:, 3));
    if ~any(valid)
        return;
    end

    cellRangeByLinIdx = nan(numel(groundHeight), 1);
    cellRangeByLinIdx(double(occupiedLinIdx(:))) = cellRange(:);
    validLinIdx = pointLinIdx(valid);
    pointGroundHeight = groundHeight(validLinIdx);
    pointState = state(validLinIdx);
    pointRange = cellRangeByLinIdx(validLinIdx);
    validGroundEstimate = isfinite(pointGroundHeight) & pointState > uint8(0) & isfinite(pointRange);
    if ~any(validGroundEstimate)
        return;
    end

    validPointRows = find(valid);
    labelRows = validPointRows(validGroundEstimate);
    residual = points(labelRows, 3) - pointGroundHeight(validGroundEstimate);
    groundTolerance = min(params.groundToleranceBase + params.groundToleranceRangeSlope .* pointRange(validGroundEstimate), ...
        params.groundToleranceMax);
    interpMask = pointState(validGroundEstimate) == uint8(2);
    groundTolerance(interpMask) = params.interpToleranceScale .* groundTolerance(interpMask);
    groundMask(labelRows) = residual >= -params.belowTolerance & residual <= groundTolerance;
end

function value = readRequiredScalarConfig(cfg, fieldName)
% readRequiredScalarConfig: Read one required finite scalar from
% config and validate that the field exists, is non-empty, and scalar.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%
% Output:
%   value: validated finite scalar double
    assert(isfield(cfg, fieldName), ...
        "Config field %s is required.", fieldName);
    assert(~isempty(cfg.(fieldName)), ...
        "Config field %s must not be empty.", fieldName);
    value = double(cfg.(fieldName));
    assert(isscalar(value) && isfinite(value), ...
        "Config field %s must be a finite scalar.", fieldName);
end

function value = readOptionalPositiveIntegerConfig(cfg, fieldName, defaultValue)
% readOptionalPositiveIntegerConfig: Read one optional positive
% integer-valued scalar from config and fall back to a provided default
% when the field is missing or empty.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%   defaultValue: scalar positive integer default
%
% Output:
%   value: validated positive integer double
    if ~isfield(cfg, fieldName) || isempty(cfg.(fieldName))
        value = double(defaultValue);
    else
        value = double(cfg.(fieldName));
    end
    assert(isscalar(value) && isfinite(value) && value == floor(value) && value >= 1, ...
        "Config field %s must be a positive integer.", fieldName);
end

function value = readOptionalNonnegativeIntegerConfig(cfg, fieldName, defaultValue)
% readOptionalNonnegativeIntegerConfig: Read one optional
% nonnegative integer-valued scalar from config and fall back to a provided
% default when the field is missing or empty.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%   defaultValue: scalar nonnegative integer default
%
% Output:
%   value: validated nonnegative integer double
    if ~isfield(cfg, fieldName) || isempty(cfg.(fieldName))
        value = double(defaultValue);
    else
        value = double(cfg.(fieldName));
    end
    assert(isscalar(value) && isfinite(value) && value == floor(value) && value >= 0, ...
        "Config field %s must be a nonnegative integer.", fieldName);
end

function value = readOptionalLogicalConfig(cfg, fieldName, defaultValue)
% readOptionalLogicalConfig: Read one optional logical scalar from
% config and fall back to a provided default when the field is missing or
% empty. Numeric scalar inputs are accepted and converted with logical.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%   defaultValue: scalar logical default
%
% Output:
%   value: validated logical scalar
    if ~isfield(cfg, fieldName) || isempty(cfg.(fieldName))
        rawValue = defaultValue;
    else
        rawValue = cfg.(fieldName);
    end
    assert(isscalar(rawValue) && (islogical(rawValue) || isnumeric(rawValue)), ...
        "Config field %s must be a logical scalar.", fieldName);
    value = logical(rawValue);
end

function value = readOptionalNonnegativeScalarConfig(cfg, fieldName, defaultValue)
% readOptionalNonnegativeScalarConfig: Read one optional nonnegative
% scalar from config and fall back to a provided default when the field is
% missing or empty.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%   defaultValue: scalar nonnegative default
%
% Output:
%   value: validated nonnegative scalar double
    if ~isfield(cfg, fieldName) || isempty(cfg.(fieldName))
        value = double(defaultValue);
    else
        value = double(cfg.(fieldName));
    end
    assert(isscalar(value) && isfinite(value) && value >= 0, ...
        "Config field %s must be a nonnegative scalar.", fieldName);
end

function value = readOptionalScalarConfig(cfg, fieldName, defaultValue)
% readOptionalScalarConfig: Read one optional finite scalar from
% config and fall back to a provided default when the field is missing or
% empty.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%   defaultValue: scalar default value
%
% Output:
%   value: validated finite scalar double
    if ~isfield(cfg, fieldName) || isempty(cfg.(fieldName))
        value = double(defaultValue);
    else
        value = double(cfg.(fieldName));
    end
    assert(isscalar(value) && isfinite(value), ...
        "Config field %s must be a finite scalar.", fieldName);
end

function value = readOptionalTwoElementConfig(cfg, fieldName, defaultValue)
% readOptionalTwoElementConfig: Read one optional two-element finite
% numeric vector from config and fall back to a provided default when the
% field is missing or empty.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%   defaultValue: [1 x 2] numeric default vector
%
% Output:
%   value: [1 x 2] validated finite double vector
    if ~isfield(cfg, fieldName) || isempty(cfg.(fieldName))
        value = double(defaultValue(:).');
    else
        value = double(cfg.(fieldName)(:).');
    end
    assert(numel(value) == 2 && all(isfinite(value)), ...
        "Config field %s must contain two finite values.", fieldName);
    value = value(1:2);
end

function value = readOptionalUnitIntervalConfig(cfg, fieldName, defaultValue)
% readOptionalUnitIntervalConfig: Read one optional scalar in the
% open interval (0, 1] from config and fall back to a provided default when
% the field is missing or empty.
%
% Input:
%   cfg: struct configuration
%   fieldName: char/string field name
%   defaultValue: scalar default value in (0, 1]
%
% Output:
%   value: validated scalar double in (0, 1]
    value = readOptionalScalarConfig(cfg, fieldName, defaultValue);
    assert(value > 0 && value <= 1, ...
        "Config field %s must be in the interval (0, 1].", fieldName);
end
