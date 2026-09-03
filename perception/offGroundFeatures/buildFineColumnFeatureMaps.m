function [columnMaps, fineVoxelGrid] = buildFineColumnFeatureMaps(voxelGrid, cfg)
% buildFineColumnFeatureMaps: Reduce the off-ground fine voxel grid to the
% 2D column feature maps that drive facade and pole detection. High-intensity
% traffic-sign voxels are removed from the structural count tensor first;
% each remaining XY column is then described by its point count, occupied
% z range, occupied z-layer count, and maximum contiguous occupied z-run
% length, together with metric column-center coordinates.
%
% Input:
%   voxelGrid: canonical off-ground voxel grid from voxelizePointCloud
%   cfg: struct from offGroundFeatureConfig
%
% Output:
%   columnMaps: struct with mapSize [Ny Nx], origin [1 x 2], dx, dy, xMap,
%       yMap, occupiedMask, pillarCounts, pillarZRange, occupiedLayerCount,
%       and maxRunLayerCount, all in the [Ny x Nx] column layout
%   fineVoxelGrid: struct with the traffic-sign-free count3D tensor, its
%       geometry, and the traffic-sign voxel channel used for refinement
    [pillarCounts, pillarZRange, maxRunLayerCount, occupiedLayerCount, xMap, yMap, occupiedMask, origin, dx, dy, fineVoxelGrid] = ...
        buildFineColumnFeatureMapArrays(voxelGrid, cfg);
    columnMaps = struct();
    columnMaps.mapSize = double(size(occupiedMask));
    columnMaps.origin = origin;
    columnMaps.dx = dx;
    columnMaps.dy = dy;
    columnMaps.xMap = xMap;
    columnMaps.yMap = yMap;
    columnMaps.occupiedMask = occupiedMask;
    columnMaps.pillarCounts = pillarCounts;
    columnMaps.pillarZRange = pillarZRange;
    columnMaps.occupiedLayerCount = occupiedLayerCount;
    columnMaps.maxRunLayerCount = maxRunLayerCount;
end

function [pillarCounts, pillarZRange, maxRunLayerCount, occupiedLayerCount, xMap, yMap, occupiedMask, origin, dx, dy, fineVoxelGrid] = buildFineColumnFeatureMapArrays(voxelGrid, cfg)
% buildFineColumnFeatureMapArrays: Resolve the canonical fine voxel grid,
% remove high-intensity traffic-sign voxels from the structural channel,
% and compute fine-column 2D maps including point counts, z range,
% occupancy, coordinates, occupied z-layer count, and maximum contiguous
% occupied z-run length.
%
% Input:
%   voxelGrid: canonical fine voxel grid or derived view with sourceVoxelGrid
%   cfg: configuration struct with traffic-sign and layer-occupancy
%       thresholds
%
% Output:
%   pillarCounts, pillarZRange, maxRunLayerCount, occupiedLayerCount,
%       xMap, yMap, occupiedMask,
%       origin, dx, dy, fineVoxelGrid: fine-column feature maps and a
%       fineVoxelGrid struct compatible with downstream refinement helpers
    pillarCounts = zeros(0, 0, "single");
    pillarZRange = zeros(0, 0, "single");
    maxRunLayerCount = zeros(0, 0, "single");
    occupiedLayerCount = zeros(0, 0, "single");
    xMap = zeros(0, 0, "single");
    yMap = zeros(0, 0, "single");
    occupiedMask = false(0, 0);
    origin = [0, 0];
    dx = 1;
    dy = 1;
    fineVoxelGrid = buildEmptyFineVoxelGrid();

    sourceVoxelGrid = resolveFineSourceVoxelGrid(voxelGrid);
    [count3D, gridOrigin, voxelSize, zCenters, isValid] = readFineCountTensor(sourceVoxelGrid);
    if ~isValid || isempty(count3D) || size(count3D, 3) < 1
        return;
    end

    trafficThreshold = resolveTrafficSignIntensityThreshold(cfg);
    [trafficSignCount3D, trafficSignPointIndices, trafficSignVoxelLinIdx, trafficSignColumnLinIdx] = buildTrafficSignCountTensorFromMetricSamples( ...
        sourceVoxelGrid, gridOrigin, voxelSize, size(count3D), trafficThreshold);
    if ~isempty(trafficSignCount3D) && isequal(size(trafficSignCount3D), size(count3D)) && any(trafficSignCount3D(:))
        count3D = max(single(count3D) - single(trafficSignCount3D), 0);
    else
        trafficSignCount3D = zeros(size(count3D), "single");
    end

    [ny, nx, ~] = size(count3D);
    dx = double(voxelSize(1));
    dy = double(voxelSize(2));
    origin = double(gridOrigin(1:2));
    [xMap, yMap] = buildCoordinateMapsFromGeometry([ny, nx], origin, dx, dy);

    occupiedLayerMinPoints = resolvePoleOccupiedLayerMinPoints(cfg);
    [pillarCounts, pillarZRange, maxRunLayerCount, occupiedLayerCount, occupiedMask] = ...
        computeFineColumnOccupancyFeatures(count3D, zCenters, occupiedLayerMinPoints);
    xMap(~occupiedMask) = NaN;
    yMap(~occupiedMask) = NaN;

    fineVoxelGrid = struct("valid", true, "count3D", single(count3D), ...
        "voxelSize", double(voxelSize(1:3)), "origin", double(gridOrigin(1:3)), ...
        "zCenters", double(zCenters(:)), "coarseOrigin", double(gridOrigin(1:2)), ...
        "coarseVoxelSize", double(voxelSize(1:2)), "coarseMapSize", [ny, nx], ...
        "sampleXBin", zeros(0, 1, "int32"), "sampleYBin", zeros(0, 1, "int32"), ...
        "sampleZ", zeros(0, 1, "single"), "trafficSignCount3D", single(trafficSignCount3D), ...
        "trafficSignPointIndices", double(trafficSignPointIndices(:)), ...
        "trafficSignVoxelLinIdx", double(trafficSignVoxelLinIdx(:)), ...
        "trafficSignColumnLinIdx", double(trafficSignColumnLinIdx(:)), ...
        "trafficSignIntensityThreshold", trafficThreshold, "occupiedLayerMinPoints", occupiedLayerMinPoints);
end

function sourceVoxelGrid = resolveFineSourceVoxelGrid(voxelGrid)
% resolveFineSourceVoxelGrid: Select the canonical fine voxel grid
% used by fine-only off-ground processing, preferring sourceVoxelGrid from
% a derived view and otherwise using the supplied voxel grid directly.
%
% Input:
%   voxelGrid: voxel-grid-like struct
%
% Output:
%   sourceVoxelGrid: voxel-grid-like struct with count and gridConfig
    sourceVoxelGrid = voxelGrid;
    if isstruct(voxelGrid) && isfield(voxelGrid, "sourceVoxelGrid") && isstruct(voxelGrid.sourceVoxelGrid) && ...
            isfield(voxelGrid.sourceVoxelGrid, "gridConfig") && isfield(voxelGrid.sourceVoxelGrid, "count")
        sourceVoxelGrid = voxelGrid.sourceVoxelGrid;
    end
end

function [count3D, gridOrigin, voxelSize, zCenters, isValid] = readFineCountTensor(sourceVoxelGrid)
% readFineCountTensor: Convert a canonical [Nx x Ny x Nz] voxel-count
% tensor into the off-ground processing [Ny x Nx x Nz] layout and extract
% lower-corner geometry plus z-center coordinates.
%
% Input:
%   sourceVoxelGrid: struct with count and gridConfig
%
% Output:
%   count3D: [Ny x Nx x Nz] single count tensor
%   gridOrigin: [1 x 3] lower voxel-grid corner
%   voxelSize: [1 x 3] voxel spacing
%   zCenters: [Nz x 1] z-center coordinates
%   isValid: logical scalar indicating successful conversion
    count3D = zeros(0, 0, 0, "single");
    gridOrigin = [0, 0, 0];
    voxelSize = [1, 1, 1];
    zCenters = zeros(0, 1);
    isValid = false;

    if ~isstruct(sourceVoxelGrid) || ~isfield(sourceVoxelGrid, "gridConfig") || ~isfield(sourceVoxelGrid, "count") || ...
            ndims(sourceVoxelGrid.count) ~= 3 || isempty(sourceVoxelGrid.count)
        return;
    end

    if isfield(sourceVoxelGrid.gridConfig, "voxelSize") && numel(sourceVoxelGrid.gridConfig.voxelSize) >= 3
        voxelSize = double(sourceVoxelGrid.gridConfig.voxelSize(1:3));
    end
    if ~all(isfinite(voxelSize)) || any(voxelSize <= 0)
        return;
    end

    if isfield(sourceVoxelGrid.gridConfig, "minCorner") && numel(sourceVoxelGrid.gridConfig.minCorner) >= 3
        gridOrigin = double(sourceVoxelGrid.gridConfig.minCorner(1:3));
    elseif isfield(sourceVoxelGrid.gridConfig, "origin") && numel(sourceVoxelGrid.gridConfig.origin) >= 3
        gridOrigin = double(sourceVoxelGrid.gridConfig.origin(1:3)) - (0.5 .* voxelSize);
    end
    if ~all(isfinite(gridOrigin))
        return;
    end

    countRaw = single(sourceVoxelGrid.count);
    countLayout = "NxNyNz";
    if isfield(sourceVoxelGrid.gridConfig, "countLayout") && ~isempty(sourceVoxelGrid.gridConfig.countLayout)
        countLayout = string(sourceVoxelGrid.gridConfig.countLayout);
    end
    if lower(countLayout) == "nynxnz"
        count3D = countRaw;
    else
        count3D = permute(countRaw, [2, 1, 3]);
    end
    zCenters = gridOrigin(3) + (((1:size(count3D, 3)).' - 0.5) .* voxelSize(3));
    isValid = true;
end

function [pillarCounts, pillarZRange, maxRunLayerCount, occupiedLayerCount, occupiedMask] = computeFineColumnOccupancyFeatures(count3D, zCenters, occupiedLayerMinPoints)
% computeFineColumnOccupancyFeatures: Reduce a fine 3D count tensor
% into fine-column 2D maps, including total point count, occupied z-range,
% maximum contiguous occupied z-run length, and occupied z-slice count.
%
% Input:
%   count3D: [Ny x Nx x Nz] single voxel-count tensor
%   zCenters: [Nz x 1] z-center coordinates
%   occupiedLayerMinPoints: scalar minimum points in a fine z voxel for
%       that z layer to count as occupied
%
% Output:
%   pillarCounts: [Ny x Nx] single total point count per fine column
%   pillarZRange: [Ny x Nx] single occupied z extent in meters
%   maxRunLayerCount: [Ny x Nx] single maximum contiguous occupied slices
%   occupiedLayerCount: [Ny x Nx] single occupied z-slice count per column
%   occupiedMask: [Ny x Nx] logical true when a column has any z layer
%       meeting occupiedLayerMinPoints
    [ny, nx, nz] = size(count3D);
    pillarCounts = single(sum(count3D, 3));
    rawOccupiedMask = pillarCounts > 0;
    occupiedMask = false(ny, nx);
    pillarZRange = zeros(ny, nx, "single");
    maxRunLayerCount = zeros(ny, nx, "single");
    occupiedLayerCount = zeros(ny, nx, "single");
    if nargin < 3 || ~isscalar(occupiedLayerMinPoints) || ~isfinite(occupiedLayerMinPoints)
        occupiedLayerMinPoints = 1;
    end
    occupiedLayerMinPoints = max(1, round(double(occupiedLayerMinPoints)));
    if nz < 1 || ~any(rawOccupiedMask(:))
        return;
    end

    zCenters = double(zCenters(:));
    voxelStep = estimateVoxelStep(zCenters);
    layerMask = count3D >= occupiedLayerMinPoints;
    occupiedLayerCount = single(sum(layerMask, 3));
    occupiedMask = occupiedLayerCount > 0;
    if ~any(occupiedMask(:))
        return;
    end

    [~, firstOccupied] = max(layerMask, [], 3);
    [~, lastOccupiedFromEnd] = max(layerMask(:, :, end:-1:1), [], 3);
    lastOccupied = nz - lastOccupiedFromEnd + 1;
    currentRun = zeros(ny, nx, "single");
    maxRun = zeros(ny, nx, "single");
    for iSlice = 1:nz
        currentRun = (currentRun + 1) .* single(layerMask(:, :, iSlice));
        maxRun = max(maxRun, currentRun);
    end

    maxRunLayerCount = single(maxRun);
    validSpan = occupiedMask & (lastOccupied >= firstOccupied);
    if any(validSpan(:))
        firstIdx = firstOccupied(validSpan);
        lastIdx = lastOccupied(validSpan);
        zRange = zCenters(lastIdx) - zCenters(firstIdx) + voxelStep;
        pillarZRange(validSpan) = single(max(0, zRange));
    end
end

function [xMap, yMap] = buildCoordinateMapsFromGeometry(mapSize, origin, dx, dy)
% buildCoordinateMapsFromGeometry: Build coarse pillar center maps
% from map size, lower-left origin, and axis-aligned cell spacing.
%
% Input:
%   mapSize: [1 x 2] coarse-map size [Ny Nx]
%   origin: [1 x 2] lower-left coarse-map origin in meters
%   dx: scalar coarse-map x spacing in meters
%   dy: scalar coarse-map y spacing in meters
%
% Output:
%   xMap: [Ny x Nx] single pillar x-coordinate map in meters
%   yMap: [Ny x Nx] single pillar y-coordinate map in meters
    ny = mapSize(1);
    nx = mapSize(2);
    xCenters = origin(1) + ((1:nx) - 0.5) .* dx;
    yCenters = origin(2) + ((1:ny) - 0.5) .* dy;
    [xMap, yMap] = meshgrid(single(xCenters), single(yCenters));
end

function count3D = accumulateFineGridBins(xBin, yBin, zBin, gridSize)
% accumulateFineGridBins: Accumulate unit counts into a fine xyz
% voxel tensor from shared bin indices.
%
% Input:
%   xBin: [K x 1] x-bin indices
%   yBin: [K x 1] y-bin indices
%   zBin: [K x 1] z-bin indices
%   gridSize: [1 x 3] size of the output [Ny Nx Nz] tensor
%
% Output:
%   count3D: [Ny x Nx x Nz] single voxel-count tensor
    if isempty(xBin) || isempty(yBin) || isempty(zBin) || numel(gridSize) < 3 || any(gridSize(1:3) < 1)
        count3D = zeros(max(0, gridSize(1)), max(0, gridSize(2)), max(0, gridSize(3)), "single");
        return;
    end
    count3D = accumarray([double(yBin(:)), double(xBin(:)), double(zBin(:))], 1, double(gridSize(1:3)), @sum, 0);
    count3D = single(count3D);
end

function [trafficSignCount3D, trafficSignPointIndices, trafficSignVoxelLinIdx, trafficSignColumnLinIdx] = buildTrafficSignCountTensorFromMetricSamples(pillarGrid, gridOrigin, voxelSize, gridSize, trafficThreshold)
% buildTrafficSignCountTensorFromMetricSamples: Build the high-
% intensity traffic-sign voxel channel in the same [Ny x Nx x Nz] frame as
% the structural count tensor, preferring canonical voxel subscripts so
% traffic-sign points can be removed before facade and pole z-layer maps
% are computed without re-binning metric coordinates.
%
% Input:
%   pillarGrid: struct that may contain fineVoxelX, fineVoxelY,
%       fineVoxelZ, and fineVoxelIntensity samples, or canonical
%       voxel-grid fields pointVoxelSub, pointIndices, and
%       pointAttributes.intensity
%   gridOrigin: [1 x 3] double lower-left-lower voxel edge in meters
%   voxelSize: [1 x 3] double voxel spacing in meters
%   gridSize: [1 x 3] size of the target [Ny x Nx x Nz] tensor
%   trafficThreshold: scalar traffic-sign intensity threshold
%
% Output:
%   trafficSignCount3D: [Ny x Nx x Nz] single count tensor of sign voxels
%   trafficSignPointIndices: [K x 1] original point indices for sign points
%   trafficSignVoxelLinIdx: [M x 1] linear indices of occupied sign voxels
%   trafficSignColumnLinIdx: [C x 1] linear indices of occupied sign columns
    trafficSignCount3D = zeros(gridSize(1), gridSize(2), gridSize(3), "single");
    trafficSignPointIndices = zeros(0, 1);
    trafficSignVoxelLinIdx = zeros(0, 1);
    trafficSignColumnLinIdx = zeros(0, 1);
    if ~isstruct(pillarGrid) || ~isfinite(trafficThreshold) || numel(gridOrigin) < 3 || numel(voxelSize) < 3 || numel(gridSize) < 3
        return;
    end
    if any(double(gridSize(1:3)) < 1)
        return;
    end
    hasCanonicalSubscripts = isfield(pillarGrid, "pointVoxelSub") && size(pillarGrid.pointVoxelSub, 2) >= 3 && ...
        isfield(pillarGrid, "pointAttributes") && isstruct(pillarGrid.pointAttributes) && ...
        isfield(pillarGrid.pointAttributes, "intensity") && ~isempty(pillarGrid.pointAttributes.intensity);
    if hasCanonicalSubscripts
        pointVoxelSub = double(pillarGrid.pointVoxelSub(:, 1:3));
        intensityVals = double(pillarGrid.pointAttributes.intensity(:));
        if numel(intensityVals) == size(pointVoxelSub, 1)
            signMask = isfinite(intensityVals) & (intensityVals > trafficThreshold) & all(isfinite(pointVoxelSub), 2);
            if any(signMask)
                xBin = round(pointVoxelSub(signMask, 1));
                yBin = round(pointVoxelSub(signMask, 2));
                zBin = round(pointVoxelSub(signMask, 3));
                valid = (xBin >= 1) & (xBin <= gridSize(2)) & ...
                    (yBin >= 1) & (yBin <= gridSize(1)) & ...
                    (zBin >= 1) & (zBin <= gridSize(3));
                if any(valid)
                    xBin = xBin(valid);
                    yBin = yBin(valid);
                    zBin = zBin(valid);
                    trafficSignCount3D = accumulateFineGridBins(xBin, yBin, zBin, gridSize);
                    trafficSignVoxelLinIdx = double(unique(sub2ind(double(gridSize(1:3)), yBin(:), xBin(:), zBin(:))));
                    trafficSignColumnLinIdx = double(unique(sub2ind(double(gridSize(1:2)), yBin(:), xBin(:))));
                    if isfield(pillarGrid, "pointIndices") && numel(pillarGrid.pointIndices) == numel(intensityVals)
                        candidatePointIndices = double(pillarGrid.pointIndices(signMask));
                        trafficSignPointIndices = candidatePointIndices(valid);
                    else
                        signLocalIdx = find(signMask);
                        trafficSignPointIndices = double(signLocalIdx(valid));
                    end
                    return;
                end
            end
        end
    end

    if ~all(isfinite(double(gridOrigin(1:3)))) || ~all(isfinite(double(voxelSize(1:3)))) || any(double(voxelSize(1:3)) <= 0)
        return;
    end
    hasMetricPointSamples = isfield(pillarGrid, "fineVoxelX") && isfield(pillarGrid, "fineVoxelY") && isfield(pillarGrid, "fineVoxelZ");
    hasMetricIntensity = isfield(pillarGrid, "fineVoxelIntensity") && ~isempty(pillarGrid.fineVoxelIntensity);
    hasCanonicalPointSamples = isfield(pillarGrid, "points") && size(pillarGrid.points, 2) == 3 && ...
        isfield(pillarGrid, "pointAttributes") && isstruct(pillarGrid.pointAttributes) && ...
        isfield(pillarGrid.pointAttributes, "intensity") && ~isempty(pillarGrid.pointAttributes.intensity);

    if hasMetricPointSamples && hasMetricIntensity
        xVals = double(pillarGrid.fineVoxelX(:));
        yVals = double(pillarGrid.fineVoxelY(:));
        zVals = double(pillarGrid.fineVoxelZ(:));
        intensityVals = double(pillarGrid.fineVoxelIntensity(:));
    elseif hasCanonicalPointSamples
        xVals = double(pillarGrid.points(:, 1));
        yVals = double(pillarGrid.points(:, 2));
        zVals = double(pillarGrid.points(:, 3));
        intensityVals = double(pillarGrid.pointAttributes.intensity(:));
    else
        return;
    end

    if isempty(xVals) || isempty(yVals) || isempty(zVals) || isempty(intensityVals) || ...
            (numel(xVals) ~= numel(yVals)) || (numel(yVals) ~= numel(zVals)) || (numel(zVals) ~= numel(intensityVals))
        return;
    end

    signMask = isfinite(xVals) & isfinite(yVals) & isfinite(zVals) & isfinite(intensityVals) & (intensityVals > trafficThreshold);
    if ~any(signMask)
        return;
    end

    xBin = floor((xVals(signMask) - double(gridOrigin(1))) ./ double(voxelSize(1))) + 1;
    yBin = floor((yVals(signMask) - double(gridOrigin(2))) ./ double(voxelSize(2))) + 1;
    zBin = floor((zVals(signMask) - double(gridOrigin(3))) ./ double(voxelSize(3))) + 1;
    valid = isfinite(xBin) & isfinite(yBin) & isfinite(zBin);
    valid = valid & (xBin >= 1) & (xBin <= gridSize(2)) & (yBin >= 1) & (yBin <= gridSize(1)) & ...
        (zBin >= 1) & (zBin <= gridSize(3));
    if ~any(valid)
        return;
    end

    xBin = xBin(valid);
    yBin = yBin(valid);
    zBin = zBin(valid);
    trafficSignCount3D = accumulateFineGridBins(xBin, yBin, zBin, gridSize);
    trafficSignVoxelLinIdx = double(unique(sub2ind(double(gridSize(1:3)), yBin(:), xBin(:), zBin(:))));
    trafficSignColumnLinIdx = double(unique(sub2ind(double(gridSize(1:2)), yBin(:), xBin(:))));
    if hasCanonicalPointSamples && isfield(pillarGrid, "pointIndices") && numel(pillarGrid.pointIndices) == numel(intensityVals)
        candidatePointIndices = double(pillarGrid.pointIndices(signMask));
        trafficSignPointIndices = candidatePointIndices(valid);
    end
end

function fineVoxelGrid = buildEmptyFineVoxelGrid()
% buildEmptyFineVoxelGrid: Create an empty placeholder for optional
% fine 3D voxel information used by pole refinement.
%
% Input:
%   none
%
% Output:
%   fineVoxelGrid: struct with fields valid, count3D, voxelSize, origin,
%       zCenters, coarseOrigin, coarseVoxelSize, coarseMapSize,
%       sampleXBin, sampleYBin, sampleZ, traffic-sign counts, and
%       traffic-sign point/voxel/column indices
    fineVoxelGrid = struct("valid", false, "count3D", zeros(0, 0, 0, "single"), ...
        "voxelSize", [1, 1, 1], "origin", [0, 0, 0], "zCenters", zeros(0, 1), ...
        "coarseOrigin", [0, 0], "coarseVoxelSize", [1, 1], "coarseMapSize", [0, 0], ...
        "sampleXBin", zeros(0, 1, "int32"), "sampleYBin", zeros(0, 1, "int32"), "sampleZ", zeros(0, 1, "single"), ...
        "trafficSignCount3D", zeros(0, 0, 0, "single"), "trafficSignIntensityThreshold", inf, ...
        "trafficSignPointIndices", zeros(0, 1), "trafficSignVoxelLinIdx", zeros(0, 1), ...
        "trafficSignColumnLinIdx", zeros(0, 1), ...
        "occupiedLayerMinPoints", 1);
end

function threshold = resolveTrafficSignIntensityThreshold(cfg)
% resolveTrafficSignIntensityThreshold: Read and sanitize the
% intensity threshold used to split traffic-sign points away from pole 3D
% refinement and expose them as a dedicated output channel.
%
% Input:
%   cfg: off-ground processing configuration struct
%
% Output:
%   threshold: scalar nonnegative intensity threshold or inf when disabled
    threshold = inf;
    if isfield(cfg, "trafficSignIntensityThreshold") && isscalar(cfg.trafficSignIntensityThreshold) && isfinite(cfg.trafficSignIntensityThreshold)
        threshold = max(0, double(cfg.trafficSignIntensityThreshold));
    end
end
