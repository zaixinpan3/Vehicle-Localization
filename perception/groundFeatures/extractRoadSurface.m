function result = extractRoadSurface(stats, energyMaps, xyView, cfg)
% extractRoadSurface: Extract road-surface XY cells by using
% detected curb cells as non-traversable barriers, selecting ego-near
% supported ground cells as seeds, and growing an 8-connected region
% through locally height-continuous candidate cells so curved and branching
% road layouts can be retained without scanline assumptions.
%
% Input:
%   stats: struct with countMap, heightMap, roughnessMap, and supportMask
%   energyMaps: struct with total and extractedMask curb-energy rasters
%   xyView: XY view struct with occupiedMask, xMap, and yMap rasters
%   cfg: struct returned by groundFeatureConfig().road
%
% Output:
%   result: struct with roadCellMask, roadSeedMask, candidateRoadMask,
%       curbCellMask, curbBarrierMask, and diagnostic cell counts
    assert(isstruct(stats) && isfield(stats, "countMap") && isfield(stats, "heightMap") && ...
        isfield(stats, "roughnessMap") && isfield(stats, "supportMask"), ...
        "stats must contain countMap, heightMap, roughnessMap, and supportMask.");
    assert(isstruct(energyMaps) && isfield(energyMaps, "total") && isfield(energyMaps, "extractedMask"), ...
        "energyMaps must contain total and extractedMask.");
    assert(isstruct(xyView) && isfield(xyView, "occupiedMask") && isfield(xyView, "xMap") && isfield(xyView, "yMap"), ...
        "xyView must contain occupiedMask, xMap, and yMap.");

    countMap = double(stats.countMap);
    heightMap = double(stats.heightMap);
    roughnessMap = double(stats.roughnessMap);
    totalEnergyMap = double(energyMaps.total);
    supportMask = logical(stats.supportMask);
    occupiedMask = logical(xyView.occupiedMask);
    curbCellMask = logical(energyMaps.extractedMask) & supportMask;
    curbBarrierMask = growMaskByChebyshevRadius(curbCellMask, cfg.curbBarrierRadiusCells);
    validGroundMask = occupiedMask & supportMask & isfinite(heightMap) & countMap >= double(cfg.minPointsPerCell);

    if isfinite(double(cfg.maxRoadEnergy))
        validGroundMask = validGroundMask & isfinite(totalEnergyMap) & totalEnergyMap <= double(cfg.maxRoadEnergy);
    end
    if isfinite(double(cfg.maxRoadRoughnessMeters))
        validGroundMask = validGroundMask & isfinite(roughnessMap) & roughnessMap <= double(cfg.maxRoadRoughnessMeters);
    end

    candidateRoadMask = validGroundMask & ~curbBarrierMask;
    roadSeedMask = buildRoadSeedMask(candidateRoadMask, xyView, cfg);
    curbInteriorMask = buildCurbBoundedInteriorMask(curbCellMask, xyView, roadSeedMask, cfg);
    if logical(cfg.curbInteriorConstraintEnabled) && any(curbInteriorMask(:))
        candidateRoadMask = candidateRoadMask & curbInteriorMask;
        roadSeedMask = roadSeedMask & candidateRoadMask;
        if ~any(roadSeedMask(:))
            roadSeedMask = buildRoadSeedMask(candidateRoadMask, xyView, cfg);
        end
    end
    roadCellMask = growRoadCellsFromSeeds(candidateRoadMask, roadSeedMask, heightMap, cfg);
    roadCellMask = removeSmallRoadComponents(roadCellMask, cfg.minRoadCells);

    result = struct();
    result.roadCellMask = roadCellMask;
    result.roadSeedMask = roadSeedMask;
    result.candidateRoadMask = candidateRoadMask;
    result.curbInteriorMask = curbInteriorMask;
    result.curbCellMask = curbCellMask;
    result.curbBarrierMask = curbBarrierMask;
    result.numRoadCells = nnz(roadCellMask);
    result.numRoadSeedCells = nnz(roadSeedMask);
    result.numCandidateRoadCells = nnz(candidateRoadMask);
    result.numCurbInteriorCells = nnz(curbInteriorMask);
    result.numCurbCells = nnz(curbCellMask);
    result.numCurbBarrierCells = nnz(curbBarrierMask);
end

function interiorMask = buildCurbBoundedInteriorMask(curbCellMask, xyView, seedMask, cfg)
% buildCurbBoundedInteriorMask: Build an approximate road-interior
% corridor by splitting detected curb cells into lower and upper boundary
% samples relative to the ego-near seed centerline, interpolating those
% boundaries over grid columns, and marking only cells between the two
% curb-side boundaries as eligible for road-surface growth.
%
% Input:
%   curbCellMask: [Ny x Nx] logical detected curb-cell mask
%   xyView: XY view struct with y centers, yMap, and cellSize fields
%   seedMask: [Ny x Nx] logical initial road seed cells
%   cfg: struct returned by groundFeatureConfig().road
%
% Output:
%   interiorMask: [Ny x Nx] logical curb-bounded interior corridor mask
    interiorMask = true(size(curbCellMask));
    if ~logical(cfg.curbInteriorConstraintEnabled) || ~any(curbCellMask(:))
        return;
    end

    [numRows, numCols] = size(curbCellMask);
    yCenters = double(xyView.yCenters(:));
    yMap = double(xyView.yMap);
    referenceY = double(cfg.fallbackSeedReferenceXYMeters(2));
    seedYValues = yMap(logical(seedMask) & isfinite(yMap));
    if ~isempty(seedYValues)
        referenceY = median(seedYValues, "omitnan");
    end

    [curbRows, curbCols] = find(logical(curbCellMask));
    lower = yCenters(curbRows) < referenceY;
    upper = yCenters(curbRows) > referenceY;
    lowerRowsByCol = accumarray(curbCols(lower),curbRows(lower),[numCols 1],@max,NaN);
    upperRowsByCol = accumarray(curbCols(upper),curbRows(upper),[numCols 1],@min,NaN);

    minBoundaryCells = max(2, round(double(cfg.curbInteriorMinBoundaryCells)));
    validLowerCols = find(isfinite(lowerRowsByCol));
    validUpperCols = find(isfinite(upperRowsByCol));
    if numel(validLowerCols) < minBoundaryCells || numel(validUpperCols) < minBoundaryCells
        return;
    end

    allCols = (1:numCols).';
    lowerRows = interpolateBoundaryRowsWithEndpointHold(validLowerCols, lowerRowsByCol(validLowerCols), allCols);
    upperRows = interpolateBoundaryRowsWithEndpointHold(validUpperCols, upperRowsByCol(validUpperCols), allCols);
    maxExtrapolationCells = max(0, round(double(cfg.curbInteriorMaxExtrapolationMeters) ./ double(xyView.cellSize(1))));
    lowerSupported = allCols >= min(validLowerCols) - maxExtrapolationCells & allCols <= max(validLowerCols) + maxExtrapolationCells;
    upperSupported = allCols >= min(validUpperCols) - maxExtrapolationCells & allCols <= max(validUpperCols) + maxExtrapolationCells;
    supportedCols = lowerSupported & upperSupported & isfinite(lowerRows) & isfinite(upperRows) & lowerRows < upperRows;
    clearanceCells = max(0, round(double(cfg.curbInteriorClearanceCells)));
    rowGrid = repmat((1:numRows).', 1, numCols);
    lowerGrid = repmat((lowerRows(:).' + clearanceCells), numRows, 1);
    upperGrid = repmat((upperRows(:).' - clearanceCells), numRows, 1);
    interiorMask = rowGrid >= lowerGrid & rowGrid <= upperGrid;
    interiorMask(:, ~supportedCols) = false;
end

function boundaryRows = interpolateBoundaryRowsWithEndpointHold(validCols, validRows, queryCols)
% interpolateBoundaryRowsWithEndpointHold: Interpolate curb-boundary
% row samples inside their observed column span and hold the nearest
% endpoint outside that span so missing near-field curb samples do not
% create linearly extrapolated road corridors wider than the observed
% curb-side boundary.
%
% Input:
%   validCols: [K x 1] numeric grid columns with observed boundary rows
%   validRows: [K x 1] numeric observed boundary rows
%   queryCols: [Q x 1] numeric grid columns to evaluate
%
% Output:
%   boundaryRows: [Q x 1] numeric interpolated boundary rows
    validCols = double(validCols(:));
    validRows = double(validRows(:));
    queryCols = double(queryCols(:));
    boundaryRows = interp1(validCols, validRows, queryCols, "linear", "extrap");
    lowerSide = queryCols < validCols(1);
    upperSide = queryCols > validCols(end);
    boundaryRows(lowerSide) = validRows(1);
    boundaryRows(upperSide) = validRows(end);
end

function seedMask = buildRoadSeedMask(candidateRoadMask, xyView, cfg)
% buildRoadSeedMask: Select initial road-region seeds from supported
% candidate cells in a configurable ego-forward center corridor, with an
% optional nearest-cell fallback when the preferred corridor is empty.
%
% Input:
%   candidateRoadMask: [Ny x Nx] logical cells eligible for road growth
%   xyView: XY view struct with xMap and yMap metric center rasters
%   cfg: struct returned by groundFeatureConfig().road
%
% Output:
%   seedMask: [Ny x Nx] logical seed cells aligned with candidateRoadMask
    xMap = double(xyView.xMap);
    yMap = double(xyView.yMap);
    seedXLimits = sort(double(cfg.seedXLimitsMeters(:)).');
    seedMask = candidateRoadMask & xMap >= seedXLimits(1) & xMap <= seedXLimits(2) & ...
        abs(yMap) <= double(cfg.seedAbsYMaxMeters);
    if any(seedMask(:)) || ~logical(cfg.fallbackSeedEnabled)
        return;
    end

    candidateIdx = find(candidateRoadMask & isfinite(xMap) & isfinite(yMap));
    if isempty(candidateIdx)
        return;
    end

    referenceXY = double(cfg.fallbackSeedReferenceXYMeters(:)).';
    distanceSquared = (xMap(candidateIdx) - referenceXY(1)) .^ 2 + (yMap(candidateIdx) - referenceXY(2)) .^ 2;
    [~, bestIdx] = min(distanceSquared);
    seedMask(candidateIdx(bestIdx)) = true;
end

function roadCellMask = growRoadCellsFromSeeds(candidateRoadMask, seedMask, heightMap, cfg)
% growRoadCellsFromSeeds: Grow a road-cell region from one or more
% seed cells through 8-connected candidate cells while enforcing local
% neighbor-to-neighbor height continuity and optional global seed-height
% deviation gating.
%
% Input:
%   candidateRoadMask: [Ny x Nx] logical cells eligible for road growth
%   seedMask: [Ny x Nx] logical initial seed cells
%   heightMap: [Ny x Nx] numeric representative ground height map
%   cfg: struct returned by groundFeatureConfig().road
%
% Output:
%   roadCellMask: [Ny x Nx] logical grown road-region cells
    candidateRoadMask = logical(candidateRoadMask) & isfinite(heightMap);
    seedIdx = find(logical(seedMask) & candidateRoadMask);
    roadCellMask = false(size(candidateRoadMask));
    if isempty(seedIdx)
        return;
    end

    [numRows, numCols] = size(candidateRoadMask);
    queue = zeros(nnz(candidateRoadMask), 1);
    queueCount = numel(seedIdx);
    queueHead = 1;
    queue(1:queueCount) = seedIdx(:);
    roadCellMask(seedIdx) = true;
    seedHeight = median(double(heightMap(seedIdx)), "omitnan");
    maxNeighborStep = double(cfg.maxNeighborHeightStepMeters);
    maxSeedDeviation = double(cfg.maxSeedHeightDeviationMeters);
    if isfield(cfg, "useNativeKernels") && cfg.useNativeKernels
        roadCellMask = perceptionKernelsMex('growRoad', candidateRoadMask, ...
            double(seedIdx), double(heightMap), [seedHeight, maxNeighborStep, maxSeedDeviation]);
        return;
    end

    while queueHead <= queueCount
        currentIdx = queue(queueHead);
        queueHead = queueHead + 1;
        [rowIdx, colIdx] = ind2sub([numRows, numCols], currentIdx);
        currentHeight = double(heightMap(currentIdx));
        for rowOffset = -1:1
            neighborRow = rowIdx + rowOffset;
            if neighborRow < 1 || neighborRow > numRows
                continue;
            end
            for colOffset = -1:1
                if rowOffset == 0 && colOffset == 0
                    continue;
                end
                neighborCol = colIdx + colOffset;
                if neighborCol < 1 || neighborCol > numCols
                    continue;
                end
                neighborIdx = neighborRow + ((neighborCol - 1) * numRows);
                if roadCellMask(neighborIdx) || ~candidateRoadMask(neighborIdx)
                    continue;
                end
                neighborHeight = double(heightMap(neighborIdx));
                if isfinite(maxNeighborStep) && abs(neighborHeight - currentHeight) > maxNeighborStep
                    continue;
                end
                if isfinite(maxSeedDeviation) && abs(neighborHeight - seedHeight) > maxSeedDeviation
                    continue;
                end
                queueCount = queueCount + 1;
                queue(queueCount) = neighborIdx;
                roadCellMask(neighborIdx) = true;
            end
        end
    end
end

function filteredMask = removeSmallRoadComponents(roadCellMask, minRoadCells)
% removeSmallRoadComponents: Remove small connected road-cell islands
% after seeded growth while retaining all sufficiently large branches that
% remain connected to the accepted seed-grown region.
%
% Input:
%   roadCellMask: [Ny x Nx] logical road-cell mask
%   minRoadCells: scalar minimum component size in cells
%
% Output:
%   filteredMask: [Ny x Nx] logical road-cell mask after size filtering
    components = connectedComponents8(logical(roadCellMask));
    filteredMask = false(size(roadCellMask));
    minRoadCells = max(1, round(double(minRoadCells)));
    for k = 1:numel(components)
        componentIdx = components{k};
        if numel(componentIdx) >= minRoadCells
            filteredMask(componentIdx) = true;
        end
    end
end

function grownMask = growMaskByChebyshevRadius(mask, radiusCells)
% growMaskByChebyshevRadius: Dilate a logical raster by a square
% Chebyshev-radius neighborhood using convolution so curb cells can be
% expanded into a configurable non-traversable barrier region.
%
% Input:
%   mask: [Ny x Nx] logical raster mask
%   radiusCells: scalar nonnegative dilation radius in cells
%
% Output:
%   grownMask: [Ny x Nx] logical dilated mask
    radiusCells = max(0, round(double(radiusCells)));
    mask = logical(mask);
    if radiusCells == 0 || isempty(mask)
        grownMask = mask;
        return;
    end

    kernel = ones(2 * radiusCells + 1, 2 * radiusCells + 1);
    grownMask = conv2(double(mask), kernel, "same") > 0;
end
