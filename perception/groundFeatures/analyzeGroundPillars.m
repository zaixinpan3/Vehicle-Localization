function coarseGround = analyzeGroundPillars(groundContext, groundCfg, coarseCfg)
% analyzeGroundPillars: Classify curb and road-marking support
% strictly on the 2D ground-cell raster. The function retains the tuned curb
% energy and road-adjacency stages but skips curb point selection, dominant-
% boundary thinning, and point-level semantic output.
%
% Input:
%   groundContext: ground branch context built by perceiveFrame
%   groundCfg: struct from groundFeatureConfig
%   coarseCfg: struct from coarseSemanticProbabilityCloudConfig
%
% Output:
%   coarseGround: cell masks, evidence probabilities, and source geometry
%       used to build sparse 2D NDT components
    assert(isstruct(groundContext) && all(isfield(groundContext, ...
        ["groundPoints", "groundCellLinIdx", "groundXYView", "groundReflectivity"])), ...
        "groundContext is missing coarse ground-feature inputs.");

    curbCfg = configureFastRefinement(groundCfg.curb, coarseCfg);
    [cellStats, energyMaps] = buildCurbEnergyMaps(groundContext, curbCfg);
    initialRoad = extractRoadSurface( ...
        cellStats, energyMaps, groundContext.groundXYView, groundCfg.road);
    if logical(curbCfg.roadAdjacencyFilterEnabled)
        energyMaps = refineCurbCellsByRoadAdjacency( ...
            energyMaps, initialRoad.roadCellMask, curbCfg, ...
            groundContext.groundXYView, initialRoad.roadSeedMask);
        road = extractRoadSurface( ...
            cellStats, energyMaps, groundContext.groundXYView, groundCfg.road);
    else
        road = initialRoad;
    end

    minimumEnergy = max(0, min(1, double(coarseCfg.curbMinimumTotalEnergy)));
    curbCellMask = logical(energyMaps.extractedMask) & ...
        isfinite(energyMaps.total) & double(energyMaps.total) >= minimumEnergy;
    curbProbability = probabilityAboveThreshold( ...
        double(energyMaps.total), curbCellMask, minimumEnergy, 1, coarseCfg);

    pointCellLinIdx = double(groundContext.groundCellLinIdx(:));
    roadPointMask = sampleCellMapAtPoints(pointCellLinIdx, road.roadCellMask) > 0;
    reflectivity = double(groundContext.groundReflectivity(:));
    reflectivityThreshold = resolveRoadReflectivityThreshold( ...
        reflectivity, roadPointMask, groundCfg.roadMarking);
    maximumReflectivityMap = aggregateCellMaximum( ...
        reflectivity, pointCellLinIdx, size(road.roadCellMask));
    roadMarkingCellMask = logical(road.roadCellMask) & ...
        isfinite(maximumReflectivityMap) & maximumReflectivityMap > reflectivityThreshold;
    finiteRoadReflectivity = maximumReflectivityMap( ...
        logical(road.roadCellMask) & isfinite(maximumReflectivityMap));
    upperReflectivity = reflectivityThreshold;
    if ~isempty(finiteRoadReflectivity)
        upperReflectivity = max(finiteRoadReflectivity);
    end
    roadMarkingProbability = probabilityAboveThreshold( ...
        maximumReflectivityMap, roadMarkingCellMask, reflectivityThreshold, ...
        upperReflectivity, coarseCfg);

    coarseGround = struct();
    coarseGround.cellMapSize = double(size(road.roadCellMask));
    coarseGround.cellOrigin = double(groundContext.groundXYView.origin(1:2));
    coarseGround.cellSize = double(groundContext.groundXYView.cellSize(1:2));
    coarseGround.pillarOffset = [0 0];
    if isfield(groundContext.groundXYView,"pillarOffset")
        coarseGround.pillarOffset = groundContext.groundXYView.pillarOffset;
    end
    coarseGround.curbCellMask = curbCellMask;
    coarseGround.curbProbability = single(curbProbability);
    coarseGround.roadMarkingCellMask = roadMarkingCellMask;
    coarseGround.roadMarkingProbability = single(roadMarkingProbability);
    coarseGround.roadCellMask = logical(road.roadCellMask);
    coarseGround.roadMarkingReflectivityThreshold = double(reflectivityThreshold);
    coarseGround.maximumReflectivityMap = single(maximumReflectivityMap);
    coarseGround.stats = cellStats;
    % Convert the internal [Nx Ny] ground indexing to the public [Ny Nx] layout.
    dims = size(road.roadCellMask);
    [xBin, yBin] = ind2sub(dims([2, 1]), pointCellLinIdx);
    cellRows = sub2ind(dims, yBin, xBin);
    coarseGround.moments = aggregatePlanarCellMoments( ...
        groundContext.groundPoints, cellRows, prod(dims), coarseCfg.projectionRotation, coarseCfg.projectionTranslation);
    coarseGround.energyMaps = energyMaps;
    coarseGround.initialRoadResult = initialRoad;
    coarseGround.roadResult = road;
end

function curbCfg = configureFastRefinement(curbCfg, coarseCfg)
% configureFastRefinement: Apply optional research ablations. The production
% default preserves every pillar topology stage; point refinement is separate.
    if ~isfield(coarseCfg, "curbDisabledRefinementStages")
        return;
    end
    disabledStages = string(coarseCfg.curbDisabledRefinementStages(:));
    for stageName = disabledStages.'
        if isfield(curbCfg, stageName)
            curbCfg.(stageName) = false;
        end
    end
end

function maximumMap = aggregateCellMaximum(values, pointCellLinIdx, mapSize)
% aggregateCellMaximum: Aggregate finite point values into the ground
% raster while respecting its internal [Nx Ny] point-index convention.
    numRows = double(mapSize(1));
    numCols = double(mapSize(2));
    numCells = numRows .* numCols;
    values = double(values(:));
    pointCellLinIdx = double(pointCellLinIdx(:));
    valid = isfinite(values) & isfinite(pointCellLinIdx) & ...
        pointCellLinIdx >= 1 & pointCellLinIdx <= numCells & ...
        pointCellLinIdx == floor(pointCellLinIdx);
    maximumVector = -inf(numCells, 1);
    if any(valid)
        maximumVector = accumarray( ...
            pointCellLinIdx(valid), values(valid), [numCells, 1], @max, -inf);
    end
    maximumMap = reshape(maximumVector, numCols, numRows).';
end

function probabilityMap = probabilityAboveThreshold(valueMap, selectionMask, lowerValue, upperValue, cfg)
% probabilityAboveThreshold: Map accepted evidence monotonically into the
% configured probability interval without assigning mass to rejected cells.
    probabilityMap = zeros(size(valueMap));
    if ~any(selectionMask(:))
        return;
    end
    probabilityFloor = max(0, min(1, double(cfg.minimumSemanticProbability)));
    scale = max(double(upperValue) - double(lowerValue), eps);
    normalized = (double(valueMap) - double(lowerValue)) ./ scale;
    normalized = min(max(normalized, 0), 1);
    probabilityMap(selectionMask) = probabilityFloor + ...
        ((1 - probabilityFloor) .* normalized(selectionMask));
    probabilityMap(~isfinite(probabilityMap)) = 0;
end
