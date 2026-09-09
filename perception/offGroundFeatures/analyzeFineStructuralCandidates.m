function fineOffGround = analyzeFineStructuralCandidates(voxelGrid, offGroundCfg, productCfg)
% analyzeFineStructuralCandidates: OFFLINE ONLY detailed structural candidates.
% Fine perception rebuilds this independent index after the coarse result.
% Detect poles, facades, and traffic signs from XY pillars and their
% sparse height histograms. Semantic decisions use vertical runs, neighboring
% pillar contrast and footprint shape. No dense 3D tensor or point-level
% feature refinement is constructed.
%
% Input:
%   voxelGrid: compact off-ground voxel metadata from perceiveFrame
%   offGroundCfg: struct from fineStructuralConfig
%   productCfg: struct from coarseSemanticProbabilityCloudConfig
%
% Output:
%   fineOffGround: pole cell mask/probability, sparse point-to-column
%       assignments, and column diagnostics
    assert(isstruct(voxelGrid) && all(isfield(voxelGrid, ...
        ["gridConfig", "points", "pointVoxelSub", "pointAttributes"])), ...
        "voxelGrid is missing coarse off-ground inputs.");

    [columnMaps, sparseFineGrid] = ...
        buildSparseColumnMaps(voxelGrid, offGroundCfg, productCfg);
    columnMaps.runLayerMap = single(columnMaps.maxRunLayerCount);
    columnMaps.supportEvidence = columnMaps.runLayerMap;
    columnMaps.rawLayerCount = single(columnMaps.runLayerMap);
    columnMaps.supportScore = single(columnMaps.runLayerMap);
    columnMaps.pointScore = zeros(columnMaps.mapSize,"single");
    columnMaps.lineScore = zeros(columnMaps.mapSize,"single");
    if any(ismember(string(productCfg.semanticNames),["pole","facade"]))
        [columnMaps.pointScore, columnMaps.lineScore, ...
            columnMaps.normalOrientation, columnMaps.blobness] = ...
            buildPillarShapeScores(columnMaps.runLayerMap, ...
            columnMaps.occupiedMask, offGroundCfg, columnMaps.dx, columnMaps.dy);
    end

    facade = struct("mask",false(columnMaps.mapSize));
    if any(string(productCfg.semanticNames)=="facade")
        facadeCfg = offGroundCfg;
        facade = extractFacadeFeatures(columnMaps, facadeCfg);
        facade = expandFacadePillarSupport(facade,columnMaps,offGroundCfg);
    end

    probabilityFloor = max(0, min(1, double(productCfg.minimumSemanticProbability)));
    poleCellMask = false(columnMaps.mapSize);
    poleProbability = zeros(columnMaps.mapSize);
    poleParams = struct(); candidates = struct();
    if any(string(productCfg.semanticNames)=="pole")
        poleParams = resolvePoleDetectionParams(offGroundCfg);
        candidates = detectPoleCandidates( ...
            columnMaps, facade.mask, sparseFineGrid, poleParams);
        % Preserve candidate footprints as objects. Independent per-pillar
        % post-gates can remove one half of a pole crossing a grid boundary.
        poleCellMask = logical(candidates.candidateMask);
        footprints = bwconncomp(poleCellMask,8);
        for footprint = footprints.PixelIdxList
            cells = footprint{1};
            if mean(double(columnMaps.pointScore(cells))) < productCfg.poleMinimumFootprintScore
                poleCellMask(cells) = false;
            end
        end

        runScale = max(1, double(poleParams.coreMinRunLayerThreshold));
        runConfidence = min(double(columnMaps.runLayerMap) ./ runScale, 1);
        shapeConfidence = min(max(double(columnMaps.pointScore), 0), 1) .* ...
            (1 - min(max(double(columnMaps.lineScore), 0), 1));
        poleProbability = zeros(columnMaps.mapSize);
        combinedConfidence = min(max(shapeConfidence .* runConfidence, 0), 1);
        poleProbability(poleCellMask) = probabilityFloor + ...
            ((1 - probabilityFloor) .* combinedConfidence(poleCellMask));
    end

    fineOffGround = struct();
    fineOffGround.facade = facade;
    fineOffGround.facadeCellMask = facade.mask;
    fineOffGround.facadeProbability = single(facade.mask .* ...
        (probabilityFloor + (1-probabilityFloor).*double(columnMaps.lineScore)));
    fineOffGround.trafficSignCellMask = columnMaps.trafficSignCellMask;
    fineOffGround.trafficSignProbability = single(columnMaps.trafficSignCellMask .* ...
        (probabilityFloor + (1-probabilityFloor).*columnMaps.trafficSignEvidence));
    fineOffGround.poleCellMask = poleCellMask;
    fineOffGround.poleProbability = single(poleProbability);
    fineOffGround.columnMaps = columnMaps;
    fineOffGround.candidates = candidates;
    fineOffGround.poleParams = poleParams;
end

function [columnMaps, sparseFineGrid] = buildSparseColumnMaps(voxelGrid, cfg, productCfg)
% buildSparseColumnMaps: Compute the same column occupancy statistics as the
% dense fine-grid branch from sorted occupied voxels and 2D accumulations.
    dims = round(double(voxelGrid.gridConfig.dims(1:3)));
    voxelSize = double(voxelGrid.gridConfig.voxelSize(1:3));
    origin = double(voxelGrid.gridConfig.minCorner(1:3));
    nx = max(0, dims(1));
    ny = max(0, dims(2));
    nz = max(0, dims(3));
    mapSize = [ny, nx];
    pointVoxelSub = double(voxelGrid.pointVoxelSub(:, 1:3));
    numPoints = size(pointVoxelSub, 1);
    validPoint = numPoints > 0 & all(isfinite(pointVoxelSub), 2) & ...
        pointVoxelSub(:, 1) >= 1 & pointVoxelSub(:, 1) <= nx & ...
        pointVoxelSub(:, 2) >= 1 & pointVoxelSub(:, 2) <= ny & ...
        pointVoxelSub(:, 3) >= 1 & pointVoxelSub(:, 3) <= nz;
    projectionRotation = productCfg.projectionRotation;
    projectionTranslation = productCfg.projectionTranslation;
    useNative = isfield(cfg,"useNativeKernels") && cfg.useNativeKernels;
    intensity = nan(numPoints,1);
    structuralPointMask = validPoint;
    trafficThreshold = resolveTrafficThreshold(cfg);
    if isfield(voxelGrid.pointAttributes, "intensity") && ...
            numel(voxelGrid.pointAttributes.intensity) == numPoints
        intensity = double(voxelGrid.pointAttributes.intensity(:));
        structuralPointMask = structuralPointMask & ...
            ~(isfinite(intensity) & intensity > trafficThreshold);
    end

    if ~any(ismember(string(productCfg.semanticNames),["pole","facade"]))
        structuralPointMask(:) = false;
    end
    occupiedLayerMinPoints = resolvePoleOccupiedLayerMinPoints(cfg);
    [pillarCounts, occupiedLayerCount, maxRunLayerCount, pillarZRange, sparseVoxels] = ...
        accumulateSparseOccupancy(pointVoxelSub(structuralPointMask, :), ...
        mapSize, nz, voxelSize(3), occupiedLayerMinPoints);
    occupiedMask = occupiedLayerCount > 0;
    [xMap, yMap] = coordinateMaps(mapSize, origin(1:2), voxelSize(1:2));
    xMap(~occupiedMask) = NaN;
    yMap(~occupiedMask) = NaN;

    columnMaps = struct();
    cellRows = sub2ind(mapSize, pointVoxelSub(structuralPointMask, 2), pointVoxelSub(structuralPointMask, 1));
    columnMaps.moments = aggregatePlanarCellMoments(voxelGrid.points(structuralPointMask, :), cellRows, prod(mapSize), projectionRotation, projectionTranslation, useNative);
    % Radiometric evidence selects whole pillars. Their Gaussian contains
    % every off-ground member, including nonreflective support at other heights.
    columnMaps.trafficSignCellMask = false(mapSize);
    columnMaps.trafficSignEvidence = zeros(mapSize);
    if any(string(productCfg.semanticNames) == "trafficSign")
        allColumns = sub2ind(mapSize, pointVoxelSub(validPoint,2), pointVoxelSub(validPoint,1));
        values = intensity(validPoint); values(~isfinite(values)) = 0;
        maxima = accumarray(allColumns,values,[prod(mapSize),1],@max,0);
        columnMaps.trafficSignCellMask = reshape(maxima > trafficThreshold,mapSize);
        columnMaps.trafficSignEvidence = reshape(max(0,1-trafficThreshold./max(maxima,eps)),mapSize);
        candidateMembers = columnMaps.trafficSignCellMask(allColumns);
        validRows = find(validPoint);
        columnMaps.trafficSignMoments = aggregatePlanarCellMoments(voxelGrid.points(validRows(candidateMembers),:), ...
            allColumns(candidateMembers),prod(mapSize),projectionRotation,projectionTranslation,useNative);
    end
    columnMaps.voxelStatistics = sparseVoxels;
    columnMaps.mapSize = double(mapSize);
    columnMaps.origin = double(origin(1:2));
    columnMaps.dx = double(voxelSize(1));
    columnMaps.dy = double(voxelSize(2));
    columnMaps.xMap = xMap;
    columnMaps.yMap = yMap;
    columnMaps.occupiedMask = occupiedMask;
    columnMaps.pillarCounts = pillarCounts;
    columnMaps.pillarZRange = pillarZRange;
    columnMaps.occupiedLayerCount = occupiedLayerCount;
    columnMaps.maxRunLayerCount = maxRunLayerCount;
    columnMaps.occupiedLayerMinPoints = double(occupiedLayerMinPoints);
    sparseFineGrid = struct( ...
        "count3D", zeros(0, 0, 0, "single"), ...
        "occupiedLayerMinPoints", double(occupiedLayerMinPoints), ...
        "sparseVoxelColumnLinIdx", sparseVoxels.columnLinIdx, ...
        "sparseVoxelZBin", sparseVoxels.zBin, ...
        "sparseVoxelCount", sparseVoxels.count, ...
        "sparseMapSize", double(mapSize), ...
        "sparseNumZLayers", double(nz));
end

function [pillarCounts, occupiedLayerCount, maxRunLayerCount, pillarZRange, sparseVoxels] = ...
        accumulateSparseOccupancy(pointVoxelSub, mapSize, nz, dz, occupiedLayerMinPoints)
% accumulateSparseOccupancy: Reduce structural point bins to dense 2D maps
% while keeping the vertical occupancy representation sparse.
    ny = double(mapSize(1));
    nx = double(mapSize(2));
    numColumns = ny .* nx;
    pillarCounts = zeros(mapSize, "single");
    occupiedLayerCount = zeros(mapSize, "single");
    maxRunLayerCount = zeros(mapSize, "single");
    pillarZRange = zeros(mapSize, "single");
    sparseVoxels = struct( ...
        "columnLinIdx", zeros(0, 1, "int32"), ...
        "zBin", zeros(0, 1, "int32"), ...
        "count", zeros(0, 1, "single"));
    if isempty(pointVoxelSub)
        return;
    end
    if any([numColumns, nz] < 1)
        return;
    end

    xBin = double(pointVoxelSub(:, 1));
    yBin = double(pointVoxelSub(:, 2));
    zBin = double(pointVoxelSub(:, 3));
    columnLinIdx = yBin + ((xBin - 1) .* ny);
    pillarCountVector = accumarray(columnLinIdx, 1, [numColumns, 1], @sum, 0);
    pillarCounts = reshape(single(pillarCountVector), mapSize);

    voxelLinIdx = columnLinIdx + ((zBin - 1) .* numColumns);
    sortedVoxelLinIdx = sort(voxelLinIdx);
    voxelStartMask = [true; diff(sortedVoxelLinIdx) ~= 0];
    voxelStartIdx = find(voxelStartMask);
    voxelCounts = diff([voxelStartIdx; numel(sortedVoxelLinIdx) + 1]);
    uniqueVoxelLinIdx = sortedVoxelLinIdx(voxelStartIdx);
    voxelColumn = mod(uniqueVoxelLinIdx - 1, numColumns) + 1;
    voxelZ = floor((uniqueVoxelLinIdx - 1) ./ numColumns) + 1;
    sparseVoxels.columnLinIdx = int32(voxelColumn(:));
    sparseVoxels.zBin = int32(voxelZ(:));
    sparseVoxels.count = single(voxelCounts(:));
    qualifiedMask = voxelCounts >= occupiedLayerMinPoints;
    qualifiedVoxelLinIdx = uniqueVoxelLinIdx(qualifiedMask);
    if isempty(qualifiedVoxelLinIdx)
        return;
    end

    qualifiedColumn = mod(qualifiedVoxelLinIdx - 1, numColumns) + 1;
    qualifiedZ = floor((qualifiedVoxelLinIdx - 1) ./ numColumns) + 1;
    occupiedLayerVector = accumarray( ...
        qualifiedColumn, 1, [numColumns, 1], @sum, 0);
    minimumZ = accumarray(qualifiedColumn, qualifiedZ, ...
        [numColumns, 1], @min, NaN);
    maximumZ = accumarray(qualifiedColumn, qualifiedZ, ...
        [numColumns, 1], @max, NaN);

    orderedPairs = sortrows([qualifiedColumn, qualifiedZ], [1, 2]);
    newRun = [true; diff(orderedPairs(:, 1)) ~= 0 | diff(orderedPairs(:, 2)) ~= 1];
    runStartIdx = find(newRun);
    runLengths = diff([runStartIdx; size(orderedPairs, 1) + 1]);
    runColumns = orderedPairs(runStartIdx, 1);
    maximumRunVector = accumarray( ...
        runColumns, runLengths, [numColumns, 1], @max, 0);

    zRangeVector = zeros(numColumns, 1);
    validSpan = isfinite(minimumZ) & isfinite(maximumZ);
    zRangeVector(validSpan) = ...
        ((maximumZ(validSpan) - minimumZ(validSpan)) + 1) .* double(dz);
    occupiedLayerCount = reshape(single(occupiedLayerVector), mapSize);
    maxRunLayerCount = reshape(single(maximumRunVector), mapSize);
    pillarZRange = reshape(single(zRangeVector), mapSize);
end

function [xMap, yMap] = coordinateMaps(mapSize, origin, voxelSize)
% coordinateMaps: Build metric XY center maps for a [Ny Nx] raster.
    xCenters = origin(1) + (((1:mapSize(2)) - 0.5) .* voxelSize(1));
    yCenters = origin(2) + (((1:mapSize(1)) - 0.5) .* voxelSize(2));
    [xMap, yMap] = meshgrid(single(xCenters), single(yCenters));
end

function threshold = resolveTrafficThreshold(cfg)
% resolveTrafficThreshold: Read the high-intensity structural exclusion.
    threshold = inf;
    if isfield(cfg, "trafficSignIntensityThreshold") && ...
            isscalar(cfg.trafficSignIntensityThreshold) && ...
            isfinite(cfg.trafficSignIntensityThreshold)
        threshold = max(0, double(cfg.trafficSignIntensityThreshold));
    end
end

function facade = expandFacadePillarSupport(facade,maps,cfg)
% Include the refinement halo into the offline candidate stage so every
% point subsequently validated already belongs to a published candidate.
% This expansion uses only pillar occupancy and distances between XY centers.
    if ~any(facade.mask(:)), return; end
    radius = max(0,round(cfg.facadeRefineHorizontalSupportRadiusVoxels));
    seedMap = facade.lineMap;
    bestDistance = inf(maps.mapSize);
    [cols,rows] = meshgrid(1:maps.mapSize(2),1:maps.mapSize(1));
    x = maps.origin(1)+(cols-0.5)*maps.dx;
    y = maps.origin(2)+(rows-0.5)*maps.dy;
    for k = 1:size(facade.detectedLines,1)
        support = imdilate(seedMap==k,true(2*radius+1)) & maps.pillarCounts>0;
        line = facade.detectedLines(k,:);
        direction = line(3:4)-line(1:2);
        distance = abs((x-line(1))*direction(2)-(y-line(2))*direction(1))/max(norm(direction),eps);
        update = support & distance<bestDistance;
        facade.lineMap(update) = uint16(k);
        bestDistance(update) = distance(update);
    end
    facade.mask = facade.lineMap>0;
    facade.pillarLinIdx = find(facade.mask);
    facade.pillarLineIdx = double(facade.lineMap(facade.pillarLinIdx));
end
