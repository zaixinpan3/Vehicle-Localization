function groundFeatures = extractGroundFeatures(groundContext, frame, cfg)
% extractGroundFeatures: Run the ground-point feature branch of the
% perception module on the segmented ground points of one frame. Curb
% evidence is scored as energy maps on the ground XY raster and thresholded
% into curb cells; the road surface is grown from ego-near seeds with those
% curb cells as barriers; the curb cells are then refined by road adjacency
% (boundary continuation, gap completion, same-side duplicate and shadow
% suppression, shoulder recovery); the ground points inside the accepted
% cells become curb points and are thinned to the dominant road boundary on
% each side; the road surface is recomputed against the refined curb cells;
% and high-reflectivity road points are selected as road markings. Output
% channels are linear indices into the original organized frame.
%
% Input:
%   groundContext: struct from perceiveFrame with groundPoints [N x 3],
%       groundCellLinIdx [N x 1], groundXYView, groundOriginalPointIdx
%       [N x 1], and groundReflectivity [N x 1]
%   frame: organized point-cloud frame with reflectivity or intensity
%       fields, used when groundContext carries no reflectivity
%   cfg: struct from groundFeatureConfig with curb, road, and roadMarking
%
% Output:
%   groundFeatures: struct with channels.curb and channels.roadMarking as
%       original point-cloud linear indices, plus the cell statistics, curb
%       energy maps, road-surface results, and point masks aligned with
%       groundContext.groundPoints
    assert(isstruct(groundContext) && isfield(groundContext, "groundPoints") && ...
        isfield(groundContext, "groundCellLinIdx") && isfield(groundContext, "groundXYView"), ...
        "groundContext must contain groundPoints, groundCellLinIdx, and groundXYView.");
    curbCfg = cfg.curb;
    roadCfg = cfg.road;
    markingCfg = cfg.roadMarking;

    groundPoints = double(groundContext.groundPoints);
    groundCellLinIdx = double(groundContext.groundCellLinIdx(:));
    groundXYView = groundContext.groundXYView;
    groundOriginalPointIdx = resolveGroundOriginalPointIndices(groundContext, size(groundPoints, 1));
    groundReflectivity = resolveGroundReflectivityForProcessing(groundContext, frame, groundOriginalPointIdx);

    % Curb evidence on the ground raster and the initial road surface
    [cellStats, energyMaps] = buildCurbEnergyMaps(groundContext, curbCfg);
    initialRoadResult = extractRoadSurface(cellStats, energyMaps, groundXYView, roadCfg);

    % Curb cell refinement by adjacency to the road surface
    energyMaps = refineCurbCellsByRoadAdjacency( ...
        energyMaps, initialRoadResult.roadCellMask, curbCfg, groundXYView, initialRoadResult.roadSeedMask);
    extractedPointMask = sampleCellMapAtPoints(groundCellLinIdx, energyMaps.extractedMask) > 0;

    % Curb points: every ground point in an accepted cell, thinned to the dominant boundary
    curbPointMask = selectCurbPointsFromCells( ...
        extractedPointMask, groundPoints, groundCellLinIdx, cellStats, energyMaps, ...
        initialRoadResult.roadCellMask, curbCfg);
    curbPointMask = thinCurbPointsToDominantBoundary( ...
        curbPointMask, groundPoints, groundCellLinIdx, groundXYView, ...
        initialRoadResult.roadSeedMask, initialRoadResult.roadCellMask, energyMaps, curbCfg);

    % Road surface against the refined curb cells, then road markings by reflectivity
    roadResult = extractRoadSurface(cellStats, energyMaps, groundXYView, roadCfg);
    roadPointMask = sampleCellMapAtPoints(groundCellLinIdx, roadResult.roadCellMask) > 0;
    [roadMarkingPointMask, roadMarkingReflectivityThreshold] = extractRoadMarkings( ...
        groundReflectivity, roadPointMask, markingCfg);

    groundFeatures = struct();
    groundFeatures.channels = struct();
    groundFeatures.channels.curb = double(groundOriginalPointIdx(curbPointMask));
    groundFeatures.channels.roadMarking = double(groundOriginalPointIdx(roadMarkingPointMask));
    groundFeatures.channelNames = ["curb", "roadMarking"];
    groundFeatures.stats = cellStats;
    groundFeatures.energyMaps = energyMaps;
    groundFeatures.totalEnergyValues = sampleCellMapAtPoints(groundCellLinIdx, energyMaps.total);
    groundFeatures.initialRoadResult = initialRoadResult;
    groundFeatures.roadResult = roadResult;
    groundFeatures.roadCellMask = roadResult.roadCellMask;
    groundFeatures.roadPointMask = roadPointMask;
    groundFeatures.roadPoints = groundPoints(roadPointMask, :);
    groundFeatures.roadMarkingReflectivityThreshold = roadMarkingReflectivityThreshold;
    groundFeatures.roadMarkingPointMask = roadMarkingPointMask;
    groundFeatures.roadMarkingPoints = groundPoints(roadMarkingPointMask, :);
    groundFeatures.extractedPointMask = extractedPointMask;
    groundFeatures.extractedPoints = groundPoints(extractedPointMask, :);
    groundFeatures.curbPointMask = curbPointMask;
    groundFeatures.curbPoints = groundPoints(curbPointMask, :);
    groundFeatures.groundOriginalPointIdx = groundOriginalPointIdx;
    groundFeatures.groundCellLinIdx = groundCellLinIdx;
    groundFeatures.groundXYView = groundXYView;
    groundFeatures.curbCellMask = logical(energyMaps.extractedMask);
end

function groundOriginalPointIdx = resolveGroundOriginalPointIndices(groundGrid, numGroundPoints)
% resolveGroundOriginalPointIndices: Validate and return the vector
% that maps each row of groundGrid.groundPoints back to its corresponding
% linear index in the original point-cloud frame. The function accepts an
% explicit groundOriginalPointIdx field, a row-aligned groundPointIdx
% field, or the voxel-grid fields used by the visualization context.
%
% Input:
%   groundGrid: struct that stores point-index mapping metadata
%   numGroundPoints: scalar number of rows in groundGrid.groundPoints
%
% Output:
%   groundOriginalPointIdx: [N x 1] double original point-cloud indices
%       aligned with groundGrid.groundPoints
    if isfield(groundGrid, "groundOriginalPointIdx")
        groundOriginalPointIdx = double(groundGrid.groundOriginalPointIdx(:));
    elseif isfield(groundGrid, "groundPointIdx") && numel(groundGrid.groundPointIdx) == numGroundPoints
        groundOriginalPointIdx = double(groundGrid.groundPointIdx(:));
    elseif isfield(groundGrid, "groundVoxelGrid") && isfield(groundGrid.groundVoxelGrid, "pointIndices") && ...
            isfield(groundGrid, "groundPointIdx") && isfield(groundGrid, "groundXYView") && ...
            isfield(groundGrid.groundXYView, "pointCellLinIdx")
        pointIndices = double(groundGrid.groundVoxelGrid.pointIndices(:));
        pointCellLinIdx = double(groundGrid.groundXYView.pointCellLinIdx(:));
        groundPointMask = ismember(pointIndices, double(groundGrid.groundPointIdx(:)));
        validGroundMap = groundPointMask & isfinite(pointCellLinIdx) & pointCellLinIdx >= 1;
        groundOriginalPointIdx = double(pointIndices(validGroundMap));
    else
        error("groundGrid must contain groundOriginalPointIdx aligned with groundPoints.");
    end

    assert(numel(groundOriginalPointIdx) == numGroundPoints, ...
        "groundOriginalPointIdx must have one index per ground point.");
    assert(all(isfinite(groundOriginalPointIdx)) && all(groundOriginalPointIdx >= 1) && ...
        all(groundOriginalPointIdx == floor(groundOriginalPointIdx)), ...
        "groundOriginalPointIdx must contain positive integer linear indices.");
end

function groundReflectivity = resolveGroundReflectivityForProcessing(groundGrid, originalPointCloud, groundOriginalPointIdx)
% resolveGroundReflectivityForProcessing: Return point-level
% reflectivity values aligned with groundGrid.groundPoints, either from
% the ground grid itself or by sampling the original point cloud at the
% supplied original linear indices. If reflectivity is unavailable, the
% function falls back to intensity so the marking channel remains usable on
% datasets that expose only that scalar return channel.
%
% Input:
%   groundGrid: struct with optional groundReflectivity or groundIntensity
%       [N x 1] scalar point channels
%   originalPointCloud: original frame struct with reflectivity or
%       intensity fields
%   groundOriginalPointIdx: [N x 1] original point-cloud linear indices
%
% Output:
%   groundReflectivity: [N x 1] double scalar channel aligned with
%       groundGrid.groundPoints
    numGroundPoints = numel(groundOriginalPointIdx);
    if isfield(groundGrid, "groundReflectivity") && numel(groundGrid.groundReflectivity) == numGroundPoints
        groundReflectivity = double(groundGrid.groundReflectivity(:));
        return;
    end
    if isstruct(originalPointCloud) && isfield(originalPointCloud, "reflectivity")
        reflectivityValues = double(originalPointCloud.reflectivity(:));
        assert(all(groundOriginalPointIdx <= numel(reflectivityValues)), ...
            "groundOriginalPointIdx contains indices outside originalPointCloud.reflectivity.");
        groundReflectivity = reflectivityValues(groundOriginalPointIdx);
        groundReflectivity = double(groundReflectivity(:));
        return;
    end
    if isfield(groundGrid, "groundIntensity") && numel(groundGrid.groundIntensity) == numGroundPoints
        groundReflectivity = double(groundGrid.groundIntensity(:));
        return;
    end
    assert(isstruct(originalPointCloud) && isfield(originalPointCloud, "intensity"), ...
        "originalPointCloud must contain reflectivity or intensity.");
    intensityValues = double(originalPointCloud.intensity(:));
    assert(all(groundOriginalPointIdx <= numel(intensityValues)), ...
        "groundOriginalPointIdx contains indices outside originalPointCloud.intensity.");
    groundReflectivity = intensityValues(groundOriginalPointIdx);
    groundReflectivity = double(groundReflectivity(:));
end
