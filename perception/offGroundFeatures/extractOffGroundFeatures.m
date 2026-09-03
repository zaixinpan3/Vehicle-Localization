function offGround = extractOffGroundFeatures(voxelGrid, cfg)
% extractOffGroundFeatures: Run the non-ground feature branch of the
% perception module on the off-ground voxel grid of one frame. The fine
% voxel grid is reduced to column feature maps after removing the
% high-intensity traffic-sign channel; the maximum contiguous occupied z-run
% of each column is the pole feature and local second moments of that map
% give point-versus-line shape scores. Facades are extracted first
% (global Hough lines, local fine-grid planarity), poles are then detected
% among the remaining columns (global column analysis) and validated slice
% by slice in the local 3D fine grid, and each accepted pole component is
% collapsed to one representative column.
%
% Input:
%   voxelGrid: canonical off-ground voxel grid from voxelizePointCloud
%   cfg: struct from offGroundFeatureConfig
%
% Output:
%   offGround: struct with columnMaps, trafficSign, facade, and pole result
%       structs; all masks use the fine-column [Ny x Nx] layout
    assert(nargin >= 2 && isstruct(cfg) && ~isempty(cfg), ...
        "cfg must be a non-empty struct from offGroundFeatureConfig.");
    assert(isstruct(voxelGrid) && isfield(voxelGrid, "gridConfig") && isfield(voxelGrid, "count") && ...
        ndims(voxelGrid.count) == 3 && isequal(size(voxelGrid.count), double(voxelGrid.gridConfig.dims)), ...
        "voxelGrid must be a canonical 3D output from voxelizePointCloud.");

    [columnMaps, fineVoxelGrid] = buildFineColumnFeatureMaps(voxelGrid, cfg);
    if isempty(columnMaps.pillarCounts) || ~any(columnMaps.occupiedMask(:))
        offGround = emptyOffGroundResult(columnMaps, fineVoxelGrid, cfg);
        return;
    end

    % Column features: vertical run-layer pole evidence and local shape scores
    [columnMaps.runLayerMap, runLayerDiagnostics] = buildFineRunLayerMap(columnMaps.maxRunLayerCount, columnMaps.occupiedMask);
    columnMaps.rawLayerCount = runLayerDiagnostics.rawLayerCount;
    columnMaps.supportScore = runLayerDiagnostics.supportScore;
    [columnMaps.pointScore, columnMaps.lineScore, columnMaps.normalOrientation, columnMaps.blobness] = ...
        buildFineColumnShapeScores(columnMaps.runLayerMap, columnMaps.occupiedMask, cfg, columnMaps.dx, columnMaps.dy);

    % Traffic-sign channel (high intensity), facades, then poles on the remaining columns
    trafficSign = buildTrafficSignResult(fineVoxelGrid, columnMaps.mapSize);
    facade = extractFacadeFeatures(columnMaps, fineVoxelGrid, cfg);
    poleParams = resolvePoleDetectionParams(cfg);
    candidates = detectPoleCandidates(columnMaps, facade.mask, fineVoxelGrid, poleParams);
    if any(candidates.candidateMask(:))
        [poleMask, refineDiagnostics] = refinePolesWithFineGrid( ...
            candidates.candidateMask, facade.mask, fineVoxelGrid, columnMaps.pointScore, columnMaps.lineScore, cfg);
    else
        poleMask = false(size(candidates.candidateMask));
        refineDiagnostics = buildEmptyPoleRefineDiagnostics();
        refineDiagnostics.reason = "emptyPoleMask";
    end
    representativeMask = buildPoleRepresentativeMask( ...
        poleMask, columnMaps.pillarCounts, columnMaps.pillarZRange, columnMaps.xMap, columnMaps.yMap);

    pole = struct();
    pole.mask = poleMask;
    pole.representativeMask = representativeMask;
    pole.pillarLinIdx = find(representativeMask);
    pole.fineVoxelMask = refineDiagnostics.finalFineVoxelMask;
    pole.fineOrigin = refineDiagnostics.finalFineOrigin;
    pole.fineVoxelSize = refineDiagnostics.finalFineVoxelSize;
    pole.fineZCenters = refineDiagnostics.finalFineZCenters;
    pole.candidates = candidates;
    pole.refineDiagnostics = refineDiagnostics;
    pole.params = poleParams;

    offGround = struct();
    offGround.columnMaps = columnMaps;
    offGround.trafficSign = trafficSign;
    offGround.facade = facade;
    offGround.pole = pole;
end

function trafficSign = buildTrafficSignResult(fineVoxelGrid, mapSize)
% buildTrafficSignResult: Package the high-intensity traffic-sign channel as
% a column mask, original point indices, and the fine voxel mask with its
% geometry, so downstream stages can recover sign points directly.
%
% Input:
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
%   mapSize: [1 x 2] fine-column map size [Ny Nx]
%
% Output:
%   trafficSign: struct with mask, pillarLinIdx, pointIndices,
%       fineVoxelLinIdx, fineColumnLinIdx, fineVoxelMask, fineOrigin,
%       fineVoxelSize, fineZCenters, and intensityThreshold
    [mask, fineVoxelMask, fineOrigin, fineVoxelSize, fineZCenters] = extractTrafficSignChannel(fineVoxelGrid, mapSize);
    trafficSign = struct();
    trafficSign.mask = mask;
    trafficSign.pillarLinIdx = find(mask);
    trafficSign.pointIndices = fineVoxelGrid.trafficSignPointIndices;
    trafficSign.fineVoxelLinIdx = fineVoxelGrid.trafficSignVoxelLinIdx;
    trafficSign.fineColumnLinIdx = fineVoxelGrid.trafficSignColumnLinIdx;
    trafficSign.fineVoxelMask = fineVoxelMask;
    trafficSign.fineOrigin = fineOrigin;
    trafficSign.fineVoxelSize = fineVoxelSize;
    trafficSign.fineZCenters = fineZCenters;
    trafficSign.intensityThreshold = fineVoxelGrid.trafficSignIntensityThreshold;
end

function offGround = emptyOffGroundResult(columnMaps, fineVoxelGrid, cfg)
% emptyOffGroundResult: Build the off-ground result for a frame without any
% occupied off-ground column, preserving the public field set.
%
% Input:
%   columnMaps: struct from buildFineColumnFeatureMaps (possibly empty maps)
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
%   cfg: struct from offGroundFeatureConfig
%
% Output:
%   offGround: struct with empty columnMaps, trafficSign, facade, and pole
    mapSize = size(columnMaps.occupiedMask);
    columnMaps.runLayerMap = zeros(mapSize, "single");
    columnMaps.rawLayerCount = zeros(mapSize, "single");
    columnMaps.supportScore = zeros(mapSize, "single");
    columnMaps.pointScore = zeros(mapSize, "single");
    columnMaps.lineScore = zeros(mapSize, "single");
    columnMaps.normalOrientation = zeros(mapSize, "single");
    columnMaps.blobness = zeros(mapSize, "single");

    trafficSign = struct();
    trafficSign.mask = false(mapSize);
    trafficSign.pillarLinIdx = zeros(0, 1);
    trafficSign.pointIndices = zeros(0, 1);
    trafficSign.fineVoxelLinIdx = zeros(0, 1);
    trafficSign.fineColumnLinIdx = zeros(0, 1);
    trafficSign.fineVoxelMask = false(0, 0, 0);
    trafficSign.fineOrigin = [0, 0, 0];
    trafficSign.fineVoxelSize = [1, 1, 1];
    trafficSign.fineZCenters = zeros(0, 1);
    trafficSign.intensityThreshold = fineVoxelGrid.trafficSignIntensityThreshold;

    facade = struct();
    facade.enabled = isFacadeDetectionEnabled(cfg);
    facade.mask = false(mapSize);
    facade.lineMap = zeros(mapSize, "uint16");
    facade.pillarLinIdx = zeros(0, 1);
    facade.pillarLineIdx = zeros(0, 1);
    facade.detectedLines = zeros(0, 4);
    facade.detectedLinesRaw = zeros(0, 4);
    facade.detectorMask = false(mapSize);
    facade.detectorDiagnostics = struct();
    facade.refineDiagnostics = struct("enabled", false, "reason", "emptyOffGroundGrid");

    pole = struct();
    pole.mask = false(mapSize);
    pole.representativeMask = false(mapSize);
    pole.pillarLinIdx = zeros(0, 1);
    pole.fineVoxelMask = false(0, 0, 0);
    pole.fineOrigin = [0, 0, 0];
    pole.fineVoxelSize = [1, 1, 1];
    pole.fineZCenters = zeros(0, 1);
    pole.candidates = struct();
    pole.refineDiagnostics = buildEmptyPoleRefineDiagnostics();
    pole.refineDiagnostics.reason = "emptyOffGroundGrid";
    pole.params = resolvePoleDetectionParams(cfg);

    offGround = struct();
    offGround.columnMaps = columnMaps;
    offGround.trafficSign = trafficSign;
    offGround.facade = facade;
    offGround.pole = pole;
end

function [runLayerMap, runLayerDebug] = buildFineRunLayerMap(maxRunLayerCount, occupiedMask)
% buildFineRunLayerMap: Build the fine-column pole feature directly
% from the maximum contiguous occupied z-layer count in each XY column.
%
% Input:
%   maxRunLayerCount: [Ny x Nx] numeric maximum occupied z-run layer count
%   occupiedMask: [Ny x Nx] logical occupied fine-column mask
%
% Output:
%   runLayerMap: [Ny x Nx] single maximum contiguous occupied z-layer count
%   runLayerDebug: struct with raw run-layer count and compatibility fields
    runLayerMap = single(maxRunLayerCount);
    runLayerMap(~occupiedMask) = 0;
    runLayerMap(~isfinite(runLayerMap)) = 0;
    runLayerDebug = struct("rawLayerCount", single(runLayerMap), "supportScore", single(runLayerMap), ...
        "seedMask", false(size(occupiedMask)), "supportMask", false(size(occupiedMask)), ...
        "reconstructMask", false(size(occupiedMask)));
end

function representativeMask = buildPoleRepresentativeMask(poleMask, pillarCounts, pillarZRange, xMap, yMap)
% buildPoleRepresentativeMask: Collapse each connected pole support
% component into one representative pillar while keeping the full support
% mask intact for downstream point labeling and visualization.
%
% Input:
%   poleMask: [Ny x Nx] logical final pole support mask
%   pillarCounts: [Ny x Nx] single per-pillar point counts
%   pillarZRange: [Ny x Nx] single per-pillar z range in meters
%   xMap: [Ny x Nx] single pillar x-coordinate map in meters
%   yMap: [Ny x Nx] single pillar y-coordinate map in meters
%
% Output:
%   representativeMask: [Ny x Nx] logical one-pillar representative mask
%       with exactly one selected pillar per connected pole component
    representativeMask = false(size(poleMask));
    if ~any(poleMask(:))
        return;
    end

    ccPole = bwconncomp(logical(poleMask), 8);
    for iComp = 1:ccPole.NumObjects
        compLinIdx = ccPole.PixelIdxList{iComp};
        if isempty(compLinIdx)
            continue;
        end

        compWeights = double(pillarCounts(compLinIdx)) .* max(double(pillarZRange(compLinIdx)), eps);
        compWeights(~isfinite(compWeights)) = 0;
        if ~any(compWeights > 0)
            compWeights = ones(size(compLinIdx));
        end

        compX = double(xMap(compLinIdx));
        compY = double(yMap(compLinIdx));
        centerX = sum(compX .* compWeights) ./ sum(compWeights);
        centerY = sum(compY .* compWeights) ./ sum(compWeights);
        repScore = ((compX - centerX) .^ 2) + ((compY - centerY) .^ 2) - (1e-6 .* compWeights);
        repScore(~isfinite(repScore)) = inf;
        [~, bestLocalIdx] = min(repScore);
        representativeMask(compLinIdx(bestLocalIdx)) = true;
    end
end

function debugStruct = buildEmptyPoleRefineDiagnostics()
% buildEmptyPoleRefineDiagnostics: Create a stable empty refinement-debug
% struct with the full field set expected by downstream merge and debug
% consumers so skipped passes do not drop rich fields from later passes.
%
% Input:
%   none
%
% Output:
%   debugStruct: struct with the canonical pole-refinement debug fields
    debugStruct = struct("enabled", true, "applied", false, "usedFineGrid", false, ...
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
        "preFinalCoarseMask", false(0, 0), "preFinalFineVoxelMask", false(0, 0, 0), ...
        "finalProjectionBypassMask", false(0, 0), ...
        "finalFineVoxelMask", false(0, 0, 0), "finalFineOrigin", [NaN, NaN, NaN], ...
        "finalFineVoxelSize", [NaN, NaN, NaN], "finalFineZCenters", zeros(0, 1));
end
