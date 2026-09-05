function perception = perceiveFrame(frame, cfg)
% perceiveFrame: Shared pillar analysis for online localization and mapping.
% coarseProbabilityCloud (default) returns semantic pillar candidates and
% empirical planar Gaussian components, without point feature refinement.
% offline (alias full) recovers ground or structural members of candidate
% pillars and explicitly evaluates each point for mapping. Its featureMasks
% address the original organized frame. Ground segmentation is common
% preprocessing in both modes, not point-level semantic feature refinement.
% legacyFull reproduces historical outputs solely for baseline comparisons.
% cfg is produced by perceptionConfig; frame requires x, y, z fields.
    assert(isstruct(frame) && all(isfield(frame, ["x", "y", "z"])), ...
        "frame must be an organized point-cloud struct with x, y, and z fields.");
    assert(isstruct(cfg) && all(isfield(cfg, ["voxel", "groundSegmentation", "groundFeatures", "offGroundFeatures"])), ...
        "cfg must be a struct from perceptionConfig.");

    mode = string(cfg.executionMode);
    legacyMode = mode == "legacyFull";
    assert(any(mode == ["coarseProbabilityCloud", "offline", "full", "legacyFull"]), ...
        "Unknown perception execution mode.");
    if legacyMode
        voxelGrid = voxelizePointCloud(frame, cfg.voxel);
    else
        voxelGrid = pillarizePointCloud(frame, cfg.voxel);
    end
    groundPointIdx = segmentGround(voxelGrid, cfg.groundSegmentation);
    [groundContext, offGroundVoxelGrid] = buildBranchInputs( ...
        frame, voxelGrid, groundPointIdx, cfg.voxel.voxelSize(1:2), legacyMode);

    % Explicit compatibility path for historical regression artifacts only.
    if legacyMode
        ground = extractGroundFeatures(groundContext, frame, cfg.groundFeatures);
        offGround = extractOffGroundFeatures(offGroundVoxelGrid, cfg.offGroundFeatures);
        perception = struct("voxelGrid", voxelGrid, "groundPointIdx", groundPointIdx, ...
            "groundContext", groundContext, "offGroundVoxelGrid", offGroundVoxelGrid, ...
            "ground", ground, "offGround", offGround);
        perception.featureMasks = buildFeaturePointMasks(frame, ground, offGround, offGroundVoxelGrid, groundPointIdx);
        return;
    end

    coarseCfg = resolveCoarseProbabilityCloudConfig(cfg);
    ground = analyzeGroundPillars(groundContext, cfg.groundFeatures, coarseCfg);
    offGround = analyzeStructuralPillars(offGroundVoxelGrid, cfg.offGroundFeatures, coarseCfg);
    candidates = buildPerceptionCandidates(voxelGrid, ground, offGround);
    probabilityCloud = buildCoarseSemanticProbabilityCloud(ground, offGround, coarseCfg);
    perception = struct("executionMode", "coarseProbabilityCloud", ...
        "probabilityCloud", probabilityCloud, "candidates", candidates);
    perception.sourceSummary = struct("numFramePoints", double(numel(frame.x)), ...
        "numRetainedPoints", double(voxelGrid.numFilteredPoints), ...
        "numGroundPoints", double(size(groundContext.groundPoints, 1)), ...
        "numOffGroundPoints", double(size(offGroundVoxelGrid.points, 1)));
    if logical(coarseCfg.storeDiagnostics)
        perception.diagnostics = struct("ground", ground, "offGround", offGround);
    end
    if any(mode == ["offline", "full"])
        context = struct("voxelGrid", voxelGrid, "groundContext", groundContext, ...
            "offGroundVoxelGrid", offGroundVoxelGrid, "ground", ground, "offGround", offGround);
        fine = refinePerceptionCandidates(frame, candidates, context, cfg);
        perception.executionMode = "offline";
        perception.featureMasks = fine.featureMasks;
        perception.refinement = fine.refinement;
    end
end

function [groundContext, offGroundVoxelGrid] = buildBranchInputs(frame, voxelGrid, groundPointIdx, cellSizeXY, storeDenseOffGroundCount)
% buildBranchInputs: Split the voxelized frame at the ground segmentation
% into the inputs of the two feature branches: the ground context (ground
% points, their XY raster cells, and reflectivity) and the canonical voxel
% grid of the non-ground points, which is a compact reindexed subset of the
% frame grid because both share the same voxel size.
%
% Input:
%   frame: organized point-cloud frame with x, y, and z fields
%   voxelGrid: canonical voxel grid of the frame
%   groundPointIdx: original point indices returned by segmentGround
%   cellSizeXY: [1 x 2] XY cell size of the ground raster in meters
%   storeDenseOffGroundCount: logical scalar selecting the dense 3D count
%       tensor required by the full point-refinement branch
%
% Output:
%   groundContext: struct accepted by extractGroundFeatures
%   offGroundVoxelGrid: canonical off-ground voxel grid
    if nargin < 5
        storeDenseOffGroundCount = true;
    end
    groundXYView = buildGroundXYView(voxelGrid, cellSizeXY);
    pointIndices = double(voxelGrid.pointIndices(:));
    pointCellLinIdx = double(groundXYView.pointCellLinIdx(:));
    numFramePoints = numel(frame.x);
    groundMaskOriginal = buildMaskFromIndices(groundPointIdx, numFramePoints);
    groundPointMask = false(size(pointIndices));
    validPointIdx = isfinite(pointIndices) & pointIndices >= 1 & pointIndices <= numFramePoints & pointIndices == floor(pointIndices);
    groundPointMask(validPointIdx) = groundMaskOriginal(pointIndices(validPointIdx));
    validGroundMap = groundPointMask & isfinite(pointCellLinIdx) & pointCellLinIdx >= 1;
    offGroundPointMask = ~groundPointMask & isfinite(pointCellLinIdx) & pointCellLinIdx >= 1;

    groundContext = struct();
    groundContext.groundVoxelGrid = voxelGrid;
    groundContext.groundPointIdx = double(groundPointIdx(:));
    groundContext.groundXYView = groundXYView;
    groundContext.groundPoints = double(voxelGrid.points(validGroundMap, :));
    groundContext.groundOriginalPointIdx = double(pointIndices(validGroundMap));
    groundContext.groundIntensity = extractFrameScalar(frame, voxelGrid.pointIndices, validGroundMap, "intensity", "reflectivity");
    groundContext.groundReflectivity = extractFrameScalar(frame, voxelGrid.pointIndices, validGroundMap, "reflectivity", "intensity");
    groundContext.groundCellLinIdx = double(pointCellLinIdx(validGroundMap));

    offGroundVoxelGrid = deriveOffGroundVoxelGrid( ...
        voxelGrid, find(offGroundPointMask), logical(storeDenseOffGroundCount));
end

function xyView = buildGroundXYView(voxelGrid, cellSizeXY)
% buildGroundXYView: Build the XY pillar view required by
% extractGroundFeatures directly from a canonical voxel grid, including
% per-point XY cell assignments and per-cell point lookup metadata.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output with retained points
%   cellSizeXY: [1 x 2] XY cell size in meters
%
% Output:
%   xyView: struct with countMap, zRangeMap, geometry, point-to-cell
%       assignments, and occupied cell lookup arrays
    cellSizeXY = double(cellSizeXY(:).');
    if isscalar(cellSizeXY)
        cellSizeXY = [cellSizeXY, cellSizeXY];
    else
        cellSizeXY = cellSizeXY(1:2);
    end
    assert(all(isfinite(cellSizeXY)) && all(cellSizeXY > 0), "cellSizeXY must contain positive finite values.");
    points = double(voxelGrid.points);
    gridCfg = voxelGrid.gridConfig;
    minCorner = double(gridCfg.minCorner(1:2));
    maxCorner = double(gridCfg.maxCorner(1:2));
    dimsXY = max(ceil((maxCorner - minCorner) ./ cellSizeXY), 1);
    xEdges = minCorner(1) + (0:dimsXY(1)) .* cellSizeXY(1);
    yEdges = minCorner(2) + (0:dimsXY(2)) .* cellSizeXY(2);
    xCenters = xEdges(1:end-1) + (0.5 .* cellSizeXY(1));
    yCenters = yEdges(1:end-1) + (0.5 .* cellSizeXY(2));
    [xMap, yMap] = meshgrid(single(xCenters), single(yCenters));

    xyView = struct();
    xyView.viewType = "xyPillar";
    xyView.origin = double(minCorner(:).');
    xyView.cellSize = double(cellSizeXY(:).');
    xyView.gridSize = double(dimsXY(:).');
    xyView.xEdges = double(xEdges(:).');
    xyView.yEdges = double(yEdges(:).');
    xyView.xCenters = double(xCenters(:).');
    xyView.yCenters = double(yCenters(:).');
    xyView.xMap = xMap;
    xyView.yMap = yMap;
    xyView.countMap = zeros(dimsXY(2), dimsXY(1), "single");
    xyView.zRangeMap = zeros(dimsXY(2), dimsXY(1), "single");
    xyView.occupiedMask = false(dimsXY(2), dimsXY(1));
    xyView.pointLocalIdx = zeros(0, 1, "int32");
    xyView.pointIndices = zeros(0, 1, "int32");
    xyView.pointCellXBin = zeros(0, 1, "int32");
    xyView.pointCellYBin = zeros(0, 1, "int32");
    xyView.pointCellLinIdx = zeros(0, 1, "int32");
    xyView.occupiedCellLinIdx = zeros(0, 1, "int32");
    xyView.cellPointOffsets = int32(1);
    xyView.cellPointLocalIdx = zeros(0, 1, "int32");
    xyView.cellPointIndices = zeros(0, 1, "int32");
    xyView.sourceVoxelGrid = voxelGrid;
    if isempty(points)
        return;
    end
    xBin = floor((points(:, 1) - minCorner(1)) ./ cellSizeXY(1)) + 1;
    yBin = floor((points(:, 2) - minCorner(2)) ./ cellSizeXY(2)) + 1;
    valid = isfinite(xBin) & isfinite(yBin);
    valid = valid & xBin >= 1 & xBin <= dimsXY(1) & yBin >= 1 & yBin <= dimsXY(2);
    if ~any(valid)
        return;
    end
    pointLocalIdx = int32(find(valid));
    pointIndices = int32(voxelGrid.pointIndices(valid));
    pointCellXBin = int32(xBin(valid));
    pointCellYBin = int32(yBin(valid));
    pointCellLinIdx = int32(sub2ind(dimsXY, double(pointCellXBin), double(pointCellYBin)));
    zVals = points(valid, 3);
    numCells = prod(dimsXY);
    countsVec = accumarray(double(pointCellLinIdx), 1, [numCells, 1], @sum, 0);
    zMinVec = accumarray(double(pointCellLinIdx), zVals, [numCells, 1], @min, NaN);
    zMaxVec = accumarray(double(pointCellLinIdx), zVals, [numCells, 1], @max, NaN);
    zRangeVec = zeros(numCells, 1);
    validSpan = isfinite(zMinVec) & isfinite(zMaxVec);
    zRangeVec(validSpan) = max(0, zMaxVec(validSpan) - zMinVec(validSpan));
    [sortedCellLinIdx, sortOrder] = sort(double(pointCellLinIdx(:)));
    occupiedMask = [true; diff(sortedCellLinIdx) ~= 0];
    occupiedStartIdx = find(occupiedMask);
    xyView.countMap = reshape(single(countsVec), dimsXY(1), dimsXY(2)).';
    xyView.zRangeMap = reshape(single(zRangeVec), dimsXY(1), dimsXY(2)).';
    xyView.occupiedMask = xyView.countMap > 0;
    xyView.xMap(~xyView.occupiedMask) = NaN;
    xyView.yMap(~xyView.occupiedMask) = NaN;
    xyView.pointLocalIdx = pointLocalIdx;
    xyView.pointIndices = pointIndices;
    xyView.pointCellXBin = pointCellXBin;
    xyView.pointCellYBin = pointCellYBin;
    xyView.pointCellLinIdx = pointCellLinIdx;
    xyView.occupiedCellLinIdx = int32(sortedCellLinIdx(occupiedStartIdx));
    xyView.cellPointOffsets = int32([occupiedStartIdx; numel(sortedCellLinIdx) + 1]);
    xyView.cellPointLocalIdx = int32(pointLocalIdx(sortOrder));
    xyView.cellPointIndices = int32(pointIndices(sortOrder));
end

function scalarValues = extractFrameScalar(frame, pointIndices, pointMask, primaryField, fallbackField)
% extractFrameScalar: Extract a scalar frame channel aligned with
% selected retained voxel-grid points, preferring the requested primary
% field and falling back to a secondary field when available.
%
% Input:
%   frame: organized point-cloud frame
%   pointIndices: [K x 1] original frame linear indices
%   pointMask: [K x 1] logical selector aligned with pointIndices
%   primaryField: string scalar preferred frame field name
%   fallbackField: string scalar fallback frame field name
%
% Output:
%   scalarValues: [N x 1] double values aligned with selected points
    selectedPointIdx = double(pointIndices(logical(pointMask(:))));
    scalarValues = NaN(numel(selectedPointIdx), 1);
    sourceField = "";
    if isfield(frame, primaryField)
        sourceField = string(primaryField);
    elseif isfield(frame, fallbackField)
        sourceField = string(fallbackField);
    end
    if strlength(sourceField) == 0
        return;
    end
    sourceValues = double(frame.(sourceField)(:));
    validIdx = selectedPointIdx >= 1 & selectedPointIdx <= numel(sourceValues) & selectedPointIdx == floor(selectedPointIdx);
    assert(all(validIdx), "Selected point indices must map into frame scalar fields.");
    scalarValues = sourceValues(selectedPointIdx);
    scalarValues = double(scalarValues(:));
end

function offGroundVoxelGrid = deriveOffGroundVoxelGrid(sourceVoxelGrid, sourceLocalIdx, storeDenseCount)
% deriveOffGroundVoxelGrid: Build a compact canonical voxel grid for
% selected off-ground points by cropping and reindexing the source frame
% voxel grid.
%
% Input:
%   sourceVoxelGrid: canonical voxelizePointCloud output for the frame
%   sourceLocalIdx: [K x 1] local retained-point indices to keep
%   storeDenseCount: logical scalar; false keeps only sparse point/voxel
%       lookup arrays for the coarse probability-cloud path
%
% Output:
%   offGroundVoxelGrid: canonical voxelizePointCloud-compatible subset
    if nargin < 3
        storeDenseCount = true;
    end
    sourceLocalIdx = double(sourceLocalIdx(:));
    sourceLocalIdx = sourceLocalIdx(isfinite(sourceLocalIdx) & sourceLocalIdx >= 1 & sourceLocalIdx <= size(sourceVoxelGrid.points, 1) & sourceLocalIdx == floor(sourceLocalIdx));
    sourceLocalIdx = unique(sourceLocalIdx, "stable");
    voxelSize = double(sourceVoxelGrid.gridConfig.voxelSize(1:3));
    sourceMinCorner = double(sourceVoxelGrid.gridConfig.minCorner(1:3));
    offGroundVoxelGrid = emptyDerivedVoxelGrid(sourceVoxelGrid, voxelSize);
    if isempty(sourceLocalIdx)
        return;
    end
    sourceSub = double(sourceVoxelGrid.pointVoxelSub(sourceLocalIdx, 1:3));
    minSub = min(sourceSub, [], 1);
    maxSub = max(sourceSub, [], 1);
    dims = maxSub - minSub + 1;
    shiftedSub = sourceSub - minSub + 1;
    pointVoxelLinIdx = sub2ind(double(dims), shiftedSub(:, 1), shiftedSub(:, 2), shiftedSub(:, 3));
    if storeDenseCount
        count = single(accumarray(shiftedSub, 1, double(dims), @sum, 0));
    else
        count = zeros(0, 0, 0, "single");
    end
    [occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets, voxelPointLocalIdx] = buildDerivedVoxelPointMapping(pointVoxelLinIdx, dims);
    selectedPointIndices = int32(sourceVoxelGrid.pointIndices(sourceLocalIdx));
    voxelPointIndices = int32(selectedPointIndices(double(voxelPointLocalIdx(:))));
    minCorner = sourceMinCorner + ((minSub - 1) .* voxelSize);
    maxCorner = minCorner + (dims .* voxelSize);

    offGroundVoxelGrid.gridConfig.dims = double(dims(:).');
    offGroundVoxelGrid.gridConfig.origin = double(minCorner(:).' + (0.5 .* voxelSize(:).'));
    offGroundVoxelGrid.gridConfig.minCorner = double(minCorner(:).');
    offGroundVoxelGrid.gridConfig.maxCorner = double(maxCorner(:).');
    offGroundVoxelGrid.gridConfig.roiLimits = [double(minCorner(1)), double(maxCorner(1)), double(minCorner(2)), double(maxCorner(2)), double(minCorner(3)), double(maxCorner(3))];
    offGroundVoxelGrid.count = count;
    offGroundVoxelGrid.sumX = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumY = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumXX = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumYY = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumZZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumXY = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumXZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumYZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.points = double(sourceVoxelGrid.points(sourceLocalIdx, :));
    offGroundVoxelGrid.pointIndices = selectedPointIndices;
    offGroundVoxelGrid.pointVoxelSub = int32(shiftedSub);
    offGroundVoxelGrid.pointVoxelLinIdx = int32(pointVoxelLinIdx(:));
    offGroundVoxelGrid.occupiedVoxelLinIdx = int32(occupiedVoxelLinIdx(:));
    offGroundVoxelGrid.occupiedVoxelSub = int32(occupiedVoxelSub);
    offGroundVoxelGrid.voxelPointOffsets = int32(voxelPointOffsets(:));
    offGroundVoxelGrid.voxelPointLocalIdx = int32(voxelPointLocalIdx(:));
    offGroundVoxelGrid.voxelPointIndices = int32(voxelPointIndices(:));
    offGroundVoxelGrid.pointAttributes = filterDerivedPointAttributes(sourceVoxelGrid.pointAttributes, sourceLocalIdx);
    offGroundVoxelGrid.numFilteredPoints = double(numel(sourceLocalIdx));
    offGroundVoxelGrid.numOccupiedVoxels = double(numel(occupiedVoxelLinIdx));
end

function voxelGrid = emptyDerivedVoxelGrid(sourceVoxelGrid, voxelSize)
% emptyDerivedVoxelGrid: Create an empty voxelizePointCloud-compatible
% struct that can be filled by deriveOffGroundVoxelGrid or returned
% directly when no off-ground points survive the ground split.
%
% Input:
%   sourceVoxelGrid: source canonical voxel grid for metadata defaults
%   voxelSize: [1 x 3] voxel size in meters
%
% Output:
%   voxelGrid: empty canonical voxel-grid struct
    voxelGrid = struct();
    voxelGrid.spatialIndexType = "voxelGrid";
    voxelGrid.gridConfig = struct("dims", [0, 0, 0], "voxelSize", double(voxelSize(:).'), "origin", [0, 0, 0], "minCorner", [0, 0, 0], "maxCorner", [0, 0, 0], "roiLimits", [0, 0, 0, 0, 0, 0], "countLayout", "NxNyNz");
    voxelGrid.count = zeros(0, 0, 0, "single");
    voxelGrid.sumX = zeros(0, 0, 0, "single");
    voxelGrid.sumY = zeros(0, 0, 0, "single");
    voxelGrid.sumZ = zeros(0, 0, 0, "single");
    voxelGrid.sumXX = zeros(0, 0, 0, "single");
    voxelGrid.sumYY = zeros(0, 0, 0, "single");
    voxelGrid.sumZZ = zeros(0, 0, 0, "single");
    voxelGrid.sumXY = zeros(0, 0, 0, "single");
    voxelGrid.sumXZ = zeros(0, 0, 0, "single");
    voxelGrid.sumYZ = zeros(0, 0, 0, "single");
    voxelGrid.points = zeros(0, 3);
    voxelGrid.pointIndices = zeros(0, 1, "int32");
    voxelGrid.pointVoxelSub = zeros(0, 3, "int32");
    voxelGrid.pointVoxelLinIdx = zeros(0, 1, "int32");
    voxelGrid.occupiedVoxelLinIdx = zeros(0, 1, "int32");
    voxelGrid.occupiedVoxelSub = zeros(0, 3, "int32");
    voxelGrid.voxelPointOffsets = int32(1);
    voxelGrid.voxelPointLocalIdx = zeros(0, 1, "int32");
    voxelGrid.voxelPointIndices = zeros(0, 1, "int32");
    voxelGrid.pointAttributes = filterDerivedPointAttributes(sourceVoxelGrid.pointAttributes, zeros(0, 1));
    voxelGrid.inputType = "derived";
    voxelGrid.inputSize = [0, 1];
    voxelGrid.numInputPoints = double(sourceVoxelGrid.numFilteredPoints);
    voxelGrid.numFilteredPoints = 0;
    voxelGrid.numOccupiedVoxels = 0;
end

function pointAttributes = filterDerivedPointAttributes(sourceAttributes, sourceLocalIdx)
% filterDerivedPointAttributes: Filter aligned per-point attribute
% vectors from a source voxel grid to a derived off-ground voxel subset.
%
% Input:
%   sourceAttributes: struct of source point attributes
%   sourceLocalIdx: [K x 1] local retained-point indices to keep
%
% Output:
%   pointAttributes: struct with filtered point attributes
    pointAttributes = struct("range", zeros(0, 1));
    if ~isstruct(sourceAttributes)
        return;
    end
    fieldNames = string(fieldnames(sourceAttributes));
    for fieldIdx = 1:numel(fieldNames)
        fieldName = char(fieldNames(fieldIdx));
        values = sourceAttributes.(fieldName);
        if isvector(values) && max([sourceLocalIdx(:); 0]) <= numel(values)
            pointAttributes.(fieldName) = values(sourceLocalIdx);
        end
    end
end

function [occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets, voxelPointLocalIdx] = buildDerivedVoxelPointMapping(pointVoxelLinIdx, dims)
% buildDerivedVoxelPointMapping: Build CSR-style occupied-voxel
% lookup arrays from compact derived point voxel indices.
%
% Input:
%   pointVoxelLinIdx: [K x 1] linear voxel indices in compact grid layout
%   dims: [1 x 3] compact voxel grid dimensions
%
% Output:
%   occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets,
%       voxelPointLocalIdx: voxelizePointCloud-compatible mapping arrays
    occupiedVoxelLinIdx = zeros(0, 1, "int32");
    occupiedVoxelSub = zeros(0, 3, "int32");
    voxelPointOffsets = int32(1);
    voxelPointLocalIdx = zeros(0, 1, "int32");
    if isempty(pointVoxelLinIdx)
        return;
    end
    [sortedLinIdx, sortOrder] = sort(double(pointVoxelLinIdx(:)));
    voxelPointLocalIdx = int32(sortOrder(:));
    occupiedMask = [true; diff(sortedLinIdx) ~= 0];
    occupiedStartIdx = find(occupiedMask);
    occupiedVoxelLinIdx = int32(sortedLinIdx(occupiedStartIdx));
    voxelPointOffsets = int32([occupiedStartIdx; numel(sortedLinIdx) + 1]);
    [xSub, ySub, zSub] = ind2sub(double(dims), double(occupiedVoxelLinIdx));
    occupiedVoxelSub = int32([xSub(:), ySub(:), zSub(:)]);
end

function coarseCfg = resolveCoarseProbabilityCloudConfig(cfg)
% resolveCoarseProbabilityCloudConfig: Select the configured coarse product
% settings or their repository defaults.
    if isfield(cfg, "coarseProbabilityCloud") && isstruct(cfg.coarseProbabilityCloud)
        coarseCfg = cfg.coarseProbabilityCloud;
    else
        coarseCfg = coarseSemanticProbabilityCloudConfig();
    end
end
