function perception = perceiveFrame(frame, cfg)
% perceiveFrame: Shared pillar analysis for online localization and mapping.
% coarseProbabilityCloud (default) returns semantic pillar candidates and
% empirical planar Gaussian components, without point feature refinement.
% offline (alias full) independently reconstructs detailed structural
% candidates and explicitly evaluates their points for mapping. Its featureMasks
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
    featureNames = validatePerceptionFeatureNames(cfg.featureNames);
    if ~legacyMode
        assert(~isfield(cfg.offGroundFeatures,"facadeDetectionEnabled") && ...
            (~isfield(cfg,"coarseProbabilityCloud") || ~isfield(cfg.coarseProbabilityCloud,"semanticNames")), ...
            "perception:ObsoleteFeatureSelection", ...
            "Select all invocation channels with cfg.featureNames; remove the old nested selectors.");
    end
    backend = "auto";
    if isfield(cfg, "executionBackend"), backend = cfg.executionBackend; end
    useNative = ~legacyMode && perceptionNativeAvailable(backend);
    if ~legacyMode
        cfg.groundSegmentation.slopeGridXYCellSize=cfg.voxel.voxelSize(1:2);
    elseif numel(cfg.voxel.voxelSize)==2
        legacyGeometry=frameVoxelizationConfig();
        cfg.voxel.voxelSize=[cfg.voxel.voxelSize,legacyGeometry.voxelSize(3)];
        cfg.voxel.statisticsMode=legacyGeometry.statisticsMode;
    end
    cfg.voxel.useNativeKernels = useNative;
    cfg.groundSegmentation.useNativeKernels = useNative;
    cfg.groundFeatures.road.useNativeKernels = useNative;
    cfg.groundFeatures.curb.useNativeKernels = useNative;
    cfg.groundFeatures.curb.compactRaster = ~legacyMode && ...
        (~isfield(cfg,"compactGroundRaster") || cfg.compactGroundRaster);
    cfg.offGroundFeatures.useNativeKernels = useNative;
    assert(any(mode == ["coarseProbabilityCloud", "offline", "full", "legacyFull"]), ...
        "Unknown perception execution mode.");
    if legacyMode
        legacyOffGround = offGroundFeatureConfig();
        supplied = fieldnames(cfg.offGroundFeatures);
        for k=1:numel(supplied)
            name=supplied{k};
            if isfield(legacyOffGround,name) && ~strcmp(name,'facadeRefineEnabled')
                legacyOffGround.(name)=cfg.offGroundFeatures.(name);
            end
        end
        if ~isfield(cfg.offGroundFeatures,'facadeDetectionEnabled')
            legacyOffGround.facadeDetectionEnabled=any(featureNames=="facade");
        end
        cfg.offGroundFeatures=legacyOffGround;
        if ~isfield(cfg.offGroundFeatures,"facadeDetectionEnabled")
            cfg.offGroundFeatures.facadeDetectionEnabled = any(featureNames=="facade");
        end
        voxelGrid = voxelizePointCloud(frame, cfg.voxel);
    else
        voxelGrid = pillarizePointCloud(frame, cfg.voxel);
    end
    groundPointIdx = segmentGround(voxelGrid, cfg.groundSegmentation);
    [groundContext, offGroundVoxelGrid] = buildBranchInputs( ...
        frame, voxelGrid, groundPointIdx, cfg.voxel.voxelSize(1:2), legacyMode, cfg.groundFeatures.curb);

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
    ground = struct("roadMarkingReflectivityThreshold",NaN);
    if any(ismember(featureNames,["curb","roadMarking"]))
        ground = analyzeGroundPillars(groundContext, cfg.groundFeatures, coarseCfg);
    end
    offGround = struct();
    if any(ismember(featureNames,["pole","facade","trafficSign"]))
        offGround = analyzeStructuralPillars(offGroundVoxelGrid, cfg.offGroundFeatures, coarseCfg);
    end
    candidates = buildPerceptionCandidates(voxelGrid, ground, offGround, coarseCfg.semanticNames);
    probabilityCloud = buildCoarseSemanticProbabilityCloud(ground, offGround, coarseCfg);
    perception = struct("executionMode", "coarseProbabilityCloud", ...
        "probabilityCloud", probabilityCloud, "candidates", candidates, ...
        "featureNames",featureNames);
    perception.sourceSummary = struct("numFramePoints", double(numel(frame.x)), ...
        "numRetainedPoints", double(voxelGrid.numFilteredPoints), ...
        "numGroundPoints", double(size(groundContext.groundPoints, 1)), ...
        "numOffGroundPoints", double(size(offGroundVoxelGrid.points, 1)));
    if logical(coarseCfg.storeDiagnostics)
        perception.diagnostics = struct("ground", ground, "offGround", offGround, "pillars", voxelGrid.statistics);
    end
    if any(mode == ["offline", "full"])
        context = struct("voxelGrid", voxelGrid, "groundContext", groundContext, ...
            "offGroundVoxelGrid", offGroundVoxelGrid, "ground", ground, "offGround", offGround);
        fine = refinePerceptionCandidates(frame, candidates, context, cfg);
        perception.executionMode = "offline";
        perception.featureMasks = fine.featureMasks;
        perception.refinement = fine.refinement;
        perception.fineCandidates = fine.candidates;
    end
end

function [groundContext, offGroundVoxelGrid] = buildBranchInputs(frame, voxelGrid, groundPointIdx, cellSizeXY, storeDenseOffGroundCount, curbCfg)
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
    pointIndices = double(voxelGrid.pointIndices(:));
    numFramePoints = numel(frame.x);
    groundMaskOriginal = buildMaskFromIndices(groundPointIdx, numFramePoints);
    groundPointMask = false(size(pointIndices));
    validPointIdx = isfinite(pointIndices) & pointIndices >= 1 & pointIndices <= numFramePoints & pointIndices == floor(pointIndices);
    groundPointMask(validPointIdx) = groundMaskOriginal(pointIndices(validPointIdx));
    groundXYView = buildGroundXYView(voxelGrid, cellSizeXY, groundPointMask, curbCfg);
    pointCellLinIdx = double(groundXYView.pointCellLinIdx(:));
    validGroundMap = groundPointMask & isfinite(pointCellLinIdx) & pointCellLinIdx >= 1;
    offGroundPointMask = ~groundPointMask;

    groundContext = struct();
    groundContext.groundVoxelGrid = voxelGrid;
    groundContext.groundPointIdx = double(groundPointIdx(:));
    groundContext.groundXYView = groundXYView;
    groundContext.groundPoints = double(voxelGrid.points(validGroundMap, :));
    groundContext.groundOriginalPointIdx = double(pointIndices(validGroundMap));
    groundContext.groundIntensity = extractFrameScalar(frame, voxelGrid.pointIndices, validGroundMap, "intensity", "reflectivity");
    groundContext.groundReflectivity = extractFrameScalar(frame, voxelGrid.pointIndices, validGroundMap, "reflectivity", "intensity");
    groundContext.groundCellLinIdx = double(pointCellLinIdx(validGroundMap));

    if storeDenseOffGroundCount
        offGroundVoxelGrid = deriveLegacyOffGroundGrid(voxelGrid, find(offGroundPointMask), true);
    else
        offGroundVoxelGrid = subsetOffGroundPillars(voxelGrid, offGroundPointMask);
    end
end

function xyView = buildGroundXYView(voxelGrid, cellSizeXY, groundPointMask, curbCfg)
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
    % Keep the original lattice phase when trimming empty ground margins.
    % All ground points and a halo survive; off-ground points outside this
    % raster remain in the separate structural branch.
    xBin = floor((points(:,1)-minCorner(1))./cellSizeXY(1))+1;
    yBin = floor((points(:,2)-minCorner(2))./cellSizeXY(2))+1;
    first = [1 1]; last = dimsXY;
    if isfield(curbCfg,"compactRaster") && curbCfg.compactRaster && any(groundPointMask)
        padding = ceil(max([curbCfg.detrendBoxRadiusCells+1, curbCfg.relativeHeightRadiusCells, ...
            curbCfg.linearityRadiusMeters./cellSizeXY, curbCfg.directionalLineRadiusCells+1, ...
            curbCfg.componentFillSupportRadiusCells, curbCfg.extractionStandaloneBridgeRadiusCells]))+2;
        first = max([min(xBin(groundPointMask)),min(yBin(groundPointMask))]-padding,1);
        last = min([max(xBin(groundPointMask)),max(yBin(groundPointMask))]+padding,dimsXY);
        if prod(last-first+1)>0.85*prod(dimsXY), first=[1 1]; last=dimsXY; end
    end
    xCenters = xCenters(first(1):last(1)); yCenters = yCenters(first(2):last(2));
    xEdges = xEdges(first(1):last(1)+1); yEdges = yEdges(first(2):last(2)+1);
    minCorner = minCorner+(first-1).*cellSizeXY;
    dimsXY = last-first+1;
    xBin = xBin-first(1)+1; yBin = yBin-first(2)+1;
    [xMap, yMap] = meshgrid(single(xCenters), single(yCenters));

    xyView = struct();
    xyView.viewType = "xyPillar";
    xyView.pillarOffset = first-1;
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
    xyView.countMap = reshape(single(countsVec), dimsXY(1), dimsXY(2)).';
    xyView.zRangeMap = reshape(single(zRangeVec), dimsXY(1), dimsXY(2)).';
    xyView.occupiedMask = xyView.countMap > 0;
    xyView.xMap(~xyView.occupiedMask) = NaN;
    xyView.yMap(~xyView.occupiedMask) = NaN;
    xyView.pointLocalIdx = pointLocalIdx;
    xyView.pointIndices = pointIndices;
    xyView.pointCellXBin = pointCellXBin;
    xyView.pointCellYBin = pointCellYBin;
    xyView.pointCellLinIdx = zeros(size(points,1),1,"int32");
    xyView.pointCellLinIdx(valid) = pointCellLinIdx;
    xyView.occupiedCellLinIdx = int32(find(countsVec>0));
    if ~isfield(curbCfg,"compactRaster") || ~curbCfg.compactRaster
        [sortedCellLinIdx, sortOrder] = sort(double(pointCellLinIdx(:)));
        occupiedStartIdx = find([true; diff(sortedCellLinIdx) ~= 0]);
        xyView.cellPointOffsets = int32([occupiedStartIdx; numel(sortedCellLinIdx) + 1]);
        xyView.cellPointLocalIdx = int32(pointLocalIdx(sortOrder));
        xyView.cellPointIndices = int32(pointIndices(sortOrder));
    end
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

function pillars = subsetOffGroundPillars(source, selected)
% subsetOffGroundPillars: Retain branch members on the original XY lattice.
% Crop empty XY margins only; never create a vertical index or a finer cell.
    pillars = source;
    pillars.points=source.points(selected,:);
    pillars.pointIndices=source.pointIndices(selected);
    pillars.pointAttributes=filterPerceptionAttributes(source.pointAttributes,find(selected));
    bins=double(source.pointPillarSub(selected,:));
    spacing=source.pillarGeometry.cellSize;
    first=[1 1]; last=[1 1];
    if ~isempty(bins), first=min(bins,[],1); last=max(bins,[],1); end
    bins=bins-first+1;
    dims=last-first+1;
    lower=source.pillarGeometry.origin+(first-1).*spacing;
    pillars.pillarGeometry=struct('origin',lower,'cellSize',spacing,'mapSize',dims([2 1]),'layout',"NyNx");
    pillars.gridConfig=struct('dims',dims,'voxelSize',spacing,'minCorner',lower, ...
        'maxCorner',lower+dims.*spacing,'origin',lower+spacing/2);
    pillars.pointPillarSub=int32(bins);
    pillars.pointPillarLinIdx=int32(sub2ind(dims([2 1]),bins(:,2),bins(:,1)));
    pillars.numFilteredPoints=size(bins,1);
    pillars=rmfield(pillars,'statistics');
end

function coarseCfg = resolveCoarseProbabilityCloudConfig(cfg)
% resolveCoarseProbabilityCloudConfig: Select the configured coarse product
% settings or their repository defaults.
    if isfield(cfg, "coarseProbabilityCloud") && isstruct(cfg.coarseProbabilityCloud)
        coarseCfg = cfg.coarseProbabilityCloud;
    else
        coarseCfg = coarseSemanticProbabilityCloudConfig();
    end
    coarseCfg.semanticNames = validatePerceptionFeatureNames(cfg.featureNames);
    calibration=lidarFrameCalibrationConfig();
    if isfield(cfg,'frameCalibration'), calibration=validateLidarFrameCalibration(cfg.frameCalibration); end
    if ~isfield(coarseCfg,'projectionTranslation'), coarseCfg.projectionTranslation=[0 0 0]; end
    coarseCfg.projectionTranslation=coarseCfg.projectionTranslation+calibration.translation*coarseCfg.projectionRotation.';
    coarseCfg.projectionRotation=coarseCfg.projectionRotation*calibration.rotation;
    coarseCfg.frameCalibration=calibration;
end
