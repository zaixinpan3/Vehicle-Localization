function semanticGrid = buildSemanticVoxelGrid(perception)
% buildSemanticVoxelGrid: Fuse the outputs of both feature branches into the
% coarse real-time perception product: semantic tags on the fixed 3D voxel
% grid of the frame. Ground road, curb, and boundary evidence and off-ground
% facade and pole evidence are lifted from their rasters into voxel cells,
% road-marking and traffic-sign candidates are assigned from per-cell
% maximum reflectivity and intensity, and one primary tag is chosen per
% occupied voxel while the multi-label masks are retained.
%
% Input:
%   perception: struct returned by perceiveFrame
%
% Output:
%   semanticGrid: struct with semanticNames, geometry, masks, linear index
%       sidecars, primaryTagVolume, recovery metadata, and source summary
    semanticGrid = buildSemanticVoxelGridFromParts(perception.voxelGrid, perception.ground, ...
        perception.offGround, perception.groundContext, perception.offGroundVoxelGrid);
end

function semanticGrid = buildSemanticVoxelGridFromParts(voxelGrid, groundResult, offGroundResult, groundGrid, offGroundVoxelGrid)
% buildSemanticVoxelGridFromParts: Build the primary coarse real-time
% perception product as semantic tags on a fixed 3D voxel grid. Existing
% ground and off-ground detectors may provide 2D evidence for structural
% tags, while traffic-sign and road-marking candidate tags are assigned from
% per-cell maximum intensity and reflectivity feature values on the
% canonical 3D voxel cells. The product is cell-level only and avoids
% expensive connected-component and derived map construction so the coarse
% stage remains suitable for real-time use.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output for the frame, with
%       count, gridConfig, pointIndices, pointVoxelLinIdx, and
%       pointVoxelSub fields
%   groundResult: optional struct returned by extractGroundFeatures
%   offGroundResult: optional struct returned by extractOffGroundFeatures
%   groundGrid: optional ground context struct with groundCellLinIdx,
%       groundOriginalPointIdx, and groundVoxelGrid fields
%   offGroundVoxelGrid: optional canonical off-ground voxelizePointCloud
%       output used to map off-ground feature evidence into voxelGrid
%
% Output:
%   semanticGrid: struct with 3D geometry, cellFeatures, semanticNames,
%       primaryTagVolume, masks, recovery metadata, candidateThresholds, and
%       sourceSummary diagnostics
    if nargin < 2
        groundResult = struct();
    end
    if nargin < 3
        offGroundResult = struct();
    end
    if nargin < 4
        groundGrid = struct();
    end
    if nargin < 5
        offGroundVoxelGrid = struct();
    end
    voxelGrid = resolveCanonicalVoxelGrid(voxelGrid, groundGrid, offGroundVoxelGrid);
    assert(isCanonicalVoxelGrid(voxelGrid), "voxelGrid must be a canonical voxelizePointCloud output.");

    geometry = buildVoxelGridGeometry(voxelGrid);
    dims = double(geometry.dims);
    occupiedMask = logical(voxelGrid.count > 0);
    semanticNames = ["empty", "unknown", "lowConfidence", "roadSurface", "curbCandidate", ...
        "roadBoundary", "roadMarkingCandidate", "facadeCandidate", "poleCandidate", ...
        "trafficSignCandidate"];
    masks = initializeSemanticVolumes(dims, semanticNames);

    masks.empty = ~occupiedMask;
    masks = addGroundTagEvidence(masks, voxelGrid, groundResult, groundGrid);
    masks = addOffGroundTagEvidence(masks, voxelGrid, offGroundResult, offGroundVoxelGrid);
    [masks, cellFeatures, candidateThresholds] = addCellFeatureCandidateTags( ...
        masks, voxelGrid, groundResult, offGroundResult, offGroundVoxelGrid, occupiedMask);
    masks = finalizeSemanticMasks(masks, occupiedMask);
    primaryTagVolume = buildPrimaryTagVolume(masks, semanticNames, occupiedMask);

    semanticGrid = struct();
    semanticGrid.productType = "voxelCellSemanticGridProduct";
    semanticGrid.stage = "coarse3DCellLevelValidation";
    semanticGrid.indexMapType = "voxelCell3D";
    semanticGrid.intendedUse = "realTimeLocalization";
    semanticGrid.requiresPointLevelOutput = false;
    semanticGrid.semanticNames = semanticNames;
    semanticGrid.geometry = geometry;
    semanticGrid.cellFeatures = cellFeatures;
    semanticGrid.candidateThresholds = candidateThresholds;
    semanticGrid.primaryTagVolume = primaryTagVolume;
    semanticGrid.masks = masks;
    semanticGrid.recovery = buildRecovery(voxelGrid, masks);
    semanticGrid.sourceSummary = buildSourceSummary(voxelGrid);
end

function voxelGrid = resolveCanonicalVoxelGrid(voxelGrid, groundGrid, offGroundVoxelGrid)
% resolveCanonicalVoxelGrid: Resolve the voxel grid that defines the
% 3D semantic product, preferring the explicit input and falling back to a
% ground context source grid or the off-ground voxel grid for compatibility
% callers.
%
% Input:
%   voxelGrid: explicit candidate canonical voxel grid
%   groundGrid: optional ground context struct
%   offGroundVoxelGrid: optional off-ground voxel grid
%
% Output:
%   voxelGrid: resolved voxel grid struct
    if isCanonicalVoxelGrid(voxelGrid)
        return;
    end
    if isstruct(groundGrid) && isfield(groundGrid, "groundVoxelGrid") && isCanonicalVoxelGrid(groundGrid.groundVoxelGrid)
        voxelGrid = groundGrid.groundVoxelGrid;
        return;
    end
    if isCanonicalVoxelGrid(offGroundVoxelGrid)
        voxelGrid = offGroundVoxelGrid;
    end
end

function tf = isCanonicalVoxelGrid(candidate)
% isCanonicalVoxelGrid: Determine whether a struct has the canonical
% voxel-grid fields required for 3D semantic volume construction.
%
% Input:
%   candidate: value to inspect
%
% Output:
%   tf: logical true when candidate is a usable voxel-grid struct
    tf = isstruct(candidate) && isfield(candidate, "gridConfig") && isfield(candidate.gridConfig, "dims") && ...
        numel(candidate.gridConfig.dims) >= 3 && isfield(candidate, "count") && ...
        isfield(candidate, "pointIndices") && isfield(candidate, "pointVoxelLinIdx") && ...
        isfield(candidate, "pointVoxelSub");
end

function geometry = buildVoxelGridGeometry(voxelGrid)
% buildVoxelGridGeometry: Build geometry and layout metadata for the
% canonical 3D semantic voxel grid.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output
%
% Output:
%   geometry: struct with dims, voxelSize, origin, minCorner, maxCorner,
%       center vectors, numCells, and countLayout fields
    dims = double(voxelGrid.gridConfig.dims(1:3));
    voxelSize = [1, 1, 1];
    origin = [0, 0, 0];
    minCorner = [0, 0, 0];
    maxCorner = dims;
    countLayout = "NxNyNz";
    if isfield(voxelGrid, "gridConfig")
        cfg = voxelGrid.gridConfig;
        if isfield(cfg, "dims") && numel(cfg.dims) >= 3
            dims = double(cfg.dims(1:3));
        end
        if isfield(cfg, "voxelSize") && numel(cfg.voxelSize) >= 3
            voxelSize = double(cfg.voxelSize(1:3));
        end
        if isfield(cfg, "origin") && numel(cfg.origin) >= 3
            origin = double(cfg.origin(1:3));
        end
        if isfield(cfg, "minCorner") && numel(cfg.minCorner) >= 3
            minCorner = double(cfg.minCorner(1:3));
        else
            minCorner = origin - (0.5 .* voxelSize);
        end
        if isfield(cfg, "maxCorner") && numel(cfg.maxCorner) >= 3
            maxCorner = double(cfg.maxCorner(1:3));
        else
            maxCorner = minCorner + (dims .* voxelSize);
        end
        if isfield(cfg, "countLayout")
            countLayout = string(cfg.countLayout);
        end
    end
    geometry = struct();
    geometry.dims = double(dims(:).');
    geometry.voxelSize = double(voxelSize(:).');
    geometry.origin = double(origin(:).');
    geometry.minCorner = double(minCorner(:).');
    geometry.maxCorner = double(maxCorner(:).');
    geometry.xCenters = minCorner(1) + ((1:dims(1)) - 0.5) .* voxelSize(1);
    geometry.yCenters = minCorner(2) + ((1:dims(2)) - 0.5) .* voxelSize(2);
    geometry.zCenters = minCorner(3) + ((1:dims(3)) - 0.5) .* voxelSize(3);
    geometry.numCells = prod(dims);
    geometry.countLayout = countLayout;
end

function volumes = initializeSemanticVolumes(dims, semanticNames)
% initializeSemanticVolumes: Create false logical 3D volumes for all
% configured semantic names.
%
% Input:
%   dims: [1 x 3] voxel grid dimensions
%   semanticNames: string vector of semantic field names
%
% Output:
%   volumes: struct of logical [Nx x Ny x Nz] volumes
    volumes = struct();
    for k = 1:numel(semanticNames)
        volumes.(char(semanticNames(k))) = false(dims);
    end
    volumes.linearIndices = struct();
end

function masks = addGroundTagEvidence(masks, voxelGrid, groundResult, groundGrid)
% addGroundTagEvidence: Lift ground-origin road, curb, and
% road-boundary evidence into canonical 3D voxel-cell tag masks while leaving
% road-marking candidates to direct voxel-cell reflectivity features.
%
% Input:
%   masks: current semantic mask volumes
%   voxelGrid: canonical voxelizePointCloud output
%   groundResult: struct returned by extractGroundFeatures
%   groundGrid: optional ground context struct
%
% Output:
%   masks: updated semantic mask volumes
    if ~isstruct(groundResult)
        return;
    end
    groundCellLinIdx = resolveGroundCellLinIdx(groundResult, groundGrid);
    groundOriginalPointIdx = resolveGroundOriginalPointIdx(groundResult, groundGrid, numel(groundCellLinIdx));
    if isempty(groundCellLinIdx) || isempty(groundOriginalPointIdx)
        return;
    end
    mapSize = resolveGroundMapSize(groundResult, groundGrid);
    if any(mapSize == 0)
        return;
    end

    canonicalVoxelLinIdx = mapOriginalPointIdxToVoxelLin(voxelGrid, groundOriginalPointIdx);
    roadCellMask = resolveGroundLogicalMap(groundResult, "roadCellMask", mapSize);
    curbCellMask = resolveGroundCurbCellMask(groundResult, mapSize);
    roadPointMask = sampleGroundCellMaskAtPoints(groundCellLinIdx, roadCellMask, mapSize);
    curbPointMask = sampleGroundCellMaskAtPoints(groundCellLinIdx, curbCellMask, mapSize);

    [masks.roadSurface, masks.linearIndices.roadSurface] = updateSemanticMaskVolume(masks.roadSurface, canonicalVoxelLinIdx, roadPointMask);
    [masks.curbCandidate, masks.linearIndices.curbCandidate] = updateSemanticMaskVolume(masks.curbCandidate, canonicalVoxelLinIdx, curbPointMask);
    [masks.roadBoundary, masks.linearIndices.roadBoundary] = updateSemanticMaskVolume(masks.roadBoundary, canonicalVoxelLinIdx, curbPointMask);
end

function masks = addOffGroundTagEvidence(masks, voxelGrid, offGround, offGroundVoxelGrid)
% addOffGroundTagEvidence: Lift off-ground facade and pole evidence into
% canonical 3D voxel-cell tag masks while leaving traffic-sign candidates to
% direct voxel-cell intensity features. Column masks in the fine [Ny x Nx]
% layout are sampled at the off-ground points, which are then mapped to
% their canonical voxels.
%
% Input:
%   masks: struct of semantic logical volumes and linear index sidecars
%   voxelGrid: canonical voxel grid defining the semantic volumes
%   offGround: struct returned by extractOffGroundFeatures
%   offGroundVoxelGrid: canonical off-ground voxel grid
%
% Output:
%   masks: updated semantic volumes with facadeCandidate and poleCandidate
    if ~isstruct(offGround) || ~isCanonicalVoxelGrid(offGroundVoxelGrid)
        return;
    end
    mapSize = resolveOffGroundMapSize(offGround, offGroundVoxelGrid);
    if any(mapSize == 0)
        return;
    end

    offGroundOriginalPointIdx = double(offGroundVoxelGrid.pointIndices(:));
    canonicalVoxelLinIdx = mapOriginalPointIdxToVoxelLin(voxelGrid, offGroundOriginalPointIdx);
    pointColumnLinIdx = resolveOffGroundPointColumnLinIdx(offGroundVoxelGrid, mapSize);
    facadeColumnMask = resolveColumnMask(offGround.facade.mask, mapSize);
    poleColumnMask = resolvePoleColumnMask(offGround.pole, mapSize);
    facadePointMask = sampleColumnMaskAtPoints(pointColumnLinIdx, facadeColumnMask, mapSize);
    polePointMask = sampleColumnMaskAtPoints(pointColumnLinIdx, poleColumnMask, mapSize);

    [masks.facadeCandidate, masks.linearIndices.facadeCandidate] = updateSemanticMaskVolume(masks.facadeCandidate, canonicalVoxelLinIdx, facadePointMask);
    [masks.poleCandidate, masks.linearIndices.poleCandidate] = updateSemanticMaskVolume(masks.poleCandidate, canonicalVoxelLinIdx, polePointMask);
end

function [masks, cellFeatures, candidateThresholds] = addCellFeatureCandidateTags(masks, voxelGrid, groundResult, offGroundResult, offGroundVoxelGrid, occupiedMask)
% addCellFeatureCandidateTags: Build direct voxel-cell feature volumes
% and assign road-marking and traffic-sign candidate tags from maximum
% reflectivity and intensity values in each canonical 3D voxel cell.
%
% Input:
%   masks: current semantic mask volumes with structural evidence
%   voxelGrid: canonical voxelizePointCloud output
%   groundResult: struct returned by extractGroundFeatures
%   offGroundResult: struct returned by extractOffGroundFeatures
%   offGroundVoxelGrid: optional canonical off-ground voxel grid
%   occupiedMask: [Nx x Ny x Nz] logical occupied voxel mask
%
% Output:
%   masks: updated semantic mask volumes
%   cellFeatures: struct with maxIntensity and maxReflectivity volumes
%   candidateThresholds: struct with scalar feature thresholds
    cellFeatures = buildVoxelCellFeatureValues(voxelGrid);
    candidateThresholds = struct();
    candidateThresholds.roadMarkingReflectivity = resolveRoadMarkingCandidateThreshold( ...
        groundResult, cellFeatures.maxReflectivity, masks.roadSurface, occupiedMask);
    candidateThresholds.trafficSignIntensity = resolveTrafficSignCandidateThreshold(offGroundResult);

    roadMarkingSupportMask = occupiedMask;
    if any(masks.roadSurface(:))
        roadMarkingSupportMask = masks.roadSurface;
    end
    trafficSignSupportMask = resolveTrafficSignCandidateSupportMask(voxelGrid, offGroundVoxelGrid, occupiedMask);

    [roadMarkingCandidateMask, masks.linearIndices.roadMarkingCandidate] = buildFeatureCandidateMask( ...
        cellFeatures.maxReflectivity, candidateThresholds.roadMarkingReflectivity, roadMarkingSupportMask, ...
        cellFeatures.maxReflectivityVoxelLinIdx, cellFeatures.maxReflectivityValues);
    masks.roadMarkingCandidate = masks.roadMarkingCandidate | roadMarkingCandidateMask;
    [trafficSignCandidateMask, masks.linearIndices.trafficSignCandidate] = buildFeatureCandidateMask( ...
        cellFeatures.maxIntensity, candidateThresholds.trafficSignIntensity, trafficSignSupportMask, ...
        cellFeatures.maxIntensityVoxelLinIdx, cellFeatures.maxIntensityValues);
    masks.trafficSignCandidate = masks.trafficSignCandidate | trafficSignCandidateMask;
end

function cellFeatures = buildVoxelCellFeatureValues(voxelGrid)
% buildVoxelCellFeatureValues: Accumulate dense per-cell scalar feature
% volumes directly from canonical point-to-voxel membership by taking the
% maximum point intensity and maximum point reflectivity in each 3D cell.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output
%
% Output:
%   cellFeatures: struct with maxIntensity and maxReflectivity [Nx x Ny x
%       Nz] single volumes with NaN for cells that lack the source channel,
%       plus occupied-cell linear indices and values for sparse candidate
%       thresholding
    dims = double(voxelGrid.gridConfig.dims(1:3));
    cellFeatures = struct();
    [cellFeatures.maxIntensity, cellFeatures.maxIntensityVoxelLinIdx, cellFeatures.maxIntensityValues] = ...
        buildVoxelCellMaxFeatureVolume(voxelGrid, "intensity", dims);
    [cellFeatures.maxReflectivity, cellFeatures.maxReflectivityVoxelLinIdx, cellFeatures.maxReflectivityValues] = ...
        buildVoxelCellMaxFeatureVolume(voxelGrid, "reflectivity", dims);
end

function [featureVolume, featureLinIdx, featureValues] = buildVoxelCellMaxFeatureVolume(voxelGrid, attributeName, dims)
% buildVoxelCellMaxFeatureVolume: Convert one retained point attribute
% into a dense canonical 3D voxel-cell feature volume by taking the maximum
% finite value among all points assigned to each cell. The occupied-cell
% index/value vectors mirror the dense volume so downstream candidate masks
% can be built without scanning the full dense grid.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output
%   attributeName: string scalar pointAttributes field name
%   dims: [1 x 3] canonical voxel grid dimensions
%
% Output:
%   featureVolume: [Nx x Ny x Nz] single maximum feature volume
%   featureLinIdx: [K x 1] double voxel linear indices with finite values
%   featureValues: [K x 1] single maximum feature values at featureLinIdx
    featureVolume = single(NaN(dims));
    featureLinIdx = zeros(0, 1);
    featureValues = zeros(0, 1, "single");
    if ~isfield(voxelGrid, "pointAttributes") || ~isstruct(voxelGrid.pointAttributes) || ...
            ~isfield(voxelGrid.pointAttributes, attributeName)
        return;
    end

    values = double(voxelGrid.pointAttributes.(attributeName)(:));
    voxelLinIdx = double(voxelGrid.pointVoxelLinIdx(:));
    numCells = prod(double(dims));
    if numel(values) ~= numel(voxelLinIdx) || numCells == 0
        return;
    end

    valid = isfinite(values) & isfinite(voxelLinIdx) & voxelLinIdx >= 1 & ...
        voxelLinIdx <= numCells & voxelLinIdx == floor(voxelLinIdx);
    if ~any(valid)
        return;
    end

    validVoxelLinIdx = voxelLinIdx(valid);
    validValues = values(valid);
    [sortedVoxelLinIdx, sortOrder] = sort(validVoxelLinIdx);
    sortedValues = validValues(sortOrder);
    groupStartMask = [true; diff(sortedVoxelLinIdx) ~= 0];
    groupId = cumsum(groupStartMask);
    featureLinIdx = sortedVoxelLinIdx(groupStartMask);
    featureValues = single(accumarray(groupId, sortedValues, [], @max, NaN));
    featureVolume(featureLinIdx) = featureValues;
end

function threshold = resolveRoadMarkingCandidateThreshold(groundResult, maxReflectivity, roadSurfaceMask, occupiedMask)
% resolveRoadMarkingCandidateThreshold: Resolve the reflectivity
% threshold for coarse road-marking cells from ground processing diagnostics
% when available, otherwise estimate it from cell-level reflectivity values
% and the configured road-marking highlight policy.
%
% Input:
%   groundResult: struct returned by extractGroundFeatures
%   maxReflectivity: [Nx x Ny x Nz] single cell-level reflectivity feature
%   roadSurfaceMask: [Nx x Ny x Nz] logical road-surface support
%   occupiedMask: [Nx x Ny x Nz] logical occupied voxel mask
%
% Output:
%   threshold: scalar reflectivity threshold, inf when unavailable
    threshold = readFiniteScalarField(groundResult, "roadMarkingReflectivityThreshold", inf);
    if isfinite(threshold)
        return;
    end
    if exist("groundFeatureConfig().roadMarking", "file") ~= 2
        return;
    end

    cfg = groundFeatureConfig().roadMarking();
    supportMask = occupiedMask;
    if any(roadSurfaceMask(:))
        supportMask = roadSurfaceMask;
    end
    threshold = resolveRelativeCellReflectivityThreshold(maxReflectivity, supportMask, cfg);
end

function threshold = resolveTrafficSignCandidateThreshold(offGround)
% resolveTrafficSignCandidateThreshold: Read the intensity threshold that
% separates traffic-sign points from the off-ground result, falling back to
% the configured default when the result carries none.
%
% Input:
%   offGround: struct returned by extractOffGroundFeatures
%
% Output:
%   threshold: scalar intensity threshold, inf when unavailable
    threshold = inf;
    if isstruct(offGround) && isfield(offGround, "trafficSign") && isstruct(offGround.trafficSign)
        threshold = readFiniteScalarField(offGround.trafficSign, "intensityThreshold", inf);
    end
    if isfinite(threshold)
        return;
    end
    cfg = offGroundFeatureConfig();
    threshold = readFiniteScalarField(cfg, "trafficSignIntensityThreshold", inf);
end

function supportMask = resolveTrafficSignCandidateSupportMask(voxelGrid, offGroundVoxelGrid, occupiedMask)
% resolveTrafficSignCandidateSupportMask: Resolve the coarse support
% domain for traffic-sign feature thresholding from all off-ground points
% mapped into canonical cells, falling back to all occupied cells when no
% off-ground voxel grid is available.
%
% Input:
%   voxelGrid: canonical full-frame voxelizePointCloud output
%   offGroundVoxelGrid: optional canonical off-ground voxelizePointCloud
%       output
%   occupiedMask: [Nx x Ny x Nz] logical occupied voxel mask
%
% Output:
%   supportMask: [Nx x Ny x Nz] logical traffic-sign support domain
    supportMask = occupiedMask;
    if ~isCanonicalVoxelGrid(offGroundVoxelGrid)
        return;
    end

    candidateSupport = false(size(occupiedMask));
    offGroundOriginalPointIdx = double(offGroundVoxelGrid.pointIndices(:));
    canonicalVoxelLinIdx = mapOriginalPointIdxToVoxelLin(voxelGrid, offGroundOriginalPointIdx);
    candidateSupport = updateSemanticMaskVolume(candidateSupport, canonicalVoxelLinIdx, true(numel(canonicalVoxelLinIdx), 1));
    if any(candidateSupport(:))
        supportMask = candidateSupport;
    end
end

function [candidateMask, candidateLinIdx] = buildFeatureCandidateMask(featureVolume, threshold, supportMask, featureLinIdx, featureValues)
% buildFeatureCandidateMask: Threshold a dense voxel-cell feature
% volume inside a support mask to produce a logical candidate semantic mask.
% When occupied-cell feature vectors are provided, thresholding is restricted
% to cells that actually contain finite feature evidence.
%
% Input:
%   featureVolume: [Nx x Ny x Nz] numeric voxel-cell feature volume
%   threshold: scalar feature threshold
%   supportMask: [Nx x Ny x Nz] logical support domain
%   featureLinIdx: optional [K x 1] voxel linear indices with finite values
%   featureValues: optional [K x 1] feature values at featureLinIdx
%
% Output:
%   candidateMask: [Nx x Ny x Nz] logical candidate semantic mask
%   candidateLinIdx: [K x 1] double voxel linear indices marked true
    candidateMask = false(size(supportMask));
    candidateLinIdx = zeros(0, 1);
    threshold = double(threshold);
    if isempty(featureVolume) || ~isequal(size(featureVolume), size(supportMask)) || ...
            isempty(threshold) || ~isfinite(threshold(1))
        return;
    end

    if nargin >= 5 && ~isempty(featureLinIdx) && ~isempty(featureValues)
        featureLinIdx = double(featureLinIdx(:));
        featureValues = double(featureValues(:));
        if numel(featureLinIdx) == numel(featureValues)
            valid = isfinite(featureLinIdx) & featureLinIdx >= 1 & featureLinIdx <= numel(supportMask) & ...
                featureLinIdx == floor(featureLinIdx) & isfinite(featureValues) & featureValues > threshold(1);
            if any(valid)
                validFeatureLinIdx = featureLinIdx(valid);
                supported = logical(supportMask(validFeatureLinIdx));
                candidateLinIdx = unique(validFeatureLinIdx(supported), "stable");
                candidateMask(candidateLinIdx) = true;
            end
        end
        return;
    end

    candidateMask = logical(supportMask) & isfinite(featureVolume) & double(featureVolume) > threshold(1);
    candidateLinIdx = find(candidateMask);
end

function threshold = resolveRelativeCellReflectivityThreshold(maxReflectivity, supportMask, cfg)
% resolveRelativeCellReflectivityThreshold: Estimate a reflectivity
% threshold from cell-level road-support values using the same quantile,
% robust spread, and absolute minimum configuration fields used by the
% offline road-marking point channel.
%
% Input:
%   maxReflectivity: [Nx x Ny x Nz] numeric cell-level reflectivity feature
%   supportMask: [Nx x Ny x Nz] logical support mask
%   cfg: struct from groundFeatureConfig().roadMarking
%
% Output:
%   threshold: scalar reflectivity threshold, inf when no support exists
    values = double(maxReflectivity(logical(supportMask) & isfinite(maxReflectivity)));
    if isempty(values)
        threshold = inf;
        return;
    end

    quantileLevel = readFiniteScalarField(cfg, "roadReflectivityHighlightQuantile", 0.90);
    quantileLevel = min(max(quantileLevel, 0), 1);
    madScale = readFiniteScalarField(cfg, "roadReflectivityHighlightMadScale", 2.5);
    madScale = max(madScale, 0);
    quantileThreshold = computeFeatureQuantile(values, quantileLevel);
    medianValue = median(values);
    medianAbsDeviation = median(abs(values - medianValue));
    robustSpread = double(1.4826 .* medianAbsDeviation);
    if isfinite(robustSpread) && robustSpread > 0
        threshold = max(quantileThreshold, medianValue + (madScale .* robustSpread));
    else
        threshold = quantileThreshold;
    end

    minimumThreshold = readFiniteScalarField(cfg, "roadReflectivityHighlightMinimum", 0);
    threshold = max(threshold, max(0, minimumThreshold));
    if ~isfinite(threshold)
        threshold = inf;
    end
end

function value = readFiniteScalarField(source, fieldName, defaultValue)
% readFiniteScalarField: Read one finite scalar field from a struct,
% returning a caller-provided default when the field is absent, empty, or
% non-finite.
%
% Input:
%   source: struct to inspect
%   fieldName: character vector or string scalar field name
%   defaultValue: scalar fallback value
%
% Output:
%   value: scalar finite value or defaultValue
    value = defaultValue;
    if ~isstruct(source) || ~isfield(source, fieldName) || isempty(source.(fieldName))
        return;
    end
    candidate = double(source.(fieldName));
    candidate = candidate(:);
    if isempty(candidate) || ~isfinite(candidate(1))
        return;
    end
    value = candidate(1);
end

function qValue = computeFeatureQuantile(values, q)
% computeFeatureQuantile: Estimate a scalar quantile by sorting finite
% samples and linearly interpolating between adjacent order statistics.
%
% Input:
%   values: numeric vector
%   q: scalar quantile in [0, 1]
%
% Output:
%   qValue: scalar quantile estimate, or NaN when no finite samples exist
    values = double(values(:));
    values = values(isfinite(values));
    if isempty(values)
        qValue = NaN;
        return;
    end
    q = min(max(double(q), 0), 1);
    values = sort(values);
    n = numel(values);
    if n == 1
        qValue = values(1);
        return;
    end

    pos = 1 + ((n - 1) .* q);
    lo = floor(pos);
    hi = ceil(pos);
    if lo == hi
        qValue = values(lo);
    else
        alpha = pos - lo;
        qValue = ((1 - alpha) .* values(lo)) + (alpha .* values(hi));
    end
end

function [maskVolume, markedLinIdx] = updateSemanticMaskVolume(maskVolume, voxelLinIdx, pointMask)
% updateSemanticMaskVolume: Mark semantic voxels using point support
% as a voxel-cell recovery mechanism without score accumulation.
%
% Input:
%   maskVolume: current logical semantic volume
%   voxelLinIdx: [N x 1] canonical voxel linear indices
%   pointMask: [N x 1] logical selector for supported points
%
% Output:
%   maskVolume: updated logical semantic volume
%   markedLinIdx: [K x 1] double voxel linear indices marked true
    markedLinIdx = zeros(0, 1);
    if isempty(voxelLinIdx) || isempty(pointMask)
        return;
    end
    voxelLinIdx = double(voxelLinIdx(:));
    pointMask = logical(pointMask(:));
    valid = pointMask & isfinite(voxelLinIdx) & voxelLinIdx >= 1 & ...
        voxelLinIdx <= numel(maskVolume) & voxelLinIdx == floor(voxelLinIdx);
    if any(valid)
        markedLinIdx = unique(voxelLinIdx(valid), "stable");
        maskVolume(markedLinIdx) = true;
    end
end

function masks = finalizeSemanticMasks(masks, occupiedMask)
% finalizeSemanticMasks: Fill unknown and low-confidence semantic
% volumes after all feature evidence has been lifted into 3D cells.
%
% Input:
%   masks: semantic mask volumes with feature evidence
%   occupiedMask: [Nx x Ny x Nz] logical occupied voxel mask
%
% Output:
%   masks: semantic mask volumes with empty, unknown, and lowConfidence set
    masks.empty = ~occupiedMask;
    masks.lowConfidence = occupiedMask & logical(masks.lowConfidence);
    masks.unknown = occupiedMask & ~masks.lowConfidence;
    featureUnionLinIdx = collectFeatureMaskLinearIndices(masks);
    if ~isempty(featureUnionLinIdx)
        masks.unknown(featureUnionLinIdx) = false;
    end
end

function primaryTagVolume = buildPrimaryTagVolume(masks, semanticNames, occupiedMask)
% buildPrimaryTagVolume: Assign a single primary semantic tag id per
% occupied 3D voxel while retaining multi-label masks separately and
% leaving empty cells as zero in the primary tag volume.
%
% Input:
%   masks: semantic mask volumes
%   semanticNames: string vector in priority order
%   occupiedMask: [Nx x Ny x Nz] logical occupied voxel mask
%
% Output:
%   primaryTagVolume: [Nx x Ny x Nz] uint8 primary tag ids
    primaryTagVolume = zeros(size(masks.empty), "uint8");
    unknownTagId = uint8(find(semanticNames == "unknown", 1, "first"));
    if ~isempty(unknownTagId)
        primaryTagVolume(logical(occupiedMask)) = unknownTagId;
    end

    priorityNames = ["lowConfidence", "roadSurface", "curbCandidate", "roadBoundary", ...
        "roadMarkingCandidate", "facadeCandidate", "poleCandidate", "trafficSignCandidate"];
    for k = 1:numel(priorityNames)
        fieldName = char(priorityNames(k));
        tagId = find(semanticNames == priorityNames(k), 1, "first");
        if ~isempty(tagId) && isfield(masks, fieldName)
            if isfield(masks, "linearIndices") && isstruct(masks.linearIndices) && isfield(masks.linearIndices, fieldName)
                markedLinIdx = double(masks.linearIndices.(fieldName)(:));
                valid = isfinite(markedLinIdx) & markedLinIdx >= 1 & markedLinIdx <= numel(primaryTagVolume) & markedLinIdx == floor(markedLinIdx);
                primaryTagVolume(markedLinIdx(valid)) = uint8(tagId);
            else
                primaryTagVolume(logical(masks.(fieldName))) = uint8(tagId);
            end
        end
    end
end

function featureUnionLinIdx = collectFeatureMaskLinearIndices(masks)
% collectFeatureMaskLinearIndices: Collect marked semantic feature
% voxel indices from the sparse index sidecar stored beside dense semantic
% masks so unknown-mask construction can avoid repeated full-volume scans.
%
% Input:
%   masks: semantic mask struct with optional linearIndices sidecar
%
% Output:
%   featureUnionLinIdx: [K x 1] double unique feature voxel linear indices
    featureUnionLinIdx = zeros(0, 1);
    if ~isfield(masks, "linearIndices") || ~isstruct(masks.linearIndices)
        return;
    end

    featureNames = ["roadSurface", "curbCandidate", "roadBoundary", ...
        "roadMarkingCandidate", "facadeCandidate", "poleCandidate", ...
        "trafficSignCandidate"];
    featureIndexLists = cell(numel(featureNames), 1);
    for k = 1:numel(featureNames)
        fieldName = char(featureNames(k));
        if isfield(masks.linearIndices, fieldName)
            featureIndexLists{k} = double(masks.linearIndices.(fieldName)(:));
        end
    end
    featureUnionLinIdx = vertcat(featureIndexLists{:});
    if ~isempty(featureUnionLinIdx)
        featureUnionLinIdx = unique(featureUnionLinIdx(isfinite(featureUnionLinIdx) & featureUnionLinIdx >= 1 ...
            & featureUnionLinIdx == floor(featureUnionLinIdx)), "stable");
    end
end

function recovery = buildRecovery(voxelGrid, ~)
% buildRecovery: Store point-to-voxel metadata so coarse visualization
% and deferred fine validation can recover source points from coarse
% selected 3D cells without constructing point-level semantic masks in the
% real-time coarse stage.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output
%   masks: unused semantic mask volumes retained only by the call signature
% Output:
%   recovery: struct with pointVoxelLinIdx and originalPointIdx fields
    pointVoxelLinIdx = double(voxelGrid.pointVoxelLinIdx(:));
    originalPointIdx = double(voxelGrid.pointIndices(:));
    recovery = struct();
    recovery.pointVoxelLinIdx = pointVoxelLinIdx;
    recovery.originalPointIdx = originalPointIdx;
end

function sourceSummary = buildSourceSummary(voxelGrid)
% buildSourceSummary: Summarize input point and occupied voxel counts
% for lightweight coarse-stage diagnostics without scanning every semantic
% mask volume.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output
%
% Output:
%   sourceSummary: struct with input point and occupied voxel counts
    sourceSummary = struct();
    sourceSummary.numInputPoints = double(numel(voxelGrid.pointIndices));
    sourceSummary.numOccupiedVoxels = double(numel(voxelGrid.occupiedVoxelLinIdx));
end

function voxelLinIdx = mapOriginalPointIdxToVoxelLin(voxelGrid, originalPointIdx)
% mapOriginalPointIdxToVoxelLin: Map original point indices from a
% detector-specific point set back to canonical voxel linear indices.
%
% Input:
%   voxelGrid: canonical voxelizePointCloud output
%   originalPointIdx: [N x 1] original frame point indices
%
% Output:
%   voxelLinIdx: [N x 1] double canonical voxel linear indices, zero when
%       an original point is not present in voxelGrid
    originalPointIdx = double(originalPointIdx(:));
    voxelLinIdx = zeros(numel(originalPointIdx), 1);
    if isempty(originalPointIdx)
        return;
    end

    sourcePointIdx = double(voxelGrid.pointIndices(:));
    sourceVoxelLinIdx = double(voxelGrid.pointVoxelLinIdx(:));
    validSource = isfinite(sourcePointIdx) & sourcePointIdx >= 1 & sourcePointIdx == floor(sourcePointIdx) & ...
        isfinite(sourceVoxelLinIdx) & sourceVoxelLinIdx >= 1 & sourceVoxelLinIdx == floor(sourceVoxelLinIdx);
    validQuery = isfinite(originalPointIdx) & originalPointIdx >= 1 & originalPointIdx == floor(originalPointIdx);
    if ~any(validSource) || ~any(validQuery)
        return;
    end

    maxLookupIdx = max([sourcePointIdx(validSource); originalPointIdx(validQuery)]);
    if maxLookupIdx <= 5000000
        pointToVoxelLinIdx = zeros(maxLookupIdx, 1);
        pointToVoxelLinIdx(sourcePointIdx(validSource)) = sourceVoxelLinIdx(validSource);
        voxelLinIdx(validQuery) = pointToVoxelLinIdx(originalPointIdx(validQuery));
    else
        [isMember, location] = ismember(originalPointIdx(validQuery), sourcePointIdx(validSource));
        sourceVoxelLinIdx = sourceVoxelLinIdx(validSource);
        validQueryIdx = find(validQuery);
        voxelLinIdx(validQueryIdx(isMember)) = sourceVoxelLinIdx(location(isMember));
    end
end

function groundCellLinIdx = resolveGroundCellLinIdx(groundResult, groundGrid)
% resolveGroundCellLinIdx: Resolve ground point-to-XY-evidence-cell
% indices from ground processing outputs for lifting evidence into 3D.
%
% Input:
%   groundResult: struct returned by extractGroundFeatures
%   groundGrid: optional ground context struct
%
% Output:
%   groundCellLinIdx: [N x 1] double ground evidence cell indices
    groundCellLinIdx = zeros(0, 1);
    if isstruct(groundResult) && isfield(groundResult, "groundCellLinIdx")
        groundCellLinIdx = double(groundResult.groundCellLinIdx(:));
        return;
    end
    if isstruct(groundGrid) && isfield(groundGrid, "groundCellLinIdx")
        groundCellLinIdx = double(groundGrid.groundCellLinIdx(:));
    end
end

function groundOriginalPointIdx = resolveGroundOriginalPointIdx(groundResult, groundGrid, numPoints)
% resolveGroundOriginalPointIdx: Resolve original point indices
% aligned with ground evidence point rows.
%
% Input:
%   groundResult: struct returned by extractGroundFeatures
%   groundGrid: optional ground context struct
%   numPoints: scalar expected point count
%
% Output:
%   groundOriginalPointIdx: [N x 1] double original point indices
    groundOriginalPointIdx = zeros(0, 1);
    if isstruct(groundResult) && isfield(groundResult, "groundOriginalPointIdx") && numel(groundResult.groundOriginalPointIdx) == numPoints
        groundOriginalPointIdx = double(groundResult.groundOriginalPointIdx(:));
        return;
    end
    if isstruct(groundGrid) && isfield(groundGrid, "groundOriginalPointIdx") && numel(groundGrid.groundOriginalPointIdx) == numPoints
        groundOriginalPointIdx = double(groundGrid.groundOriginalPointIdx(:));
    end
end

function mapSize = resolveGroundMapSize(groundResult, groundGrid)
% resolveGroundMapSize: Resolve the source ground evidence map size
% used only to lift ground-origin evidence into the 3D voxel grid.
%
% Input:
%   groundResult: struct returned by extractGroundFeatures
%   groundGrid: optional ground context struct
%
% Output:
%   mapSize: [1 x 2] double source map size [Ny Nx]
    mapSize = [0, 0];
    mapFields = ["roadCellMask", "curbCellMask", "roadBoundaryCellMask"];
    for k = 1:numel(mapFields)
        fieldName = char(mapFields(k));
        if isstruct(groundResult) && isfield(groundResult, fieldName) && ~isempty(groundResult.(fieldName))
            mapSize = size(groundResult.(fieldName));
            return;
        end
    end
    if isstruct(groundResult) && isfield(groundResult, "energyMaps") && isstruct(groundResult.energyMaps) && isfield(groundResult.energyMaps, "total") && ~isempty(groundResult.energyMaps.total)
        mapSize = size(groundResult.energyMaps.total);
        return;
    end
    if isstruct(groundGrid) && isfield(groundGrid, "groundXYView") && isstruct(groundGrid.groundXYView) && isfield(groundGrid.groundXYView, "countMap")
        mapSize = size(groundGrid.groundXYView.countMap);
    end
end

function map = resolveGroundLogicalMap(groundResult, fieldName, mapSize)
% resolveGroundLogicalMap: Read a logical ground evidence map when it
% matches the source evidence map size.
%
% Input:
%   groundResult: struct returned by extractGroundFeatures
%   fieldName: character vector field name
%   mapSize: [1 x 2] source map size
%
% Output:
%   map: [Ny x Nx] logical map
    map = false(mapSize);
    if isstruct(groundResult) && isfield(groundResult, fieldName) && isequal(size(groundResult.(fieldName)), mapSize)
        map = logical(groundResult.(fieldName));
    end
end

function map = resolveGroundCurbCellMask(groundResult, mapSize)
% resolveGroundCurbCellMask: Resolve source curb candidate evidence
% as a ground map before lifting it into 3D voxels.
%
% Input:
%   groundResult: struct returned by extractGroundFeatures
%   mapSize: [1 x 2] source map size
%
% Output:
%   map: [Ny x Nx] logical curb candidate map
    map = resolveGroundLogicalMap(groundResult, "curbCellMask", mapSize);
    if ~any(map(:)) && isfield(groundResult, "energyMaps") && isstruct(groundResult.energyMaps) && isfield(groundResult.energyMaps, "extractedMask") && isequal(size(groundResult.energyMaps.extractedMask), mapSize)
        map = logical(groundResult.energyMaps.extractedMask);
    end
end

function pointMask = sampleGroundCellMaskAtPoints(cellLinIdx, cellMask, mapSize)
% sampleGroundCellMaskAtPoints: Sample a [Ny x Nx] source evidence
% map at point-cell indices stored in [Nx x Ny] linear layout.
%
% Input:
%   cellLinIdx: [N x 1] numeric source cell indices
%   cellMask: [Ny x Nx] logical source evidence map
%   mapSize: [1 x 2] source map size
%
% Output:
%   pointMask: [N x 1] logical point-row support mask
    pointMask = false(numel(cellLinIdx), 1);
    if isempty(cellLinIdx) || isempty(cellMask) || any(mapSize == 0)
        return;
    end
    values = logical(cellMask).';
    values = values(:);
    cellLinIdx = double(cellLinIdx(:));
    valid = isfinite(cellLinIdx) & cellLinIdx >= 1 & cellLinIdx <= numel(values) & cellLinIdx == floor(cellLinIdx);
    pointMask(valid) = values(double(cellLinIdx(valid)));
end

function mapSize = resolveOffGroundMapSize(offGround, offGroundVoxelGrid)
% resolveOffGroundMapSize: Resolve the fine-column [Ny Nx] map size from the
% off-ground column maps, falling back to the off-ground voxel grid dims.
%
% Input:
%   offGround: struct returned by extractOffGroundFeatures
%   offGroundVoxelGrid: canonical off-ground voxel grid
%
% Output:
%   mapSize: [1 x 2] double column map size [Ny Nx]
    mapSize = [0, 0];
    if isstruct(offGround) && isfield(offGround, "columnMaps") && isfield(offGround.columnMaps, "occupiedMask") && ...
            ~isempty(offGround.columnMaps.occupiedMask)
        mapSize = double(size(offGround.columnMaps.occupiedMask));
        return;
    end
    if isCanonicalVoxelGrid(offGroundVoxelGrid)
        dims = double(offGroundVoxelGrid.gridConfig.dims(:).');
        mapSize = [dims(2), dims(1)];
    end
end

function pointColumnLinIdx = resolveOffGroundPointColumnLinIdx(offGroundVoxelGrid, mapSize)
% resolveOffGroundPointColumnLinIdx: Convert off-ground voxel
% subscripts [x y z] into source column linear indices in [Ny x Nx] map
% layout.
%
% Input:
%   offGroundVoxelGrid: canonical off-ground voxel grid
%   mapSize: [1 x 2] source map size [Ny Nx]
%
% Output:
%   pointColumnLinIdx: [N x 1] double source column linear indices
    pointSub = double(offGroundVoxelGrid.pointVoxelSub);
    xSub = pointSub(:, 1);
    ySub = pointSub(:, 2);
    numRows = double(mapSize(1));
    numCols = double(mapSize(2));
    pointColumnLinIdx = zeros(size(xSub));
    valid = isfinite(xSub) & isfinite(ySub) & xSub >= 1 & xSub <= numCols & ySub >= 1 & ySub <= numRows;
    pointColumnLinIdx(valid) = ySub(valid) + ((xSub(valid) - 1) .* numRows);
end

function columnMask = resolveColumnMask(mask, mapSize)
% resolveColumnMask: Accept a fine-column logical mask when it matches the
% expected map size and otherwise return an all-false mask.
%
% Input:
%   mask: [Ny x Nx] logical column mask candidate
%   mapSize: [1 x 2] expected column map size [Ny Nx]
%
% Output:
%   columnMask: [Ny x Nx] logical column mask
    columnMask = false(mapSize);
    if ~isempty(mask) && isequal(size(mask), mapSize)
        columnMask = logical(mask);
    end
end

function columnMask = resolvePoleColumnMask(pole, mapSize)
% resolvePoleColumnMask: Resolve pole support columns, preferring the full
% validated pole mask over representative-only columns for recall.
%
% Input:
%   pole: pole result struct from extractOffGroundFeatures
%   mapSize: [1 x 2] column map size [Ny Nx]
%
% Output:
%   columnMask: [Ny x Nx] logical pole column mask
    columnMask = resolveColumnMask(pole.mask, mapSize);
    if ~any(columnMask(:))
        columnMask = resolveColumnMask(pole.representativeMask, mapSize);
    end
end

function pointMask = sampleColumnMaskAtPoints(pointColumnLinIdx, columnMask, mapSize)
% sampleColumnMaskAtPoints: Sample a [Ny x Nx] source column mask at
% off-ground point column indices.
%
% Input:
%   pointColumnLinIdx: [N x 1] numeric source column indices
%   columnMask: [Ny x Nx] logical source column mask
%   mapSize: [1 x 2] source map size
%
% Output:
%   pointMask: [N x 1] logical point-row support mask
    pointMask = false(numel(pointColumnLinIdx), 1);
    if isempty(pointColumnLinIdx) || isempty(columnMask) || any(mapSize == 0)
        return;
    end
    values = logical(columnMask(:));
    pointColumnLinIdx = double(pointColumnLinIdx(:));
    valid = isfinite(pointColumnLinIdx) & pointColumnLinIdx >= 1 & pointColumnLinIdx <= numel(values) & pointColumnLinIdx == floor(pointColumnLinIdx);
    pointMask(valid) = values(double(pointColumnLinIdx(valid)));
end
