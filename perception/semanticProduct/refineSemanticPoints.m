function pointProduct = refineSemanticPoints(semanticGrid, perception)
% refineSemanticPoints: Build the offline point-level semantic product from
% the coarse voxel product. Original points are recovered only from the
% coarse-selected 3D cells and then gated by the precise point-level and
% fine-voxel validation outputs of the two feature branches, yielding
% per-class original point index lists and the map primitives used by
% offline map construction.
%
% Input:
%   semanticGrid: struct returned by buildSemanticVoxelGrid
%   perception: struct returned by perceiveFrame
%
% Output:
%   pointProduct: struct with semanticPointIdx, localPointMasks,
%       candidateRecovery, mapPrimitives, and productBoundary
    pointProduct = refineSemanticPointsFromParts(semanticGrid, perception.ground, perception.offGround, ...
        perception.groundContext, perception.offGroundVoxelGrid);
end

function pointProduct = refineSemanticPointsFromParts(coarseProduct, groundResult, offGroundResult, groundGrid, offGroundVoxelGrid)
% refineSemanticPointsFromParts: Build the offline point-level refined semantic
% product by recovering original points only from coarse-selected 3D voxel
% cells, then applying existing precise point-level or fine-voxel masks
% inside those candidate cells. This product is for offline map
% construction and is not required by the real-time 3D semantic grid path.
%
% Input:
%   coarseProduct: struct returned by buildSemanticVoxelGrid with semanticGrid
%   groundResult: struct returned by extractGroundFeatures
%   offGroundResult: struct returned by extractOffGroundFeatures
%   groundGrid: optional ground context struct used for compatibility
%   offGroundVoxelGrid: optional canonical off-ground voxelizePointCloud
%       output used to resolve fine off-ground point masks
%
% Output:
%   pointProduct: struct with point-level semantic labels, local point
%       masks, candidate recovery metadata, and offline map primitives
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
    semanticGrid = resolveSemanticGrid(coarseProduct);
    assert(isstruct(semanticGrid) && isfield(semanticGrid, "recovery") && isfield(semanticGrid.recovery, "originalPointIdx"), ...
        "coarseProduct must contain semanticGrid from buildSemanticVoxelGrid.");

    canonicalOriginalPointIdx = double(semanticGrid.recovery.originalPointIdx(:));
    candidateMasks = buildCandidatePointMasks(semanticGrid, numel(canonicalOriginalPointIdx));
    preciseMasks = buildPrecisePointMasks(canonicalOriginalPointIdx, groundResult, offGroundResult, groundGrid, offGroundVoxelGrid);
    localPointMasks = buildFinalPointMasks(candidateMasks, preciseMasks, numel(canonicalOriginalPointIdx));
    semanticPointIdx = buildSemanticPointIdx(canonicalOriginalPointIdx, localPointMasks);

    pointProduct = struct();
    pointProduct.productType = "pointLevelRefinedSemanticProduct";
    pointProduct.stage = "finePointLevelValidation";
    pointProduct.intendedUse = "offlineMapConstruction";
    pointProduct.requiresCoarseGridProduct = true;
    pointProduct.semanticPointIdx = semanticPointIdx;
    pointProduct.localPointMasks = localPointMasks;
    pointProduct.candidateRecovery = semanticGrid.recovery;
    pointProduct.mapPrimitives = buildMapPrimitives(semanticGrid, semanticPointIdx);
    pointProduct.productBoundary = buildProductBoundary();
end

function candidateMasks = buildCandidatePointMasks(semanticGrid, numPoints)
% buildCandidatePointMasks: Recover coarse candidate point masks from
% semantic grid cell masks only when fine point validation is executed, so
% the real-time coarse stage does not precompute point-level semantic
% masks.
%
% Input:
%   semanticGrid: coarse 3D semantic grid with masks and recovery metadata
%   numPoints: scalar number of recovered canonical point rows
%
% Output:
%   candidateMasks: struct of [N x 1] logical coarse candidate masks by
%       semantic field name
    candidateMasks = struct();
    pointVoxelLinIdx = zeros(numPoints, 1);
    if isstruct(semanticGrid) && isfield(semanticGrid, "recovery") && ...
            isstruct(semanticGrid.recovery) && isfield(semanticGrid.recovery, "pointVoxelLinIdx") && ...
            numel(semanticGrid.recovery.pointVoxelLinIdx) == numPoints
        pointVoxelLinIdx = double(semanticGrid.recovery.pointVoxelLinIdx(:));
    end

    semanticNames = ["roadSurface", "curbCandidate", "roadBoundary", ...
        "roadMarkingCandidate", "facadeCandidate", "poleCandidate", "trafficSignCandidate"];
    for k = 1:numel(semanticNames)
        fieldName = char(semanticNames(k));
        candidateMasks.(fieldName) = false(numPoints, 1);
        if ~isstruct(semanticGrid) || ~isfield(semanticGrid, "masks") || ...
                ~isstruct(semanticGrid.masks) || ~isfield(semanticGrid.masks, fieldName)
            continue;
        end
        candidateMasks.(fieldName) = sampleSemanticVolumeAtPoints(pointVoxelLinIdx, semanticGrid.masks.(fieldName), numPoints);
    end
end

function pointMask = sampleSemanticVolumeAtPoints(pointVoxelLinIdx, volume, numPoints)
% sampleSemanticVolumeAtPoints: Sample a coarse semantic voxel mask at
% recovered point voxel indices.
%
% Input:
%   pointVoxelLinIdx: [N x 1] numeric voxel linear indices
%   volume: logical semantic voxel mask
%   numPoints: scalar expected number of point rows
%
% Output:
%   pointMask: [N x 1] logical point mask
    pointMask = false(numPoints, 1);
    if isempty(pointVoxelLinIdx) || isempty(volume)
        return;
    end
    pointVoxelLinIdx = double(pointVoxelLinIdx(:));
    if numel(pointVoxelLinIdx) ~= numPoints
        return;
    end
    valid = isfinite(pointVoxelLinIdx) & pointVoxelLinIdx >= 1 & ...
        pointVoxelLinIdx <= numel(volume) & pointVoxelLinIdx == floor(pointVoxelLinIdx);
    if ~any(valid)
        return;
    end
    values = logical(volume(:));
    pointMask(valid) = values(pointVoxelLinIdx(valid));
end

function semanticGrid = resolveSemanticGrid(coarseProduct)
% resolveSemanticGrid: Resolve the primary 3D semantic grid from a
% coarse product or accept the semantic grid directly.
%
% Input:
%   coarseProduct: coarse validation product or semantic grid product
%
% Output:
%   semanticGrid: 3D voxel-cell semantic grid product
    semanticGrid = struct();
    if isstruct(coarseProduct) && isfield(coarseProduct, "semanticGrid")
        semanticGrid = coarseProduct.semanticGrid;
    elseif isstruct(coarseProduct) && isfield(coarseProduct, "primaryTagVolume") && isfield(coarseProduct, "recovery")
        semanticGrid = coarseProduct;
    end
end

function preciseMasks = buildPrecisePointMasks(canonicalOriginalPointIdx, groundFeatures, offGround, groundContext, offGroundVoxelGrid)
% buildPrecisePointMasks: Convert the precise point-level and fine-voxel
% outputs of both feature branches into canonical point-row masks.
%
% Input:
%   canonicalOriginalPointIdx: [K x 1] original indices of canonical points
%   groundFeatures: struct returned by extractGroundFeatures
%   offGround: struct returned by extractOffGroundFeatures
%   groundContext: ground context struct from perceiveFrame
%   offGroundVoxelGrid: canonical off-ground voxel grid
%
% Output:
%   preciseMasks: struct of [K x 1] logical masks per semantic name
    numPoints = numel(canonicalOriginalPointIdx);
    preciseMasks = struct();
    preciseMasks.roadSurface = false(numPoints, 1);
    preciseMasks.curbCandidate = false(numPoints, 1);
    preciseMasks.roadBoundary = false(numPoints, 1);
    preciseMasks.roadMarkingCandidate = false(numPoints, 1);
    preciseMasks.facadeCandidate = false(numPoints, 1);
    preciseMasks.poleCandidate = false(numPoints, 1);
    preciseMasks.trafficSignCandidate = false(numPoints, 1);

    groundOriginalPointIdx = resolveGroundOriginalPointIdx(groundFeatures, groundContext);
    preciseMasks.roadSurface = mapSourcePointMaskToCanonical(canonicalOriginalPointIdx, groundOriginalPointIdx, resolveSourceMask(groundFeatures, "roadPointMask", numel(groundOriginalPointIdx)));
    preciseMasks.curbCandidate = mapSourcePointMaskToCanonical(canonicalOriginalPointIdx, groundOriginalPointIdx, resolveSourceMask(groundFeatures, "curbPointMask", numel(groundOriginalPointIdx)));
    preciseMasks.roadBoundary = preciseMasks.curbCandidate;
    preciseMasks.roadMarkingCandidate = mapSourcePointMaskToCanonical(canonicalOriginalPointIdx, groundOriginalPointIdx, resolveSourceMask(groundFeatures, "roadMarkingPointMask", numel(groundOriginalPointIdx)));

    offGroundOriginalPointIdx = resolveOffGroundOriginalPointIdx(offGroundVoxelGrid);
    poleLocalMask = resolvePoleFinePointMask(offGround, offGroundVoxelGrid, numel(offGroundOriginalPointIdx));
    signLocalMask = resolveTrafficSignFinePointMask(offGround, offGroundVoxelGrid, offGroundOriginalPointIdx);
    preciseMasks.poleCandidate = mapSourcePointMaskToCanonical(canonicalOriginalPointIdx, offGroundOriginalPointIdx, poleLocalMask);
    preciseMasks.trafficSignCandidate = mapSourcePointMaskToCanonical(canonicalOriginalPointIdx, offGroundOriginalPointIdx, signLocalMask);
end

function localPointMasks = buildFinalPointMasks(candidateMasks, preciseMasks, numPoints)
% buildFinalPointMasks: Gate precise point masks through coarse 3D
% candidate cell recovery masks, falling back to the coarse candidate
% points when a precise mask is unavailable for that semantic.
%
% Input:
%   candidateMasks: struct of coarse candidate point masks by semantic name
%   preciseMasks: struct of precise point masks by semantic name
%   numPoints: scalar number of canonical point rows
%
% Output:
%   localPointMasks: struct of final [N x 1] point-level semantic masks
    mapping = [
        "roadSurface", "roadSurface"
        "curb", "curbCandidate"
        "roadBoundary", "roadBoundary"
        "roadMarking", "roadMarkingCandidate"
        "facade", "facadeCandidate"
        "pole", "poleCandidate"
        "trafficSign", "trafficSignCandidate"];
    localPointMasks = struct();
    for k = 1:size(mapping, 1)
        outputName = char(mapping(k, 1));
        semanticName = char(mapping(k, 2));
        candidateMask = resolveCandidateMask(candidateMasks, semanticName, numPoints);
        preciseMask = false(numPoints, 1);
        if isfield(preciseMasks, semanticName)
            preciseMask = logical(preciseMasks.(semanticName));
        end
        if any(preciseMask)
            localPointMasks.(outputName) = candidateMask & preciseMask;
        else
            localPointMasks.(outputName) = candidateMask;
        end
    end
end

function semanticPointIdx = buildSemanticPointIdx(originalPointIdx, localPointMasks)
% buildSemanticPointIdx: Convert final local point masks into
% original point-index lists by semantic name.
%
% Input:
%   originalPointIdx: [N x 1] original point indices
%   localPointMasks: struct of final local point masks
%
% Output:
%   semanticPointIdx: struct of original point-index vectors
    semanticPointIdx = struct();
    fieldNames = string(fieldnames(localPointMasks));
    for k = 1:numel(fieldNames)
        fieldName = char(fieldNames(k));
        semanticPointIdx.(fieldName) = double(originalPointIdx(logical(localPointMasks.(fieldName))));
    end
end

function mapPrimitives = buildMapPrimitives(semanticGrid, semanticPointIdx)
% buildMapPrimitives: Collect 3D coarse components, derived line
% structures, component centers, and refined point memberships for offline
% map construction.
%
% Input:
%   semanticGrid: 3D voxel-cell semantic grid product
%   semanticPointIdx: struct of refined semantic point indices
%
% Output:
%   mapPrimitives: struct with 3D support regions and refined points
    mapPrimitives = struct();
    mapPrimitives.roadBoundaryComponents = resolveComponentField(semanticGrid, "roadBoundary");
    mapPrimitives.curbCandidateComponents = resolveComponentField(semanticGrid, "curbCandidate");
    mapPrimitives.facadeComponents = resolveComponentField(semanticGrid, "facadeCandidate");
    mapPrimitives.poleComponents = resolveComponentField(semanticGrid, "poleCandidate");
    mapPrimitives.trafficSignComponents = resolveComponentField(semanticGrid, "trafficSignCandidate");
    mapPrimitives.facadeLines = resolveFacadeLines(semanticGrid);
    mapPrimitives.componentCenters = resolveComponentCenters(semanticGrid);
    mapPrimitives.refinedSemanticPointIdx = semanticPointIdx;
end

function components = resolveComponentField(semanticGrid, semanticName)
% resolveComponentField: Resolve optional 3D coarse component
% summaries from a semantic grid, returning an empty component array for
% lightweight real-time coarse products.
%
% Input:
%   semanticGrid: 3D voxel-cell semantic grid product
%   semanticName: character vector semantic field name
%
% Output:
%   components: struct array of component summaries
    components = struct("semanticName", {}, "componentId", {}, "cellCount", {}, ...
        "voxelLinearIdx", {}, "voxelSub", {}, "centerXYZ", {}, "meanScore", {}, "maxScore", {}, ...
        "minSub", {}, "maxSub", {});
    if isstruct(semanticGrid) && isfield(semanticGrid, "components") && ...
            isstruct(semanticGrid.components) && isfield(semanticGrid.components, semanticName)
        components = semanticGrid.components.(semanticName);
    end
end

function facadeLines = resolveFacadeLines(semanticGrid)
% resolveFacadeLines: Resolve optional facade line structures from a
% semantic grid, returning an empty line array when derived views are not
% present.
%
% Input:
%   semanticGrid: 3D voxel-cell semantic grid product
%
% Output:
%   facadeLines: [N x 4] line segment array
    facadeLines = zeros(0, 4);
    if isstruct(semanticGrid) && isfield(semanticGrid, "derivedViews") && ...
            isstruct(semanticGrid.derivedViews) && isfield(semanticGrid.derivedViews, "lineStructures") && ...
            isstruct(semanticGrid.derivedViews.lineStructures) && ...
            isfield(semanticGrid.derivedViews.lineStructures, "detectedLines")
        facadeLines = double(semanticGrid.derivedViews.lineStructures.detectedLines);
    end
end

function componentCenters = resolveComponentCenters(semanticGrid)
% resolveComponentCenters: Resolve optional component-center
% summaries from a semantic grid, returning an empty struct when not
% present.
%
% Input:
%   semanticGrid: 3D voxel-cell semantic grid product
%
% Output:
%   componentCenters: struct of component-center arrays
    componentCenters = struct();
    if isstruct(semanticGrid) && isfield(semanticGrid, "derivedViews") && ...
            isstruct(semanticGrid.derivedViews) && isfield(semanticGrid.derivedViews, "componentCenters")
        componentCenters = semanticGrid.derivedViews.componentCenters;
    end
end

function boundary = buildProductBoundary()
% buildProductBoundary: Describe the offline point validation
% responsibility boundary encoded by the refined product.
%
% Input:
%   none
%
% Output:
%   boundary: struct with coarse dependency and offline-only policy fields
    boundary = struct();
    boundary.coarseInput = "voxelCellSemanticGridProduct";
    boundary.pointRecoveryPolicy = "onlyPointsInsideCoarseSelected3DCells";
    boundary.realTimeLocalizationDependency = "none";
    boundary.offlineMappingOutput = "refinedSemanticPointsAndMapPrimitives";
end

function groundOriginalPointIdx = resolveGroundOriginalPointIdx(groundResult, groundGrid)
% resolveGroundOriginalPointIdx: Resolve original point indices
% aligned with groundResult point-level masks.
%
% Input:
%   groundResult: struct returned by extractGroundFeatures
%   groundGrid: optional ground context struct
%
% Output:
%   groundOriginalPointIdx: [N x 1] double original point indices
    groundOriginalPointIdx = zeros(0, 1);
    if isstruct(groundResult) && isfield(groundResult, "groundOriginalPointIdx")
        groundOriginalPointIdx = double(groundResult.groundOriginalPointIdx(:));
        return;
    end
    if isstruct(groundGrid) && isfield(groundGrid, "groundOriginalPointIdx")
        groundOriginalPointIdx = double(groundGrid.groundOriginalPointIdx(:));
    end
end

function offGroundOriginalPointIdx = resolveOffGroundOriginalPointIdx(offGroundVoxelGrid)
% resolveOffGroundOriginalPointIdx: Resolve original point indices
% aligned with off-ground voxel-grid point rows.
%
% Input:
%   offGroundVoxelGrid: canonical off-ground voxelizePointCloud output
%
% Output:
%   offGroundOriginalPointIdx: [N x 1] double original point indices
    offGroundOriginalPointIdx = zeros(0, 1);
    if isstruct(offGroundVoxelGrid) && isfield(offGroundVoxelGrid, "pointIndices")
        offGroundOriginalPointIdx = double(offGroundVoxelGrid.pointIndices(:));
    end
end

function pointMask = resolveSourceMask(source, fieldName, numPoints)
% resolveSourceMask: Resolve a source point-level mask when it is
% present and aligned to the expected source point count.
%
% Input:
%   source: struct with optional point mask field
%   fieldName: character vector point mask field name
%   numPoints: scalar expected point count
%
% Output:
%   pointMask: [N x 1] logical point mask
    pointMask = false(numPoints, 1);
    if isstruct(source) && isfield(source, fieldName) && numel(source.(fieldName)) == numPoints
        pointMask = logical(source.(fieldName)(:));
    end
end

function canonicalMask = mapSourcePointMaskToCanonical(canonicalOriginalPointIdx, sourceOriginalPointIdx, sourcePointMask)
% mapSourcePointMaskToCanonical: Convert a source point-row mask into
% a canonical voxel-grid point-row mask using original point indices.
%
% Input:
%   canonicalOriginalPointIdx: [N x 1] canonical original point indices
%   sourceOriginalPointIdx: [M x 1] source original point indices
%   sourcePointMask: [M x 1] logical source point mask
%
% Output:
%   canonicalMask: [N x 1] logical canonical point mask
    canonicalMask = false(numel(canonicalOriginalPointIdx), 1);
    if isempty(sourceOriginalPointIdx) || isempty(sourcePointMask)
        return;
    end
    sourceOriginalPointIdx = double(sourceOriginalPointIdx(:));
    sourcePointMask = logical(sourcePointMask(:));
    if numel(sourceOriginalPointIdx) ~= numel(sourcePointMask)
        return;
    end
    selectedOriginalPointIdx = sourceOriginalPointIdx(sourcePointMask);
    if isempty(selectedOriginalPointIdx)
        return;
    end
    canonicalMask = ismember(double(canonicalOriginalPointIdx(:)), selectedOriginalPointIdx);
end

function candidateMask = resolveCandidateMask(candidateMasks, semanticName, numPoints)
% resolveCandidateMask: Resolve a coarse candidate point mask by
% semantic name from semantic-grid recovery metadata.
%
% Input:
%   candidateMasks: struct of candidate local point masks
%   semanticName: character vector semantic field name
%   numPoints: scalar expected point count
%
% Output:
%   candidateMask: [N x 1] logical candidate point mask
    candidateMask = false(numPoints, 1);
    if isstruct(candidateMasks) && isfield(candidateMasks, semanticName) && numel(candidateMasks.(semanticName)) == numPoints
        candidateMask = logical(candidateMasks.(semanticName)(:));
    end
end

function pointMask = resolvePoleFinePointMask(offGround, offGroundVoxelGrid, numPoints)
% resolvePoleFinePointMask: Recover refined pole points from the final pole
% fine-voxel mask of the off-ground result.
%
% Input:
%   offGround: struct returned by extractOffGroundFeatures
%   offGroundVoxelGrid: canonical off-ground voxel grid
%   numPoints: number of retained off-ground points
%
% Output:
%   pointMask: [numPoints x 1] logical pole point mask
    pointMask = false(numPoints, 1);
    if isstruct(offGround) && isfield(offGround, "pole") && isfield(offGround.pole, "fineVoxelMask")
        pole = offGround.pole;
        pointMask = mapFineVoxelMaskToLocalPoints(offGroundVoxelGrid, pole.fineVoxelMask, pole.fineOrigin, pole.fineVoxelSize, numPoints);
    end
end

function pointMask = resolveTrafficSignFinePointMask(offGround, offGroundVoxelGrid, originalPointIdx)
% resolveTrafficSignFinePointMask: Recover traffic-sign points from the
% direct original point indices of the traffic-sign channel, or from its
% fine-voxel mask when no direct indices exist.
%
% Input:
%   offGround: struct returned by extractOffGroundFeatures
%   offGroundVoxelGrid: canonical off-ground voxel grid
%   originalPointIdx: [K x 1] original indices of the off-ground points
%
% Output:
%   pointMask: [K x 1] logical traffic-sign point mask
    pointMask = false(numel(originalPointIdx), 1);
    if ~isstruct(offGround) || ~isfield(offGround, "trafficSign")
        return;
    end
    trafficSign = offGround.trafficSign;
    signPointIdx = double(trafficSign.pointIndices(:));
    if ~isempty(signPointIdx)
        pointMask = ismember(double(originalPointIdx(:)), signPointIdx);
        return;
    end
    pointMask = mapFineVoxelMaskToLocalPoints(offGroundVoxelGrid, trafficSign.fineVoxelMask, trafficSign.fineOrigin, trafficSign.fineVoxelSize, numel(originalPointIdx));
end

function pointMask = mapFineVoxelMaskToLocalPoints(offGroundVoxelGrid, voxelMask, origin, voxelSize, numPoints)
% mapFineVoxelMaskToLocalPoints: Project a [Ny x Nx x Nz] fine-voxel
% semantic mask to retained off-ground point rows using metric point
% coordinates.
%
% Input:
%   offGroundVoxelGrid: canonical off-ground voxel grid with points
%   voxelMask: [Ny x Nx x Nz] logical fine-voxel mask
%   origin: [1 x 3] lower metric edge for voxelMask
%   voxelSize: [1 x 3] voxel size in meters for voxelMask
%   numPoints: scalar expected point count
%
% Output:
%   pointMask: [N x 1] logical point mask
    pointMask = false(numPoints, 1);
    if ~isstruct(offGroundVoxelGrid) || ~isfield(offGroundVoxelGrid, "points") || isempty(voxelMask) || ndims(voxelMask) ~= 3
        return;
    end
    if numel(origin) < 3 || numel(voxelSize) < 3
        return;
    end
    points = double(offGroundVoxelGrid.points);
    if size(points, 1) ~= numPoints
        return;
    end
    origin = double(origin(1:3));
    voxelSize = double(voxelSize(1:3));
    if ~all(isfinite(origin)) || ~all(isfinite(voxelSize)) || ~all(voxelSize > 0)
        return;
    end
    xBin = floor((points(:, 1) - origin(1)) ./ voxelSize(1)) + 1;
    yBin = floor((points(:, 2) - origin(2)) ./ voxelSize(2)) + 1;
    zBin = floor((points(:, 3) - origin(3)) ./ voxelSize(3)) + 1;
    validBin = isfinite(xBin) & isfinite(yBin) & isfinite(zBin) & ...
        xBin >= 1 & xBin <= size(voxelMask, 2) & ...
        yBin >= 1 & yBin <= size(voxelMask, 1) & ...
        zBin >= 1 & zBin <= size(voxelMask, 3);
    if ~any(validBin)
        return;
    end
    voxelLinIdx = sub2ind(size(voxelMask), yBin(validBin), xBin(validBin), zBin(validBin));
    pointMask(validBin) = logical(voxelMask(voxelLinIdx));
end
