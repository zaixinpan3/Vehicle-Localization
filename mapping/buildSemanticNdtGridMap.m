function ndtMap = buildSemanticNdtGridMap(frame, coarseProduct, cfg)
% buildSemanticNdtGridMap: Project coarse 3D voxel semantic tags onto a
% fixed 2D XY grid, aggregate original point XY coordinates by semantic
% class in each 2D cell, and fit one regularized 2D NDT Gaussian per valid
% semantic-cell group. The output keeps dense per-layer maps for inspection
% and flattened component arrays for real-time localization matching.
%
% Input:
%   frame: organized point-cloud frame with x and y fields
%   coarseProduct: buildSemanticVoxelGrid product or semanticGrid struct with
%       primaryTagVolume, semanticNames, and recovery point mappings
%   cfg: semanticNdtGridMapConfig struct with fixed 2D grid, semantic class,
%       point-count, covariance, and storage controls
%
% Output:
%   ndtMap: struct containing grid geometry, semantic layers, flattened NDT
%       Gaussian components, and source/diagnostic summaries
    if nargin < 3 || isempty(cfg)
        cfg = semanticNdtGridMapConfig();
    end
    semanticGrid = resolveSemanticGrid(coarseProduct);
    cfg = validateConfig(cfg);
    geometry = buildGridGeometry(cfg);
    [pointsXY, pointTags, sourceSummary, blockIds] = recoverTaggedPoints(frame, semanticGrid);
    [selectedNames, selectedTagIds] = resolveSelectedSemantics(cfg.semanticNames, semanticGrid.semanticNames);

    layers = repmat(emptyLayer(geometry), numel(selectedNames), 1);
    for semanticIdx = 1:numel(selectedNames)
        semanticPointMask = pointTags == uint16(selectedTagIds(semanticIdx));
        layers(semanticIdx) = buildSemanticLayer(pointsXY(semanticPointMask, :), ...
            selectedNames(semanticIdx), uint16(selectedTagIds(semanticIdx)), geometry, cfg, blockIds(semanticPointMask));
    end

    components = concatenateComponents(layers);
    ndtMap = struct();
    ndtMap.mapType = "semanticNDTGridMap2D";
    ndtMap.coordinateFrame = string(cfg.coordinateFrame);
    ndtMap.semanticNames = selectedNames(:);
    ndtMap.semanticTagIds = uint16(selectedTagIds(:));
    ndtMap.geometry = geometry;
    ndtMap.layers = layers;
    ndtMap.components = components;
    ndtMap.sourceSummary = sourceSummary;
    ndtMap.configSummary = buildConfigSummary(cfg);
    if logical(cfg.storePointAssignments)
        ndtMap.pointAssignments = buildPointAssignments(pointsXY, pointTags, selectedTagIds, geometry);
    end
end

function semanticGrid = resolveSemanticGrid(coarseProduct)
% resolveSemanticGrid: Resolve the semantic grid from either the
% coarse validation product or a direct semanticGrid input.
%
% Input:
%   coarseProduct: coarse product or semantic grid struct
%
% Output:
%   semanticGrid: semantic grid struct with primaryTagVolume and recovery
    if isstruct(coarseProduct) && isfield(coarseProduct, "semanticGrid")
        semanticGrid = coarseProduct.semanticGrid;
    else
        semanticGrid = coarseProduct;
    end
    assert(isstruct(semanticGrid) && isfield(semanticGrid, "primaryTagVolume") && ...
        isfield(semanticGrid, "semanticNames") && isfield(semanticGrid, "recovery"), ...
        "coarseProduct must contain a semanticGrid with primaryTagVolume, semanticNames, and recovery.");
    assert(isfield(semanticGrid.recovery, "originalPointIdx") && isfield(semanticGrid.recovery, "pointVoxelLinIdx"), ...
        "semanticGrid.recovery must contain originalPointIdx and pointVoxelLinIdx.");
end

function cfg = validateConfig(cfg)
% validateConfig: Normalize and validate semantic NDT grid
% configuration fields before map construction.
%
% Input:
%   cfg: raw semantic NDT configuration struct
%
% Output:
%   cfg: normalized semantic NDT configuration struct
    requiredFields = ["xMin", "xMax", "yMin", "yMax", "resolution", ...
        "semanticNames", "minPointsPerGaussian", "minCovarianceEigenvalue", ...
        "maxCovarianceEigenvalue", "regularizationVariance", ...
        "storePointAssignments", "coordinateFrame"];
    for fieldName = requiredFields
        assert(isfield(cfg, char(fieldName)), "semantic NDT cfg is missing field %s.", char(fieldName));
    end
    cfg.xMin = double(cfg.xMin);
    cfg.xMax = double(cfg.xMax);
    cfg.yMin = double(cfg.yMin);
    cfg.yMax = double(cfg.yMax);
    cfg.resolution = double(cfg.resolution);
    cfg.semanticNames = string(cfg.semanticNames(:));
    cfg.minPointsPerGaussian = max(1, round(double(cfg.minPointsPerGaussian)));
    cfg.minCovarianceEigenvalue = max(0, double(cfg.minCovarianceEigenvalue));
    cfg.maxCovarianceEigenvalue = max(cfg.minCovarianceEigenvalue, double(cfg.maxCovarianceEigenvalue));
    cfg.regularizationVariance = max(0, double(cfg.regularizationVariance));
    cfg.storePointAssignments = logical(cfg.storePointAssignments);
    cfg.coordinateFrame = string(cfg.coordinateFrame);
    assert(isfinite(cfg.xMin) && isfinite(cfg.xMax) && cfg.xMin < cfg.xMax, ...
        "semantic NDT cfg xMin/xMax must define finite increasing bounds.");
    assert(isfinite(cfg.yMin) && isfinite(cfg.yMax) && cfg.yMin < cfg.yMax, ...
        "semantic NDT cfg yMin/yMax must define finite increasing bounds.");
    assert(isfinite(cfg.resolution) && cfg.resolution > 0, ...
        "semantic NDT cfg resolution must be positive and finite.");
    assert(all(isfinite([cfg.minCovarianceEigenvalue, cfg.maxCovarianceEigenvalue, cfg.regularizationVariance])), ...
        "semantic NDT covariance limits must be finite.");
end

function geometry = buildGridGeometry(cfg)
% buildGridGeometry: Build fixed 2D grid geometry used to aggregate
% semantic points into NDT cells.
%
% Input:
%   cfg: validated semantic NDT configuration
%
% Output:
%   geometry: struct with metric bounds, resolution, edges, centers, dims,
%       and total cell count
    numCols = max(1, ceil((cfg.xMax - cfg.xMin) ./ cfg.resolution));
    numRows = max(1, ceil((cfg.yMax - cfg.yMin) ./ cfg.resolution));
    xEdges = cfg.xMin + (0:numCols) .* cfg.resolution;
    yEdges = cfg.yMin + (0:numRows) .* cfg.resolution;
    geometry = struct();
    geometry.xMin = cfg.xMin;
    geometry.xMax = xEdges(end);
    geometry.yMin = cfg.yMin;
    geometry.yMax = yEdges(end);
    geometry.resolution = cfg.resolution;
    geometry.dims = [numRows, numCols];
    geometry.numCells = numRows .* numCols;
    geometry.xEdges = double(xEdges(:).');
    geometry.yEdges = double(yEdges(:).');
    geometry.xCenters = double(xEdges(1:end-1) + (0.5 .* cfg.resolution));
    geometry.yCenters = double(yEdges(1:end-1) + (0.5 .* cfg.resolution));
end

function [pointsXY, pointTags, sourceSummary, blockIds] = recoverTaggedPoints(frame, semanticGrid)
% recoverTaggedPoints: Recover original point XY coordinates and
% corresponding coarse 3D voxel semantic tag ids from semanticGrid recovery
% metadata.
%
% Input:
%   frame: organized point-cloud frame with x and y fields
%   semanticGrid: coarse semantic grid with primaryTagVolume and recovery
%
% Output:
%   pointsXY: [N x 2] original point XY coordinates
%   pointTags: [N x 1] uint16 coarse semantic tag ids
%   sourceSummary: struct with source and retained point counts
    assert(isstruct(frame) && isfield(frame, "x") && isfield(frame, "y"), ...
        "frame must contain x and y fields.");
    x = double(frame.x(:));
    y = double(frame.y(:));
    originalPointIdx = double(semanticGrid.recovery.originalPointIdx(:));
    pointVoxelLinIdx = double(semanticGrid.recovery.pointVoxelLinIdx(:));
    tagVolume = uint16(semanticGrid.primaryTagVolume(:));
    valid = isfinite(originalPointIdx) & originalPointIdx >= 1 & originalPointIdx <= numel(x) & ...
        originalPointIdx == floor(originalPointIdx) & isfinite(pointVoxelLinIdx) & ...
        pointVoxelLinIdx >= 1 & pointVoxelLinIdx <= numel(tagVolume) & pointVoxelLinIdx == floor(pointVoxelLinIdx);
    validOriginalIdx = originalPointIdx(valid);
    validVoxelLinIdx = pointVoxelLinIdx(valid);
    pointX = x(validOriginalIdx);
    pointY = y(validOriginalIdx);
    finiteXY = isfinite(pointX) & isfinite(pointY);
    pointsXY = [pointX(finiteXY), pointY(finiteXY)];
    pointTags = tagVolume(validVoxelLinIdx(finiteXY));
    blockIds=repmat("unspecified",size(pointsXY,1),1);
    if isfield(frame,'observationBlockId')
        ids=string(frame.observationBlockId(:));
        assert(all(~ismissing(ids) & strlength(ids)>0),'Invalid observation block IDs.');
        if isscalar(ids), blockIds=repmat(ids,size(pointsXY,1),1);
        else
            assert(numel(ids)==numel(x),'One block ID is required per original point.');
            blockIds=ids(validOriginalIdx(finiteXY));
        end
    end
    sourceSummary = struct();
    sourceSummary.numFramePoints = double(numel(x));
    sourceSummary.numRecoveredPoints = double(numel(originalPointIdx));
    sourceSummary.numValidRecoveredPoints = double(nnz(valid));
    sourceSummary.numFiniteTaggedPoints = double(size(pointsXY, 1));
end

function [selectedNames, selectedTagIds] = resolveSelectedSemantics(requestedNames, allSemanticNames)
% resolveSelectedSemantics: Resolve requested semantic class names into
% semanticGrid tag ids while preserving configured class order.
%
% Input:
%   requestedNames: string vector of requested semantic names
%   allSemanticNames: semanticGrid semanticNames vector
%
% Output:
%   selectedNames: string vector of names present in semanticGrid
%   selectedTagIds: numeric vector of 1-based tag ids matching selectedNames
    requestedNames = string(requestedNames(:));
    allSemanticNames = string(allSemanticNames(:));
    if isempty(requestedNames)
        requestedNames = setdiff(allSemanticNames, ["empty", "unknown", "lowConfidence"], "stable");
    end
    selectedNamesBuffer = strings(numel(requestedNames), 1);
    selectedTagIdsBuffer = zeros(numel(requestedNames), 1);
    selectedCount = 0;
    for k = 1:numel(requestedNames)
        tagIdx = find(allSemanticNames == requestedNames(k), 1, "first");
        if ~isempty(tagIdx)
            selectedCount = selectedCount + 1;
            selectedNamesBuffer(selectedCount) = requestedNames(k);
            selectedTagIdsBuffer(selectedCount) = tagIdx;
        end
    end
    selectedNames = selectedNamesBuffer(1:selectedCount);
    selectedTagIds = selectedTagIdsBuffer(1:selectedCount);
end

function layer = emptyLayer(geometry)
% emptyLayer: Create an empty semantic NDT layer with dense maps and
% flattened component fields initialized to valid sizes.
%
% Input:
%   geometry: semantic NDT grid geometry struct
%
% Output:
%   layer: scalar semantic NDT layer struct
    mapSize = double(geometry.dims);
    layer = struct();
    layer.semanticName = "";
    layer.semanticTagId = uint16(0);
    layer.countMap = zeros(mapSize, "uint16");
    layer.validMask = false(mapSize);
    layer.meanX = NaN(mapSize);
    layer.meanY = NaN(mapSize);
    layer.covarianceXX = NaN(mapSize);
    layer.covarianceXY = NaN(mapSize);
    layer.covarianceYY = NaN(mapSize);
    layer.invCovarianceXX = NaN(mapSize);
    layer.invCovarianceXY = NaN(mapSize);
    layer.invCovarianceYY = NaN(mapSize);
    layer.determinant = NaN(mapSize);
    layer.logNormalizationConstant = NaN(mapSize);
    layer.cellLinIdx = zeros(0, 1, "int32");
    layer.cellSub = zeros(0, 2, "int32");
    layer.count = zeros(0, 1, "uint16");
    layer.mean = zeros(0, 2);
    layer.covariance = zeros(2, 2, 0);
    layer.invCovariance = zeros(2, 2, 0);
    layer.componentLogNormalizationConstant = zeros(0, 1);
    layer.blockStatistics=struct('observationBlockId',strings(0,1),'cellLinIdx',zeros(0,1), ...
        'count',zeros(0,1),'mean',zeros(0,2),'centeredScatter',zeros(2,2,0));
end

function layer = buildSemanticLayer(pointsXY, semanticName, semanticTagId, geometry, cfg, blockIds)
% buildSemanticLayer: Aggregate one semantic class into fixed XY cells
% and compute regularized 2D Gaussian statistics for valid cells.
%
% Input:
%   pointsXY: [N x 2] point coordinates for one semantic class
%   semanticName: string scalar semantic class name
%   semanticTagId: uint16 semanticGrid tag id
%   geometry: fixed 2D grid geometry
%   cfg: semantic NDT configuration
%
% Output:
%   layer: semantic NDT layer with dense maps and flattened components
    layer = emptyLayer(geometry);
    layer.semanticName = string(semanticName);
    layer.semanticTagId = uint16(semanticTagId);
    if isempty(pointsXY)
        return;
    end

    [cellLinIdx, inGrid] = assignPointsToCells(pointsXY, geometry);
    pointsXY = double(pointsXY(inGrid, :));
    cellLinIdx = double(cellLinIdx(inGrid));
    blockIds=blockIds(inGrid);
    if isempty(cellLinIdx)
        return;
    end

    numCells = double(geometry.numCells);
    layer.blockStatistics=aggregateBlocks(pointsXY,blockIds,cellLinIdx);
    [cellRow,cellCol]=ind2sub(geometry.dims,cellLinIdx);
    localOrigin=[geometry.xMin+(cellCol-1)*geometry.resolution,geometry.yMin+(cellRow-1)*geometry.resolution];
    pointsXY=pointsXY-localOrigin;
    countVec = accumarray(cellLinIdx, 1, [numCells, 1], @sum, 0);
    sumX = accumarray(cellLinIdx, pointsXY(:, 1), [numCells, 1], @sum, 0);
    sumY = accumarray(cellLinIdx, pointsXY(:, 2), [numCells, 1], @sum, 0);
    sumXX = accumarray(cellLinIdx, pointsXY(:, 1) .* pointsXY(:, 1), [numCells, 1], @sum, 0);
    sumXY = accumarray(cellLinIdx, pointsXY(:, 1) .* pointsXY(:, 2), [numCells, 1], @sum, 0);
    sumYY = accumarray(cellLinIdx, pointsXY(:, 2) .* pointsXY(:, 2), [numCells, 1], @sum, 0);
    validCellLinIdx = find(countVec >= double(cfg.minPointsPerGaussian));
    if isempty(validCellLinIdx)
        layer.countMap = reshape(uint16(min(countVec, double(intmax("uint16")))), geometry.dims);
        return;
    end

    count = double(countVec(validCellLinIdx));
    meanX = sumX(validCellLinIdx) ./ count;
    meanY = sumY(validCellLinIdx) ./ count;
    covarianceXX = (sumXX(validCellLinIdx) - (sumX(validCellLinIdx) .* sumX(validCellLinIdx) ./ count)) ./ max(count - 1, 1);
    covarianceXY = (sumXY(validCellLinIdx) - (sumX(validCellLinIdx) .* sumY(validCellLinIdx) ./ count)) ./ max(count - 1, 1);
    covarianceYY = (sumYY(validCellLinIdx) - (sumY(validCellLinIdx) .* sumY(validCellLinIdx) ./ count)) ./ max(count - 1, 1);
    [cellRow,cellCol]=ind2sub(geometry.dims,validCellLinIdx);
    meanX=meanX+geometry.xMin+(cellCol-1)*geometry.resolution;
    meanY=meanY+geometry.yMin+(cellRow-1)*geometry.resolution;
    [covariance, invCovariance, determinant, logNorm] = regularizeCovariances( ...
        covarianceXX, covarianceXY, covarianceYY, cfg);

    layer = fillLayerMaps(layer, geometry, countVec, validCellLinIdx, meanX, meanY, covariance, invCovariance, determinant, logNorm);
end

function stats=aggregateBlocks(points,ids,cells)
% Preserve unregularized centered sufficient statistics, including small cells.
    [names,~,block]=unique(ids); [keys,~,group]=unique([block cells],'rows');
    count=accumarray(group,1); means=zeros(size(keys,1),2); scatter=zeros(2,2,size(keys,1));
    for j=1:size(keys,1)
        values=points(group==j,:); origin=values(1,:); local=values-origin;
        center=mean(local,1); residual=local-center;
        means(j,:)=center+origin; scatter(:,:,j)=residual.'*residual;
    end
    stats=struct('observationBlockId',names(keys(:,1)),'cellLinIdx',keys(:,2), ...
        'count',count,'mean',means,'centeredScatter',scatter);
end

function [cellLinIdx, inGrid] = assignPointsToCells(pointsXY, geometry)
% assignPointsToCells: Assign point XY coordinates to fixed 2D grid
% linear cell indices in [Ny x Nx] layout.
%
% Input:
%   pointsXY: [N x 2] numeric XY coordinates
%   geometry: fixed 2D grid geometry struct
%
% Output:
%   cellLinIdx: [N x 1] double linear cell indices
%   inGrid: [N x 1] logical selector for points inside the grid
    xBin = floor((double(pointsXY(:, 1)) - geometry.xMin) ./ geometry.resolution) + 1;
    yBin = floor((double(pointsXY(:, 2)) - geometry.yMin) ./ geometry.resolution) + 1;
    inGrid = isfinite(xBin) & isfinite(yBin) & xBin >= 1 & xBin <= geometry.dims(2) & ...
        yBin >= 1 & yBin <= geometry.dims(1);
    cellLinIdx = zeros(size(xBin));
    cellLinIdx(inGrid) = yBin(inGrid) + ((xBin(inGrid) - 1) .* geometry.dims(1));
end

function [covariance, invCovariance, determinant, logNorm] = regularizeCovariances(covarianceXX, covarianceXY, covarianceYY, cfg)
% regularizeCovariances: Apply symmetric 2D covariance regularization,
% eigenvalue bounds, inverse-covariance computation, and Gaussian log
% normalization constants for each valid NDT cell.
%
% Input:
%   covarianceXX, covarianceXY, covarianceYY: [K x 1] raw covariance terms
%   cfg: semantic NDT configuration with covariance safeguards
%
% Output:
%   covariance: [2 x 2 x K] bounded covariance matrices
%   invCovariance: [2 x 2 x K] inverse covariance matrices
%   determinant: [K x 1] covariance determinants
%   logNorm: [K x 1] 2D Gaussian log normalization constants
    numComponents = numel(covarianceXX);
    covariance = zeros(2, 2, numComponents);
    invCovariance = zeros(2, 2, numComponents);
    determinant = zeros(numComponents, 1);
    logNorm = zeros(numComponents, 1);
    for k = 1:numComponents
        covMat = [covarianceXX(k), covarianceXY(k); covarianceXY(k), covarianceYY(k)];
        covMat(~isfinite(covMat)) = 0;
        covMat = (covMat + covMat.') ./ 2;
        covMat = covMat + (cfg.regularizationVariance .* eye(2));
        [vectors, values] = eig(covMat);
        eigValues = diag(values);
        eigValues(~isfinite(eigValues)) = cfg.minCovarianceEigenvalue;
        eigValues = min(max(eigValues, cfg.minCovarianceEigenvalue), cfg.maxCovarianceEigenvalue);
        covMat = vectors * diag(eigValues) * vectors.';
        covMat = (covMat + covMat.') ./ 2;
        invMat = covMat \ eye(2);
        detValue = det(covMat);
        covariance(:, :, k) = covMat;
        invCovariance(:, :, k) = (invMat + invMat.') ./ 2;
        determinant(k) = detValue;
        logNorm(k) = -log(2 .* pi) - (0.5 .* log(detValue));
    end
end

function layer = fillLayerMaps(layer, geometry, countVec, validCellLinIdx, meanX, meanY, covariance, invCovariance, determinant, logNorm)
% fillLayerMaps: Populate dense per-cell maps and flattened component
% arrays for one semantic NDT layer.
%
% Input:
%   layer: initialized semantic NDT layer
%   geometry: fixed 2D grid geometry
%   countVec: [numCells x 1] point counts for all cells
%   validCellLinIdx: [K x 1] valid cell linear indices
%   meanX, meanY: [K x 1] Gaussian mean coordinates
%   covariance, invCovariance: [2 x 2 x K] Gaussian matrices
%   determinant, logNorm: [K x 1] Gaussian determinant and log constants
%
% Output:
%   layer: populated semantic NDT layer
    layer.countMap = reshape(uint16(min(countVec, double(intmax("uint16")))), geometry.dims);
    layer.validMask(validCellLinIdx) = true;
    layer.meanX(validCellLinIdx) = meanX;
    layer.meanY(validCellLinIdx) = meanY;
    layer.covarianceXX(validCellLinIdx) = squeeze(covariance(1, 1, :));
    layer.covarianceXY(validCellLinIdx) = squeeze(covariance(1, 2, :));
    layer.covarianceYY(validCellLinIdx) = squeeze(covariance(2, 2, :));
    layer.invCovarianceXX(validCellLinIdx) = squeeze(invCovariance(1, 1, :));
    layer.invCovarianceXY(validCellLinIdx) = squeeze(invCovariance(1, 2, :));
    layer.invCovarianceYY(validCellLinIdx) = squeeze(invCovariance(2, 2, :));
    layer.determinant(validCellLinIdx) = determinant;
    layer.logNormalizationConstant(validCellLinIdx) = logNorm;
    [cellRows, cellCols] = ind2sub(geometry.dims, validCellLinIdx);
    layer.cellLinIdx = int32(validCellLinIdx(:));
    layer.cellSub = int32([cellRows(:), cellCols(:)]);
    layer.count = layer.countMap(validCellLinIdx);
    layer.mean = [meanX(:), meanY(:)];
    layer.covariance = covariance;
    layer.invCovariance = invCovariance;
    layer.componentLogNormalizationConstant = logNorm(:);
end

function components = concatenateComponents(layers)
% concatenateComponents: Flatten all semantic-layer NDT Gaussians into
% component arrays suited for fast real-time matching and class filtering.
%
% Input:
%   layers: semantic NDT layer struct array
%
% Output:
%   components: struct with semantic ids, cell ids, means, covariances,
%       inverse covariances, determinants, log constants, and counts
    totalComponents = sum(arrayfun(@(layer) numel(layer.cellLinIdx), layers));
    components = struct();
    components.semanticName = strings(totalComponents, 1);
    components.semanticTagId = zeros(totalComponents, 1, "uint16");
    components.layerIdx = zeros(totalComponents, 1, "uint16");
    components.cellLinIdx = zeros(totalComponents, 1, "int32");
    components.cellSub = zeros(totalComponents, 2, "int32");
    components.count = zeros(totalComponents, 1, "uint16");
    components.mean = zeros(totalComponents, 2);
    components.covariance = zeros(2, 2, totalComponents);
    components.invCovariance = zeros(2, 2, totalComponents);
    components.logNormalizationConstant = zeros(totalComponents, 1);
    writeStart = 1;
    for layerIdx = 1:numel(layers)
        numLayerComponents = numel(layers(layerIdx).cellLinIdx);
        if numLayerComponents == 0
            continue;
        end
        writeIdx = writeStart:(writeStart + numLayerComponents - 1);
        components.semanticName(writeIdx) = layers(layerIdx).semanticName;
        components.semanticTagId(writeIdx) = layers(layerIdx).semanticTagId;
        components.layerIdx(writeIdx) = uint16(layerIdx);
        components.cellLinIdx(writeIdx) = layers(layerIdx).cellLinIdx;
        components.cellSub(writeIdx, :) = layers(layerIdx).cellSub;
        components.count(writeIdx) = layers(layerIdx).count;
        components.mean(writeIdx, :) = layers(layerIdx).mean;
        components.covariance(:, :, writeIdx) = layers(layerIdx).covariance;
        components.invCovariance(:, :, writeIdx) = layers(layerIdx).invCovariance;
        components.logNormalizationConstant(writeIdx) = layers(layerIdx).componentLogNormalizationConstant;
        writeStart = writeStart + numLayerComponents;
    end
    components.numComponents = double(totalComponents);
end

function configSummary = buildConfigSummary(cfg)
% buildConfigSummary: Store scalar semantic NDT construction settings
% on the output map without retaining unrelated configuration state.
%
% Input:
%   cfg: validated semantic NDT configuration
%
% Output:
%   configSummary: struct with semantic NDT construction settings
    configSummary = struct();
    configSummary.semanticNames = string(cfg.semanticNames(:));
    configSummary.minPointsPerGaussian = double(cfg.minPointsPerGaussian);
    configSummary.minCovarianceEigenvalue = double(cfg.minCovarianceEigenvalue);
    configSummary.maxCovarianceEigenvalue = double(cfg.maxCovarianceEigenvalue);
    configSummary.regularizationVariance = double(cfg.regularizationVariance);
end

function pointAssignments = buildPointAssignments(pointsXY, pointTags, selectedTagIds, geometry)
% buildPointAssignments: Optionally store finite tagged source points,
% semantic tag ids, and fixed-grid cell indices for diagnostics.
%
% Input:
%   pointsXY: [N x 2] finite recovered source point coordinates
%   pointTags: [N x 1] uint16 semantic tag ids
%   selectedTagIds: selected semantic tag ids
%   geometry: fixed 2D grid geometry
%
% Output:
%   pointAssignments: struct with pointsXY, semanticTagId, cellLinIdx, and
%       selectedMask fields
    [cellLinIdx, inGrid] = assignPointsToCells(pointsXY, geometry);
    selectedMask = ismember(pointTags(:), uint16(selectedTagIds(:)));
    pointAssignments = struct();
    pointAssignments.pointsXY = pointsXY;
    pointAssignments.semanticTagId = uint16(pointTags(:));
    pointAssignments.cellLinIdx = int32(cellLinIdx(:));
    pointAssignments.inGrid = logical(inGrid(:));
    pointAssignments.selectedMask = logical(selectedMask(:));
end
