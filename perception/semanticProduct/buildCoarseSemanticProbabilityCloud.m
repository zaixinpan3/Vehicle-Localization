function probabilityCloud = buildCoarseSemanticProbabilityCloud(coarseGround, coarseOffGround, cfg)
% buildCoarseSemanticProbabilityCloud: Aggregate pillar-classified curb,
% road-marking, and pole support into a sparse semantic cloud with XYZ
% moments and an exact XY marginal. Each
% component stores a regularized Gaussian, semantic evidence probability,
% hit-based occupancy probability, and normalized mixture weight without
% allocating dense semantic layers.
%
% Input:
%   coarseGround: output from analyzeGroundPillars
%   coarseOffGround: output from analyzeStructuralPillars
%   cfg: struct from coarseSemanticProbabilityCloudConfig
%
% Output:
%   probabilityCloud: mean/covariance are XY; meanXYZ/covarianceXYZ retain
%       height in components for which heightAvailable is true
    cfg = validateConfig(cfg);
    geometry = buildGeometry(cfg);
    semanticNames = string(cfg.semanticNames(:));
    componentSets = cell(numel(semanticNames), 1);
    sourceCounts = zeros(numel(semanticNames), 1);
    sourceCellCounts = zeros(numel(semanticNames), 1);

    for semanticIdx = 1:numel(semanticNames)
        observations = ...
            selectSemanticObservations(semanticNames(semanticIdx), ...
            coarseGround, coarseOffGround);
        componentSets{semanticIdx} = aggregateComponents( ...
            observations, semanticNames(semanticIdx), ...
            semanticIdx, geometry, cfg);
        sourceCounts(semanticIdx) = sum(observations.count);
        sourceCellCounts(semanticIdx) = size(observations.mean, 1);
    end

    components = concatenateComponents(componentSets);
    components = normalizeMixtureWeights(components);
    layers = summarizeLayers(components, semanticNames);
    probabilityCloud = struct();
    probabilityCloud.mapType = "semanticNDTProbabilityCloud2D";
    probabilityCloud.representation = "sparseGaussianMixture";
    probabilityCloud.classificationStage = "pillarOnlyCoarseValidation";
    probabilityCloud.coordinateFrame = string(cfg.coordinateFrame);
    probabilityCloud.spatialCoordinateFrame = "projectionRotationAppliedToSensorXYZ";
    probabilityCloud.projectionRotation = cfg.projectionRotation;
    probabilityCloud.dimension = 2;
    probabilityCloud.spatialDimension = 3;
    probabilityCloud.heightModel = "jointGaussianWithExactXYMarginal";
    probabilityCloud.weightSemantics = "normalizedSemanticEvidenceTimesHitSupport";
    probabilityCloud.probabilityCalibration = "uncalibratedEvidence";
    probabilityCloud.geometry = geometry;
    probabilityCloud.semanticNames = semanticNames;
    probabilityCloud.layers = layers;
    probabilityCloud.components = components;
    probabilityCloud.sourceSummary = struct( ...
        "semanticName", semanticNames, ...
        "selectedHitCount", sourceCounts, ...
        "selectedSourceCellCount", sourceCellCounts);
    probabilityCloud.configSummary = struct( ...
        "minimumPointsPerComponent", double(cfg.minimumPointsPerComponent), ...
        "minimumSemanticProbability", double(cfg.minimumSemanticProbability), ...
        "occupancySaturationPointCount", double(cfg.occupancySaturationPointCount), ...
        "minCovarianceEigenvalue", double(cfg.minCovarianceEigenvalue), ...
        "maxCovarianceEigenvalue", double(cfg.maxCovarianceEigenvalue), ...
        "regularizationVariance", double(cfg.regularizationVariance));
end

function cfg = validateConfig(cfg)
% validateConfig: Normalize the required probability-cloud settings.
    if ~isfield(cfg, "minimumConditionalHeightVariance")
        cfg.minimumConditionalHeightVariance = 1.0e-4;
    end
    assert(isscalar(cfg.minimumConditionalHeightVariance) && ...
        isfinite(cfg.minimumConditionalHeightVariance) && cfg.minimumConditionalHeightVariance > 0);
    requiredFields = ["xMin", "xMax", "yMin", "yMax", "resolution", ...
        "coordinateFrame", "semanticNames", "minimumSemanticProbability", ...
        "occupancySaturationPointCount", "minimumPointsPerComponent", ...
        "minCovarianceEigenvalue", "maxCovarianceEigenvalue", ...
        "regularizationVariance"];
    assert(isstruct(cfg) && all(isfield(cfg, requiredFields)), ...
        "cfg must be produced by coarseSemanticProbabilityCloudConfig.");
    cfg.xMin = double(cfg.xMin);
    cfg.xMax = double(cfg.xMax);
    cfg.yMin = double(cfg.yMin);
    cfg.yMax = double(cfg.yMax);
    cfg.resolution = double(cfg.resolution);
    cfg.semanticNames = string(cfg.semanticNames(:));
    cfg.minimumSemanticProbability = max(0, min(1, double(cfg.minimumSemanticProbability)));
    cfg.occupancySaturationPointCount = max(eps, double(cfg.occupancySaturationPointCount));
    cfg.minimumPointsPerComponent = max(1, round(double(cfg.minimumPointsPerComponent)));
    cfg.minCovarianceEigenvalue = max(eps, double(cfg.minCovarianceEigenvalue));
    cfg.maxCovarianceEigenvalue = max(cfg.minCovarianceEigenvalue, double(cfg.maxCovarianceEigenvalue));
    cfg.regularizationVariance = max(0, double(cfg.regularizationVariance));
    assert(all(isfinite([cfg.xMin, cfg.xMax, cfg.yMin, cfg.yMax, cfg.resolution])) && ...
        cfg.xMin < cfg.xMax && cfg.yMin < cfg.yMax && cfg.resolution > 0, ...
        "Probability-cloud grid bounds and resolution must be finite and valid.");
end

function geometry = buildGeometry(cfg)
% buildGeometry: Create the fixed 2D grid metadata without dense layers.
    numCols = max(1, ceil((cfg.xMax - cfg.xMin) ./ cfg.resolution));
    numRows = max(1, ceil((cfg.yMax - cfg.yMin) ./ cfg.resolution));
    geometry = struct();
    geometry.xMin = cfg.xMin;
    geometry.xMax = cfg.xMin + (numCols .* cfg.resolution);
    geometry.yMin = cfg.yMin;
    geometry.yMax = cfg.yMin + (numRows .* cfg.resolution);
    geometry.resolution = cfg.resolution;
    geometry.dims = [numRows, numCols];
    geometry.numCells = numRows .* numCols;
end

function observations = selectSemanticObservations(semanticName, coarseGround, coarseOffGround)
% selectSemanticObservations: Convert accepted source cells into weighted
% Gaussian sufficient statistics without assigning semantics to raw points.
    switch lower(string(semanticName))
        case "curb"
            observations = selectGroundCells( ...
                coarseGround, coarseGround.curbCellMask, ...
                coarseGround.curbProbability);
        case "roadmarking"
            observations = selectGroundCells( ...
                coarseGround, coarseGround.roadMarkingCellMask, ...
                coarseGround.roadMarkingProbability);
        case "pole"
            observations = selectOffGroundColumns( ...
                coarseOffGround, coarseOffGround.poleCellMask, ...
                coarseOffGround.poleProbability);
        otherwise
            error("Unsupported coarse semantic class: %s", semanticName);
    end
end

function observations = selectGroundCells(ground, cellMask, probabilityMap)
% selectGroundCells: Represent each accepted ground cell by its center,
% hit count, evidence probability, and uniform-cell covariance.
    mapSize = size(cellMask);
    selectedCell = find(logical(cellMask));
    selectedCell = selectedCell(:);
    [row, col] = ind2sub(mapSize, selectedCell);
    row = row(:); col = col(:);
    meanXY = [ground.cellOrigin(1) + ((double(col) - 0.5) .* ground.cellSize(1)), ...
        ground.cellOrigin(2) + ((double(row) - 0.5) .* ground.cellSize(2))];
    count = double(ground.stats.countMap(selectedCell));
    probability = double(probabilityMap(selectedCell));
    count = count(:); probability = probability(:);
    valid = count > 0 & all(isfinite(meanXY), 2) & isfinite(probability);
    observations = observationStruct( ...
        meanXY(valid, :), count(valid), probability(valid), ground.cellSize);
    if isfield(ground, "moments")
        observations = useEmpiricalMoments(observations, ground.moments, selectedCell(valid));
    end
end

function observations = selectOffGroundColumns(offGround, cellMask, probabilityMap)
% selectOffGroundColumns: Represent accepted pole columns by sparse column
% sufficient statistics rather than point labels.
    selectedCell = find(logical(cellMask));
    maps = offGround.columnMaps;
    x = double(maps.xMap(selectedCell)); y = double(maps.yMap(selectedCell));
    meanXY = [x(:), y(:)];
    count = double(maps.pillarCounts(selectedCell));
    probability = double(probabilityMap(selectedCell));
    count = count(:); probability = probability(:);
    valid = count > 0 & all(isfinite(meanXY), 2) & isfinite(probability);
    observations = observationStruct( ...
        meanXY(valid, :), count(valid), probability(valid), [maps.dx, maps.dy]);
    if isfield(maps, "moments")
        observations = useEmpiricalMoments(observations, maps.moments, selectedCell(valid));
    end
end

function observations = useEmpiricalMoments(observations, moments, rows)
% useEmpiricalMoments: Carry geometric scatter independently of class decisions.
    observations.mean = moments.mean(rows, :);
    observations.covarianceXX = moments.covariance(rows, 1);
    observations.covarianceXY = moments.covariance(rows, 2);
    observations.covarianceYY = moments.covariance(rows, 3);
    if isfield(moments, "meanZ")
        observations.meanZ = moments.meanZ(rows);
        observations.heightCovariance = moments.heightCovariance(rows, :);
        observations.heightAvailable(:) = true;
    end
end

function observations = observationStruct(meanXY, count, probability, cellSize)
% observationStruct: Package voxel-level first and second moments.
    numObservations = size(meanXY, 1);
    observations = struct();
    observations.mean = double(meanXY);
    observations.count = double(count(:));
    observations.probability = double(probability(:));
    observations.covarianceXX = repmat((double(cellSize(1)).^2) ./ 12, numObservations, 1);
    observations.covarianceXY = zeros(numObservations, 1);
    observations.covarianceYY = repmat((double(cellSize(2)).^2) ./ 12, numObservations, 1);
    observations.meanZ = zeros(numObservations, 1);
    observations.heightCovariance = zeros(numObservations, 3);
    observations.heightAvailable = false(numObservations, 1);
end

function components = aggregateComponents(observations, semanticName, semanticId, geometry, cfg)
% aggregateComponents: Fit one regularized 2D Gaussian per occupied output
% cell by weighted moment matching of source-cell distributions.
    components = emptyComponents();
    if isempty(observations.mean)
        return;
    end
    [cellLinIdx, inGrid] = assignToGrid(observations.mean, geometry);
    meanXY = double(observations.mean(inGrid, :));
    sourceCount = double(observations.count(inGrid));
    evidenceProbability = double(observations.probability(inGrid));
    sourceCovarianceXX = double(observations.covarianceXX(inGrid));
    sourceCovarianceXY = double(observations.covarianceXY(inGrid));
    sourceCovarianceYY = double(observations.covarianceYY(inGrid));
    cellLinIdx = double(cellLinIdx(inGrid));
    if isempty(cellLinIdx)
        return;
    end

    numCells = double(geometry.numCells);
    countVector = accumarray(cellLinIdx, sourceCount, [numCells, 1], @sum, 0);
    sumX = accumarray(cellLinIdx, sourceCount .* meanXY(:, 1), ...
        [numCells, 1], @sum, 0);
    sumY = accumarray(cellLinIdx, sourceCount .* meanXY(:, 2), ...
        [numCells, 1], @sum, 0);
    sumXX = accumarray(cellLinIdx, sourceCount .* ...
        (sourceCovarianceXX + meanXY(:, 1).^2), [numCells, 1], @sum, 0);
    sumXY = accumarray(cellLinIdx, sourceCount .* ...
        (sourceCovarianceXY + (meanXY(:, 1).*meanXY(:, 2))), ...
        [numCells, 1], @sum, 0);
    sumYY = accumarray(cellLinIdx, sourceCount .* ...
        (sourceCovarianceYY + meanXY(:, 2).^2), [numCells, 1], @sum, 0);
    sumProbability = accumarray( ...
        cellLinIdx, sourceCount .* evidenceProbability, ...
        [numCells, 1], @sum, 0);
    validCell = find(countVector >= cfg.minimumPointsPerComponent);
    if isempty(validCell)
        return;
    end

    count = countVector(validCell);
    meanX = sumX(validCell) ./ count;
    meanY = sumY(validCell) ./ count;
    covarianceXX = (sumXX(validCell) ./ count) - (meanX.^2);
    covarianceXY = (sumXY(validCell) ./ count) - (meanX.*meanY);
    covarianceYY = (sumYY(validCell) ./ count) - (meanY.^2);
    [covariance, inverseCovariance, determinant, logNormalization] = ...
        regularizeCovariances(covarianceXX, covarianceXY, covarianceYY, cfg);
    semanticProbability = min(max(sumProbability(validCell) ./ count, 0), 1);
    occupancyProbability = 1 - exp(-count ./ cfg.occupancySaturationPointCount);
    unnormalizedWeight = semanticProbability .* occupancyProbability;
    [cellRow, cellCol] = ind2sub(geometry.dims, validCell);

    numComponents = numel(validCell);
    components.semanticName = repmat(string(semanticName), numComponents, 1);
    components.semanticId = repmat(uint8(semanticId), numComponents, 1);
    components.cellLinIdx = int32(validCell(:));
    components.cellSub = int32([cellRow(:), cellCol(:)]);
    components.count = uint32(count(:));
    components.mean = [meanX(:), meanY(:)];
    components.covariance = covariance;
    components.invCovariance = inverseCovariance;
    components.determinant = determinant(:);
    components.logNormalizationConstant = logNormalization(:);
    components.semanticProbability = semanticProbability(:);
    components.occupancyProbability = occupancyProbability(:);
    components.unnormalizedWeight = unnormalizedWeight(:);
    components.mixtureWeight = zeros(numComponents, 1);
    components.numComponents = double(numComponents);
    % Recenter source means at the output mean before accumulating height
    % scatter; this avoids subtracting large squared map coordinates.
    countHeight = accumarray(cellLinIdx, sourceCount.*observations.heightAvailable(inGrid), [numCells, 1]);
    z = observations.meanZ(inGrid);
    zSum = accumarray(cellLinIdx, sourceCount.*z, [numCells, 1]);
    zMean = zSum./max(countVector, 1);
    xMean = sumX./max(countVector, 1);
    yMean = sumY./max(countVector, 1);
    dz = z-zMean(cellLinIdx);
    hc = observations.heightCovariance(inGrid, :);
    values = [hc(:, 1)+(meanXY(:, 1)-xMean(cellLinIdx)).*dz, ...
        hc(:, 2)+(meanXY(:, 2)-yMean(cellLinIdx)).*dz, hc(:, 3)+dz.^2];
    heightScatter = zeros(numComponents, 3);
    for entry = 1:3
        sums = accumarray(cellLinIdx, sourceCount.*values(:, entry), [numCells, 1]);
        heightScatter(:, entry) = sums(validCell)./count;
    end
    cross = heightScatter(:, 1:2);
    ia = reshape(inverseCovariance(1, 1, :), [], 1);
    ib = reshape(inverseCovariance(1, 2, :), [], 1);
    id = reshape(inverseCovariance(2, 2, :), [], 1);
    explained = ia.*cross(:, 1).^2+2*ib.*cross(:, 1).*cross(:, 2)+id.*cross(:, 2).^2;
    varianceZ = max(heightScatter(:, 3)+cfg.regularizationVariance, ...
        explained+cfg.minimumConditionalHeightVariance);
    components.meanXYZ = [components.mean, zMean(validCell)];
    components.covarianceXYZ = zeros(3, 3, numComponents);
    components.covarianceXYZ(1:2, 1:2, :) = covariance;
    components.covarianceXYZ(1:2, 3, :) = reshape(cross.', 2, 1, []);
    components.covarianceXYZ(3, 1:2, :) = reshape(cross.', 1, 2, []);
    components.covarianceXYZ(3, 3, :) = reshape(varianceZ, 1, 1, []);
    components.heightAvailable = countHeight(validCell) == count;
end

function [cellLinIdx, inGrid] = assignToGrid(pointsXY, geometry)
% assignToGrid: Assign XY samples to the fixed [Ny Nx] output grid.
    xBin = floor((pointsXY(:, 1) - geometry.xMin) ./ geometry.resolution) + 1;
    yBin = floor((pointsXY(:, 2) - geometry.yMin) ./ geometry.resolution) + 1;
    inGrid = isfinite(xBin) & isfinite(yBin) & ...
        xBin >= 1 & xBin <= geometry.dims(2) & ...
        yBin >= 1 & yBin <= geometry.dims(1);
    cellLinIdx = zeros(size(xBin));
    cellLinIdx(inGrid) = yBin(inGrid) + ...
        ((xBin(inGrid) - 1) .* geometry.dims(1));
end

function [covariance, inverseCovariance, determinant, logNormalization] = ...
        regularizeCovariances(covarianceXX, covarianceXY, covarianceYY, cfg)
% regularizeCovariances: Bound covariance eigenvalues and cache inverse and
% normalization terms required by NDT matching.
    % Spectral clipping of symmetric 2-by-2 matrices in one batch. The
    % eigenprojector formula avoids per-component eig, inverse and det calls.
    a = covarianceXX(:); b = covarianceXY(:); d = covarianceYY(:);
    a(~isfinite(a)) = 0; b(~isfinite(b)) = 0; d(~isfinite(d)) = 0;
    a = a + cfg.regularizationVariance; d = d + cfg.regularizationVariance;
    center = (a+d)./2; halfDifference = (a-d)./2;
    radius = hypot(halfDifference,b);
    upper = min(max(center+radius,cfg.minCovarianceEigenvalue),cfg.maxCovarianceEigenvalue);
    lower = min(max(center-radius,cfg.minCovarianceEigenvalue),cfg.maxCovarianceEigenvalue);
    clippedCenter = (upper+lower)./2;
    scale = zeros(size(radius));
    distinct = radius>0;
    scale(distinct) = (upper(distinct)-lower(distinct))./(2.*radius(distinct));
    xx = clippedCenter + scale.*halfDifference;
    yy = clippedCenter - scale.*halfDifference;
    xy = scale.*b;
    covariance = reshape([xx,xy,xy,yy].',2,2,[]);
    determinant = xx.*yy-xy.*xy;
    inverseCovariance = reshape([yy,-xy,-xy,xx].'./determinant.',2,2,[]);
    logNormalization = -log(2*pi)-0.5.*log(determinant);
end

function components = concatenateComponents(componentSets)
% concatenateComponents: Flatten class-specific component arrays.
    components = emptyComponents();
    if isempty(componentSets)
        return;
    end
    totalComponents = sum(cellfun(@(item) item.numComponents, componentSets));
    if totalComponents == 0
        return;
    end
    components.semanticName = strings(totalComponents, 1);
    components.semanticId = zeros(totalComponents, 1, "uint8");
    components.cellLinIdx = zeros(totalComponents, 1, "int32");
    components.cellSub = zeros(totalComponents, 2, "int32");
    components.count = zeros(totalComponents, 1, "uint32");
    components.mean = zeros(totalComponents, 2);
    components.meanXYZ = zeros(totalComponents, 3);
    components.covarianceXYZ = zeros(3, 3, totalComponents);
    components.heightAvailable = false(totalComponents, 1);
    components.covariance = zeros(2, 2, totalComponents);
    components.invCovariance = zeros(2, 2, totalComponents);
    components.determinant = zeros(totalComponents, 1);
    components.logNormalizationConstant = zeros(totalComponents, 1);
    components.semanticProbability = zeros(totalComponents, 1);
    components.occupancyProbability = zeros(totalComponents, 1);
    components.unnormalizedWeight = zeros(totalComponents, 1);
    components.mixtureWeight = zeros(totalComponents, 1);
    writeStart = 1;
    for setIdx = 1:numel(componentSets)
        source = componentSets{setIdx};
        numSource = source.numComponents;
        if numSource < 1
            continue;
        end
        writeIdx = writeStart:(writeStart + numSource - 1);
        components.semanticName(writeIdx) = source.semanticName;
        components.semanticId(writeIdx) = source.semanticId;
        components.cellLinIdx(writeIdx) = source.cellLinIdx;
        components.cellSub(writeIdx, :) = source.cellSub;
        components.count(writeIdx) = source.count;
        components.mean(writeIdx, :) = source.mean;
        components.meanXYZ(writeIdx, :) = source.meanXYZ;
        components.covarianceXYZ(:, :, writeIdx) = source.covarianceXYZ;
        components.heightAvailable(writeIdx) = source.heightAvailable;
        components.covariance(:, :, writeIdx) = source.covariance;
        components.invCovariance(:, :, writeIdx) = source.invCovariance;
        components.determinant(writeIdx) = source.determinant;
        components.logNormalizationConstant(writeIdx) = ...
            source.logNormalizationConstant;
        components.semanticProbability(writeIdx) = source.semanticProbability;
        components.occupancyProbability(writeIdx) = source.occupancyProbability;
        components.unnormalizedWeight(writeIdx) = source.unnormalizedWeight;
        writeStart = writeStart + numSource;
    end
    components.numComponents = double(totalComponents);
end

function components = normalizeMixtureWeights(components)
% normalizeMixtureWeights: Normalize evidence-weighted occupancy mass over
% all semantic components.
    totalWeight = sum(double(components.unnormalizedWeight));
    if components.numComponents < 1
        return;
    end
    if totalWeight > 0 && isfinite(totalWeight)
        components.mixtureWeight = ...
            double(components.unnormalizedWeight) ./ totalWeight;
    else
        components.mixtureWeight = ...
            repmat(1 ./ components.numComponents, components.numComponents, 1);
    end
end

function layers = summarizeLayers(components, semanticNames)
% summarizeLayers: Store compact class offsets and probability mass only.
    layerTemplate = struct("semanticName", "", "semanticId", uint8(0), ...
        "componentIndices", zeros(0, 1, "int32"), "numComponents", 0, ...
        "pointSupportCount", 0, "mixtureWeight", 0);
    layers = repmat(layerTemplate, numel(semanticNames), 1);
    for semanticIdx = 1:numel(semanticNames)
        componentIdx = find(components.semanticId == uint8(semanticIdx));
        layers(semanticIdx).semanticName = semanticNames(semanticIdx);
        layers(semanticIdx).semanticId = uint8(semanticIdx);
        layers(semanticIdx).componentIndices = int32(componentIdx(:));
        layers(semanticIdx).numComponents = double(numel(componentIdx));
        layers(semanticIdx).pointSupportCount = ...
            double(sum(double(components.count(componentIdx))));
        layers(semanticIdx).mixtureWeight = ...
            double(sum(components.mixtureWeight(componentIdx)));
    end
end

function components = emptyComponents()
% emptyComponents: Create the stable sparse component schema.
    components = struct();
    components.semanticName = strings(0, 1);
    components.semanticId = zeros(0, 1, "uint8");
    components.cellLinIdx = zeros(0, 1, "int32");
    components.cellSub = zeros(0, 2, "int32");
    components.count = zeros(0, 1, "uint32");
    components.mean = zeros(0, 2);
    components.meanXYZ = zeros(0, 3);
    components.covarianceXYZ = zeros(3, 3, 0);
    components.heightAvailable = false(0, 1);
    components.covariance = zeros(2, 2, 0);
    components.invCovariance = zeros(2, 2, 0);
    components.determinant = zeros(0, 1);
    components.logNormalizationConstant = zeros(0, 1);
    components.semanticProbability = zeros(0, 1);
    components.occupancyProbability = zeros(0, 1);
    components.unnormalizedWeight = zeros(0, 1);
    components.mixtureWeight = zeros(0, 1);
    components.numComponents = 0;
end
