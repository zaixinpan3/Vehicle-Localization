function [stats, energyMaps] = buildCurbEnergyMaps(groundContext, cfg)
% buildCurbEnergyMaps: Convert the segmented ground points of one frame into
% per-cell curb evidence on the ground XY raster. Cell statistics (count,
% mean height, roughness) are aggregated, the height map is detrended with a
% NaN-aware box mean, and the curb-like signatures are scored as energy maps:
% height step, residual slope, curvature, roughness, relative height, and
% full-cell relief. The weighted total energy is then gated by local line
% shape (second-moment anisotropy, directional line support, and connected
% line-component support) and thresholded into the extracted curb cell mask.
%
% Input:
%   groundContext: struct with groundPoints [N x 3], groundCellLinIdx
%       [N x 1], and groundXYView fields
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   stats: struct of dense per-cell ground statistics maps
%   energyMaps: struct of per-feature energy maps, the base and total energy
%       maps, linearity maps, and the extractedMask of accepted curb cells
    groundPoints = double(groundContext.groundPoints);
    groundCellLinIdx = double(groundContext.groundCellLinIdx);
    groundXYView = groundContext.groundXYView;

    [stats, energyMaps] = buildGroundFeatureEnergyMaps( ...
        groundPoints, groundCellLinIdx, groundXYView, cfg);
end

function [stats, maps] = buildGroundFeatureEnergyMaps(groundPoints, groundCellLinIdx, xyView, cfg)
% buildGroundFeatureEnergyMaps: Convert segmented ground points into
% per-XY-cell height and roughness maps, detrend the height map with a
% NaN-aware box mean, evaluate realtime height-step, residual-slope,
% second-order curvature, roughness, relative-height, and full-pillar
% relief diagnostic energies, combine geometric evidence into a weighted
% base energy map with valid-neighborhood support gating, apply local
% weighted second-moment linearity energy gating, and extract cells only
% when energy, local line support, and center evidence agree.
%
% Input:
%   groundPoints: [N x 3] double ground-point coordinates
%   groundCellLinIdx: [N x 1] XY-cell linear indices in [Nx Ny] layout
%   xyView: XY view struct with geometry fields
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   stats: struct with countMap, heightMap, roughnessMap,
%       detrendedHeightMap, validMask, validNeighborCount, and supportMask
%   maps: struct with raw heightStep, residualSlope, curvature, roughness,
%       roughnessMeters, relativeHeight, fullCellRelief, totalBase,
%       linearity, total, and intermediate response maps
    mapSize = size(xyView.countMap);
    stats = buildGroundCellStats(groundPoints, groundCellLinIdx, mapSize);
    validMask = stats.countMap >= single(max(1, round(cfg.minPointsPerCell))) & isfinite(stats.heightMap);
    localMeanHeight = computeLocalMeanMap(stats.heightMap, validMask, cfg.detrendBoxRadiusCells);
    detrendedHeightMap = stats.heightMap - single(localMeanHeight);
    detrendedHeightMap(~validMask) = NaN;
    validNeighborCount = countValidNeighbors(validMask);
    supportMask = validMask & validNeighborCount >= max(0, round(cfg.minValidNeighborCount));

    [heightStepMap, heightStepTargetEnergy] = computeHeightStepFeatureMaps(detrendedHeightMap, validMask, cfg);
    [residualSlopeEnergy, residualSlopeAngleMap] = computeResidualSlopeEnergyMap(detrendedHeightMap, validMask, xyView.cellSize, cfg);
    [curvatureEnergy, curvatureMap] = computeCurvatureEnergyMap(detrendedHeightMap, validMask, xyView.cellSize, cfg);
    roughnessEnergy = computeRoughnessEnergyMap(stats.roughnessMap, validMask, cfg);
    [relativeHeightEnergy, relativeHeightMap] = computeRelativeHeightEnergyMap(stats.heightMap, supportMask, cfg);
    [fullCellReliefEnergy, ~] = computeFullCellReliefEnergyMap(xyView, cfg);
    curbSupportMask = supportMask;
    centerEvidenceMap = double(relativeHeightEnergy);

    totalEnergyBase = cfg.weights.heightStep .* double(heightStepTargetEnergy) ...
        + cfg.weights.residualSlope .* double(residualSlopeEnergy) ...
        + cfg.weights.curvature .* double(curvatureEnergy) ...
        + cfg.weights.roughness .* double(roughnessEnergy) ...
        + cfg.weights.relativeHeight .* double(relativeHeightEnergy) ...
        + cfg.weights.fullCellRelief .* double(fullCellReliefEnergy);
    totalEnergyBase = single(totalEnergyBase) .* single(curbSupportMask);
    [linearityMaps, totalEnergy] = applyLocalLinearityEnergyGate(totalEnergyBase, heightStepMap, stats.roughnessMap, curbSupportMask, xyView, cfg, centerEvidenceMap);

    stats.detrendedHeightMap = single(detrendedHeightMap);
    stats.validMask = validMask;
    stats.validNeighborCount = single(validNeighborCount);
    stats.supportMask = supportMask;
    stats.curbSupportMask = curbSupportMask;

    maps = struct();
    maps.heightStep = single(heightStepMap);
    maps.residualSlope = single(residualSlopeEnergy);
    maps.curvature = single(curvatureEnergy);
    maps.roughness = single(roughnessEnergy);
    maps.roughnessMeters = single(stats.roughnessMap);
    maps.relativeHeight = single(relativeHeightEnergy);
    maps.fullCellRelief = single(fullCellReliefEnergy);
    maps.totalBase = single(totalEnergyBase);
    maps.linearity = single(linearityMaps.linearity);
    maps.linearityMoment = single(linearityMaps.momentLinearity);
    maps.linearityDirectional = single(linearityMaps.directionalLinearity);
    maps.total = single(totalEnergy);
    maps.heightStepMeters = single(heightStepMap);
    maps.heightStepTargetEnergy = single(heightStepTargetEnergy);
    maps.relativeHeightMeters = single(relativeHeightMap);
    maps.fullCellReliefMeters = single(xyView.zRangeMap);
    maps.residualSlopeRadians = single(residualSlopeAngleMap);
    maps.curvatureResponse = single(curvatureMap);
    maps.linearityThetaRadians = single(linearityMaps.thetaRadians);
    maps.linearitySigma1Meters = single(linearityMaps.sigma1Meters);
    maps.linearitySigma2Meters = single(linearityMaps.sigma2Meters);
    maps.linearityAnisotropy = single(linearityMaps.anisotropy);
    maps.linearitySeedCount = single(linearityMaps.seedCount);
    maps.linearityValidRatio = single(linearityMaps.validRatio);
    maps.linearityEnergyMultiplier = single(linearityMaps.energyMultiplier);
    maps.linearityRefinementDelta = single(linearityMaps.refinementDelta);
    maps.linearityComponentScore = single(linearityMaps.componentScore);
    maps.linearityComponentPeakEnergy = single(linearityMaps.componentPeakEnergy);
    maps.linearityComponentSize = single(linearityMaps.componentSize);
    maps.linearityComponentCenterEvidence = single(linearityMaps.componentCenterEvidence);
    maps.linearityComponentFillGate = single(linearityMaps.componentFillGate);
    maps.linearityComponentFillSupportCount = single(linearityMaps.componentFillSupportCount);
    strongStandaloneMask = totalEnergy >= single(cfg.extractionEnergyThreshold) ...
        & single(linearityMaps.componentCenterEvidence) >= single(cfg.extractionCenterEvidenceMin) ...
        & single(heightStepMap) >= single(cfg.extractionHeightStepMinMeters) ...
        & single(stats.roughnessMap) >= single(cfg.extractionStandaloneRoughnessMinMeters);
    strongComponentMask = totalEnergy >= single(cfg.extractionEnergyThreshold) ...
        & single(linearityMaps.componentScore) >= single(cfg.extractionComponentScoreMin);
    baseStandaloneSeedMask = totalEnergyBase >= single(cfg.extractionStandaloneBaseEnergyThreshold) ...
        & single(linearityMaps.componentCenterEvidence) >= single(cfg.extractionCenterEvidenceMin) ...
        & single(heightStepMap) >= single(cfg.extractionHeightStepMinMeters) ...
        & single(stats.roughnessMap) >= single(cfg.extractionStandaloneRoughnessMinMeters);
    bridgeRadiusCells = max(0, round(double(cfg.extractionStandaloneBridgeRadiusCells)));
    baseStandaloneSeedCount = boxSumMap(double(baseStandaloneSeedMask), bridgeRadiusCells, bridgeRadiusCells) ...
        - double(baseStandaloneSeedMask);
    baseStandaloneBridgeMask = totalEnergyBase >= single(cfg.extractionStandaloneBridgeBaseEnergyThreshold) ...
        & single(linearityMaps.componentCenterEvidence) >= single(cfg.extractionStandaloneBridgeCenterEvidenceMin) ...
        & single(heightStepMap) >= single(cfg.extractionHeightStepMinMeters) ...
        & single(baseStandaloneSeedCount) >= single(cfg.extractionStandaloneBridgeMinSeedCount);
    baseStandaloneMask = baseStandaloneSeedMask | baseStandaloneBridgeMask;
    fillSupportedMask = totalEnergyBase >= single(cfg.extractionFillBaseEnergyThreshold) ...
        & single(heightStepMap) >= single(cfg.extractionFillHeightStepMinMeters) ...
        & single(linearityMaps.componentFillGate) >= single(cfg.extractionFillGateMin) ...
        & single(linearityMaps.componentFillSupportCount) >= single(cfg.extractionFillSupportCountMin) ...
        & single(linearityMaps.seedCount) >= single(cfg.extractionFillSeedCountMin) ...
        & single(linearityMaps.validRatio) >= single(cfg.extractionFillValidRatioMin);
    endpointSupportedMask = totalEnergyBase >= single(cfg.extractionEndpointBaseEnergyThreshold) ...
        & single(heightStepMap) >= single(cfg.extractionEndpointHeightStepMinMeters) ...
        & single(linearityMaps.componentCenterEvidence) >= single(cfg.extractionEndpointCenterEvidenceMin) ...
        & single(linearityMaps.componentScore) >= single(cfg.extractionComponentScoreMin) ...
        & single(linearityMaps.seedCount) >= single(cfg.extractionEndpointSeedCountMin) ...
        & single(linearityMaps.validRatio) >= single(cfg.extractionEndpointValidRatioMin);
    maps.extractionStandaloneSeedMask = baseStandaloneSeedMask;
    maps.extractionStandaloneBridgeMask = baseStandaloneBridgeMask;
    maps.extractionFillMask = fillSupportedMask;
    maps.extractionEndpointMask = endpointSupportedMask;
    strongEnergyMask = strongStandaloneMask | strongComponentMask | baseStandaloneMask | fillSupportedMask | endpointSupportedMask;
    supportedBaseMask = totalEnergyBase >= single(cfg.extractionBaseEnergyThreshold) ...
        & single(linearityMaps.linearity) >= single(cfg.extractionLinearityMin) ...
        & single(linearityMaps.componentScore) >= single(cfg.extractionComponentScoreMin) ...
        & single(linearityMaps.componentCenterEvidence) >= single(cfg.extractionCenterEvidenceMin) ...
        & single(heightStepMap) >= single(cfg.extractionHeightStepMinMeters);
    maps.extractedMask = (strongEnergyMask | supportedBaseMask) & curbSupportMask;
end

function stats = buildGroundCellStats(groundPoints, groundCellLinIdx, mapSize)
% buildGroundCellStats: Aggregate point-level ground samples into
% dense XY-cell maps for point count, mean height, within-cell vertical
% standard deviation using the same [Ny x Nx] map orientation returned by
% the active XY view builder.
%
% Input:
%   groundPoints: [N x 3] double ground-point coordinates
%   groundCellLinIdx: [N x 1] XY-cell linear indices in [Nx Ny] layout
%   mapSize: [1 x 2] output map size [Ny Nx]
%
% Output:
%   stats: struct with countMap, heightMap, and roughnessMap
    ny = double(mapSize(1));
    nx = double(mapSize(2));
    numCells = nx * ny;
    countVec = zeros(numCells, 1);
    heightVec = NaN(numCells, 1);
    roughnessVec = NaN(numCells, 1);

    if ~isempty(groundPoints) && numCells > 0
        cellIdx = double(groundCellLinIdx(:));
        zVals = double(groundPoints(:, 3));
        validPoint = isfinite(cellIdx) & cellIdx >= 1 & cellIdx <= numCells & cellIdx == floor(cellIdx) & isfinite(zVals);
        if any(validPoint)
            cellIdx = cellIdx(validPoint);
            zVals = zVals(validPoint);
            countVec = accumarray(cellIdx, 1, [numCells, 1], @sum, 0);
            sumZVec = accumarray(cellIdx, zVals, [numCells, 1], @sum, 0);
            sumZZVec = accumarray(cellIdx, zVals .* zVals, [numCells, 1], @sum, 0);
            occupied = countVec > 0;
            heightVec(occupied) = sumZVec(occupied) ./ countVec(occupied);
            varianceVec = zeros(numCells, 1);
            varianceVec(occupied) = max((sumZZVec(occupied) ./ countVec(occupied)) - (heightVec(occupied) .* heightVec(occupied)), 0);
            roughnessVec(occupied) = sqrt(varianceVec(occupied));
        end
    end

    stats = struct();
    stats.countMap = reshape(single(countVec), nx, ny).';
    stats.heightMap = reshape(single(heightVec), nx, ny).';
    stats.roughnessMap = reshape(single(roughnessVec), nx, ny).';
end

function localMeanMap = computeLocalMeanMap(valueMap, validMask, radiusCells)
% computeLocalMeanMap: Compute a NaN-aware local box mean for a
% raster map by summing only valid cells and dividing by the local valid
% count. Invalid output locations are returned as NaN so detrended maps
% keep the same effective support as their source map.
%
% Input:
%   valueMap: [Ny x Nx] numeric map
%   validMask: [Ny x Nx] logical valid-cell support
%   radiusCells: scalar nonnegative box radius in cells
%
% Output:
%   localMeanMap: [Ny x Nx] double local mean map
    radiusCells = max(0, round(double(radiusCells)));
    kernel = ones((2 * radiusCells) + 1, (2 * radiusCells) + 1);
    valueMap = double(valueMap);
    validMask = logical(validMask) & isfinite(valueMap);
    valueMap(~validMask) = 0;
    valueSum = conv2(valueMap, kernel, "same");
    validCount = conv2(double(validMask), kernel, "same");
    localMeanMap = NaN(size(valueMap));
    hasSupport = validCount > 0;
    localMeanMap(hasSupport) = valueSum(hasSupport) ./ validCount(hasSupport);
end

function [heightStepMap, targetEnergyMap] = computeHeightStepFeatureMaps(detrendedHeightMap, validMask, cfg)
% computeHeightStepFeatureMaps: Evaluate the detrended height-step
% feature by taking the largest absolute height difference over the 8-cell
% neighborhood as the raw S_h map, while separately producing the Gaussian
% target score used only by the weighted total energy.
%
% Input:
%   detrendedHeightMap: [Ny x Nx] single/double detrended height map
%   validMask: [Ny x Nx] logical valid-cell support
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   heightStepMap: [Ny x Nx] single maximum neighbor height step in meters
%   targetEnergyMap: [Ny x Nx] single target height-step score in [0, 1]
    if isfield(cfg,"useNativeKernels") && cfg.useNativeKernels
        heightStepMap = perceptionKernelsMex('neighborDifference',double(detrendedHeightMap),logical(validMask));
    else
        heightStepMap = maxNeighborAbsDiffMap(double(detrendedHeightMap), validMask);
    end
    sigmaH = max(double(cfg.heightStepSigmaMeters), eps);
    targetEnergyMap = exp(-((heightStepMap - double(cfg.heightStepTargetMeters)).^2) ./ (2 * sigmaH * sigmaH));
    inBand = heightStepMap >= double(cfg.heightStepMinMeters) & heightStepMap <= double(cfg.heightStepMaxMeters);
    targetEnergyMap = single(targetEnergyMap .* double(inBand) .* double(validMask));
    heightStepMap = heightStepMap .* double(validMask);
    heightStepMap = single(heightStepMap);
end

function [energyMap, angleMap] = computeResidualSlopeEnergyMap(detrendedHeightMap, validMask, cellSize, cfg)
% computeResidualSlopeEnergyMap: Approximate a residual normal-change
% response by central-differencing the detrended height map in X and Y,
% converting the gradient magnitude to slope angle, and applying a
% Gaussian target score around the configured curb-like residual slope.
%
% Input:
%   detrendedHeightMap: [Ny x Nx] single/double detrended height map
%   validMask: [Ny x Nx] logical valid-cell support
%   cellSize: [1 x 2] XY cell size in meters
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   energyMap: [Ny x Nx] single residual-slope energy in [0, 1]
%   angleMap: [Ny x Nx] single residual slope angle in radians
    h = double(detrendedHeightMap);
    dx = max(double(cellSize(1)), eps);
    dy = max(double(cellSize(2)), eps);
    [east, eastValid] = shiftMap(h, validMask, 0, 1);
    [west, westValid] = shiftMap(h, validMask, 0, -1);
    [north, northValid] = shiftMap(h, validMask, 1, 0);
    [south, southValid] = shiftMap(h, validMask, -1, 0);
    gradientValid = logical(validMask) & eastValid & westValid & northValid & southValid;
    gx = zeros(size(h));
    gy = zeros(size(h));
    gx(gradientValid) = (east(gradientValid) - west(gradientValid)) ./ (2 * dx);
    gy(gradientValid) = (north(gradientValid) - south(gradientValid)) ./ (2 * dy);
    angleMap = atan(hypot(gx, gy));
    thetaTarget = deg2rad(double(cfg.residualSlopeTargetDeg));
    thetaSigma = max(deg2rad(double(cfg.residualSlopeSigmaDeg)), eps);
    energyMap = exp(-((angleMap - thetaTarget).^2) ./ (2 * thetaSigma * thetaSigma));
    energyMap = single(energyMap .* double(gradientValid));
    angleMap = single(angleMap);
end

function [energyMap, curvatureMap] = computeCurvatureEnergyMap(detrendedHeightMap, validMask, cellSize, cfg)
% computeCurvatureEnergyMap: Evaluate a second-order curvature-like
% response from axial and diagonal finite differences of the detrended
% height map, using the sum of absolute hxx, hyy, and twice hxy as a fast
% raster substitute for per-cell PCA curvature and scoring closeness to a
% configured curb-like target response.
%
% Input:
%   detrendedHeightMap: [Ny x Nx] single/double detrended height map
%   validMask: [Ny x Nx] logical valid-cell support
%   cellSize: [1 x 2] XY cell size in meters
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   energyMap: [Ny x Nx] single curvature energy in [0, 1]
%   curvatureMap: [Ny x Nx] single curvature-like response
    h = double(detrendedHeightMap);
    dx = max(double(cellSize(1)), eps);
    dy = max(double(cellSize(2)), eps);
    [east, eastValid] = shiftMap(h, validMask, 0, 1);
    [west, westValid] = shiftMap(h, validMask, 0, -1);
    [north, northValid] = shiftMap(h, validMask, 1, 0);
    [south, southValid] = shiftMap(h, validMask, -1, 0);
    [xPyP, xPyPValid] = shiftMap(h, validMask, 1, 1);
    [xPyM, xPyMValid] = shiftMap(h, validMask, -1, 1);
    [xMyP, xMyPValid] = shiftMap(h, validMask, 1, -1);
    [xMyM, xMyMValid] = shiftMap(h, validMask, -1, -1);
    curvatureValid = logical(validMask) & eastValid & westValid & northValid & southValid ...
        & xPyPValid & xPyMValid & xMyPValid & xMyMValid;
    hxx = zeros(size(h));
    hyy = zeros(size(h));
    hxy = zeros(size(h));
    hxx(curvatureValid) = (east(curvatureValid) - (2 .* h(curvatureValid)) + west(curvatureValid)) ./ (dx * dx);
    hyy(curvatureValid) = (north(curvatureValid) - (2 .* h(curvatureValid)) + south(curvatureValid)) ./ (dy * dy);
    hxy(curvatureValid) = (xPyP(curvatureValid) - xPyM(curvatureValid) - xMyP(curvatureValid) + xMyM(curvatureValid)) ./ (4 * dx * dy);
    curvatureMap = (abs(hxx) + abs(hyy) + (2 .* abs(hxy))) .* double(curvatureValid);
    kTarget = double(cfg.curvatureTarget);
    kSigma = max(double(cfg.curvatureSigma), eps);
    energyMap = exp(-((curvatureMap - kTarget).^2) ./ (2 * kSigma * kSigma));
    energyMap = single(energyMap .* double(curvatureValid));
    curvatureMap = single(curvatureMap);
end

function energyMap = computeRoughnessEnergyMap(roughnessMap, validMask, cfg)
% computeRoughnessEnergyMap: Score within-cell vertical standard
% deviation by closeness to the configured curb-like roughness target so
% moderate mixed road, curb-face, and sidewalk cells score higher than
% both flat cells and overly rough cells.
%
% Input:
%   roughnessMap: [Ny x Nx] numeric cell-local Z standard deviation
%   validMask: [Ny x Nx] logical valid-cell support
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   energyMap: [Ny x Nx] single roughness energy in [0, 1]
    roughnessTarget = double(cfg.roughnessTargetMeters);
    roughnessSigma = max(double(cfg.roughnessSigmaMeters), eps);
    energyMap = exp(-((double(roughnessMap) - roughnessTarget).^2) ./ (2 * roughnessSigma * roughnessSigma));
    energyMap(~validMask | ~isfinite(energyMap)) = 0;
    energyMap = single(energyMap);
end

function [energyMap, relativeHeightMap] = computeRelativeHeightEnergyMap(heightMap, validMask, cfg)
% computeRelativeHeightEnergyMap: Score cells whose representative
% ground height rises above the local lower-envelope height, preserving
% raised curb or sidewalk ribbons that become weak after mean detrending
% because they are height-continuous along the curb direction.
%
% Input:
%   heightMap: [Ny x Nx] numeric representative ground height map
%   validMask: [Ny x Nx] logical valid-cell support
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   energyMap: [Ny x Nx] single relative-height energy in [0, 1]
%   relativeHeightMap: [Ny x Nx] single height above local minimum in meters
    localMinimumMap = computeLocalMinimumMap(heightMap, validMask, cfg.relativeHeightRadiusCells);
    relativeHeightMap = double(heightMap) - localMinimumMap;
    relativeHeightMap(~validMask | ~isfinite(relativeHeightMap) | relativeHeightMap < 0) = 0;
    energyMap = smoothStepMap(relativeHeightMap, cfg.relativeHeightMinMeters, cfg.relativeHeightSaturatedMeters);
    energyMap(~validMask | ~isfinite(energyMap)) = 0;
    energyMap = single(energyMap);
    relativeHeightMap = single(relativeHeightMap);
end

function [energyMap, supportMask] = computeFullCellReliefEnergyMap(xyView, cfg)
% computeFullCellReliefEnergyMap: Score XY cells whose full retained
% point pillar has enough vertical relief to indicate a curb face or
% raised boundary even when the ground-only segmentation does not keep the
% cell as supported ground.
%
% Input:
%   xyView: XY view struct with countMap and zRangeMap fields
%   cfg: struct from groundFeatureConfig().curb
%
% Output:
%   energyMap: [Ny x Nx] single full-pillar relief energy in [0, 1]
%   supportMask: [Ny x Nx] logical support for full-pillar relief evidence
    countMap = double(xyView.countMap);
    zRangeMap = double(xyView.zRangeMap);
    supportMask = countMap >= max(1, round(double(cfg.fullCellReliefMinPoints))) & isfinite(zRangeMap);
    energyMap = smoothStepMap(zRangeMap, cfg.fullCellReliefMinMeters, cfg.fullCellReliefSaturatedMeters);
    energyMap(~supportMask | ~isfinite(energyMap)) = 0;
    energyMap = single(energyMap);
end

function minimumMap = computeLocalMinimumMap(valueMap, validMask, radiusCells)
% computeLocalMinimumMap: Compute a finite local minimum over a square
% raster neighborhood while ignoring invalid cells so relative-height curb
% evidence can compare each cell against the nearby lower road surface.
%
% Input:
%   valueMap: [Ny x Nx] numeric map
%   validMask: [Ny x Nx] logical valid-cell support
%   radiusCells: scalar nonnegative box radius in cells
%
% Output:
%   minimumMap: [Ny x Nx] double local finite minimum with NaN where no
%       valid cell exists inside the neighborhood
    valueMap = double(valueMap);
    validMask = logical(validMask) & isfinite(valueMap);
    radiusCells = max(0, round(double(radiusCells)));
    valueMap(~validMask) = inf;
    minimumMap = movmin(valueMap, [radiusCells, radiusCells], 1, "Endpoints", "shrink");
    minimumMap = movmin(minimumMap, [radiusCells, radiusCells], 2, "Endpoints", "shrink");
    minimumMap(~isfinite(minimumMap)) = NaN;
end

function [linearityMaps, gatedEnergy] = applyLocalLinearityEnergyGate(totalEnergyBase, heightStepMap, roughnessMap, supportMask, xyView, cfg, centerEvidenceMap)
% applyLocalLinearityEnergyGate: Refine the base curb energy map by
% treating high-energy supported cells inside a local window as weighted XY
% samples, measuring their second-moment anisotropy in metric coordinates,
% gating it by length, width, support, validity, center energy, and
% optional external center-evidence constraints, and multiplying base
% energy by a powered linearity factor.
%
% Input:
%   totalEnergyBase: [Ny x Nx] single/double supported base energy in [0, 1]
%   heightStepMap: [Ny x Nx] single/double raw neighbor height step in meters
%   roughnessMap: [Ny x Nx] single/double raw within-cell height spread in
%       meters
%   supportMask: [Ny x Nx] logical valid-neighborhood support mask
%   xyView: XY view struct with x/y centers and
%       cellSize fields
%   cfg: struct from groundFeatureConfig().curb with linearity parameters
%   centerEvidenceMap: [Ny x Nx] optional numeric center evidence in [0, 1]
%
% Output:
%   linearityMaps: struct with linearity, thetaRadians, sigma1Meters,
%       sigma2Meters, anisotropy, seedCount, validRatio, energyMultiplier,
%       refinementDelta, componentScore, componentPeakEnergy,
%       componentSize, componentCenterEvidence, componentFillGate, and
%       componentFillSupportCount maps
%   gatedEnergy: [Ny x Nx] single gated total energy map in [0, 1]
    e = double(min(max(single(totalEnergyBase), single(0)), single(1)));
    supportMask = logical(supportMask) & isfinite(e);
    e(~supportMask) = 0;
    mapSize = size(e);
    linearityMaps = struct();
    linearityMaps.linearity = zeros(mapSize, "single");
    linearityMaps.momentLinearity = zeros(mapSize, "single");
    linearityMaps.directionalLinearity = zeros(mapSize, "single");
    linearityMaps.thetaRadians = zeros(mapSize, "single");
    linearityMaps.sigma1Meters = zeros(mapSize, "single");
    linearityMaps.sigma2Meters = zeros(mapSize, "single");
    linearityMaps.anisotropy = zeros(mapSize, "single");
    linearityMaps.seedCount = zeros(mapSize, "single");
    linearityMaps.validRatio = zeros(mapSize, "single");
    linearityMaps.energyMultiplier = ones(mapSize, "single");
    linearityMaps.refinementDelta = zeros(mapSize, "single");
    linearityMaps.componentScore = zeros(mapSize, "single");
    linearityMaps.componentPeakEnergy = zeros(mapSize, "single");
    linearityMaps.componentSize = zeros(mapSize, "single");
    linearityMaps.componentCenterEvidence = zeros(mapSize, "single");
    linearityMaps.componentFillGate = zeros(mapSize, "single");
    linearityMaps.componentFillSupportCount = zeros(mapSize, "single");
    gatedEnergy = single(e);

    if ~logical(cfg.linearityEnergyGateEnabled)
        return;
    end

    tau = min(max(double(cfg.linearityEnergyThreshold), 0), 1 - eps);
    weightPower = max(double(cfg.linearityWeightPower), eps);
    rowRadius = max(1, round(double(cfg.linearityRadiusMeters) ./ max(double(xyView.cellSize(2)), eps)));
    colRadius = max(1, round(double(cfg.linearityRadiusMeters) ./ max(double(xyView.cellSize(1)), eps)));
    [xGrid, yGrid] = meshgrid(double(xyView.xCenters(:).'), double(xyView.yCenters(:)));
    heightStepMap = double(heightStepMap);
    roughnessMap = double(roughnessMap);
    heightStepMap(~supportMask | ~isfinite(heightStepMap)) = 0;
    roughnessMap(~supportMask | ~isfinite(roughnessMap)) = 0;
    if nargin < 7 || isempty(centerEvidenceMap)
        centerEvidenceMap = zeros(size(e));
    end
    centerEvidenceMap = double(centerEvidenceMap);
    centerEvidenceMap(~supportMask | ~isfinite(centerEvidenceMap)) = 0;
    centerEvidenceMap = min(max(centerEvidenceMap, 0), 1);
    heightStepEvidence = smoothStepMap(heightStepMap, cfg.centerHeightStepMinMeters, cfg.centerHeightStepSaturatedMeters);
    roughnessEvidence = smoothStepMap(roughnessMap, cfg.centerRoughnessMinMeters, cfg.centerRoughnessSaturatedMeters);
    componentCenterEvidence = max(max(heightStepEvidence, roughnessEvidence), centerEvidenceMap);
    softWeight = max((e - tau) ./ max(1 - tau, eps), 0) .^ weightPower;
    weightMap = softWeight .* double(supportMask);
    seedMap = double(supportMask & e > tau);

    m00 = boxSumMap(weightMap, rowRadius, colRadius);
    m10 = boxSumMap(weightMap .* xGrid, rowRadius, colRadius);
    m01 = boxSumMap(weightMap .* yGrid, rowRadius, colRadius);
    m20 = boxSumMap(weightMap .* xGrid .* xGrid, rowRadius, colRadius);
    m02 = boxSumMap(weightMap .* yGrid .* yGrid, rowRadius, colRadius);
    m11 = boxSumMap(weightMap .* xGrid .* yGrid, rowRadius, colRadius);
    seedCount = boxSumMap(seedMap, rowRadius, colRadius);
    validCount = boxSumMap(double(supportMask), rowRadius, colRadius);
    windowCount = boxWindowCountMap(mapSize, rowRadius, colRadius);
    validRatio = validCount ./ max(windowCount, eps);

    momentValid = m00 >= max(double(cfg.linearityMinWeightSum), 0);
    invM00 = 1 ./ max(m00, eps);
    muX = m10 .* invM00;
    muY = m01 .* invM00;
    cxx = max((m20 .* invM00) - (muX .* muX), 0);
    cyy = max((m02 .* invM00) - (muY .* muY), 0);
    cxy = (m11 .* invM00) - (muX .* muY);
    traceC = max(cxx + cyy, 0);
    deltaC = sqrt(max(((cxx - cyy) .* (cxx - cyy)) + (4 .* cxy .* cxy), 0));
    lambda1 = max(0.5 .* (traceC + deltaC), 0);
    lambda2 = max(min(0.5 .* (traceC - deltaC), lambda1), 0);
    sigma1 = sqrt(lambda1);
    sigma2 = sqrt(lambda2);
    anisotropy = (lambda1 - lambda2) ./ max(lambda1 + lambda2, eps);
    anisotropy = min(max(anisotropy, 0), 1);

    lengthGate = smoothStepMap(sigma1, cfg.linearityMinLengthMeters, cfg.linearitySaturatedLengthMeters);
    widthGate = 1 - smoothStepMap(sigma2, cfg.linearityMaxWidthMeters, cfg.linearityRejectWidthMeters);
    supportGate = smoothStepMap(seedCount, cfg.linearityMinSeedCount, cfg.linearitySaturatedSeedCount);
    validGate = smoothStepMap(validRatio, cfg.linearityMinValidRatio, cfg.linearitySaturatedValidRatio);
    centerGate = smoothStepMap(e, cfg.linearityCenterEnergyMin, cfg.linearityCenterEnergySaturated);
    anisotropyPower = max(double(cfg.linearityAnisotropyPower), eps);
    momentLinearity = (anisotropy .^ anisotropyPower) .* lengthGate .* widthGate .* supportGate .* validGate .* centerGate;
    momentLinearity(~supportMask | ~momentValid | ~isfinite(momentLinearity)) = 0;
    momentLinearity = min(max(momentLinearity, 0), 1);
    [componentScore, componentPeakEnergy, componentSize] = computeLineComponentSupportMaps(e, supportMask, xGrid, yGrid, cfg);
    directionalLinearity = computeDirectionalLinearityMap(e, supportMask, componentScore, cfg);
    linearity = max(momentLinearity, directionalLinearity);
    linearity(~supportMask | ~isfinite(linearity)) = 0;
    linearity = min(max(linearity, 0), 1);

    energyPower = max(double(cfg.linearityEnergyPower), eps);
    energyFloor = min(max(double(cfg.linearityEnergyFloor), 0), 1);
    energyMultiplier = energyFloor + ((1 - energyFloor) .* (linearity .^ energyPower));
    gated = e .* energyMultiplier;
    centerEvidenceFloor = min(max(double(cfg.centerEvidenceEnergyFloor), 0), 1);
    gated = gated .* (centerEvidenceFloor + ((1 - centerEvidenceFloor) .* componentCenterEvidence));
    if logical(cfg.componentLineSupportEnabled)
        componentFillRadius = max(0, round(double(cfg.componentFillSupportRadiusCells)));
        strongLinearitySeed = double(supportMask) .* double(linearity >= double(cfg.componentFillLinearityThreshold));
        componentFillSupportCount = boxSumMap(strongLinearitySeed, componentFillRadius, componentFillRadius);
        componentFillGate = smoothStepMap(componentFillSupportCount, cfg.componentFillMinStrongLinearityCount, cfg.componentFillSaturatedStrongLinearityCount);
        componentFill = max(double(cfg.componentFillWeight), 0) .* componentScore .* componentPeakEnergy .* componentCenterEvidence .* componentFillGate;
        gated = max(gated, componentFill);
    else
        componentFillGate = zeros(size(e));
        componentFillSupportCount = zeros(size(e));
    end
    gated(~supportMask | ~isfinite(gated)) = 0;
    gated = min(max(gated, 0), 1);
    if logical(cfg.normalizeTotalEnergyToUnitMax)
        supportedEnergy = gated(logical(supportMask) & isfinite(gated));
        if ~isempty(supportedEnergy)
            maxSupportedEnergy = max(supportedEnergy);
            if maxSupportedEnergy > eps
                gated = gated ./ maxSupportedEnergy;
            end
        end
    end
    theta = 0.5 .* atan2(2 .* cxy, cxx - cyy);
    theta(~supportMask | ~momentValid | ~isfinite(theta)) = 0;

    linearityMaps.linearity = single(linearity);
    linearityMaps.momentLinearity = single(momentLinearity);
    linearityMaps.directionalLinearity = single(directionalLinearity);
    linearityMaps.thetaRadians = single(theta);
    linearityMaps.sigma1Meters = single(sigma1 .* double(supportMask) .* double(momentValid));
    linearityMaps.sigma2Meters = single(sigma2 .* double(supportMask) .* double(momentValid));
    linearityMaps.anisotropy = single(anisotropy .* double(supportMask) .* double(momentValid));
    linearityMaps.seedCount = single(seedCount);
    linearityMaps.validRatio = single(validRatio);
    linearityMaps.energyMultiplier = single(energyMultiplier);
    linearityMaps.refinementDelta = single(gated - e);
    linearityMaps.componentScore = single(componentScore);
    linearityMaps.componentPeakEnergy = single(componentPeakEnergy);
    linearityMaps.componentSize = single(componentSize);
    linearityMaps.componentCenterEvidence = single(componentCenterEvidence);
    linearityMaps.componentFillGate = single(componentFillGate);
    linearityMaps.componentFillSupportCount = single(componentFillSupportCount);
    gatedEnergy = single(gated);
end

function directionalLinearity = computeDirectionalLinearityMap(baseEnergy, supportMask, componentScore, cfg)
% computeDirectionalLinearityMap: Estimate local line support by
% summing high-base-energy supported cells with compact convolution
% kernels in four undirected raster directions and converting the maximum
% directional support into a bounded linearity score. This complements the
% weighted second-moment linearity estimate for short curb endpoints whose
% local covariance window can look too wide.
%
% Input:
%   baseEnergy: [Ny x Nx] double supported base energy in [0, 1]
%   supportMask: [Ny x Nx] logical support mask
%   componentScore: [Ny x Nx] double component-level line score in [0, 1]
%   cfg: struct from groundFeatureConfig().curb with directional parameters
%
% Output:
%   directionalLinearity: [Ny x Nx] double directional line support score
    directionalLinearity = zeros(size(baseEnergy));
    if ~logical(cfg.directionalLinearityEnabled)
        return;
    end

    radiusCells = max(1, round(double(cfg.directionalLineRadiusCells)));
    highSeed = double(logical(supportMask) & baseEnergy >= double(cfg.directionalLineEnergyThreshold));
    centerEnergyGate = smoothStepMap(baseEnergy, cfg.directionalLineCenterEnergyMin, cfg.directionalLineCenterEnergySaturated);
    componentGate = smoothStepMap(componentScore, cfg.directionalLineComponentScoreMin, cfg.directionalLineComponentScoreSaturated);
    if isfield(cfg,"useNativeKernels") && cfg.useNativeKernels
        % Only positive final gates require directional neighborhood counts.
        active=logical(supportMask) & centerEnergyGate>0 & componentGate>0;
        best=perceptionKernelsMex('directionalSupport',logical(highSeed),active, ...
            [radiusCells,cfg.directionalLineMinCount,cfg.directionalLineSaturatedCount, ...
             cfg.directionalLineThinnessMin,cfg.directionalLineThinnessSaturated]);
        directionalLinearity=best.*centerEnergyGate.*componentGate.*double(supportMask);
        directionalLinearity(~isfinite(directionalLinearity))=0;
        directionalLinearity=min(max(directionalLinearity,0),1);
        return;
    end
    kernelSet = resolveDirectionalLinearityKernels(size(baseEnergy), radiusCells);
    bestDirectionalScore = zeros(size(baseEnergy));
    for k = 1:numel(kernelSet)
        lineKernel = kernelSet(k).lineKernel;
        sideKernelPositive = kernelSet(k).sideKernelPositive;
        sideKernelNegative = kernelSet(k).sideKernelNegative;
        lineCount = conv2(highSeed, lineKernel, "same");
        sideCountPositive = conv2(highSeed, sideKernelPositive, "same");
        sideCountNegative = conv2(highSeed, sideKernelNegative, "same");
        directionValid = kernelSet(k).directionValid;
        sideCount = min(sideCountPositive, sideCountNegative);
        thinness = lineCount ./ max(lineCount + sideCount, eps);
        lineCountGate = smoothStepMap(lineCount, cfg.directionalLineMinCount, cfg.directionalLineSaturatedCount);
        thinnessGate = smoothStepMap(thinness, cfg.directionalLineThinnessMin, cfg.directionalLineThinnessSaturated);
        directionalScore = lineCountGate .* thinnessGate;
        directionalScore(~directionValid | ~isfinite(directionalScore)) = 0;
        bestDirectionalScore = max(bestDirectionalScore, directionalScore);
    end

    directionalLinearity = bestDirectionalScore .* centerEnergyGate .* componentGate .* double(supportMask);
    directionalLinearity(~isfinite(directionalLinearity)) = 0;
    directionalLinearity = min(max(directionalLinearity, 0), 1);
end

function kernelSet = resolveDirectionalLinearityKernels(mapSize, radiusCells)
% resolveDirectionalLinearityKernels: Build and cache the four
% directional line-support convolution kernels plus their edge-validity
% masks for a specific map size and search radius. Reusing these kernels
% avoids repeated allocation and repeated full-map convolutions across
% profiling runs and adjacent frames with identical grid geometry.
%
% Input:
%   mapSize: [1 x 2] raster size [Ny Nx]
%   radiusCells: scalar line-support radius in cells
%
% Output:
%   kernelSet: struct array with lineKernel, sideKernelPositive,
%       sideKernelNegative, and directionValid fields
    mapSize = double(mapSize(1:2));
    radiusCells = max(1, round(double(radiusCells)));
    persistent cachedMapSize cachedRadiusCells cachedKernelSet
    if ~isempty(cachedKernelSet) && isequal(cachedMapSize, mapSize) && cachedRadiusCells == radiusCells
        kernelSet = cachedKernelSet;
        return;
    end

    directions = [0, 1; 1, 0; 1, 1; 1, -1];
    fullMap = ones(mapSize);
    kernelSet = repmat(struct("lineKernel", [], "sideKernelPositive", [], ...
        "sideKernelNegative", [], "directionValid", false(mapSize)), size(directions, 1), 1);
    for k = 1:size(directions, 1)
        rowStep = directions(k, 1);
        colStep = directions(k, 2);
        normalRowStep = -colStep;
        normalColStep = rowStep;
        lineOffsets = (-radiusCells:radiusCells).';
        lineRowOffsets = lineOffsets .* rowStep;
        lineColOffsets = lineOffsets .* colStep;
        sideRowOffsetsPositive = normalRowStep + lineRowOffsets;
        sideColOffsetsPositive = normalColStep + lineColOffsets;
        sideRowOffsetsNegative = -normalRowStep + lineRowOffsets;
        sideColOffsetsNegative = -normalColStep + lineColOffsets;
        lineKernel = buildShiftSumKernel(lineRowOffsets, lineColOffsets);
        sideKernelPositive = buildShiftSumKernel(sideRowOffsetsPositive, sideColOffsetsPositive);
        sideKernelNegative = buildShiftSumKernel(sideRowOffsetsNegative, sideColOffsetsNegative);
        directionValid = conv2(fullMap, lineKernel, "same") >= sum(lineKernel(:)) ...
            & conv2(fullMap, sideKernelPositive, "same") >= sum(sideKernelPositive(:)) ...
            & conv2(fullMap, sideKernelNegative, "same") >= sum(sideKernelNegative(:));
        kernelSet(k).lineKernel = lineKernel;
        kernelSet(k).sideKernelPositive = sideKernelPositive;
        kernelSet(k).sideKernelNegative = sideKernelNegative;
        kernelSet(k).directionValid = directionValid;
    end

    cachedMapSize = mapSize;
    cachedRadiusCells = radiusCells;
    cachedKernelSet = kernelSet;
end

function kernel = buildShiftSumKernel(rowOffsets, colOffsets)
% buildShiftSumKernel: Build a compact convolution kernel that sums
% source cells at the same row and column offsets used by shiftMap so
% repeated raster shifts can be evaluated as one vectorized convolution.
%
% Input:
%   rowOffsets: [N x 1] numeric source row offsets from each target cell
%   colOffsets: [N x 1] numeric source column offsets from each target cell
%
% Output:
%   kernel: dense numeric convolution kernel for conv2(..., "same")
    rowOffsets = round(double(rowOffsets(:)));
    colOffsets = round(double(colOffsets(:)));
    rowRadius = max(abs(rowOffsets));
    colRadius = max(abs(colOffsets));
    kernelSize = [(2 * rowRadius) + 1, (2 * colRadius) + 1];
    centerRow = rowRadius + 1;
    centerCol = colRadius + 1;
    kernelIdx = sub2ind(kernelSize, centerRow - rowOffsets, centerCol - colOffsets);
    kernel = accumarray(kernelIdx, 1, [prod(kernelSize), 1], @sum, 0);
    kernel = reshape(kernel, kernelSize);
end

function [componentScoreMap, componentPeakMap, componentSizeMap] = computeLineComponentSupportMaps(baseEnergy, supportMask, xGrid, yGrid, cfg)
% computeLineComponentSupportMaps: Label connected high-energy cells
% and score each component by weighted metric anisotropy, component cell
% count, principal length, and cross-line width so long curb-like
% connected structures can support weak cells on the same line while
% compact or short components stay low.
%
% Input:
%   baseEnergy: [Ny x Nx] double supported base energy in [0, 1]
%   supportMask: [Ny x Nx] logical support mask
%   xGrid: [Ny x Nx] double metric x-coordinate at each cell
%   yGrid: [Ny x Nx] double metric y-coordinate at each cell
%   cfg: struct from groundFeatureConfig().curb with component parameters
%
% Output:
%   componentScoreMap: [Ny x Nx] double component line support score
%   componentPeakMap: [Ny x Nx] double maximum base energy in component
%   componentSizeMap: [Ny x Nx] double component cell count
    componentScoreMap = zeros(size(baseEnergy));
    componentPeakMap = zeros(size(baseEnergy));
    componentSizeMap = zeros(size(baseEnergy));
    if ~logical(cfg.componentLineSupportEnabled)
        return;
    end

    candidateMask = logical(supportMask) & baseEnergy >= double(cfg.componentSeedEnergyThreshold);
    components = connectedComponents8(candidateMask);
    numComponents = numel(components);
    if numComponents == 0, return; end
    counts = cellfun(@numel,components);
    indices = vertcat(components{:});
    groups = repelem((1:numComponents).',counts);
    weights = double(baseEnergy(indices));
    weightSum = accumarray(groups,weights,[numComponents 1],@sum,0);
    denominator = max(weightSum,eps);
    x = double(xGrid(indices)); y = double(yGrid(indices));
    muX = accumarray(groups,weights.*x,[numComponents 1],@sum,0)./denominator;
    muY = accumarray(groups,weights.*y,[numComponents 1],@sum,0)./denominator;
    x = x-muX(groups); y = y-muY(groups);
    cxx = accumarray(groups,weights.*x.*x,[numComponents 1],@sum,0)./denominator;
    cyy = accumarray(groups,weights.*y.*y,[numComponents 1],@sum,0)./denominator;
    cxy = accumarray(groups,weights.*x.*y,[numComponents 1],@sum,0)./denominator;
    traceC = max(cxx+cyy,0);
    deltaC = sqrt(max((cxx-cyy).*(cxx-cyy)+4.*cxy.*cxy,0));
    lambda1 = max(0.5.*(traceC+deltaC),0);
    lambda2 = max(min(0.5.*(traceC-deltaC),lambda1),0);
    anisotropy = min(max((lambda1-lambda2)./max(lambda1+lambda2,eps),0),1);
    score = (anisotropy.^max(double(cfg.componentAnisotropyPower),eps)) ...
        .*smoothStepMap(counts,cfg.componentMinCells,cfg.componentSaturatedCells) ...
        .*smoothStepMap(sqrt(lambda1),cfg.componentMinLengthMeters,cfg.componentSaturatedLengthMeters) ...
        .*(1-smoothStepMap(sqrt(lambda2),cfg.componentMaxWidthMeters,cfg.componentRejectWidthMeters));
    score = min(max(score,0),1);
    peak = accumarray(groups,weights,[numComponents 1],@max,0);
    valid = weightSum(groups)>0;
    componentScoreMap(indices(valid)) = score(groups(valid));
    componentPeakMap(indices(valid)) = peak(groups(valid));
    componentSizeMap(indices(valid)) = counts(groups(valid));
end

function smoothMap = smoothStepMap(valueMap, lowerValue, upperValue)
% smoothStepMap: Apply a cubic smooth-step transfer function that is
% zero at and below lowerValue, one at and above upperValue, and follows
% 3t^2 - 2t^3 between the two limits.
%
% Input:
%   valueMap: numeric map or scalar values to transform
%   lowerValue: scalar lower transition boundary
%   upperValue: scalar upper transition boundary
%
% Output:
%   smoothMap: double map with values clipped to [0, 1]
    valueMap = double(valueMap);
    lowerValue = double(lowerValue);
    upperValue = double(upperValue);
    if upperValue <= lowerValue
        smoothMap = double(valueMap >= upperValue);
        smoothMap(~isfinite(valueMap)) = 0;
        return;
    end

    t = (valueMap - lowerValue) ./ (upperValue - lowerValue);
    t = min(max(t, 0), 1);
    smoothMap = (3 .* t .* t) - (2 .* t .* t .* t);
    smoothMap(~isfinite(smoothMap)) = 0;
end

function sumMap = boxSumMap(valueMap, rowRadius, colRadius)
% boxSumMap: Sum finite map values inside a rectangular cell window
% centered at every output location using zero padding outside the map
% extent so edge windows naturally contain fewer valid samples.
%
% Input:
%   valueMap: [Ny x Nx] numeric map to sum
%   rowRadius: scalar nonnegative row radius in cells
%   colRadius: scalar nonnegative column radius in cells
%
% Output:
%   sumMap: [Ny x Nx] double rectangular-window sum map
    rowRadius = max(0, round(double(rowRadius)));
    colRadius = max(0, round(double(colRadius)));
    valueMap = double(valueMap);
    valueMap(~isfinite(valueMap)) = 0;
    mapSize = size(valueMap);
    if isempty(valueMap)
        sumMap = valueMap;
        return;
    end

    integralMap = zeros(mapSize(1) + 1, mapSize(2) + 1);
    integralMap(2:end, 2:end) = cumsum(cumsum(valueMap, 1), 2);
    rowStart = max((1:mapSize(1)).' - rowRadius, 1);
    rowEnd = min((1:mapSize(1)).' + rowRadius, mapSize(1));
    colStart = max((1:mapSize(2)) - colRadius, 1);
    colEnd = min((1:mapSize(2)) + colRadius, mapSize(2));
    sumMap = integralMap(rowEnd + 1, colEnd + 1) ...
        - integralMap(rowStart, colEnd + 1) ...
        - integralMap(rowEnd + 1, colStart) ...
        + integralMap(rowStart, colStart);
end

function countMap = boxWindowCountMap(mapSize, rowRadius, colRadius)
% boxWindowCountMap: Compute the number of in-bounds cells in the
% same rectangular window used by boxSumMap without materializing and
% summing an all-ones raster. Edge windows naturally contain fewer cells.
%
% Input:
%   mapSize: [1 x 2] raster size [Ny Nx]
%   rowRadius: scalar nonnegative row radius in cells
%   colRadius: scalar nonnegative column radius in cells
%
% Output:
%   countMap: [Ny x Nx] double in-bounds sample count per window
    mapSize = double(mapSize(1:2));
    rowRadius = max(0, round(double(rowRadius)));
    colRadius = max(0, round(double(colRadius)));
    if any(mapSize < 1)
        countMap = zeros(mapSize);
        return;
    end

    rowIdx = (1:mapSize(1)).';
    colIdx = 1:mapSize(2);
    rowCount = min(rowIdx + rowRadius, mapSize(1)) - max(rowIdx - rowRadius, 1) + 1;
    colCount = min(colIdx + colRadius, mapSize(2)) - max(colIdx - colRadius, 1) + 1;
    countMap = double(rowCount) .* double(colCount);
end

function maxDiffMap = maxNeighborAbsDiffMap(valueMap, validMask)
% maxNeighborAbsDiffMap: Compute the maximum absolute difference
% between each valid raster cell and all valid cells in its fixed 8-cell
% neighborhood, returning zero where no valid neighbor pair exists.
%
% Input:
%   valueMap: [Ny x Nx] numeric raster values
%   validMask: [Ny x Nx] logical valid-cell support
%
% Output:
%   maxDiffMap: [Ny x Nx] double maximum absolute neighbor difference
    offsets = [0, 1; 0, -1; 1, 0; -1, 0; 1, 1; -1, 1; 1, -1; -1, -1];
    maxDiffMap = zeros(size(valueMap));
    for k = 1:size(offsets, 1)
        [neighborMap, neighborValid] = shiftMap(valueMap, validMask, offsets(k, 1), offsets(k, 2));
        pairMask = logical(validMask) & neighborValid & isfinite(valueMap) & isfinite(neighborMap);
        diffMap = zeros(size(valueMap));
        diffMap(pairMask) = abs(valueMap(pairMask) - neighborMap(pairMask));
        maxDiffMap = max(maxDiffMap, diffMap);
    end
end

function neighborCount = countValidNeighbors(validMask)
% countValidNeighbors: Count valid cells in the fixed 8-neighborhood
% around each raster cell so the total energy map can suppress cells whose
% local geometry is underconstrained by missing or invalid neighbors.
%
% Input:
%   validMask: [Ny x Nx] logical valid-cell support
%
% Output:
%   neighborCount: [Ny x Nx] double count of valid neighboring cells
    validMask = logical(validMask);
    neighborCount = conv2(double(validMask), ones(3), "same") - double(validMask);
end

function [shiftedMap, shiftedValid] = shiftMap(valueMap, validMask, rowOffset, colOffset)
% shiftMap: Return a neighbor-aligned raster where each output cell
% stores the value and validity of the source cell displaced by the given
% row and column offsets, leaving out-of-bounds locations invalid.
%
% Input:
%   valueMap: [Ny x Nx] numeric raster values
%   validMask: [Ny x Nx] logical valid-cell support
%   rowOffset: scalar row offset from target cell to source cell
%   colOffset: scalar column offset from target cell to source cell
%
% Output:
%   shiftedMap: [Ny x Nx] neighbor-aligned numeric raster
%   shiftedValid: [Ny x Nx] logical neighbor-validity raster
    [numRows, numCols] = size(valueMap);
    shiftedMap = NaN(numRows, numCols);
    shiftedValid = false(numRows, numCols);
    targetRows = max(1, 1 - rowOffset):min(numRows, numRows - rowOffset);
    targetCols = max(1, 1 - colOffset):min(numCols, numCols - colOffset);
    if isempty(targetRows) || isempty(targetCols)
        return;
    end
    sourceRows = targetRows + rowOffset;
    sourceCols = targetCols + colOffset;
    shiftedMap(targetRows, targetCols) = valueMap(sourceRows, sourceCols);
    shiftedValid(targetRows, targetCols) = logical(validMask(sourceRows, sourceCols));
end
