function curbPointMask = selectCurbPointsFromCells(extractedPointMask, groundPoints, groundCellLinIdx, stats, energyMaps, roadCellMask, cfg)
% selectCurbPointsFromCells: Refine dense curb-channel
% output by removing weak points from curb cells that sit close to the
% initial road surface while preserving points with strong lower residual,
% base curb energy, or roughness evidence. The extracted curb-cell mask is
% left unchanged so diagnostics still show all accepted cells.
%
% Input:
%   extractedPointMask: [N x 1] logical points sampled from extracted
%       curb-energy cells
%   groundPoints: [N x 3] double ground-point coordinates
%   groundCellLinIdx: [N x 1] XY-cell linear indices aligned to
%       groundPoints
%   stats: struct with heightMap and detrendedHeightMap
%   energyMaps: struct with totalBase and roughnessMeters maps
%   roadCellMask: [Ny x Nx] logical initial road-surface raster
%   cfg: struct from groundFeatureConfig().curb with dense curb filter fields
%
% Output:
%   curbPointMask: [N x 1] logical dense curb-channel mask
    curbPointMask = logical(extractedPointMask(:));
    if ~isstruct(cfg) || ~isfield(cfg, "denseCurbNearRoadFilterEnabled") ...
            || ~logical(cfg.denseCurbNearRoadFilterEnabled) || ~any(curbPointMask)
        return;
    end

    radiusCells = 0;
    if isfield(cfg, "denseCurbNearRoadRadiusCells")
        radiusCells = max(0, round(double(cfg.denseCurbNearRoadRadiusCells)));
    end
    if radiusCells <= 0 || isempty(roadCellMask)
        return;
    end

    nearRoadCellMask = conv2(double(logical(roadCellMask)), ones((2 * radiusCells) + 1), "same") > 0;
    nearRoadPointMask = sampleCellMapAtPoints(groundCellLinIdx, nearRoadCellMask) > 0;
    if ~any(nearRoadPointMask & curbPointMask)
        return;
    end

    minResidualMeters = 0.025;
    if isfield(cfg, "denseCurbMinResidualMeters")
        minResidualMeters = max(0, double(cfg.denseCurbMinResidualMeters));
    end
    minBaseEnergy = 0.75;
    if isfield(cfg, "denseCurbMinBaseEnergy")
        minBaseEnergy = max(0, double(cfg.denseCurbMinBaseEnergy));
    end
    minRoughnessMeters = 0.030;
    if isfield(cfg, "denseCurbMinRoughnessMeters")
        minRoughnessMeters = max(0, double(cfg.denseCurbMinRoughnessMeters));
    end

    heightMap = double(stats.heightMap);
    localMeanMap = heightMap - double(stats.detrendedHeightMap);
    localMeanValues = double(sampleCellMapAtPointsPreserveNaN(groundCellLinIdx, localMeanMap));
    heightValues = double(sampleCellMapAtPointsPreserveNaN(groundCellLinIdx, heightMap));
    missingLocalMean = ~isfinite(localMeanValues);
    localMeanValues(missingLocalMean) = heightValues(missingLocalMean);
    residualValues = max(localMeanValues - double(groundPoints(:, 3)), 0);
    residualValues(~isfinite(residualValues)) = 0;
    baseValues = double(sampleCellMapAtPoints(groundCellLinIdx, energyMaps.totalBase));
    roughnessValues = double(sampleCellMapAtPoints(groundCellLinIdx, energyMaps.roughnessMeters));
    strongNearRoadMask = residualValues >= minResidualMeters ...
        | baseValues >= minBaseEnergy ...
        | roughnessValues >= minRoughnessMeters;
    curbPointMask = curbPointMask & (~nearRoadPointMask | strongNearRoadMask);
end

function values = sampleCellMapAtPointsPreserveNaN(cellLinIdx, cellMap)
% sampleCellMapAtPointsPreserveNaN: Sample a [Ny x Nx] XY-cell
% raster at point-level cell linear indices while preserving NaN values so
% callers can distinguish missing map data from a valid zero measurement.
%
% Input:
%   cellLinIdx: [N x 1] numeric XY-cell linear indices aligned to points
%   cellMap: [Ny x Nx] numeric raster map
%
% Output:
%   values: [N x 1] single sampled scalar values with invalid cell indices
%       set to NaN
    values = NaN(numel(cellLinIdx), 1, "single");
    if isempty(cellLinIdx) || isempty(cellMap)
        return;
    end

    mapValues = double(cellMap.');
    mapValues = mapValues(:);
    validCell = isfinite(cellLinIdx) & cellLinIdx >= 1 & cellLinIdx <= numel(mapValues) & cellLinIdx == floor(cellLinIdx);
    values(validCell) = single(mapValues(double(cellLinIdx(validCell))));
end
