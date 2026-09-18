function pillars = pillarizePointCloud(frame, cfg)
% pillarizePointCloud: Index whole XY pillars and retain their XYZ statistics.
% Z is never quantized. The lattice is fixed by cfg: gridDims pillars of
% voxelSize meters centered at the sensor origin (see pillarGridExtent).
% Returns outside the lattice, outside [minRange, maxRange], or inside the
% exclusionHalfSize box around the sensor are ignored.
    [xyz, indices, attributes, meta] = readPerceptionPoints(frame);
    extent = pillarGridExtent(cfg);
    spacing = double(cfg.voxelSize(:).');
    dims = double(cfg.gridDims(:).');
    lower = extent([1, 3]);
    upper = extent([2, 4]);
    bins = floor((xyz(:, 1:2) - lower)./spacing) + 1;
    keep = all(isfinite(xyz), 2) & isfinite(attributes.range);
    keep = keep & attributes.range >= cfg.minRange & attributes.range <= cfg.maxRange;
    keep = keep & ~(abs(xyz(:, 1)) < cfg.exclusionHalfSize & abs(xyz(:, 2)) < cfg.exclusionHalfSize);
    keep = keep & all(bins >= 1 & bins <= dims, 2);
    xyz = xyz(keep, :);
    indices = indices(keep);
    bins = bins(keep, :);
    names = fieldnames(attributes);
    for k = 1:numel(names), attributes.(names{k}) = attributes.(names{k})(keep); end
    mapSize = dims([2, 1]);
    ids = int32(sub2ind(mapSize, bins(:, 2), bins(:, 1)));
    pillars = struct('spatialIndexType', "xyPillars", 'points', xyz, ...
        'pointIndices', indices, 'pointAttributes', attributes, ...
        'pointPillarSub', int32(bins), 'pointPillarLinIdx', ids, ...
        'inputType', meta.inputType, 'inputSize', meta.inputSize, ...
        'numInputPoints', meta.numInputPoints, 'numFilteredPoints', size(xyz, 1));
    pillars.pillarGeometry = struct('origin', lower, 'cellSize', spacing, 'mapSize', mapSize, 'layout', "NyNx");
    % Terrain preprocessing consumes only XY geometry through this shared name.
    pillars.gridConfig = struct('dims', dims, 'voxelSize', spacing, ...
        'minCorner', lower, 'maxCorner', upper, 'origin', lower + spacing/2);
    useNative = isfield(cfg, 'useNativeKernels') && cfg.useNativeKernels;
    pillars.statistics = aggregatePillarStatistics(xyz, ids, attributes, useNative);
end
