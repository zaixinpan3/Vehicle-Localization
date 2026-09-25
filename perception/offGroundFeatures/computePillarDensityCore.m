function [coreFraction, coreHeight] = computePillarDensityCore(points, pillarIds, geometry, peakBinMeters, coreRadius)
% computePillarDensityCore: XY density peak of every whole pillar and its vertical extent.
% A pole fills one small XY spot of its pillar with returns that spread over a
% large height range, whereas foliage, walls and vehicle bodies spread their
% returns over the pillar footprint. For each pillar the densest XY location is
% located on a peakBinMeters histogram of the pillar footprint and refined by
% one mean shift inside coreRadius; coreFraction is the share of the pillar's
% returns within coreRadius of that peak and coreHeight is the z range of those
% returns. The pillar remains the unit: no finer cell is published and no
% vertical index exists. Outputs are ordered like unique(pillarIds).
%
% Input:
%   points: [N x 3] XYZ returns of the branch
%   pillarIds: [N x 1] linear pillar indices on the [Ny Nx] lattice
%   geometry: struct with origin, cellSize and mapSize of the lattice
%   peakBinMeters: histogram bin used to locate the density peak
%   coreRadius: radius in meters around the peak that defines the core
%
% Output:
%   coreFraction: [P x 1] share of pillar returns inside the core
%   coreHeight: [P x 1] z range of the core returns
    assert(isscalar(peakBinMeters) && peakBinMeters > 0 && isscalar(coreRadius) && coreRadius > 0, ...
        "perception:InvalidDensityCore", "Density-core bin and radius must be positive.");
    [ids, ~, group] = unique(double(pillarIds(:)));
    numPillars = numel(ids);
    coreFraction = zeros(numPillars, 1); coreHeight = zeros(numPillars, 1);
    if isempty(points), return; end
    points = double(points);
    mapSize = double(geometry.mapSize(:).'); cellSize = double(geometry.cellSize(:).'); origin = double(geometry.origin(:).');
    [row, col] = ind2sub(mapSize, ids);
    lower = origin + ([col row] - 1) .* cellSize;
    bins = max(1, ceil(cellSize ./ peakBinMeters));
    local = points(:, 1:2) - lower(group, :);
    sub = min(max(floor(local ./ peakBinMeters) + 1, 1), bins);
    subLinear = sub(:, 1) + (sub(:, 2) - 1) .* bins(1);
    counts = accumarray([group, subLinear], 1, [numPillars, prod(bins)]);
    [~, peakBin] = max(counts, [], 2);
    [peakX, peakY] = ind2sub(bins, peakBin);
    peak = lower + ([peakX peakY] - 0.5) .* peakBinMeters;
    % One mean shift inside the core radius sharpens the histogram peak.
    near = hypot(points(:, 1) - peak(group, 1), points(:, 2) - peak(group, 2)) <= coreRadius;
    nearCount = accumarray(group(near), 1, [numPillars 1]);
    shifted = [accumarray(group(near), points(near, 1), [numPillars 1]), ...
        accumarray(group(near), points(near, 2), [numPillars 1])] ./ max(nearCount, 1);
    peak(nearCount > 0, :) = shifted(nearCount > 0, :);
    core = hypot(points(:, 1) - peak(group, 1), points(:, 2) - peak(group, 2)) <= coreRadius;
    total = accumarray(group, 1, [numPillars 1]);
    coreFraction = accumarray(group(core), 1, [numPillars 1]) ./ max(total, 1);
    top = accumarray(group(core), points(core, 3), [numPillars 1], @max, NaN);
    bottom = accumarray(group(core), points(core, 3), [numPillars 1], @min, NaN);
    span = top - bottom; span(~isfinite(span)) = 0;
    coreHeight = span;
end
