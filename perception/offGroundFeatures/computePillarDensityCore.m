function [coreFraction, coreHeight, coreIsolation, peak, corePointCount] = computePillarDensityCore(points, pillarIds, geometry, peakBinMeters, coreRadius, isolationRadius, evaluate)
% computePillarDensityCore: XY density peak of every whole pillar, its vertical extent and isolation.
% A pole fills one small XY spot of its pillar with returns that spread over a
% large height range, whereas foliage, walls and vehicle bodies spread their
% returns over the pillar footprint. For each evaluated pillar the densest XY
% location is located on a peakBinMeters histogram of the pillar footprint and
% refined by one mean shift inside coreRadius; coreFraction is the share of the
% pillar's returns within coreRadius of that peak and coreHeight is the z range
% of those returns. coreIsolation compares the returns of the pillar and its
% neighbours within coreRadius of the peak with those between coreRadius and
% isolationRadius: a shaft standing free of hedges, walls and canopies keeps
% a concentrated share of that neighbourhood inside its core, and a shaft split over a cell
% boundary keeps its core because the partner's returns count as core. The
% pillar remains the unit: no finer cell is published and no vertical index
% exists. Outputs are ordered like unique(pillarIds); pillars that are not
% evaluated report zeros and a NaN peak.
%
% Input:
%   points: [N x 3] XYZ returns of the branch
%   pillarIds: [N x 1] linear pillar indices on the [Ny Nx] lattice
%   geometry: struct with origin, cellSize and mapSize of the lattice
%   peakBinMeters: histogram bin used to locate the density peak
%   coreRadius: radius in meters around the peak that defines the core
%   isolationRadius: outer radius of the neighbourhood compared with the core
%   evaluate: optional [P x 1] logical, ordered like unique(pillarIds),
%       selecting the pillars whose core is measured (default: all)
%
% Output:
%   coreFraction: [P x 1] share of pillar returns inside the core
%   coreHeight: [P x 1] z range of the core returns
%   coreIsolation: [P x 1] core returns over all returns within isolationRadius
%   peak: [P x 2] XY location of the density peak
%   corePointCount: [P x 1] returns inside the metric core, including neighbours
    assert(isscalar(peakBinMeters) && peakBinMeters > 0 && isscalar(coreRadius) && coreRadius > 0, ...
        "perception:InvalidDensityCore", "Density-core bin and radius must be positive.");
    if nargin < 6 || isempty(isolationRadius), isolationRadius = coreRadius; end
    assert(isscalar(isolationRadius) && isolationRadius >= coreRadius, ...
        "perception:InvalidDensityCore", "The isolation radius must not be smaller than the core radius.");
    [ids, ~, group] = unique(double(pillarIds(:)));
    numPillars = numel(ids);
    if nargin < 7 || isempty(evaluate), evaluate = true(numPillars, 1); end
    evaluate = logical(evaluate(:));
    assert(numel(evaluate) == numPillars, "perception:InvalidDensityCore", ...
        "evaluate must hold one flag per occupied pillar.");
    coreFraction = zeros(numPillars, 1); coreHeight = zeros(numPillars, 1);
    coreIsolation = zeros(numPillars, 1); peak = nan(numPillars, 2);
    corePointCount = zeros(numPillars, 1);
    selected = find(evaluate); numSelected = numel(selected);
    if isempty(points) || numSelected == 0, return; end
    points = double(points);
    mapSize = double(geometry.mapSize(:).'); cellSize = double(geometry.cellSize(:).'); origin = double(geometry.origin(:).');
    [row, col] = ind2sub(mapSize, ids);
    lower = origin + ([col row] - 1) .* cellSize;

    % Density peak and core of every evaluated pillar from its own returns.
    remap = zeros(numPillars, 1); remap(selected) = 1:numSelected;
    keep = evaluate(group);
    own = points(keep, :); k = remap(group(keep));
    bins = max(1, ceil(cellSize ./ peakBinMeters));
    local = own(:, 1:2) - lower(selected(k), :);
    sub = min(max(floor(local ./ peakBinMeters) + 1, 1), bins);
    counts = accumarray([k, sub(:, 1) + (sub(:, 2) - 1) .* bins(1)], 1, [numSelected, prod(bins)]);
    [~, peakBin] = max(counts, [], 2);
    [peakX, peakY] = ind2sub(bins, peakBin);
    centre = lower(selected, :) + ([peakX peakY] - 0.5) .* peakBinMeters;
    % One mean shift inside the core radius sharpens the histogram peak.
    near = hypot(own(:, 1) - centre(k, 1), own(:, 2) - centre(k, 2)) <= coreRadius;
    nearCount = accumarray(k(near), 1, [numSelected 1]);
    shifted = [accumarray(k(near), own(near, 1), [numSelected 1]), ...
        accumarray(k(near), own(near, 2), [numSelected 1])] ./ max(nearCount, 1);
    centre(nearCount > 0, :) = shifted(nearCount > 0, :);
    core = hypot(own(:, 1) - centre(k, 1), own(:, 2) - centre(k, 2)) <= coreRadius;
    total = accumarray(k, 1, [numSelected 1]);
    coreFraction(selected) = accumarray(k(core), 1, [numSelected 1]) ./ max(total, 1);
    top = accumarray(k(core), own(core, 3), [numSelected 1], @max, NaN);
    bottom = accumarray(k(core), own(core, 3), [numSelected 1], @min, NaN);
    span = top - bottom; span(~isfinite(span)) = 0;
    coreHeight(selected) = span;
    peak(selected, :) = centre;

    % Isolation: returns of every pillar within reach of isolationRadius are
    % gathered through one sorted point index and tested against the peak.
    [~, order] = sort(group);
    pillarCount = accumarray(group, 1, [numPillars 1]);
    first = cumsum([1; pillarCount(1:end-1)]);
    reach = ceil(isolationRadius ./ cellSize);
    [colOffset, rowOffset] = meshgrid(-reach(1):reach(1), -reach(2):reach(2));
    neighbourRow = row(selected) + rowOffset(:).'; neighbourCol = col(selected) + colOffset(:).';
    valid = neighbourRow >= 1 & neighbourRow <= mapSize(1) & neighbourCol >= 1 & neighbourCol <= mapSize(2);
    lookup = zeros(mapSize); lookup(ids) = 1:numPillars;
    neighbour = zeros(size(neighbourRow));
    neighbour(valid) = lookup(sub2ind(mapSize, neighbourRow(valid), neighbourCol(valid)));
    owner = repmat((1:numSelected).', 1, numel(rowOffset));
    pair = neighbour > 0; neighbour = neighbour(pair); owner = owner(pair);
    neighbour = neighbour(:); owner = owner(:);
    lengths = pillarCount(neighbour);
    if isempty(lengths) || sum(lengths) == 0, return; end
    % repelem of a scalar returns a row, so every expansion is reshaped.
    ownerExpanded = reshape(repelem(owner, lengths), [], 1);
    before = reshape(repelem(cumsum(lengths) - lengths, lengths), [], 1);
    start = reshape(repelem(first(neighbour), lengths), [], 1);
    pointIdx = order(start + ((1:sum(lengths)).' - before) - 1);
    d = hypot(points(pointIdx, 1) - centre(ownerExpanded, 1), points(pointIdx, 2) - centre(ownerExpanded, 2));
    coreAll = accumarray(ownerExpanded(d <= coreRadius), 1, [numSelected 1]);
    annulus = accumarray(ownerExpanded(d > coreRadius & d <= isolationRadius), 1, [numSelected 1]);
    coreIsolation(selected) = coreAll ./ max(coreAll + annulus, 1);
    corePointCount(selected) = coreAll;
end
