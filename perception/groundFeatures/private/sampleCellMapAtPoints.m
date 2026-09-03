function values = sampleCellMapAtPoints(cellLinIdx, cellMap)
% sampleCellMapAtPoints: Sample a [Ny x Nx] XY-cell raster at
% point-level cell linear indices that use the corresponding [Nx Ny]
% sub2ind layout, returning one scalar map value per point for point-cloud
% coloring or data export.
%
% Input:
%   cellLinIdx: [N x 1] numeric XY-cell linear indices aligned to points
%   cellMap: [Ny x Nx] numeric raster map
%
% Output:
%   values: [N x 1] single sampled scalar values with invalid samples set
%       to zero
    values = zeros(numel(cellLinIdx), 1, "single");
    if isempty(cellLinIdx) || isempty(cellMap)
        return;
    end

    mapValues = double(cellMap.');
    mapValues = mapValues(:);
    validCell = isfinite(cellLinIdx) & cellLinIdx >= 1 & cellLinIdx <= numel(mapValues) & cellLinIdx == floor(cellLinIdx);
    values(validCell) = single(mapValues(double(cellLinIdx(validCell))));
    values(~isfinite(values)) = 0;
end
