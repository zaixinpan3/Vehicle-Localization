function supportMeta = buildFineGridSupportMeta(fineVoxelGrid, coarseMapSize)
% buildFineGridSupportMeta: Read fine-grid geometry and the matching
% coarse-grid geometry into one struct used for resolution conversion.
%
% Input:
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
%   coarseMapSize: [1 x 2] expected coarse support-mask size [Ny Nx]
%
% Output:
%   supportMeta: struct with originXY, voxelSizeXY, coarseOriginXY,
%       coarseVoxelSizeXY, and coarseMapSize fields
    supportMeta = struct("originXY", [0, 0], "voxelSizeXY", [1, 1], ...
        "coarseOriginXY", [0, 0], "coarseVoxelSizeXY", [1, 1], "coarseMapSize", double(coarseMapSize(:).'));

    if isstruct(fineVoxelGrid) && isfield(fineVoxelGrid, "origin") && numel(fineVoxelGrid.origin) >= 2 && ...
            all(isfinite(fineVoxelGrid.origin(1:2)))
        supportMeta.originXY = double(fineVoxelGrid.origin(1:2));
    end
    if isstruct(fineVoxelGrid) && isfield(fineVoxelGrid, "voxelSize") && numel(fineVoxelGrid.voxelSize) >= 2 && ...
            all(isfinite(fineVoxelGrid.voxelSize(1:2))) && all(fineVoxelGrid.voxelSize(1:2) > 0)
        supportMeta.voxelSizeXY = double(fineVoxelGrid.voxelSize(1:2));
    end
    if isstruct(fineVoxelGrid) && isfield(fineVoxelGrid, "coarseOrigin") && numel(fineVoxelGrid.coarseOrigin) >= 2 && ...
            all(isfinite(fineVoxelGrid.coarseOrigin(1:2)))
        supportMeta.coarseOriginXY = double(fineVoxelGrid.coarseOrigin(1:2));
    else
        supportMeta.coarseOriginXY = supportMeta.originXY;
    end
    if isstruct(fineVoxelGrid) && isfield(fineVoxelGrid, "coarseVoxelSize") && numel(fineVoxelGrid.coarseVoxelSize) >= 2 && ...
            all(isfinite(fineVoxelGrid.coarseVoxelSize(1:2))) && all(fineVoxelGrid.coarseVoxelSize(1:2) > 0)
        supportMeta.coarseVoxelSizeXY = double(fineVoxelGrid.coarseVoxelSize(1:2));
    end
    if isstruct(fineVoxelGrid) && isfield(fineVoxelGrid, "coarseMapSize") && numel(fineVoxelGrid.coarseMapSize) >= 2 && ...
            all(isfinite(fineVoxelGrid.coarseMapSize(1:2))) && all(fineVoxelGrid.coarseMapSize(1:2) >= 1)
        supportMeta.coarseMapSize = double(fineVoxelGrid.coarseMapSize(1:2));
    end
end
