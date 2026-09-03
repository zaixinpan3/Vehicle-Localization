function [trafficSignMask, trafficSignFineVoxelMask, trafficSignFineOrigin, trafficSignFineVoxelSize, trafficSignFineZCenters] = extractTrafficSignChannel(fineVoxelGrid, coarseMapSize)
% extractTrafficSignChannel: Project the high-intensity traffic-sign
% fine-voxel channel, extracted from the full off-ground point set rather
% than from pole-refine candidates, onto the coarse pillar map and expose
% the fine mask for downstream point-level visualization.
%
% Input:
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
%   coarseMapSize: [1 x 2] coarse map size [Ny Nx]
%
% Output:
%   trafficSignMask: [Ny x Nx] logical coarse traffic-sign mask
%   trafficSignFineVoxelMask: [NyFine x NxFine x Nz] logical fine mask
%   trafficSignFineOrigin: [1 x 3] double fine-grid origin
%   trafficSignFineVoxelSize: [1 x 3] double fine-grid voxel size
%   trafficSignFineZCenters: [Nz x 1] double fine-grid z centers
    trafficSignMask = false(coarseMapSize);
    trafficSignFineVoxelMask = false(0, 0, 0);
    trafficSignFineOrigin = [0, 0, 0];
    trafficSignFineVoxelSize = [1, 1, 1];
    trafficSignFineZCenters = zeros(0, 1);

    if ~isstruct(fineVoxelGrid) || ~isfield(fineVoxelGrid, "trafficSignCount3D") || isempty(fineVoxelGrid.trafficSignCount3D)
        return;
    end

    trafficSignFineVoxelMask = logical(fineVoxelGrid.trafficSignCount3D > 0);
    if ~any(trafficSignFineVoxelMask(:))
        return;
    end

    supportMeta = buildFineGridSupportMeta(fineVoxelGrid, coarseMapSize);
    trafficSignMask = projectFineMaskToCoarse(any(trafficSignFineVoxelMask, 3), coarseMapSize, supportMeta);
    if isfield(fineVoxelGrid, "origin") && numel(fineVoxelGrid.origin) >= 3
        trafficSignFineOrigin = double(fineVoxelGrid.origin(1:3));
    end
    if isfield(fineVoxelGrid, "voxelSize") && numel(fineVoxelGrid.voxelSize) >= 3
        trafficSignFineVoxelSize = double(fineVoxelGrid.voxelSize(1:3));
    end
    if isfield(fineVoxelGrid, "zCenters") && ~isempty(fineVoxelGrid.zCenters)
        trafficSignFineZCenters = double(fineVoxelGrid.zCenters(:));
    end
end
