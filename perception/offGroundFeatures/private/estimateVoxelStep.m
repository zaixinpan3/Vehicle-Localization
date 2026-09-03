function voxelStep = estimateVoxelStep(zCenters)
% estimateVoxelStep: Estimate the vertical voxel spacing from a vector
% of z-center coordinates, returning 1 when the spacing is unavailable.
%
% Input:
%   zCenters: [Nz x 1] double z-center coordinates
%
% Output:
%   voxelStep: scalar positive voxel spacing in meters
    voxelStep = 1;
    zCenters = double(zCenters(:));
    if numel(zCenters) < 2
        return;
    end

    dz = diff(zCenters);
    dz = dz(isfinite(dz) & (dz > 0));
    if ~isempty(dz)
        voxelStep = median(dz);
    end
end
