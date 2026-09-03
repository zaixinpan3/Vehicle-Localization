function [count3D, zCenters, reason, supportMeta] = resolveFineGridForSupportMask(fineVoxelGrid, supportMask)
% resolveFineGridForSupportMask: Materialize a fine 3D voxel grid for
% a supplied 2D support mask using either an existing 3D tensor or
% per-point samples while preserving coarse-to-fine mapping metadata.
%
% Input:
%   fineVoxelGrid: struct from buildFineColumnFeatureMaps
%   supportMask: [Ny x Nx] logical support mask in 2D
%
% Output:
%   count3D: [NyFine x NxFine x Nz] single voxel-count tensor
%   zCenters: [Nz x 1] double z-center coordinates
%   reason: string reason when output is empty
%   supportMeta: struct with fine-grid and coarse-grid mapping metadata
    count3D = zeros(0, 0, 0, "single");
    zCenters = zeros(0, 1);
    reason = "invalidFineGrid";
    supportMeta = buildFineGridSupportMeta(fineVoxelGrid, size(supportMask));

    [ny, nx] = size(supportMask);
    supportMask = logical(supportMask);
    if ~any(supportMask(:))
        reason = "emptyFineGridNeighborhood";
        return;
    end

    hasSampleInput = isfield(fineVoxelGrid, "sampleXBin") && isfield(fineVoxelGrid, "sampleYBin") && isfield(fineVoxelGrid, "sampleZ") && ...
        ~isempty(fineVoxelGrid.sampleXBin) && ~isempty(fineVoxelGrid.sampleYBin) && ~isempty(fineVoxelGrid.sampleZ);
    if hasSampleInput
        dz = 1;
        if isfield(fineVoxelGrid, "voxelSize") && numel(fineVoxelGrid.voxelSize) >= 3 && ...
                isfinite(fineVoxelGrid.voxelSize(3)) && fineVoxelGrid.voxelSize(3) > 0
            dz = double(fineVoxelGrid.voxelSize(3));
        end
        [count3D, zCenters, isValid] = buildFineGridFromPointSamples( ...
            fineVoxelGrid.sampleXBin, fineVoxelGrid.sampleYBin, fineVoxelGrid.sampleZ, supportMask, dz);
        if ~isValid || isempty(count3D) || ~any(count3D(:))
            reason = "emptyFineGridNeighborhood";
            return;
        end
        reason = "ok";
        return;
    end

    if ~isfield(fineVoxelGrid, "count3D") || ndims(fineVoxelGrid.count3D) ~= 3
        reason = "invalidFineGrid";
        return;
    end

    count3D = single(fineVoxelGrid.count3D);
    if size(count3D, 1) == ny && size(count3D, 2) == nx
        if isempty(count3D) || size(count3D, 3) < 1 || ~any(count3D(:))
            count3D = zeros(0, 0, 0, "single");
            reason = "emptyFineGridNeighborhood";
            return;
        end
    else
        fineMaskSize = [size(count3D, 1), size(count3D, 2)];
        if ~isequal(supportMeta.coarseMapSize, [ny, nx])
            count3D = zeros(0, 0, 0, "single");
            reason = "gridSizeMismatch";
            return;
        end
        fineSupportMask = expandCoarseMaskToFine(supportMask, fineMaskSize, supportMeta);
        if ~any(fineSupportMask(:))
            count3D = zeros(0, 0, 0, "single");
            reason = "emptyFineGridNeighborhood";
            return;
        end
        count3D = count3D .* cast(fineSupportMask, "like", count3D);
        if isempty(count3D) || size(count3D, 3) < 1 || ~any(count3D(:))
            count3D = zeros(0, 0, 0, "single");
            reason = "emptyFineGridNeighborhood";
            return;
        end
    end

    if isfield(fineVoxelGrid, "zCenters") && numel(fineVoxelGrid.zCenters) == size(count3D, 3)
        zCenters = double(fineVoxelGrid.zCenters(:));
    else
        zCenters = ((1:size(count3D, 3)).' - 0.5);
    end
    reason = "ok";
end
