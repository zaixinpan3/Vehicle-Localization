function [count3D, zCenters, isValid] = buildFineGridFromPointSamples(sampleXBin, sampleYBin, sampleZ, supportMask, dz)
% buildFineGridFromPointSamples: Build [Ny x Nx x Nz] fine 3D voxel
% counts from per-point pillar bins and z values limited to a 2D support
% mask.
%
% Input:
%   sampleXBin: [K x 1] x-bin indices in [1, Nx]
%   sampleYBin: [K x 1] y-bin indices in [1, Ny]
%   sampleZ: [K x 1] z values in meters
%   supportMask: [Ny x Nx] logical support region in 2D
%   dz: scalar z-bin size in meters
%
% Output:
%   count3D: [Ny x Nx x Nz] single voxel-count tensor
%   zCenters: [Nz x 1] double z-center coordinates
%   isValid: logical scalar indicating successful construction
    count3D = zeros(0, 0, 0, "single");
    zCenters = zeros(0, 1);
    isValid = false;

    if isempty(sampleXBin) || isempty(sampleYBin) || isempty(sampleZ) || isempty(supportMask)
        return;
    end
    if ~(numel(sampleXBin) == numel(sampleYBin) && numel(sampleYBin) == numel(sampleZ))
        return;
    end

    if ~isscalar(dz) || ~isfinite(dz) || dz <= 0
        return;
    end

    [ny, nx] = size(supportMask);
    supportMask = logical(supportMask);

    xBin = double(sampleXBin(:));
    yBin = double(sampleYBin(:));
    zVals = double(sampleZ(:));
    valid = isfinite(xBin) & isfinite(yBin) & isfinite(zVals);
    valid = valid & (xBin >= 1) & (xBin <= nx) & (yBin >= 1) & (yBin <= ny);
    if ~any(valid)
        return;
    end

    xBin = round(xBin(valid));
    yBin = round(yBin(valid));
    zVals = zVals(valid);
    inSupport = supportMask(sub2ind([ny, nx], yBin, xBin));
    if ~any(inSupport)
        return;
    end
    xBin = xBin(inSupport);
    yBin = yBin(inSupport);
    zVals = zVals(inSupport);

    zMin = floor(min(zVals) ./ dz) .* dz;
    zMax = ceil(max(zVals) ./ dz) .* dz;
    nz = max(1, ceil((zMax - zMin) ./ dz));
    zBin = floor((zVals - zMin) ./ dz) + 1;
    zBin = min(max(zBin, 1), nz);

    count3D = accumarray([yBin, xBin, zBin], 1, [ny, nx, nz], @sum, 0);
    count3D = single(count3D);
    zCenters = zMin + (((1:nz).' - 0.5) .* dz);
    isValid = true;
end
