function coarseMask = projectFineMaskToCoarse(fineMask, coarseMapSize, supportMeta)
% projectFineMaskToCoarse: Project a fine logical xy mask back onto
% the coarse grid by assigning each occupied fine cell to its parent
% coarse cell using metric cell centers.
%
% Input:
%   fineMask: [NyFine x NxFine] logical fine-grid mask
%   coarseMapSize: [1 x 2] coarse-grid size [Ny Nx]
%   supportMeta: struct from buildFineGridSupportMeta
%
% Output:
%   coarseMask: [Ny x Nx] logical projected coarse-grid mask
    coarseMask = false(coarseMapSize);
    [fineRows, fineCols] = find(logical(fineMask));
    if isempty(fineRows)
        return;
    end

    xVals = supportMeta.originXY(1) + ((double(fineCols(:)) - 0.5) .* supportMeta.voxelSizeXY(1));
    yVals = supportMeta.originXY(2) + ((double(fineRows(:)) - 0.5) .* supportMeta.voxelSizeXY(2));
    coarseCols = floor((xVals - supportMeta.coarseOriginXY(1)) ./ supportMeta.coarseVoxelSizeXY(1)) + 1;
    coarseRows = floor((yVals - supportMeta.coarseOriginXY(2)) ./ supportMeta.coarseVoxelSizeXY(2)) + 1;
    valid = isfinite(coarseRows) & isfinite(coarseCols);
    valid = valid & (coarseRows >= 1) & (coarseRows <= coarseMapSize(1)) & ...
        (coarseCols >= 1) & (coarseCols <= coarseMapSize(2));
    if ~any(valid)
        return;
    end

    coarseLin = sub2ind(coarseMapSize, coarseRows(valid), coarseCols(valid));
    coarseMask(coarseLin) = true;
end
