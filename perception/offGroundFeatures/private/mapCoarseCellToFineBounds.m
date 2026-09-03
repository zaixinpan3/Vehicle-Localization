function [fineRowMin, fineRowMax, fineColMin, fineColMax] = mapCoarseCellToFineBounds(rowIdx, colIdx, fineMaskSize, supportMeta)
% mapCoarseCellToFineBounds: Convert one coarse-grid cell index into
% the inclusive fine-grid row and column bounds that cover the same metric
% support region.
%
% Input:
%   rowIdx: scalar coarse row index
%   colIdx: scalar coarse column index
%   fineMaskSize: [1 x 2] fine-grid size [NyFine NxFine]
%   supportMeta: struct from buildFineGridSupportMeta
%
% Output:
%   fineRowMin: scalar lower fine-row bound
%   fineRowMax: scalar upper fine-row bound
%   fineColMin: scalar lower fine-column bound
%   fineColMax: scalar upper fine-column bound
    yMin = supportMeta.coarseOriginXY(2) + ((double(rowIdx) - 1) .* supportMeta.coarseVoxelSizeXY(2));
    yMax = yMin + supportMeta.coarseVoxelSizeXY(2);
    xMin = supportMeta.coarseOriginXY(1) + ((double(colIdx) - 1) .* supportMeta.coarseVoxelSizeXY(1));
    xMax = xMin + supportMeta.coarseVoxelSizeXY(1);

    fineRowMin = floor((yMin - supportMeta.originXY(2)) ./ supportMeta.voxelSizeXY(2)) + 1;
    fineRowMax = ceil((yMax - supportMeta.originXY(2)) ./ supportMeta.voxelSizeXY(2));
    fineColMin = floor((xMin - supportMeta.originXY(1)) ./ supportMeta.voxelSizeXY(1)) + 1;
    fineColMax = ceil((xMax - supportMeta.originXY(1)) ./ supportMeta.voxelSizeXY(1));

    fineRowMin = min(max(fineRowMin, 1), fineMaskSize(1));
    fineRowMax = min(max(fineRowMax, 1), fineMaskSize(1));
    fineColMin = min(max(fineColMin, 1), fineMaskSize(2));
    fineColMax = min(max(fineColMax, 1), fineMaskSize(2));
end
