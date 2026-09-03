function fineMask = expandCoarseMaskToFine(coarseMask, fineMaskSize, supportMeta)
% expandCoarseMaskToFine: Expand a coarse logical mask onto the
% corresponding fine xy grid using the stored metric geometry.
%
% Input:
%   coarseMask: [Ny x Nx] logical coarse-grid support mask
%   fineMaskSize: [1 x 2] fine-grid size [NyFine NxFine]
%   supportMeta: struct from buildFineGridSupportMeta
%
% Output:
%   fineMask: [NyFine x NxFine] logical fine-grid support mask
    fineMask = false(fineMaskSize);
    [rowIdx, colIdx] = find(logical(coarseMask));
    if isempty(rowIdx)
        return;
    end

    for i = 1:numel(rowIdx)
        [fineRowMin, fineRowMax, fineColMin, fineColMax] = mapCoarseCellToFineBounds( ...
            rowIdx(i), colIdx(i), fineMaskSize, supportMeta);
        if fineRowMin <= fineRowMax && fineColMin <= fineColMax
            fineMask(fineRowMin:fineRowMax, fineColMin:fineColMax) = true;
        end
    end
end
