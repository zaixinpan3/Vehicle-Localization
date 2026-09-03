function pointMask = buildMaskFromIndices(pointIdx, numFramePoints)
% buildMaskFromIndices: Convert original organized-frame linear point
% indices into a full-frame logical mask, dropping invalid index values.
%
% Input:
%   pointIdx: numeric vector of original frame linear point indices
%   numFramePoints: total number of original frame points
%
% Output:
%   pointMask: [numFramePoints x 1] logical point mask
    pointMask = false(numFramePoints, 1);
    if isempty(pointIdx)
        return;
    end
    pointIdx = double(pointIdx(:));
    validIdx = isfinite(pointIdx) & pointIdx >= 1 & pointIdx <= numFramePoints & pointIdx == floor(pointIdx);
    if any(validIdx)
        pointMask(pointIdx(validIdx)) = true;
    end
end
