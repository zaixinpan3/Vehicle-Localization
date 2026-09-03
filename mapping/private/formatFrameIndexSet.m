function text = formatFrameIndexSet(frameIndices)
% formatFrameIndexSet: Format a frame-index vector as a compact range
% when contiguous and as an explicit list otherwise.
%
% Input:
%   frameIndices: numeric vector of one-based frame indices
%
% Output:
%   text: string scalar frame-index summary
    frameIndices = double(frameIndices(:).');
    if isempty(frameIndices)
        text = "[]";
    elseif isscalar(frameIndices)
        text = string(frameIndices(1));
    elseif all(diff(frameIndices) == 1)
        text = sprintf("%d:%d", frameIndices(1), frameIndices(end));
    else
        text = "[" + strjoin(string(frameIndices), " ") + "]";
    end
end
