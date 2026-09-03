function [frame, numFrames] = loadPointCloudFrame(matPath, frameIdx)
% loadPointCloudFrame: Load one organized point-cloud frame from a
% MAT file while supporting both struct-array and cell-array storage and
% returning the inferred total frame count.
%
% Input:
%   matPath: char/string path to the MAT file
%   frameIdx: scalar positive integer frame index starting at 1
%
% Output:
%   frame: struct with organized point-cloud fields such as x, y, and z
%   numFrames: scalar inferred number of frames stored in the MAT file
    info = whos("-file", matPath);
    [varName, varClass, varSize] = pickPointCloudVar(info);
    assert(strlength(varName) > 0, "No pointClouds-like variable found in MAT file.");
    numFrames = inferNumFrames(varSize);
    assert(frameIdx >= 1 && frameIdx <= numFrames, "frameIdx out of range. Got %d, valid range is [1, %d].", frameIdx, numFrames);
    m = matfile(matPath);
    idx = frameIndexToSubs(varSize, frameIdx);
    varNameChar = char(varName);
    if strcmp(varClass, "cell")
        frameCell = m.(varNameChar)(idx{:});
        frame = frameCell{1};
    else
        frame = m.(varNameChar)(idx{:});
    end
end

function [varName, varClass, varSize] = pickPointCloudVar(info)
% pickPointCloudVar: Select the MAT-file variable that stores point
% cloud frames, preferring a variable named pointClouds when available.
%
% Input:
%   info: struct array from whos("-file", matPath)
%
% Output:
%   varName: string scalar chosen variable name or empty string
%   varClass: string scalar MATLAB class name of the variable
%   varSize: row vector size of the chosen variable
    varName = "";
    varClass = "";
    varSize = [0, 0];
    if isempty(info)
        return;
    end
    names = string({info.name});
    preferredIdx = find(strcmpi(names, "pointClouds"), 1, "first");
    if isempty(preferredIdx)
        preferredIdx = find(contains(lower(names), "pointcloud"), 1, "first");
    end
    if isempty(preferredIdx)
        preferredIdx = 1;
    end
    varName = string(info(preferredIdx).name);
    varClass = string(info(preferredIdx).class);
    varSize = info(preferredIdx).size;
end

function numFrames = inferNumFrames(varSize)
% inferNumFrames: Infer the total frame count from the size vector of
% a MAT-file variable that stores either a vector of frames or a 2D array
% of frame entries.
%
% Input:
%   varSize: numeric row vector from a MAT-file variable size
%
% Output:
%   numFrames: scalar positive integer total number of frames
    dims = double(varSize(:).');
    dims = dims(dims > 0);
    if isempty(dims)
        numFrames = 0;
        return;
    end
    numFrames = max(dims);
end

function idx = frameIndexToSubs(varSize, frameIdx)
% frameIndexToSubs: Convert a linear frame index into MAT-file
% subscripts for a row vector, column vector, or general 2D frame array.
%
% Input:
%   varSize: numeric row vector from a MAT-file variable size
%   frameIdx: scalar positive integer frame index starting at 1
%
% Output:
%   idx: cell array of subscripts suitable for MAT-file indexing
    dims = double(varSize(:).');
    if numel(dims) == 2
        if dims(1) == 1
            idx = {1, frameIdx};
            return;
        end
        if dims(2) == 1
            idx = {frameIdx, 1};
            return;
        end
        [r, c] = ind2sub(dims, frameIdx);
        idx = {r, c};
        return;
    end
    idx = repmat({1}, 1, numel(dims));
    idx{1} = frameIdx;
end
