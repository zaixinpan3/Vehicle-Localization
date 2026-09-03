function components = connectedComponents8(mask)
% connectedComponents8: Return 8-connected component linear-index
% lists for a logical raster mask using MATLAB's optimized connected
% component implementation when available and an explicit-stack fallback
% otherwise.
%
% Input:
%   mask: [Ny x Nx] logical raster mask
%
% Output:
%   components: cell array where each element contains linear indices for
%       one 8-connected component
    mask = logical(mask);
    if exist("bwconncomp", "file") == 2
        connectedComponentStruct = bwconncomp(mask, 8);
        components = connectedComponentStruct.PixelIdxList(:);
        return;
    end

    [numRows, numCols] = size(mask);
    visited = false(numRows, numCols);
    candidateIdx = find(mask);
    components = cell(numel(candidateIdx), 1);
    numComponents = 0;
    for n = 1:numel(candidateIdx)
        seedIdx = candidateIdx(n);
        if visited(seedIdx)
            continue;
        end

        stack = zeros(numel(candidateIdx), 1);
        component = zeros(numel(candidateIdx), 1);
        stackCount = 1;
        stackHead = 1;
        componentCount = 0;
        stack(stackCount) = seedIdx;
        visited(seedIdx) = true;
        while stackHead <= stackCount
            currentIdx = stack(stackHead);
            stackHead = stackHead + 1;
            componentCount = componentCount + 1;
            component(componentCount) = currentIdx;
            [rowIdx, colIdx] = ind2sub([numRows, numCols], currentIdx);
            for rowOffset = -1:1
                neighborRow = rowIdx + rowOffset;
                if neighborRow < 1 || neighborRow > numRows
                    continue;
                end
                for colOffset = -1:1
                    if rowOffset == 0 && colOffset == 0
                        continue;
                    end
                    neighborCol = colIdx + colOffset;
                    if neighborCol < 1 || neighborCol > numCols
                        continue;
                    end
                    neighborIdx = sub2ind([numRows, numCols], neighborRow, neighborCol);
                    if mask(neighborIdx) && ~visited(neighborIdx)
                        stackCount = stackCount + 1;
                        stack(stackCount) = neighborIdx;
                        visited(neighborIdx) = true;
                    end
                end
            end
        end
        numComponents = numComponents + 1;
        components{numComponents, 1} = component(1:componentCount);
    end
    components = components(1:numComponents);
end
