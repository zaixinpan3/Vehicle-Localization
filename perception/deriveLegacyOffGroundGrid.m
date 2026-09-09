function offGroundVoxelGrid = deriveLegacyOffGroundGrid(sourceVoxelGrid, sourceLocalIdx, storeDenseCount)
% deriveLegacyOffGroundGrid: Build a compact canonical voxel grid for
% selected off-ground points by cropping and reindexing the source frame
% voxel grid.
%
% Input:
%   sourceVoxelGrid: canonical voxelizePointCloud output for the frame
%   sourceLocalIdx: [K x 1] local retained-point indices to keep
%   storeDenseCount: logical scalar; false keeps only sparse point/voxel
%       lookup arrays for the coarse probability-cloud path
%
% Output:
%   offGroundVoxelGrid: canonical voxelizePointCloud-compatible subset
    if nargin < 3
        storeDenseCount = true;
    end
    sourceLocalIdx = double(sourceLocalIdx(:));
    sourceLocalIdx = sourceLocalIdx(isfinite(sourceLocalIdx) & sourceLocalIdx >= 1 & sourceLocalIdx <= size(sourceVoxelGrid.points, 1) & sourceLocalIdx == floor(sourceLocalIdx));
    if any(diff(sourceLocalIdx)<=0), sourceLocalIdx = unique(sourceLocalIdx,"stable"); end
    voxelSize = double(sourceVoxelGrid.gridConfig.voxelSize(1:3));
    sourceMinCorner = double(sourceVoxelGrid.gridConfig.minCorner(1:3));
    offGroundVoxelGrid = emptyDerivedVoxelGrid(sourceVoxelGrid, voxelSize);
    if isempty(sourceLocalIdx)
        return;
    end
    sourceSub = double(sourceVoxelGrid.pointVoxelSub(sourceLocalIdx, 1:3));
    minSub = min(sourceSub, [], 1);
    maxSub = max(sourceSub, [], 1);
    dims = maxSub - minSub + 1;
    shiftedSub = sourceSub - minSub + 1;
    pointVoxelLinIdx = sub2ind(double(dims), shiftedSub(:, 1), shiftedSub(:, 2), shiftedSub(:, 3));
    if storeDenseCount
        count = single(accumarray(shiftedSub, 1, double(dims), @sum, 0));
    else
        count = zeros(0, 0, 0, "single");
    end
    if storeDenseCount
        [occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets, voxelPointLocalIdx] = buildDerivedVoxelPointMapping(pointVoxelLinIdx, dims);
    else
        occupiedVoxelLinIdx = zeros(0,1,"int32"); occupiedVoxelSub = zeros(0,3,"int32");
        voxelPointOffsets = int32(1); voxelPointLocalIdx = zeros(0,1,"int32");
    end
    selectedPointIndices = int32(sourceVoxelGrid.pointIndices(sourceLocalIdx));
    voxelPointIndices = int32(selectedPointIndices(double(voxelPointLocalIdx(:))));
    minCorner = sourceMinCorner + ((minSub - 1) .* voxelSize);
    maxCorner = minCorner + (dims .* voxelSize);

    offGroundVoxelGrid.gridConfig.dims = double(dims(:).');
    offGroundVoxelGrid.gridConfig.origin = double(minCorner(:).' + (0.5 .* voxelSize(:).'));
    offGroundVoxelGrid.gridConfig.minCorner = double(minCorner(:).');
    offGroundVoxelGrid.gridConfig.maxCorner = double(maxCorner(:).');
    offGroundVoxelGrid.gridConfig.roiLimits = [double(minCorner(1)), double(maxCorner(1)), double(minCorner(2)), double(maxCorner(2)), double(minCorner(3)), double(maxCorner(3))];
    offGroundVoxelGrid.count = count;
    offGroundVoxelGrid.sumX = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumY = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumXX = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumYY = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumZZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumXY = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumXZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.sumYZ = zeros(0, 0, 0, "single");
    offGroundVoxelGrid.points = double(sourceVoxelGrid.points(sourceLocalIdx, :));
    offGroundVoxelGrid.pointIndices = selectedPointIndices;
    offGroundVoxelGrid.pointVoxelSub = int32(shiftedSub);
    offGroundVoxelGrid.pointVoxelLinIdx = int32(pointVoxelLinIdx(:));
    offGroundVoxelGrid.occupiedVoxelLinIdx = int32(occupiedVoxelLinIdx(:));
    offGroundVoxelGrid.occupiedVoxelSub = int32(occupiedVoxelSub);
    offGroundVoxelGrid.voxelPointOffsets = int32(voxelPointOffsets(:));
    offGroundVoxelGrid.voxelPointLocalIdx = int32(voxelPointLocalIdx(:));
    offGroundVoxelGrid.voxelPointIndices = int32(voxelPointIndices(:));
    offGroundVoxelGrid.pointAttributes = filterPerceptionAttributes(sourceVoxelGrid.pointAttributes, sourceLocalIdx);
    offGroundVoxelGrid.numFilteredPoints = double(numel(sourceLocalIdx));
    offGroundVoxelGrid.numOccupiedVoxels = double(numel(occupiedVoxelLinIdx));
    offGroundVoxelGrid.hasPointLookup = storeDenseCount;
    if ~storeDenseCount, offGroundVoxelGrid.numOccupiedVoxels = NaN; end
end

function voxelGrid = emptyDerivedVoxelGrid(sourceVoxelGrid, voxelSize)
% emptyDerivedVoxelGrid: Create an empty voxelizePointCloud-compatible
% struct that can be filled by deriveLegacyOffGroundGrid or returned
% directly when no off-ground points survive the ground split.
%
% Input:
%   sourceVoxelGrid: source canonical voxel grid for metadata defaults
%   voxelSize: [1 x 3] voxel size in meters
%
% Output:
%   voxelGrid: empty canonical voxel-grid struct
    voxelGrid = struct();
    voxelGrid.spatialIndexType = "voxelGrid";
    voxelGrid.gridConfig = struct("dims", [0, 0, 0], "voxelSize", double(voxelSize(:).'), "origin", [0, 0, 0], "minCorner", [0, 0, 0], "maxCorner", [0, 0, 0], "roiLimits", [0, 0, 0, 0, 0, 0], "countLayout", "NxNyNz");
    voxelGrid.count = zeros(0, 0, 0, "single");
    voxelGrid.sumX = zeros(0, 0, 0, "single");
    voxelGrid.sumY = zeros(0, 0, 0, "single");
    voxelGrid.sumZ = zeros(0, 0, 0, "single");
    voxelGrid.sumXX = zeros(0, 0, 0, "single");
    voxelGrid.sumYY = zeros(0, 0, 0, "single");
    voxelGrid.sumZZ = zeros(0, 0, 0, "single");
    voxelGrid.sumXY = zeros(0, 0, 0, "single");
    voxelGrid.sumXZ = zeros(0, 0, 0, "single");
    voxelGrid.sumYZ = zeros(0, 0, 0, "single");
    voxelGrid.points = zeros(0, 3);
    voxelGrid.pointIndices = zeros(0, 1, "int32");
    voxelGrid.pointVoxelSub = zeros(0, 3, "int32");
    voxelGrid.pointVoxelLinIdx = zeros(0, 1, "int32");
    voxelGrid.occupiedVoxelLinIdx = zeros(0, 1, "int32");
    voxelGrid.occupiedVoxelSub = zeros(0, 3, "int32");
    voxelGrid.voxelPointOffsets = int32(1);
    voxelGrid.voxelPointLocalIdx = zeros(0, 1, "int32");
    voxelGrid.voxelPointIndices = zeros(0, 1, "int32");
    voxelGrid.pointAttributes = filterPerceptionAttributes(sourceVoxelGrid.pointAttributes, zeros(0, 1));
    voxelGrid.inputType = "derived";
    voxelGrid.inputSize = [0, 1];
    voxelGrid.numInputPoints = double(sourceVoxelGrid.numFilteredPoints);
    voxelGrid.numFilteredPoints = 0;
    voxelGrid.numOccupiedVoxels = 0;
end

function [occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets, voxelPointLocalIdx] = buildDerivedVoxelPointMapping(pointVoxelLinIdx, dims)
% buildDerivedVoxelPointMapping: Build CSR-style occupied-voxel
% lookup arrays from compact derived point voxel indices.
%
% Input:
%   pointVoxelLinIdx: [K x 1] linear voxel indices in compact grid layout
%   dims: [1 x 3] compact voxel grid dimensions
%
% Output:
%   occupiedVoxelLinIdx, occupiedVoxelSub, voxelPointOffsets,
%       voxelPointLocalIdx: voxelizePointCloud-compatible mapping arrays
    occupiedVoxelLinIdx = zeros(0, 1, "int32");
    occupiedVoxelSub = zeros(0, 3, "int32");
    voxelPointOffsets = int32(1);
    voxelPointLocalIdx = zeros(0, 1, "int32");
    if isempty(pointVoxelLinIdx)
        return;
    end
    [sortedLinIdx, sortOrder] = sort(double(pointVoxelLinIdx(:)));
    voxelPointLocalIdx = int32(sortOrder(:));
    occupiedMask = [true; diff(sortedLinIdx) ~= 0];
    occupiedStartIdx = find(occupiedMask);
    occupiedVoxelLinIdx = int32(sortedLinIdx(occupiedStartIdx));
    voxelPointOffsets = int32([occupiedStartIdx; numel(sortedLinIdx) + 1]);
    [xSub, ySub, zSub] = ind2sub(double(dims), double(occupiedVoxelLinIdx));
    occupiedVoxelSub = int32([xSub(:), ySub(:), zSub(:)]);
end
