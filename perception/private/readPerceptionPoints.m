function [xyz, pointIndices, pointAttributes, inputMeta] = readPerceptionPoints(pointCloud)
% readPerceptionPoints: Flatten either an organized point-cloud
% struct or an unorganized [N x 3] matrix into a common point list with
% retained original indices and aligned per-point attributes used by the
% voxelizer and downstream lookup utilities.
%
% Input:
%   pointCloud: struct with x/y/z fields or [N x 3] numeric matrix
%
% Output:
%   xyz: [N x 3] double point coordinates
%   pointIndices: [N x 1] int32 original point indices
%   pointAttributes: struct with range and optional intensity/reflectivity
%       vectors
%   inputMeta: struct with inputType, inputSize, and numInputPoints
    pointAttributes = struct("range", zeros(0, 1));
    inputMeta = struct("inputType", "unorganized", "inputSize", [0, 0], "numInputPoints", 0);

    if isnumeric(pointCloud)
        assert(ismatrix(pointCloud) && size(pointCloud, 2) == 3, ...
            "pointCloud must be an [N x 3] numeric matrix.");
        xyz = double(pointCloud);
        pointIndices = int32((1:size(xyz, 1)).');
        pointAttributes.range = sqrt(sum(xyz.^2, 2));
        inputMeta.inputType = "unorganized";
        inputMeta.inputSize = [size(xyz, 1), 1];
        inputMeta.numInputPoints = size(xyz, 1);
        return;
    end

    assert(isstruct(pointCloud) && isfield(pointCloud, "x") && isfield(pointCloud, "y") && isfield(pointCloud, "z"), ...
        "pointCloud struct input must contain x, y, and z.");

    xVals = double(pointCloud.x);
    yVals = double(pointCloud.y);
    zVals = double(pointCloud.z);
    assert(isequal(size(xVals), size(yVals), size(zVals)), ...
        "pointCloud.x, pointCloud.y, and pointCloud.z must share the same size.");

    xyz = [xVals(:), yVals(:), zVals(:)];
    pointIndices = int32((1:size(xyz, 1)).');
    if isfield(pointCloud, "pointIndices") && isvector(pointCloud.pointIndices) && ...
            numel(pointCloud.pointIndices) == size(xyz, 1)
        candidatePointIndices = int32(pointCloud.pointIndices(:));
        pointIndices = candidatePointIndices;
    elseif isfield(pointCloud, "pointIndices") && numel(pointCloud.pointIndices) == numel(pointCloud.x)
        candidatePointIndices = int32(pointCloud.pointIndices(:));
        if isvector(candidatePointIndices) && numel(candidatePointIndices) == size(xyz, 1)
            pointIndices = int32(candidatePointIndices);
        end
    end
    if isfield(pointCloud, "range") && isequal(size(pointCloud.range), size(pointCloud.x))
        pointAttributes.range = double(pointCloud.range(:));
    else
        pointAttributes.range = sqrt(sum(xyz.^2, 2));
    end
    if isfield(pointCloud, "intensity") && isequal(size(pointCloud.intensity), size(pointCloud.x))
        pointAttributes.intensity = double(pointCloud.intensity(:));
    end
    if isfield(pointCloud, "reflectivity") && isequal(size(pointCloud.reflectivity), size(pointCloud.x))
        pointAttributes.reflectivity = double(pointCloud.reflectivity(:));
    end

    inputMeta.inputType = "organized";
    inputMeta.inputSize = size(pointCloud.x);
    inputMeta.numInputPoints = numel(pointCloud.x);
end
