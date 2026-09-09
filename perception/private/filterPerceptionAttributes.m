function pointAttributes = filterPerceptionAttributes(sourceAttributes, sourceLocalIdx)
% filterPerceptionAttributes: Filter aligned per-point attribute
% vectors from a source voxel grid to a derived off-ground voxel subset.
%
% Input:
%   sourceAttributes: struct of source point attributes
%   sourceLocalIdx: [K x 1] local retained-point indices to keep
%
% Output:
%   pointAttributes: struct with filtered point attributes
    pointAttributes = struct("range", zeros(0, 1));
    if ~isstruct(sourceAttributes)
        return;
    end
    fieldNames = string(fieldnames(sourceAttributes));
    for fieldIdx = 1:numel(fieldNames)
        fieldName = char(fieldNames(fieldIdx));
        values = sourceAttributes.(fieldName);
        if isvector(values) && max([sourceLocalIdx(:); 0]) <= numel(values)
            pointAttributes.(fieldName) = values(sourceLocalIdx);
        end
    end
end
