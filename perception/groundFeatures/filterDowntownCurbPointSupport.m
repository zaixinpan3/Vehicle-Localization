function [keepPoint,metrics] = filterDowntownCurbPointSupport(points,cellLinIdx,mapSize,curbPointMask,cfg)
% ---------------------------------------------------------------------
% filterDowntownCurbPointSupport: Reject short, isolated connected curb fragments
% using final candidate points. Form eight-connected XY-cell components,
% measure their point count, distinct cell count, and horizontal extent,
% and remove components without sufficient support. Never add predictions.
%
% Input:
%   points: N-by-3 original ground coordinates in meters.
%   cellLinIdx: N-by-1 cell indices stored in [Nx Ny] ordering.
%   mapSize: [Ny Nx] dimensions of the XY raster.
%   curbPointMask: N-by-1 logical candidate curb membership.
%   cfg: downtownCurbConfig().pointSupport thresholds.
%
% Output:
%   keepPoint: N-by-1 filtered curb membership.
%   metrics: per-component point/cell counts, horizontal extent, acceptance,
%       and rejection reasons, using pre-filter component IDs.
% ---------------------------------------------------------------------
    occupied = false(fliplr(mapSize));
    occupied(cellLinIdx(curbPointMask)) = true;
    components = bwconncomp(occupied.',8);
    labelMap = labelmatrix(components).';
    selected = find(curbPointMask);
    labels = double(labelMap(cellLinIdx(selected)));
    count = components.NumObjects;
    metrics = table((1:count).',zeros(count,1),zeros(count,1),zeros(count,1), ...
        false(count,1),strings(count,1),VariableNames=["componentId","pointCount", ...
        "occupiedCells","extentMeters","accepted","rejectReason"]);
    keepPoint = false(size(curbPointMask));
    for k = 1:count
        rows = selected(labels==k);
        xy = double(points(rows,1:2));
        metrics.pointCount(k) = numel(rows);
        metrics.occupiedCells(k) = numel(components.PixelIdxList{k});
        metrics.extentMeters(k) = norm(max(xy,[],1)-min(xy,[],1));
        rejected = [metrics.pointCount(k)<cfg.minPoints, ...
            metrics.occupiedCells(k)<cfg.minOccupiedCells, ...
            metrics.extentMeters(k)<cfg.minExtentMeters];
        reasons = ["insufficientPoints","insufficientCells","shortFragment"];
        metrics.rejectReason(k) = strjoin(reasons(rejected),"+");
        metrics.accepted(k) = ~any(rejected);
        keepPoint(rows) = metrics.accepted(k);
    end
end
