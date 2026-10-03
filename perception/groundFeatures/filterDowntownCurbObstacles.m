function [keepPoint,metrics,blockedPoint] = filterDowntownCurbObstacles(points,allPoints,cellLinIdx,mapSize,curbPointMask,cfg)
% ---------------------------------------------------------------------
% filterDowntownCurbObstacles: Remove connected curb candidates dominated
% by nearby elevated obstacle returns. For each original candidate, count
% points in a horizontal cylinder and a relative height band, requiring
% vertical span and multiple occupied height bins. Reject an entire
% eight-connected XY component when the configured fraction has this evidence,
% including short edge fragments offset from the obstacle projection.
% Use all retained raw points independently of ground segmentation. This
% is conservative obstacle rejection, not a vehicle semantic classifier;
% real curbs close to structures can also be removed. Never add predictions.
%
% Input:
%   points: N-by-3 original ground coordinates in meters.
%   allPoints: M-by-3 retained original ground and off-ground coordinates.
%   cellLinIdx: N-by-1 XY indices in [Nx Ny] ordering.
%   mapSize: [Ny Nx] dimensions of the XY raster.
%   curbPointMask: N-by-1 logical candidate curb membership.
%   cfg: downtownCurbConfig().obstacleClearance thresholds; no scene-specific regions.
%
% Output:
%   keepPoint: N-by-1 filtered curb membership, a subset of the input.
%   metrics: pre-filter component IDs, support counts, blocked fraction,
%       acceptance, and rejection reason for every connected component.
%   blockedPoint: N-by-1 logical elevated-support evidence at each tested
%       candidate independently of the component-level rejection fraction.
% ---------------------------------------------------------------------
    occupied = false(fliplr(mapSize));
    occupied(cellLinIdx(curbPointMask)) = true;
    components = bwconncomp(occupied.',8);
    labelMap = labelmatrix(components).';
    selected = find(curbPointMask);
    labels = double(labelMap(cellLinIdx(selected)));
    count = components.NumObjects;
    metrics = table((1:count).',zeros(count,1),zeros(count,1),zeros(count,1), ...
        zeros(count,1),zeros(count,1),false(count,1),strings(count,1), ...
        VariableNames=["componentId","pointCount","blockedPoints", ...
        "blockedFraction","medianElevatedPoints","medianOccupiedHeightBins", ...
        "accepted","rejectReason"]);
    keepPoint = false(size(curbPointMask));
    blockedPoint = false(size(curbPointMask));
    allPoints = double(allPoints);
    radius = cfg.searchRadiusMeters;
    low = cfg.minHeightAboveCandidateMeters;
    high = cfg.maxHeightAboveCandidateMeters;
    numBins = ceil((high-low)/cfg.heightBinMeters);
    for k = 1:count
        rows = selected(labels==k);
        xyz = double(points(rows,:));
        lower = min(xyz,[],1)+[-radius,-radius,low];
        upper = max(xyz,[],1)+[radius,radius,high];
        nearby = allPoints(all(allPoints>=lower & allPoints<=upper,2),:);
        elevatedCount = zeros(numel(rows),1);
        occupiedBins = zeros(numel(rows),1);
        span = zeros(numel(rows),1);
        % Chunk the exact cylinder tests so a long curb next to a dense
        % building cannot allocate an unbounded point-pair matrix.
        blockSize = max(1,floor(250000/max(size(nearby,1),1)));
        for first = 1:blockSize:numel(rows)
            chunk = first:min(first+blockSize-1,numel(rows));
            dx = nearby(:,1)-xyz(chunk,1).';
            dy = nearby(:,2)-xyz(chunk,2).';
            dz = nearby(:,3)-xyz(chunk,3).';
            support = dx.^2+dy.^2<=radius^2 & dz>=low & dz<=high;
            elevatedCount(chunk) = sum(support,1).';
            heightBins = min(floor((dz-low)/cfg.heightBinMeters),numBins-1);
            for bin = 0:numBins-1
                occupiedBins(chunk) = occupiedBins(chunk)+any(support & heightBins==bin,1).';
            end
            minHeight = dz;
            maxHeight = dz;
            minHeight(~support) = inf;
            maxHeight(~support) = -inf;
            if ~isempty(nearby)
                span(chunk) = (max(maxHeight,[],1)-min(minHeight,[],1)).';
            end
        end
        blocked = elevatedCount>=cfg.minElevatedPoints & ...
            span>=cfg.minVerticalSpanMeters & occupiedBins>=cfg.minOccupiedHeightBins;
        blockedPoint(rows) = blocked;
        metrics.pointCount(k) = numel(rows);
        metrics.blockedPoints(k) = nnz(blocked);
        metrics.blockedFraction(k) = mean(blocked);
        metrics.medianElevatedPoints(k) = median(elevatedCount);
        metrics.medianOccupiedHeightBins(k) = median(occupiedBins);
        metrics.accepted(k) = metrics.blockedFraction(k)<cfg.minBlockedFraction;
        if ~metrics.accepted(k)
            metrics.rejectReason(k) = "elevatedObstacleSupport";
        end
        keepPoint(rows) = metrics.accepted(k);
    end
end
