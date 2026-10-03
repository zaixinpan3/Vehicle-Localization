function [keepPoint,metrics] = validateDowntownPoleContext(points,pointIds,componentIds,contextPoints,contextIds,cfg)
% ---------------------------------------------------------------------
% validateDowntownPoleContext: Reject pole-shaped columns embedded in adjoining
% point-cloud structures. Fit x(z), y(z) to each original candidate and
% examine an annulus around this axis, excluding candidate IDs and the
% observed upper/lower ends. Require external returns in many height
% slices, close lateral contacts in multiple slices, and more surrounding
% than candidate samples before rejecting the whole component. Localized
% bases, attached sign heads, and isolated background points alone do not
% trigger density rejection. Optional lateral-attachment validation traces
% measured point connections in overlapping horizontal slabs. Extensive
% branches at separated heights reject the component even when its overall
% surroundings are sparse. This geometry does not prove object semantics.
% No annotations, frame numbers, or fixed scene coordinates are consulted.
%
% Input:
%   points: N-by-3 finite candidate coordinates in meters.
%   pointIds: N-by-1 original organized-frame indices.
%   componentIds: N-by-1 native or recovered positive component IDs.
%   contextPoints: M-by-3 retained raw ground/off-ground coordinates.
%   contextIds: M-by-1 original organized-frame indices aligned to context.
%   cfg: downtownStructuralConfig().context thresholds.
%
% Output:
%   keepPoint: N-by-1 logical subset retaining entire accepted components.
%   metrics: per-component context density, vertical persistence, contact
%       support, object/context ratio, acceptance, and rejection reason.
%       Optional branch diagnostics report supported windows, vertical
%       separation, lateral reach, and external connected-point support.
% ---------------------------------------------------------------------
    assert(size(points,2)==3 && size(points,1)==numel(pointIds) && numel(pointIds)==numel(componentIds));
    assert(size(contextPoints,2)==3 && size(contextPoints,1)==numel(contextIds));
    components = unique(componentIds(:));
    count = numel(components);
    keepPoint = false(size(componentIds(:)));
    metrics = table(components,zeros(count,1),zeros(count,1),zeros(count,1), ...
        zeros(count,1),zeros(count,1),zeros(count,1),nan(count,1), ...
        false(count,1),strings(count,1),VariableNames=["componentId","pointCount", ...
        "contextPoints","heightSlices","crowdedSlices","crowdedSliceFraction", ...
        "contactSlices","objectToContextPointRatio","accepted","rejectReason"]);
    contextPoints = double(contextPoints);
    checkAttachment = isfield(cfg,"lateralAttachment") && cfg.lateralAttachment.enabled;
    if checkAttachment
        metrics.branchWindows = zeros(count,1);
        metrics.branchHeightSpanMeters = zeros(count,1);
        metrics.maxBranchReachMeters = zeros(count,1);
        metrics.maxBranchExternalPoints = zeros(count,1);
    end
    for k = 1:count
        selected = componentIds==components(k);
        xyz = double(points(selected,:));
        metrics.pointCount(k) = size(xyz,1);
        centerZ = mean(xyz(:,3));
        design = [xyz(:,3)-centerZ,ones(size(xyz,1),1)];
        if rank(design)<2
            metrics.rejectReason(k) = "degenerateAxis";
            continue;
        end
        model = design\xyz(:,1:2);
        low = min(xyz(:,3))+cfg.endMarginMeters;
        high = max(xyz(:,3))-cfg.endMarginMeters;
        if high<=low
            metrics.rejectReason(k) = "insufficientInteriorHeight";
            continue;
        end
        countSlices = ceil((high-low)/cfg.sliceHeightMeters);
        axisXY = [contextPoints(:,3)-centerZ,ones(size(contextPoints,1),1)]*model;
        distance = vecnorm(contextPoints(:,1:2)-axisXY,2,2);
        nearby = contextPoints(:,3)>=low & contextPoints(:,3)<=high & ...
            distance>cfg.coreRadiusMeters & distance<=cfg.contextRadiusMeters & ...
            ~ismember(contextIds,pointIds(selected));
        sliceIds = min(floor((contextPoints(nearby,3)-low)/cfg.sliceHeightMeters)+1,countSlices);
        sliceCounts = accumarray(sliceIds,1,[countSlices,1]);
        contactCounts = accumarray(sliceIds,double(distance(nearby)<=cfg.contactRadiusMeters),[countSlices,1]);
        metrics.contextPoints(k) = nnz(nearby);
        metrics.heightSlices(k) = countSlices;
        metrics.crowdedSlices(k) = nnz(sliceCounts>=cfg.minContextPointsPerSlice);
        metrics.crowdedSliceFraction(k) = metrics.crowdedSlices(k)/countSlices;
        metrics.contactSlices(k) = nnz(contactCounts>=cfg.minContactPointsPerSlice);
        metrics.objectToContextPointRatio(k) = size(xyz,1)/(size(xyz,1)+nnz(nearby));
        embedded = metrics.crowdedSliceFraction(k)>=cfg.minCrowdedSliceFraction && ...
            metrics.contactSlices(k)>=cfg.minContactSlices && ...
            metrics.objectToContextPointRatio(k)<cfg.minObjectToContextPointRatio;
        attached = false;
        if checkAttachment
            [attached,branch] = validateLateralAttachment(contextPoints, ...
                ismember(contextIds,pointIds(selected)),distance,low,high, ...
                cfg.coreRadiusMeters,cfg.lateralAttachment);
            metrics.branchWindows(k) = branch.windows;
            metrics.branchHeightSpanMeters(k) = branch.heightSpanMeters;
            metrics.maxBranchReachMeters(k) = branch.maxReachMeters;
            metrics.maxBranchExternalPoints(k) = branch.maxExternalPoints;
        end
        metrics.accepted(k) = ~(embedded || attached);
        if embedded
            metrics.rejectReason(k) = "persistentNearbyStructure";
        elseif attached
            metrics.rejectReason(k) = "lateralStructureConnections";
        end
        keepPoint(selected) = metrics.accepted(k);
    end
end

function [attached,metrics] = validateLateralAttachment(points,isCandidate,distance,low,high,coreRadius,cfg)
% ---------------------------------------------------------------------
% validateLateralAttachment: Trace local measured-point connectivity from
% candidate pole returns into surrounding horizontal structures. Each thin
% overlapping height slab builds a graph whose edges join samples within
% the configured 3-D gap. Only graph components containing real candidate
% samples contribute external support. Require enough external points and
% lateral reach at vertically separated heights, so a single localized
% attachment or a nearby disconnected surface alone does not reject a pole.
%
% Input:
%   points: M-by-3 finite retained coordinates in meters.
%   isCandidate: M-by-1 original-ID membership for the current component.
%   distance: M-by-1 horizontal distance from the fitted candidate axis.
%   low, high: interior observed height bounds after excluding both ends.
%   coreRadius: radius excluded from external branch support, in meters.
%   cfg: downtownStructuralConfig().context.lateralAttachment thresholds.
%
% Output:
%   attached: true when measured branches span the minimum height separation.
%   metrics: supported window count, height span, maximum branch radius,
%       and maximum external connected-point count in a supported window.
% ---------------------------------------------------------------------
    nearby = distance<=cfg.contextRadiusMeters & points(:,3)>=low & points(:,3)<=high;
    points = points(nearby,:);
    isCandidate = isCandidate(nearby);
    distance = distance(nearby);
    centers = (low:cfg.heightStepMeters:high).';
    supported = false(size(centers));
    reach = zeros(size(centers));
    support = zeros(size(centers));
    for j = 1:numel(centers)
        inSlab = abs(points(:,3)-centers(j))<=cfg.halfHeightMeters;
        samples = points(inSlab,:);
        candidate = isCandidate(inSlab);
        radius = distance(inSlab);
        if ~any(candidate) || size(samples,1)<=cfg.minExternalPoints
            continue;
        end
        squaredDistance = (samples(:,1)-samples(:,1).').^2 + ...
            (samples(:,2)-samples(:,2).').^2 + (samples(:,3)-samples(:,3).').^2;
        neighbors = sparse(triu(squaredDistance<=cfg.maxPointGapMeters^2,1));
        groups = conncomp(graph(neighbors,"upper")).';
        connected = ismember(groups,groups(candidate));
        external = connected & ~candidate & radius>coreRadius;
        if nnz(external)<cfg.minExternalPoints
            continue;
        end
        if max(radius(external))<cfg.minReachMeters
            continue;
        end
        supported(j) = true;
        reach(j) = max(radius(external));
        support(j) = nnz(external);
    end
    metrics = struct("windows",nnz(supported),"heightSpanMeters",0, ...
        "maxReachMeters",0,"maxExternalPoints",0);
    if any(supported)
        metrics.heightSpanMeters = max(centers(supported))-min(centers(supported));
        metrics.maxReachMeters = max(reach(supported));
        metrics.maxExternalPoints = max(support(supported));
    end
    attached = metrics.heightSpanMeters>=cfg.minHeightSeparationMeters;
end
