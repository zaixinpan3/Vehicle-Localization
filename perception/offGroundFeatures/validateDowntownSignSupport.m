function [keepPoint,metrics] = validateDowntownSignSupport(points,componentIds,groundPoints,cfg)
% ---------------------------------------------------------------------
% validateDowntownSignSupport: Reject weak high-reflectivity components
% using original-point count and local height above ground. Balance nearby
% ground samples by XY cells, represent each cell with a lower height
% quantile, and robustly fit a local plane by iterative residual trimming.
% Require sufficient nearby inlier cells, a bounded slope and residual,
% and minimum clearance for every point in an accepted component. Reject
% missing/ambiguous ground support rather than substituting sensor height.
% No frame, location, index, or manually annotated region is hardcoded.
%
% Input:
%   points: N-by-3 original high-intensity candidate coordinates in meters.
%   componentIds: N-by-1 positive native XY component IDs.
%   groundPoints: M-by-3 original points classified as ground.
%   cfg: downtownStructuralConfig().signSupport structure.
%
% Output:
%   keepPoint: N-by-1 logical accepted membership; never adds points.
%   metrics: component count, ground-fit support/quality, clearance, final
%       acceptance, and explicit rejection reasons. Accepted points remain
%       geometric candidates, not proven traffic-sign identities.
% ---------------------------------------------------------------------
    assert(size(points,2)==3 && size(points,1)==numel(componentIds));
    assert(size(groundPoints,2)==3 && all(isfinite(groundPoints),"all"));
    assert(all(isfinite(points),"all") && all(componentIds>0));
    components = unique(componentIds(:));
    count = numel(components);
    keepPoint = false(size(componentIds(:)));
    metrics = table(components,zeros(count,1),zeros(count,1),zeros(count,1), ...
        zeros(count,1),nan(count,1),nan(count,1),nan(count,1),nan(count,1),nan(count,1), ...
        false(count,1),strings(count,1),VariableNames=["componentId","pointCount", ...
        "groundPointCount","groundCellCount","groundInlierCells","nearestGroundMeters", ...
        "groundRmsMeters","groundSlope","minHeightAboveGroundMeters", ...
        "medianHeightAboveGroundMeters","accepted","rejectReason"]);
    groundPoints = double(groundPoints);
    for k = 1:count
        selected = componentIds==components(k);
        xyz = double(points(selected,:));
        center = median(xyz(:,1:2),1);
        metrics.pointCount(k) = size(xyz,1);
        distances = vecnorm(groundPoints(:,1:2)-center,2,2);
        local = groundPoints(distances<=cfg.groundSearchRadiusMeters,:);
        metrics.groundPointCount(k) = size(local,1);
        if size(local,1)<cfg.minGroundPoints
            metrics.rejectReason(k) = "insufficientGroundSupport";
            continue;
        end
        [~,~,groups] = unique(floor(local(:,1:2)/cfg.groundCellSizeMeters),"rows");
        cellX = accumarray(groups,local(:,1),[],@median);
        cellY = accumarray(groups,local(:,2),[],@median);
        cellZ = accumarray(groups,local(:,3),[],@(z) quantile(z,cfg.groundCellHeightQuantile));
        metrics.groundCellCount(k) = numel(cellZ);
        design = [cellX-center(1),cellY-center(2),ones(size(cellZ))];
        inliers = true(size(cellZ));
        for iteration = 1:cfg.groundFitIterations
            if nnz(inliers)<cfg.minGroundCells || rank(design(inliers,:))<3
                break;
            end
            model = design(inliers,:)\cellZ(inliers);
            residual = cellZ-design*model;
            nextInliers = abs(residual-median(residual(inliers)))<=cfg.groundResidualToleranceMeters;
            if isequal(nextInliers,inliers)
                break;
            end
            inliers = nextInliers;
        end
        metrics.groundInlierCells(k) = nnz(inliers);
        if nnz(inliers)<cfg.minGroundCells || mean(inliers)<cfg.minGroundInlierFraction || rank(design(inliers,:))<3
            metrics.rejectReason(k) = "insufficientGroundSupport";
            continue;
        end
        model = design(inliers,:)\cellZ(inliers);
        residual = cellZ(inliers)-design(inliers,:)*model;
        metrics.groundRmsMeters(k) = sqrt(mean(residual.^2));
        metrics.groundSlope(k) = norm(model(1:2));
        metrics.nearestGroundMeters(k) = min(vecnorm(design(inliers,1:2),2,2));
        height = xyz(:,3)-[xyz(:,1:2)-center,ones(size(xyz,1),1)]*model;
        metrics.minHeightAboveGroundMeters(k) = min(height);
        metrics.medianHeightAboveGroundMeters(k) = median(height);
        rejected = [metrics.pointCount(k)<cfg.minCandidatePoints, ...
            metrics.nearestGroundMeters(k)>cfg.maxNearestGroundDistanceMeters, ...
            metrics.groundRmsMeters(k)>cfg.maxGroundRmsMeters, ...
            metrics.groundSlope(k)>cfg.maxGroundSlope, ...
            metrics.minHeightAboveGroundMeters(k)<cfg.minHeightAboveGroundMeters];
        reasons = ["insufficientPoints","distantGroundSupport","unstableGroundFit", ...
            "steepGroundFit","lowReflector"];
        metrics.rejectReason(k) = strjoin(reasons(rejected),"+");
        metrics.accepted(k) = ~any(rejected);
        keepPoint(selected) = metrics.accepted(k);
    end
end
