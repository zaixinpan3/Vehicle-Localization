function [curbMask,metrics,addedMask] = extendDowntownCurbEndpoints(points,allPoints,cellLinIdx,mapSize,energyMaps,anchorMask,cfg,obstacleCfg,eligible)
% ---------------------------------------------------------------------
% extendDowntownCurbEndpoints: Add measured ground samples just beyond a validated
% curb's endpoints. Use long connected anchors free of widespread
% elevated obstacles; robustly fit local y(x) and z(x) models at each end.
% Only narrow, height-consistent candidates with curb-cell evidence qualify.
% Reject candidate points with elevated-obstacle support, stop at the first
% excessive longitudinal sample gap, and cap distance from the original
% endpoint. New points never become anchors for a second extension pass.
% If direct continuation fails, a nearby separated run must independently
% satisfy long-run support, narrow line fitting, height-step evidence, and
% obstacle clearance. Only its measured points are added; gaps stay empty.
% Designed for locally longitudinal road boundaries; ambiguous, curved,
% sparse, or occluded anchors keep their existing labels without extension.
%
% Input:
%   points: N-by-3 original ground coordinates in meters.
%   allPoints: M-by-3 retained raw ground/off-ground coordinates in meters.
%   cellLinIdx: N-by-1 ground-cell indices in [Nx Ny] ordering.
%   mapSize: [Ny Nx] raster dimensions.
%   energyMaps: curb total/base energy, height-step, relative-height, and
%       linearityComponentCenterEvidence rasters from ground processing.
%   anchorMask: N-by-1 accepted curb membership after all rejection gates.
%   cfg: downtownCurbConfig().endpointExtension thresholds.
%   obstacleCfg: downtownCurbConfig().obstacleClearance used for pointwise validation.
%   eligible: optional N-by-1 immutable coarse-candidate membership. Every
%       continuation/search point must belong to this envelope.
%
% Output:
%   curbMask: N-by-1 original anchors plus accepted continuation samples.
%   metrics: direction, component support, local fit quality, added count/extent, and
%       reason when a component cannot safely be extended.
%   addedMask: N-by-1 membership of newly selected original ground points.
% ---------------------------------------------------------------------
    if nargin<9, eligible=true(size(anchorMask)); end
    anchorMask = anchorMask & eligible;
    addedMask = false(size(anchorMask));
    records = cell(numel(cfg.directions),1);
    for j = 1:numel(cfg.directions)
        direction = cfg.directions(j);
        transformed = double(points).*[direction,1,1];
        context = double(allPoints).*[direction,1,1];
        [~,record,added] = extendOneCurbDirection(transformed,context,cellLinIdx,mapSize,energyMaps,anchorMask,cfg,obstacleCfg,eligible);
        record.direction = repmat(direction,height(record),1);
        records{j} = record;
        addedMask = addedMask | added;
    end
    metrics = vertcat(records{:});
    curbMask = anchorMask | addedMask;
end

function [curbMask,metrics,addedMask] = extendOneCurbDirection(points,allPoints,cellLinIdx,mapSize,energyMaps,anchorMask,cfg,obstacleCfg,eligible)
% ---------------------------------------------------------------------
% extendOneCurbDirection: Fit and follow the positive-x endpoint of each
% connected curb anchor in a direction-transformed coordinate system.
% Apply original cell evidence, narrow local geometry, pointwise obstacle
% rejection, and an observed-sample gap/distance bound.
%
% Input:
%   points,allPoints: N-by-3 ground and M-by-3 context in transformed meters.
%   cellLinIdx,mapSize,energyMaps: unchanged original XY-cell mappings/maps.
%   anchorMask: N-by-1 original accepted curb membership.
%   cfg,obstacleCfg: endpoint and obstacle threshold structures.
%
% Output:
%   curbMask,addedMask: N-by-1 final and newly selected original-point masks.
%   metrics: one row per original anchor with fit and continuation status.
% ---------------------------------------------------------------------
    points = double(points);
    occupied = false(fliplr(mapSize));
    occupied(cellLinIdx(anchorMask)) = true;
    components = bwconncomp(occupied.',8);
    labelMap = labelmatrix(components).';
    anchorRows = find(anchorMask);
    labels = double(labelMap(cellLinIdx(anchorRows)));
    count = components.NumObjects;
    metrics = table((1:count).',zeros(count,1),zeros(count,1),zeros(count,1), ...
        nan(count,1),nan(count,1),zeros(count,1),zeros(count,1),strings(count,1), ...
        VariableNames=["componentId","anchorPoints","anchorLengthMeters", ...
        "anchorBlockedFraction","fitRmsMeters","lateralSlope","addedPoints", ...
        "extensionMeters","status"]);
    addedMask = false(size(anchorMask));
    [~,~,blockedAnchors] = filterDowntownCurbObstacles(points,allPoints,cellLinIdx,mapSize,anchorMask,obstacleCfg);
    evidence = eligible;
    fields = ["total","totalBase","heightStepMeters","linearityComponentCenterEvidence"];
    thresholds = [cfg.minTotalEnergy,cfg.minBaseEnergy,cfg.minHeightStepMeters,cfg.minCenterEvidence];
    for j = 1:numel(fields)
        values = energyMaps.(fields(j)).';
        evidence = evidence & values(cellLinIdx)>=thresholds(j);
    end
    relative = energyMaps.relativeHeightMeters.';
    evidence = evidence & relative(cellLinIdx)<=cfg.maxRelativeHeightMeters;
    for k = 1:count
        rows = anchorRows(labels==k);
        xyz = points(rows,:);
        endpoint = max(xyz(:,1));
        metrics.anchorPoints(k) = numel(rows);
        metrics.anchorLengthMeters(k) = endpoint-min(xyz(:,1));
        metrics.anchorBlockedFraction(k) = mean(blockedAnchors(rows));
        if numel(rows)<cfg.minAnchorPoints || metrics.anchorLengthMeters(k)<cfg.minAnchorLengthMeters || ...
                metrics.anchorBlockedFraction(k)>cfg.maxAnchorBlockedFraction
            metrics.status(k) = "weakOrObstructedAnchor";
            continue;
        end
        fitPoints = xyz(xyz(:,1)>=endpoint-cfg.fitLengthMeters,:);
        design = [fitPoints(:,1)-endpoint,ones(size(fitPoints,1),1)];
        inliers = true(size(fitPoints,1),1);
        for iteration = 1:cfg.fitIterations
            if nnz(inliers)<cfg.minFitPoints || rank(design(inliers,:))<2
                break;
            end
            yFit = design(inliers,:)\fitPoints(inliers,2);
            next = abs(fitPoints(:,2)-design*yFit)<=cfg.fitResidualMeters;
            if isequal(next,inliers)
                break;
            end
            inliers = next;
        end
        if nnz(inliers)<cfg.minFitPoints || mean(inliers)<cfg.minFitInlierFraction || rank(design(inliers,:))<2
            metrics.status(k) = "insufficientFitSupport";
            continue;
        end
        yFit = design(inliers,:)\fitPoints(inliers,2);
        zFit = design(inliers,:)\fitPoints(inliers,3);
        residual = fitPoints(inliers,2)-design(inliers,:)*yFit;
        metrics.fitRmsMeters(k) = sqrt(mean(residual.^2));
        metrics.lateralSlope(k) = yFit(1);
        if metrics.fitRmsMeters(k)>cfg.maxFitRmsMeters || abs(yFit(1))>cfg.maxAbsLateralSlope || abs(zFit(1))>cfg.maxAbsHeightSlope
            metrics.status(k) = "ambiguousLocalShape";
            continue;
        end
        distance = points(:,1)-endpoint;
        groundDesign = [distance,ones(size(distance))];
        zResidual = points(:,3)-groundDesign*zFit;
        limits = quantile(fitPoints(inliers,3)-design(inliers,:)*zFit,cfg.heightQuantiles)+ ...
            [-cfg.heightPaddingMeters,cfg.heightPaddingMeters];
        candidate = ~anchorMask & evidence & distance>0 & distance<=cfg.maxExtensionMeters & ...
            abs(points(:,2)-groundDesign*yFit)<=cfg.lateralToleranceMeters & ...
            zResidual>=limits(1) & zResidual<=limits(2);
        [~,~,blocked] = filterDowntownCurbObstacles(points,allPoints,cellLinIdx,mapSize,candidate,obstacleCfg);
        candidateRows = find(candidate & ~blocked);
        [sortedDistance,order] = sort(distance(candidateRows));
        firstGap = find(diff([0;sortedDistance])>cfg.maxSampleGapMeters,1);
        if ~isempty(firstGap)
            order = order(1:firstGap-1);
        end
        candidateRows = candidateRows(order);
        if numel(candidateRows)<cfg.minAddedPoints
            steps = energyMaps.heightStepMeters.';
            search = ~anchorMask & evidence & distance>0 & distance<=cfg.maxExtensionMeters & ...
                abs(points(:,2)-groundDesign*yFit)<=cfg.runSearchToleranceMeters & ...
                zResidual>=limits(1)-cfg.runHeightPaddingMeters & zResidual<=limits(2)+cfg.runHeightPaddingMeters & ...
                steps(cellLinIdx)>=cfg.minRunHeightStepMeters;
            [~,~,blocked] = filterDowntownCurbObstacles(points,allPoints,cellLinIdx,mapSize,search,obstacleCfg);
            candidateRows = findSeparatedCurbRun(points,distance,search & ~blocked,yFit(1),cfg);
            if isempty(candidateRows)
                metrics.status(k) = "insufficientContinuousSamples";
                continue;
            end
            metrics.status(k) = "separatedMeasuredRun";
        else
            metrics.status(k) = "extended";
        end
        addedMask(candidateRows) = true;
        metrics.addedPoints(k) = numel(candidateRows);
        metrics.extensionMeters(k) = max(distance(candidateRows));
    end
    curbMask = anchorMask | addedMask;
end

function rows = findSeparatedCurbRun(points,distance,candidateMask,anchorSlope,cfg)
% ---------------------------------------------------------------------
% findSeparatedCurbRun: Validate the first independently supported run in
% a narrow search corridor beyond a curb endpoint. Split at sample gaps,
% require a substantial measured length and population, robustly fit the
% lateral position, and publish only narrow inliers consistent with the
% anchor heading. Do not label any intervening empty space.
%
% Input:
%   points: N-by-3 transformed ground coordinates in meters.
%   distance: N-by-1 forward distances from the original anchor endpoint.
%   candidateMask: N-by-1 evidence/height/obstacle-qualified search points.
%   anchorSlope: local anchor dy/dx in the transformed coordinates.
%   cfg: downtownCurbConfig().endpointExtension thresholds.
%
% Output:
%   rows: original ground-point rows of the first validated separated run.
% ---------------------------------------------------------------------
    rows = zeros(0,1);
    candidates = find(candidateMask);
    [d,order] = sort(distance(candidates));
    candidates = candidates(order);
    starts = [1;find(diff(d)>cfg.maxSampleGapMeters)+1];
    ends = [starts(2:end)-1;numel(d)];
    for k = 1:numel(starts)
        group = candidates(starts(k):ends(k));
        if numel(group)<cfg.minRunPoints
            continue;
        end
        x = distance(group);
        if min(x)>cfg.maxRunStartDistanceMeters || max(x)-min(x)<cfg.minRunLengthMeters
            continue;
        end
        design = [x-mean(x),ones(size(x))];
        y = points(group,2);
        inliers = true(size(x));
        for iteration = 1:cfg.fitIterations
            if nnz(inliers)<cfg.minRunPoints || rank(design(inliers,:))<2
                break;
            end
            model = design(inliers,:)\y(inliers);
            inliers = abs(y-design*model)<=cfg.fitResidualMeters;
        end
        if nnz(inliers)<cfg.minRunPoints || mean(inliers)<cfg.minFitInlierFraction || rank(design(inliers,:))<2
            continue;
        end
        model = design(inliers,:)\y(inliers);
        residual = y-design*model;
        if sqrt(mean(residual(inliers).^2))>cfg.maxFitRmsMeters || ...
                abs(model(1)-anchorSlope)>cfg.maxRunLateralSlopeDifference
            continue;
        end
        accepted = abs(residual)<=cfg.lateralToleranceMeters;
        acceptedX = sort(x(accepted));
        if numel(acceptedX)>=cfg.minRunPoints && max(acceptedX)-min(acceptedX)>=cfg.minRunLengthMeters && ...
                ~any(diff(acceptedX)>cfg.maxSampleGapMeters)
            rows = group(accepted);
            return;
        end
    end
end
