function [keepPoint,metrics] = validateDowntownPoleShape(points,componentIds,cfg)
% ---------------------------------------------------------------------
% validateDowntownPoleShape: Reject native pole components whose original
% points lack vertical, slender support. Fit each component's covariance
% principal axes; the leading eigenvalue fraction measures how strongly
% the observed distribution follows one axis. Check that axis against Z
% and require sample count and the height of a continuously sampled run.
% Reject components with excessive gaps between sorted point heights so
% distant fragments cannot inflate apparent pole height. Frozen profiles
% without a gap limit retain the original whole-component height behavior.
% Optional height-profile validation takes median XY positions in height
% slices, removes the fitted linear lean, and rejects repeated lateral
% jumps between successive populated slices. Tall but misaligned fragments
% cannot pass solely because their global covariance is slender.
% Never add pole points.
%
% Input:
%   points: N-by-3 finite original point coordinates in meters.
%   componentIds: N-by-1 positive native XY-component IDs.
%   cfg: downtownStructuralConfig().pointShape with geometry and sample thresholds.
%
% Output:
%   keepPoint: N-by-1 logical accepted membership.
%   metrics: per-component sample count, observed and continuous height,
%       largest gap, run count, covariance ratio, axis tilt, acceptance,
%       and explicit rejection reason table. Optional profile diagnostics
%       expose populated slice count, RMS residual, maximum residual step,
%       and number of steps exceeding the configured lateral tolerance.
% ---------------------------------------------------------------------
    assert(size(points,2)==3 && size(points,1)==numel(componentIds));
    assert(all(isfinite(points),"all") && all(componentIds>0));
    components = unique(componentIds(:));
    gapLimit = inf;
    if isfield(cfg,"maxVerticalGapMeters")
        gapLimit = cfg.maxVerticalGapMeters;
    end
    keepPoint = false(size(componentIds(:)));
    metrics = table(components,zeros(numel(components),1),zeros(numel(components),1), ...
        zeros(numel(components),1),nan(numel(components),1),false(numel(components),1), ...
        strings(numel(components),1),VariableNames=["componentId","pointCount","heightMeters", ...
        "principalVarianceFraction","axisTiltDegrees","accepted","rejectReason"]);
    metrics.continuousHeightMeters = zeros(numel(components),1);
    metrics.maxVerticalGapMeters = zeros(numel(components),1);
    metrics.verticalRunCount = zeros(numel(components),1);
    checkProfile = isfield(cfg,"axisProfile") && cfg.axisProfile.enabled;
    if checkProfile
        metrics.profileSlices = zeros(numel(components),1);
        metrics.profileRmsMeters = nan(numel(components),1);
        metrics.maxProfileStepMeters = nan(numel(components),1);
        metrics.largeProfileSteps = zeros(numel(components),1);
    end
    for k = 1:numel(components)
        selected = componentIds==components(k);
        xyz = double(points(selected,:));
        metrics.pointCount(k) = size(xyz,1);
        metrics.heightMeters(k) = max(xyz(:,3))-min(xyz(:,3));
        heights = sort(xyz(:,3));
        gaps = diff(heights);
        boundaries = find(gaps>gapLimit);
        runStarts = [1;boundaries+1];
        runEnds = [boundaries;numel(heights)];
        metrics.continuousHeightMeters(k) = max(heights(runEnds)-heights(runStarts));
        metrics.maxVerticalGapMeters(k) = max([0;gaps]);
        metrics.verticalRunCount(k) = numel(runStarts);
        if size(xyz,1)<cfg.minPoints
            metrics.rejectReason(k) = "insufficientPoints";
            continue;
        end
        centered = xyz-mean(xyz,1);
        [vectors,values] = eig(centered.'*centered,"vector");
        values = max(real(values),0);
        [largest,index] = max(values);
        if sum(values)<=0
            metrics.rejectReason(k) = "degenerateShape";
            continue;
        end
        metrics.principalVarianceFraction(k) = largest/sum(values);
        metrics.axisTiltDegrees(k) = acosd(min(1,abs(vectors(3,index))));
        rejected = [metrics.continuousHeightMeters(k)<cfg.minHeightMeters, ...
            metrics.principalVarianceFraction(k)<cfg.minPrincipalVarianceFraction, ...
            metrics.axisTiltDegrees(k)>cfg.maxAxisTiltDegrees, ...
            metrics.maxVerticalGapMeters(k)>gapLimit,false];
        reasons = ["insufficientHeight","notSlender","notVertical","verticalGap","axisProfileDiscontinuity"];
        if checkProfile
            bins = floor((xyz(:,3)-min(xyz(:,3)))/cfg.axisProfile.sliceHeightMeters)+1;
            count = max(bins);
            profile = [accumarray(bins,xyz(:,1),[count,1],@median,NaN), ...
                accumarray(bins,xyz(:,2),[count,1],@median,NaN), ...
                accumarray(bins,xyz(:,3),[count,1],@median,NaN)];
            profile = profile(all(isfinite(profile),2),:);
            metrics.profileSlices(k) = size(profile,1);
            profileRejected = true;
            if size(profile,1)>=3
                design = [profile(:,3)-mean(profile(:,3)),ones(size(profile,1),1)];
                residual = profile(:,1:2)-design*(design\profile(:,1:2));
                steps = vecnorm(diff(residual,1,1),2,2);
                metrics.profileRmsMeters(k) = sqrt(mean(sum(residual.^2,2)));
                metrics.maxProfileStepMeters(k) = max(steps);
                metrics.largeProfileSteps(k) = nnz(steps>cfg.axisProfile.maxResidualStepMeters);
                profileRejected = metrics.largeProfileSteps(k)>cfg.axisProfile.maxLargeStepCount;
            end
            rejected(5) = profileRejected;
        end
        metrics.rejectReason(k) = strjoin(reasons(rejected),"+");
        metrics.accepted(k) = ~any(rejected);
        keepPoint(selected) = metrics.accepted(k);
    end
end
