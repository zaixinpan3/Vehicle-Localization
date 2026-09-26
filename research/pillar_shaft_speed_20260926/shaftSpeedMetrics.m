function metrics = shaftSpeedMetrics(actual, expected, reference)
% shaftSpeedMetrics: Compare outputs without treating references as truth.
    actualIds = actual.poleCells; expectedIds = expected.poleCells;
    shared = numel(intersect(actualIds,expectedIds));
    joined = numel(union(actualIds,expectedIds));
    metrics = struct('baselineCount',numel(expectedIds), ...
        'candidateCount',numel(actualIds),'sharedCount',shared, ...
        'lostCount',numel(setdiff(expectedIds,actualIds)), ...
        'addedCount',numel(setdiff(actualIds,expectedIds)), ...
        'retention',ratio(shared,numel(expectedIds)), ...
        'jaccard',ratio(shared,joined), ...
        'maskExact',isequal(actualIds,expectedIds), ...
        'sourceExact',isequaln(actual.sourceSummary,expected.sourceSummary), ...
        'componentsExact',isequaln(actual.components,expected.components), ...
        'nonPoleMasksExact',NaN,'nonPoleComponentsExact',true, ...
        'sharedComponentCount',0,'maximumMeanError',0, ...
        'maximumCovarianceError',0,'maximumProbabilityError',0, ...
        'maximumMixtureWeightError',0, ...
        'legacyCount',NaN,'legacyRetained',NaN, ...
        'fineCount',NaN,'fineMatched',NaN,'baselineFineMatched',NaN);
    if isfield(expected,'semanticNames')
        metrics.nonPoleMasksExact = true;
        for name=expected.semanticNames(expected.semanticNames~="pole").'
            ai=find(actual.semanticNames==name,1); ei=find(expected.semanticNames==name,1);
            metrics.nonPoleMasksExact = metrics.nonPoleMasksExact && ...
                ~isempty(ai) && isequal(actual.pillarIndices{ai},expected.pillarIndices{ei});
        end
    end
    a = actual.components; b = expected.components;
    [~,ia,ib] = intersect([double(a.semanticId),double(a.cellLinIdx)], ...
        [double(b.semanticId),double(b.cellLinIdx)],'rows');
    metrics.sharedComponentCount = numel(ia);
    for field={'mean','meanXYZ'}
        metrics.maximumMeanError=max(metrics.maximumMeanError, ...
            maximumDifference(a.(field{1})(ia,:),b.(field{1})(ib,:)));
    end
    for field={'covariance','covarianceXYZ','invCovariance'}
        metrics.maximumCovarianceError=max(metrics.maximumCovarianceError, ...
            maximumDifference(a.(field{1})(:,:,ia),b.(field{1})(:,:,ib)));
    end
    for field={'semanticProbability','occupancyProbability'}
        metrics.maximumProbabilityError=max(metrics.maximumProbabilityError, ...
            maximumDifference(a.(field{1})(ia,:),b.(field{1})(ib,:)));
    end
    metrics.maximumMixtureWeightError = ...
        maximumDifference(a.mixtureWeight(ia,:),b.mixtureWeight(ib,:));
    aa = a.semanticName~="pole"; bb = b.semanticName~="pole";
    for field=fieldnames(a).'
        key=field{1};
        % Global mixture normalization can change when the pole set changes.
        if ismember(key,{'numComponents','mixtureWeight'}), continue; end
        x=a.(key); y=b.(key);
        if ismember(key,{'covariance','covarianceXYZ','invCovariance'})
            exact=isequaln(x(:,:,aa),y(:,:,bb));
        else
            exact=isequaln(x(aa,:),y(bb,:));
        end
        metrics.nonPoleComponentsExact=metrics.nonPoleComponentsExact && exact;
    end
    if ~isempty(reference)
        metrics.legacyCount=numel(reference.legacyCells);
        metrics.legacyRetained=numel(intersect(actualIds,reference.legacyCells));
        if reference.hasFineReference
            metrics.fineCount=numel(reference.fineCells);
            metrics.fineMatched=numel(intersect(actualIds,reference.fineCells));
            metrics.baselineFineMatched=numel(intersect(expectedIds,reference.fineCells));
        end
    end
end

function value = ratio(numerator,denominator)
    if denominator==0, value=1; else, value=numerator/denominator; end
end

function value = maximumDifference(a,b)
    if isempty(a), value=0; return; end
    if ~isequal(isnan(a),isnan(b)) || ~isequal(isinf(a),isinf(b))
        value=Inf; return;
    end
    valid=isfinite(a)&isfinite(b);
    value=max([0;abs(double(a(valid))-double(b(valid)))]);
end
