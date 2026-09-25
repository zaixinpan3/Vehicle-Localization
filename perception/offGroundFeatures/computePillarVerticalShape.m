function shape=computePillarVerticalShape(points,pillarIds,evaluate)
% computePillarVerticalShape: Height order statistics of complete pillars.
% These dimensionless distribution summaries do not create vertical bins or
% change point ownership. Outputs follow unique(pillarIds). A zero-height
% pillar has zero shape values. Unevaluated pillars also report zeros.
% gapRatio: largest adjacent height gap divided by the full height span.
% interquartileRatio: central 50 percent height span divided by full span.
% skewness and kurtosis: population standardized third and fourth moments.
% lowerHalfHeight and upperHalfHeight: median-to-5/95-percentile spans [m].
    assert(isnumeric(points) && size(points,2)==3 && all(isfinite(points(:))) && ...
        numel(pillarIds)==size(points,1) && all(isfinite(pillarIds(:))) && ...
        all(pillarIds(:)>=1 & pillarIds(:)==fix(pillarIds(:))),'perception:InvalidPillarShape', ...
        'Finite XYZ points and one pillar index per point are required.');
    [ids,~,group]=unique(double(pillarIds(:))); n=numel(ids);
    if nargin<3 || isempty(evaluate), evaluate=true(n,1); end
    assert(islogical(evaluate) && numel(evaluate)==n,'perception:InvalidPillarShape', ...
        'evaluate must contain one logical flag per occupied pillar.');
    names={'gapRatio','interquartileRatio','skewness','kurtosis','lowerHalfHeight','upperHalfHeight'};
    for name=names, shape.(name{1})=zeros(n,1); end
    shape.pillarIndices=ids;
    if isempty(points) || ~any(evaluate), return; end
    ordered=sortrows([group,double(points(:,3))],[1 2]);
    counts=accumarray(group,1); first=cumsum([1;counts(1:end-1)]);
    for j=find(evaluate(:)).'
        z=ordered(first(j):first(j)+counts(j)-1,2); span=z(end)-z(1);
        if span<=0, continue; end
        q=quantile(z,[.05 .25 .5 .75 .95]); centered=z-mean(z);
        sd=sqrt(mean(centered.^2));
        values=[max(diff(z))/span,(q(4)-q(2))/span, ...
            mean((centered/sd).^3),mean((centered/sd).^4),q(3)-q(1),q(5)-q(3)];
        for k=1:numel(names), shape.(names{k})(j)=values(k); end
    end
end
