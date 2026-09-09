function stats=aggregatePillarStatistics(points, pillarIds, attributes, useNative)
% aggregatePillarStatistics: Sufficient statistics of every whole pillar.
% Each row addresses one occupied pillar. covarianceXYZ is the population
% covariance packed as [xx xy yy xz yz zz], retaining all XYZ correlations.
% Bounds and radiometric summaries include every member; no subcells exist.
    if nargin<3, attributes=struct(); end
    if nargin<4, useNative=false; end
    [ids,~,group]=unique(pillarIds(:));
    n=numel(ids);
    moments=aggregatePlanarCellMoments(points,group,n,eye(3),[0 0 0],useNative);
    lower=zeros(n,3); upper=lower;
    if ~isempty(points)
        for axisIndex=1:3
            lower(:,axisIndex)=accumarray(group,points(:,axisIndex),[n 1],@min);
            upper(:,axisIndex)=accumarray(group,points(:,axisIndex),[n 1],@max);
        end
    end
    stats=struct('pillarIndices',int32(ids),'count',moments.count, ...
        'meanXYZ',[moments.mean moments.meanZ], ...
        'covarianceXYZ',[moments.covariance moments.heightCovariance], ...
        'covarianceOrder',"xx xy yy xz yz zz",'minimumXYZ',lower,'maximumXYZ',upper);
    names=intersect(fieldnames(attributes),{'intensity','reflectivity'});
    for k=1:numel(names)
        values=double(attributes.(names{k})(:));
        finite=isfinite(values);
        counts=accumarray(group(finite),1,[n 1],@sum,0);
        maxima=accumarray(group(finite),values(finite),[n 1],@max,NaN);
        stats.(names{k})=struct('count',counts,'maximum',maxima);
    end
end
