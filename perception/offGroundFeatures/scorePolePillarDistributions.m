function evidence=scorePolePillarDistributions(points,pillarIds,geometry,peaks,evaluate,settings)
% scorePolePillarDistributions: Continuous shaft evidence for whole pillars.
% A compact pole concentrates mass near its density peak and has no dominant
% gap in the ordered heights of that mass. The score is geometric evidence,
% not a calibrated semantic class probability. Original point ownership and
% all-point Gaussian moments are unchanged; no finer semantic grid is built.
% Neighboring whole pillars supply evidence when the shaft crosses a boundary.
    if nargin<6 || isempty(settings)
        cfg=structuralPillarConfig(.6); settings=cfg.pole.distribution;
    end
    assert(settings.coreRadius>0 && settings.contextRadius>=settings.coreRadius && ...
        settings.minimumRobustHeight>0 && settings.minimumCorePoints>0, ...
        'perception:InvalidPoleDistribution','Distribution radii and support scales must be positive.');
    [ids,~,group]=unique(double(pillarIds(:))); n=numel(ids);
    if nargin<5 || isempty(evaluate), evaluate=true(n,1); end
    assert(isequal(size(peaks),[n 2]) && numel(evaluate)==n, ...
        'perception:InvalidPoleDistribution','One peak and evaluation flag per occupied pillar are required.');
    evidence=struct('pillarIndices',ids,'score',zeros(n,1),'concentration',zeros(n,1), ...
        'continuity',zeros(n,1),'robustHeight',zeros(n,1),'coreCount',zeros(n,1));
    selected=find(evaluate(:) & all(isfinite(peaks),2));
    if isempty(points) || isempty(selected), return; end
    p=double(points); counts=accumarray(group,1); [~,order]=sort(group);
    first=cumsum([1;counts(1:end-1)]); [row,col]=ind2sub(geometry.mapSize,ids);
    lookup=zeros(geometry.mapSize); lookup(ids)=1:n;
    reach=ceil(settings.contextRadius./geometry.cellSize);
    for j=selected.'
        rr=max(1,row(j)-reach(2)):min(geometry.mapSize(1),row(j)+reach(2));
        cc=max(1,col(j)-reach(1)):min(geometry.mapSize(2),col(j)+reach(1));
        neighbors=lookup(rr,cc); neighbors=neighbors(neighbors>0);
        pieces=arrayfun(@(b) order(first(b):first(b)+counts(b)-1),neighbors(:),'UniformOutput',false);
        q=p(vertcat(pieces{:}),:); distance=vecnorm(q(:,1:2)-peaks(j,:),2,2);
        z=sort(q(distance<=settings.coreRadius,3)); evidence.coreCount(j)=numel(z);
        if numel(z)<3 || z(end)<=z(1), continue; end
        concentration=numel(z)/max(1,nnz(distance<=settings.contextRadius));
        continuity=max(0,1-max(diff(z))/(z(end)-z(1)));
        heights=quantile(z,[.05 .95]); robustHeight=diff(heights);
        evidence.concentration(j)=concentration; evidence.continuity(j)=continuity;
        evidence.robustHeight(j)=robustHeight;
        evidence.score(j)=concentration*continuity*min(1,robustHeight/settings.minimumRobustHeight)*min(1,numel(z)/settings.minimumCorePoints);
    end
end
