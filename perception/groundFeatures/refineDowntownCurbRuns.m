function pointsByFrame = refineDowntownCurbRuns(pointsByFrame,path,cfg)
% Refine registered fine curb returns using straight stretches of the driven path.
% Vote by distinct longitudinal stations, so a dense patch or a stopped car
% cannot establish a road boundary merely by repeating the same observation.
% Short turns and stretches without a supported model retain single-scan labels.
    path=double(path);
    assert(size(path,2)==2 && size(path,1)==numel(pointsByFrame) && all(isfinite(path),'all'), ...
        'One finite planar mapping pose is required per acquisition.');
    counts=cellfun(@(p) size(p,1),pointsByFrame(:));
    if numel(counts)<2 || ~any(counts), return; end
    breaks=straightStretches(path,cfg);
    lengths=vecnorm(diff(path(breaks,:),1,1),2,2);
    if ~any(lengths>=cfg.minLength), return; end
    points=double(vertcat(pointsByFrame{:}));
    xy=points(:,1:2); frame=repelem((1:numel(counts)).',counts);
    travel=[0;cumsum(vecnorm(diff(path,1,1),2,2))];
    reach=vecnorm(xy-path(frame,:),2,2)+2*cfg.pathTolerance;
    best=inf(size(frame)); owner=zeros(size(frame));
    station=zeros(size(frame)); across=station;
    for j=find(lengths>0).'
        a=breaks(j); b=breaks(j+1); L=lengths(j);
        t=(path(b,:)-path(a,:))/L; delta=xy-path(a,:);
        s=delta*t.'; y=delta*[-t(2);t(1)];
        distance=y.^2+(s-min(max(s,0),L)).^2;
        % Limit assignment at revisits and crossings to reachable route pieces.
        reachable=travel(frame)>=travel(a)-reach & travel(frame)<=travel(b)+reach;
        closer=reachable & distance<best;
        owner(closer)=j; best(closer)=distance(closer);
        station(closer)=s(closer); across(closer)=y(closer);
    end
    keep=true(size(frame));
    for j=find(lengths>=cfg.minLength).'
        rows=find(owner==j);
        keep(rows)=onBoundaries(station(rows),across(rows),lengths(j),cfg);
    end
    parts=mat2cell(keep,counts,1);
    for k=1:numel(counts)
        pointsByFrame{k}=pointsByFrame{k}(parts{k},:);
    end
end

function breaks=straightStretches(path,cfg)
    breaks=[1,size(path,1)]; j=1;
    while j<numel(breaks)
        a=breaks(j); b=breaks(j+1); delta=path(a:b,:)-path(a,:);
        chord=delta(end,:); lengthMeters=norm(chord);
        t=chord/max(eps,lengthMeters);
        residual=delta-min(max(delta*t.',0),lengthMeters).*t;
        [error,index]=max(vecnorm(residual,2,2));
        if error>cfg.pathTolerance
            breaks=[breaks(1:j),a+index-1,breaks(j+1:end)]; %#ok<AGROW>
        else
            j=j+1;
        end
    end
    j=1;
    while j+2<=numel(breaks)
        v=diff(path(breaks(j:j+2),:),1,1);
        chord=sum(v,1); t=chord/max(eps,norm(chord));
        deviation=(path(breaks(j):breaks(j+2),:)-path(breaks(j),:))*[-t(2);t(1)];
        if dot(v(1,:),v(2,:))>=cosd(cfg.mergeAngleDeg)*prod(vecnorm(v,2,2)) && ...
                max(abs(deviation))<=2*cfg.pathTolerance
            breaks(j+1)=[];
        else
            j=j+1;
        end
    end
end

function keep=onBoundaries(s,y,L,cfg)
    keep=true(size(s));
    slopes=nan(1,2); runs=zeros(1,2); samples=cell(1,2);
    for k=1:2
        rows=s>0 & s<L & y*(2*k-3)>0;
        samples{k}=cfg.sampleSpacing*unique(round([s(rows)-L/2,y(rows)]/cfg.sampleSpacing),'rows');
        slopesToTry=tand(-cfg.angleLimitDeg:cfg.angleStepDeg:cfg.angleLimitDeg);
        [slopes(k),~,runs(k)]=boundaryFit(samples{k},slopesToTry,L,cfg,true);
    end
    if all(~isfinite(slopes)), return; end
    [~,anchor]=max(runs); commonSlope=slopes(anchor);
    keep=false(size(s)); resolved=false;
    for k=find(isfinite(slopes))
        slope=slopes(k);
        % Align nearly parallel sides to the better measured direction.
        if abs(atand(slope)-atand(commonSlope))<=cfg.parallelAngleDeg, slope=commonSlope; end
        [~,offset]=boundaryFit(samples{k},slope,L,cfg,false);
        if ~isfinite(offset) && slope~=slopes(k)
            slope=slopes(k); [~,offset]=boundaryFit(samples{k},slope,L,cfg,false);
        end
        if isfinite(offset)
            resolved=true;
            % Use the wider band between observed anchor stations. Extrapolated
            % edges must meet the original fit tolerance.
            sample=samples{k}; scale=hypot(1,slope);
            tight=abs(sample(:,2)-slope*sample(:,1)-offset)<=cfg.fitBand*scale;
            bins=unique(floor((sample(tight,1)+L/2)/cfg.stationLength));
            observed=false(size(s));
            if ~isempty(bins)
                at=floor(s/cfg.stationLength);
                observed=at>=min(bins)-cfg.maxStationGap & at<=max(bins)+cfg.maxStationGap;
            end
            residual=abs(y-slope*(s-L/2)-offset)/scale;
            keep=keep | residual<=cfg.fitBand | (observed & residual<=cfg.acceptBand);
        end
    end
    if ~resolved, keep(:)=true; end
end

function [slope,offset,runLength]=boundaryFit(samples,slopes,L,cfg,refit)
    slope=nan; offset=nan; runLength=0;
    if isempty(samples), return; end
    s=samples(:,1); y=samples(:,2); station=floor((s+L/2)/cfg.stationLength);
    best=-inf;
    for m=slopes
        h=y-m*s;
        centers=(unique(floor(h/cfg.sampleSpacing))+.5)*cfg.sampleSpacing;
        for c=centers.'
            selected=abs(h-c)<cfg.fitBand;
            bins=unique(station(selected));
            if numel(bins)<max(cfg.minStations,ceil(cfg.minCoverage*L/cfg.stationLength)), continue; end
            boundaries=[1;find(diff(bins)>cfg.maxStationGap)+1;numel(bins)+1];
            [connected,k]=max(diff(boundaries));
            if connected<cfg.minConnectedStations, continue; end
            score=numel(bins)+2*connected;
            if score<=best, continue; end
            best=score; slope=m; runLength=connected; inliers=selected;
            longest=bins(boundaries(k):boundaries(k+1)-1);
        end
    end
    if ~isfinite(slope), return; end
    if refit, inliers=inliers & ismember(station,longest); end
    [~,~,group]=unique(station(inliers));
    x=accumarray(group,s(inliers),[],@median);
    z=accumarray(group,y(inliers),[],@median);
    if refit
        fit=polyfit(x,z,1); slope=fit(1); offset=fit(2);
    else
        offset=median(z-slope*x);
    end
end
