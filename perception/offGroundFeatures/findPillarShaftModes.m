function evidence=findPillarShaftModes(points,pillarIds,geometry,cfg,evaluate)
% findPillarShaftModes: Bounded, compact, continuous modes within mixed pillars.
% Multiple fit scales seed height-balanced axes. Each axis/radius searches
% bounded intervals of gap-connected returns, retaining short valid segments
% even when larger segments contain clutter. Finite endpoints limit runtime.
% Evidence is owned by a pillar only if that pillar supplies shaft returns.
    [ids,~,group]=unique(double(pillarIds(:)));n=numel(ids);
    if nargin<5 || isempty(evaluate),evaluate=true(n,1);end
    assert(numel(evaluate)==n,'perception:InvalidShaftModes','One evaluation flag per occupied pillar is required.');
    thresholds=repmat(cfg.minimumScore,n,1);
    if isfield(cfg,'columnMinimumScores'),thresholds=max(thresholds,double(cfg.columnMinimumScores(:)));end
    assert(numel(thresholds)==n && all(isfinite(thresholds)),'perception:InvalidShaftModes','Column score thresholds must be finite and match occupied pillars.');
    names={'score','supportCount','ownCount','height','robustHeight','radialRms', ...
        'maximumGap','contrast','radius','minimumZ','maximumZ','axisZ','ownHeight','peakContrast', ...
        'excessCount','peakSignificance','heightCoverage','fitScaleCount','radialSignificance'};
    evidence=struct('pillarIndices',ids,'found',false(n,1),'axisXY',nan(n,2),'slopeXY',zeros(n,2));
    for name=names,evidence.(name{1})=zeros(n,1);end
    if isempty(points) || ~any(evaluate),return;end
    if isfield(cfg,'useNativeKernels') && cfg.useNativeKernels
        fields={'maximumSeeds','seedSeparation','fitHeight','heightHypotheses','neighborhoodRadius', ...
            'minimumPoints','minimumOwnPoints','minimumOwnHeight','minimumHeight','minimumRobustHeight', ...
            'maximumTiltDegrees','maximumGap','rangeGapScale','maximumRadialRms', ...
            'minimumPeakContrast','minimumExcessPoints','maximumIntervalEndpoints','minimumScore', ...
            'saturatedHeight','saturatedPoints','axisMergeDistance','axisMergeSlope','minimumFitScales','minimumSingleScaleScore','verticalHypotheses', ...
            'minimumRadialSignificance','maximumRelativeRadialRms','minimumHalfRadialSignificance','minimumHalfPeakContrast'};
        parameters=cellfun(@(name)double(cfg.(name)),fields);
        nativeThreads=1;
        if isfield(cfg,'nativeThreads'),nativeThreads=double(cfg.nativeThreads);end
        measured=perceptionKernelsMex('shaftModes',double(points),double(group),ids, ...
            double(geometry.mapSize),double(geometry.cellSize),parameters,cfg.radii,cfg.fitRadii,logical(evaluate(:)),thresholds,nativeThreads);
        columns=[1:14 19:23];
        for k=1:numel(names),evidence.(names{k})=measured(:,columns(k));end
        evidence.found=evidence.supportCount>0;evidence.axisXY=measured(:,15:16);evidence.slopeXY=measured(:,17:18);
        return;
    end
    p=double(points);counts=accumarray(group,1);[~,order]=sort(group);first=cumsum([1;counts(1:end-1)]);
    [row,col]=ind2sub(geometry.mapSize,ids);lookup=zeros(geometry.mapSize);lookup(ids)=1:n;
    reach=ceil(cfg.neighborhoodRadius./geometry.cellSize);
    for j=find(evaluate(:) & counts>=cfg.minimumOwnPoints).'
        own=sortrows(p(order(first(j):first(j)+counts(j)-1),:),[1 2 3]);
        if max(own(:,3))-min(own(:,3))<cfg.minimumOwnHeight,continue;end
        rr=max(1,row(j)-reach(2)):min(geometry.mapSize(1),row(j)+reach(2));
        cc=max(1,col(j)-reach(1)):min(geometry.mapSize(2),col(j)+reach(1));
        neighbors=lookup(rr,cc);neighbors=neighbors(neighbors>0);
        pieces=arrayfun(@(b)order(first(b):first(b)+counts(b)-1),neighbors(:),'UniformOutput',false);
        index=vertcat(pieces{:});q=p(index,:);owned=group(index)==j;
        [~,zOrder]=sort(q(:,3));q=q(zOrder,:);owned=owned(zOrder);
        seeds=coveringSeeds(own(:,1:2),cfg.maximumSeeds,cfg.seedSeparation);axes=zeros(0,5);scaleMasks=zeros(0,1,'uint32');
        for seed=1:size(seeds,1)
            for fitIndex=1:numel(cfg.fitRadii)
                fitRadius=cfg.fitRadii(fitIndex);
                initial=own(vecnorm(own(:,1:2)-seeds(seed,:),2,2)<=fitRadius,:);
                if size(initial,1)<cfg.minimumOwnPoints,continue;end
                lo=min(initial(:,3));hi=max(initial(:,3));if hi-lo<cfg.minimumOwnHeight,continue;end
                starts=linspace(lo,max(lo,hi-cfg.fitHeight),cfg.heightHypotheses);
                windows=unique([lo hi;starts(:),min(starts(:)+cfg.fitHeight,hi)],'rows','stable');
                for w=1:size(windows,1)
                    fitting=initial(initial(:,3)>=windows(w,1) & initial(:,3)<=windows(w,2),:);
                    if size(fitting,1)<cfg.minimumOwnPoints || max(fitting(:,3))-min(fitting(:,3))<cfg.minimumOwnHeight,continue;end
                    for vertical=0:double(cfg.verticalHypotheses)
                        axis=fitAxis(fitting,vertical);
                        if norm(axis(4:5))>tand(cfg.maximumTiltDegrees),continue;end
                        for iteration=1:2
                            distance=axisDistance(q,axis);
                            inside=distance<=fitRadius & q(:,3)>=windows(w,1) & q(:,3)<=windows(w,2);
                            if nnz(inside)<cfg.minimumPoints,break;end
                            candidate=fitAxis(q(inside,:),vertical);
                            if norm(candidate(4:5))>tand(cfg.maximumTiltDegrees),break;end
                            axis=candidate;
                        end
                        shifted=axes(:,1:2)+(axis(3)-axes(:,3)).*axes(:,4:5);
                        duplicate=vecnorm(shifted-axis(1:2),2,2)<=cfg.axisMergeDistance & ...
                            vecnorm(axes(:,4:5)-axis(4:5),2,2)<=cfg.axisMergeSlope;
                        if any(duplicate)
                            a=find(duplicate,1);scaleMasks(a)=bitset(scaleMasks(a),fitIndex);continue;
                        end
                        axes(end+1,:)=axis; %#ok<AGROW>
                        scaleMasks(end+1,1)=bitset(uint32(0),fitIndex); %#ok<AGROW>
                    end
                end
            end
        end
        for a=1:size(axes,1)
            scaleCount=sum(bitget(scaleMasks(a),1:numel(cfg.fitRadii)));
            intervalCfg=cfg;intervalCfg.minimumScore=thresholds(j);
            if scaleCount<cfg.minimumFitScales,intervalCfg.minimumScore=max(intervalCfg.minimumScore,cfg.minimumSingleScaleScore);end
            axis=axes(a,:);distance=axisDistance(q,axis);
            for radius=cfg.radii
                values=bestInterval(q,owned,distance,axis,radius,intervalCfg,evidence.score(j));
                if isempty(values),continue;end
                values=[values(1:17),scaleCount,values(18)];
                evidence.found(j)=true;evidence.axisXY(j,:)=axis(1:2);evidence.slopeXY(j,:)=axis(4:5);
                for c=1:numel(names),evidence.(names{c})(j)=values(c);end
            end
        end
    end
end

function axis=fitAxis(p,vertical)
    if vertical,axis=[median(p,1),0,0];else,axis=balancedAxis(p);end
end

function best=bestInterval(q,owned,distance,axis,radius,cfg,bestScore)
    nearby=distance<=2*radius*(1+1e-14);q=q(nearby,:);owned=owned(nearby);distance=distance(nearby);
    core=find(distance<=radius);z=q(core,3);best=[];
    if numel(core)<cfg.minimumPoints,return;end
    directions=[1 0;0 1;1 1;1 -1;-1 0;0 -1;-1 -1;-1 1];directions=directions./vecnorm(directions,2,2);
    residual=q(:,1:2)-axis(1:2)-(q(:,3)-axis(3)).*axis(4:5);
    probes=zeros(size(q,1),10);probes(:,1)=distance<=radius;probes(:,10)=distance<=2*radius;
    for k=1:8,probes(:,k+1)=sum((residual-radius*directions(k,:)).^2,2)<=radius^2;end
    prefix=[zeros(1,10);cumsum(probes,1)];
    ownPrefix=[0;cumsum(owned(core))];squarePrefix=[0;cumsum(distance(core).^2)];
    coveragePrefix=[0;cumsum(min(diff(z),.20))];
    gapLimit=cfg.maximumGap+cfg.rangeGapScale*norm(axis(1:2));
    cuts=[0;find(diff(z)>gapLimit);numel(z)];
    for run=1:numel(cuts)-1
        first=cuts(run)+1;last=cuts(run+1);if last-first+1<cfg.minimumPoints,continue;end
        % Sampling observed ranks creates no height lattice. Include endpoints.
        starts=unique(round(linspace(first,last,min(last-first+1,cfg.maximumIntervalEndpoints))));
        ends=starts;
        for a=starts
            for b=ends(ends>=a+cfg.minimumPoints-1)
                height=z(b)-z(a);if height<cfg.minimumHeight,continue;end
                ownerCount=ownPrefix(b+1)-ownPrefix(a);if ownerCount<cfg.minimumOwnPoints,continue;end
                ownedPart=core(a:b);ownedPart=ownedPart(owned(ownedPart));ownHeight=q(ownedPart(end),3)-q(ownedPart(1),3);
                if ownHeight<cfg.minimumOwnHeight,continue;end
                zz=z(a:b);robust=diff(quantile(zz,[.05 .95]));if robust<cfg.minimumRobustHeight,continue;end
                radial=sqrt(max(0,(squarePrefix(b+1)-squarePrefix(a))/(b-a+1)));
                if radial>min(cfg.maximumRadialRms,cfg.maximumRelativeRadialRms*radius),continue;end
                low=find(q(:,3)>=zz(1),1);high=find(q(:,3)<=zz(end),1,'last');
                mass=prefix(high+1,:)-prefix(low,:);side=max(mass(2:9));
                peak=mass(1)/max(side,1);excess=mass(1)-side;
                if peak<cfg.minimumPeakContrast || excess<cfg.minimumExcessPoints,continue;end
                radialSignificance=(mass(1)-.25*mass(10))/sqrt(max(.1875*mass(10),1));
                if radialSignificance<cfg.minimumRadialSignificance,continue;end
                middle=find(q(:,3)<=.5*(zz(1)+zz(end)),1,'last');
                halves=[prefix(middle+1,:)-prefix(low,:);prefix(high+1,:)-prefix(middle+1,:)];
                halfRadial=(halves(:,1)-.25*halves(:,10))./sqrt(max(.1875*halves(:,10),1));
                halfPeak=halves(:,1)./max(max(halves(:,2:9),[],2),1);
                if any(halfRadial<cfg.minimumHalfRadialSignificance | halfPeak<cfg.minimumHalfPeakContrast),continue;end
                coverage=(coveragePrefix(b)-coveragePrefix(a))/height;
                significance=excess/sqrt(max(mass(1)+side,1));
                score=min(1,robust/cfg.saturatedHeight)*sqrt(min(1,numel(zz)/cfg.saturatedPoints))* ...
                    exp(-(radial/cfg.maximumRadialRms)^2)*sqrt(coverage)*min(1,(peak-1)/.6);
                if score<cfg.minimumScore || score<=bestScore+1e-12,continue;end
                gap=max(diff(zz));
                bestScore=score;
                best=[score,numel(zz),ownerCount,height,robust,radial,gap,peak,radius,zz(1),zz(end),axis(3),ownHeight,peak,excess,significance,coverage,radialSignificance];
            end
        end
    end
end

function seeds=coveringSeeds(xy,maximum,separation)
    seeds=zeros(maximum,2);seeds(1,:)=xy(1,:);distance=inf(size(xy,1),1);count=1;
    for k=2:maximum
        distance=min(distance,sum((xy-seeds(k-1,:)).^2,2));[far,index]=max(distance);
        if far<separation^2,break;end
        seeds(k,:)=xy(index,:);count=k;
    end
    seeds=seeds(1:count,:);
end

function axis=balancedAxis(p)
    [z,~,group]=unique(p(:,3));counts=accumarray(group,1);
    if isscalar(z),axis=[mean(p(:,1:2),1),z,0,0];return;end
    gaps=diff(z);weights=([gaps;0]+[0;gaps])/2;weights=weights(group)./counts(group);weights=weights/sum(weights);
    center=sum(weights.*p,1);dz=p(:,3)-center(3);
    slope=sum(weights.*dz.*(p(:,1:2)-center(1:2)),1)/max(sum(weights.*dz.^2),eps);
    axis=[center,slope];
end

function distance=axisDistance(p,axis)
    distance=vecnorm(p(:,1:2)-axis(1:2)-(p(:,3)-axis(3)).*axis(4:5),2,2);
end
