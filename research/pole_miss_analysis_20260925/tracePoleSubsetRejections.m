function [evidence,trace]=tracePoleSubsetRejections(points,pillarIds,geometry,cfg,evaluate)
% tracePoleSubsetRejections: Instrument the c722914 subset detector without changing gates.
% Research-only copy; detection output is checked against the native implementation.
% Stage records the furthest successful gate among all hypotheses, not a
% unique physical cause. Independent maxima need not belong to one hypothesis.
% Each pillar is accepted when at least one independently supported point
% bundle qualifies. Neither whole-pillar scatter nor a fraction of all pillar
% or neighborhood returns is a veto. Local radial density contrast excludes
% arbitrary strips cut from a broad surface; only the shaft's height interval
% and immediate transverse neighborhood participate in that test.
% All input points retain their original pillar. No finer XY/Z lattice or
% semantic point mask is created. Output Gaussian moments remain all-point.
    [ids,~,group]=unique(double(pillarIds(:))); n=numel(ids);
    if nargin<5 || isempty(evaluate), evaluate=true(n,1); end
    assert(numel(evaluate)==n,'perception:InvalidPoleSubset','One evaluation flag per occupied pillar is required.');
    names={'score','supportCount','ownCount','height','robustHeight','radialRms', ...
        'maximumGap','contrast','radius','minimumZ','maximumZ','axisZ','ownHeight','peakContrast'};
    evidence=struct('pillarIndices',ids,'found',false(n,1),'axisXY',nan(n,2),'slopeXY',zeros(n,2));
    for name=names, evidence.(name{1})=zeros(n,1); end
    cfg.useNativeKernels=false;
    trace=struct('pillarIndices',ids,'stage',zeros(n,1));
    traceNames={'ownPoints','ownSpan','maximumCylinderPoints','maximumQualifiedPoints', ...
        'maximumRunPoints','maximumRunOwnPoints','longestRunHeight','longestRunRobustHeight', ...
        'longestRunOwnHeight','longestRunPoints','longestRunOwnPoints','longestRunRadialRms','maximumPeakContrast'};
    for name=traceNames, trace.(name{1})=zeros(n,1); end
    trace.minimumSeedTiltDegrees=inf(n,1);trace.minimumRadialRms=inf(n,1);
    if isempty(points) || ~any(evaluate), return; end
    p=double(points); counts=accumarray(group,1); [~,order]=sort(group);
    first=cumsum([1;counts(1:end-1)]); [row,col]=ind2sub(geometry.mapSize,ids);
    lookup=zeros(geometry.mapSize); lookup(ids)=1:n;
    minimum=accumarray(group,p(:,3),[],@min); maximum=accumarray(group,p(:,3),[],@max);
    trace.ownPoints=counts;trace.ownSpan=maximum-minimum;
    selected=find(evaluate(:) & counts>=cfg.minimumOwnPoints & maximum-minimum>=cfg.minimumOwnHeight);
    reach=ceil(cfg.neighborhoodRadius./geometry.cellSize);
    for j=selected.'
        trace.stage(j)=1;
        own=p(order(first(j):first(j)+counts(j)-1),:); own=sortrows(own,[1 2 3]);
        rr=max(1,row(j)-reach(2)):min(geometry.mapSize(1),row(j)+reach(2));
        cc=max(1,col(j)-reach(1)):min(geometry.mapSize(2),col(j)+reach(1));
        neighbors=lookup(rr,cc); neighbors=neighbors(neighbors>0);
        pieces=arrayfun(@(b) order(first(b):first(b)+counts(b)-1),neighbors(:),'UniformOutput',false);
        index=vertcat(pieces{:}); q=p(index,:); owned=group(index)==j;
        seeds=coveringSeeds(own(:,1:2),cfg.maximumSeeds,cfg.seedSeparation);
        for s=1:size(seeds,1)
            near=vecnorm(own(:,1:2)-seeds(s,:),2,2)<=cfg.fitRadius;
            initial=own(near,:);
            if size(initial,1)<cfg.minimumOwnPoints, continue; end
            lo=min(initial(:,3)); hi=max(initial(:,3));
            if hi-lo<cfg.minimumOwnHeight, continue; end
            % Whole support and overlapping physical windows seed fits. They
            % are regression hypotheses, not vertical voxel occupancies.
            starts=linspace(lo,max(lo,hi-cfg.fitHeight),cfg.heightHypotheses);
            intervals=unique([lo hi;starts(:) min(starts(:)+cfg.fitHeight,hi)],'rows','stable');
            for w=1:size(intervals,1)
                fitPoints=initial(initial(:,3)>=intervals(w,1) & initial(:,3)<=intervals(w,2),:);
                if size(fitPoints,1)<cfg.minimumOwnPoints || max(fitPoints(:,3))-min(fitPoints(:,3))<cfg.minimumOwnHeight, continue; end
                trace.stage(j)=max(trace.stage(j),2);
                [center,slope,z0]=balancedAxis(fitPoints);
                trace.minimumSeedTiltDegrees(j)=min(trace.minimumSeedTiltDegrees(j),atand(norm(slope)));
                if norm(slope)>tand(cfg.maximumTiltDegrees), continue; end
                trace.stage(j)=max(trace.stage(j),3);
                % Refine within this hypothesis's height interval; a dense
                % crown elsewhere must not pull the shaft axis toward it.
                for iteration=1:2
                    distance=vecnorm(q(:,1:2)-center-(q(:,3)-z0).*slope,2,2);
                    inlier=distance<=cfg.fitRadius & q(:,3)>=intervals(w,1) & q(:,3)<=intervals(w,2);
                    if nnz(inlier)<cfg.minimumPoints, break; end
                    [newCenter,newSlope,newZ]=balancedAxis(q(inlier,:));
                    if norm(newSlope)>tand(cfg.maximumTiltDegrees), break; end
                    center=newCenter; slope=newSlope; z0=newZ;
                end
                distance=vecnorm(q(:,1:2)-center-(q(:,3)-z0).*slope,2,2);
                for radius=cfg.radii
                    core=find(distance<=radius); [z,orderZ]=sort(q(core,3)); core=core(orderZ);
                    trace.maximumCylinderPoints(j)=max(trace.maximumCylinderPoints(j),numel(core));
                    if numel(core)<cfg.minimumPoints, continue; end
                    trace.stage(j)=max(trace.stage(j),4);
                    inner=heightWindowCount(z,z,cfg.heightHalfWindow);
                    outer=heightWindowCount(q(distance<=2*radius,3),z,cfg.heightHalfWindow);
                    contrast=3*inner./max(outer-inner,1);
                    qualified=inner>=cfg.minimumWindowPoints & contrast>=cfg.minimumContrast;
                    core=core(qualified); z=z(qualified); contrast=contrast(qualified);
                    trace.maximumQualifiedPoints(j)=max(trace.maximumQualifiedPoints(j),numel(core));
                    if numel(core)<cfg.minimumPoints, continue; end
                    trace.stage(j)=max(trace.stage(j),5);
                    gapLimit=cfg.maximumGap+cfg.rangeGapScale*norm(center);
                    cuts=[0;find(diff(z)>gapLimit);numel(z)];
                    for run=1:numel(cuts)-1
                        range=(cuts(run)+1):cuts(run+1); part=core(range); zz=z(range);
                        trace.maximumRunPoints(j)=max(trace.maximumRunPoints(j),numel(part));
                        if numel(part)<cfg.minimumPoints, continue; end
                        trace.stage(j)=max(trace.stage(j),6);
                        ownPart=part(owned(part));
                        trace.maximumRunOwnPoints(j)=max(trace.maximumRunOwnPoints(j),numel(ownPart));
                        if numel(ownPart)<cfg.minimumOwnPoints, continue; end
                        trace.stage(j)=max(trace.stage(j),7);
                        height=zz(end)-zz(1); ownHeight=max(q(ownPart,3))-min(q(ownPart,3));
                        robust=diff(quantile(zz,[.05 .95]));
                        if height>trace.longestRunHeight(j)
                            trace.longestRunHeight(j)=height;trace.longestRunRobustHeight(j)=robust;
                            trace.longestRunOwnHeight(j)=ownHeight;trace.longestRunPoints(j)=numel(part);
                            trace.longestRunOwnPoints(j)=numel(ownPart);trace.longestRunRadialRms(j)=sqrt(mean(distance(part).^2));
                        end
                        if height<cfg.minimumHeight || robust<cfg.minimumRobustHeight || ownHeight<cfg.minimumOwnHeight, continue; end
                        trace.stage(j)=max(trace.stage(j),8);
                        radial=sqrt(mean(distance(part).^2));
                        trace.minimumRadialRms(j)=min(trace.minimumRadialRms(j),radial);
                        if radial>cfg.maximumRadialRms, continue; end
                        trace.stage(j)=max(trace.stage(j),9);
                        gap=max(diff(zz)); localContrast=median(contrast(range));
                        inHeight=q(:,3)>=zz(1) & q(:,3)<=zz(end);
                        transverse=q(inHeight,1:2)-center-(q(inHeight,3)-z0).*slope;
                        peakContrast=densityPeakContrast(transverse,radius);
                        trace.maximumPeakContrast(j)=max(trace.maximumPeakContrast(j),peakContrast);
                        if peakContrast<cfg.minimumPeakContrast, continue; end
                        trace.stage(j)=10;
                        score=min(1,robust/cfg.saturatedHeight)*min(1,numel(part)/cfg.saturatedPoints)* ...
                            (1-gap/height)*exp(-(radial/cfg.maximumRadialRms)^2)*min(1,localContrast/cfg.saturatedContrast)*min(1,peakContrast/2);
                        if score<=evidence.score(j)+1e-12, continue; end
                        evidence.found(j)=true; evidence.axisXY(j,:)=center; evidence.slopeXY(j,:)=slope;
                        values=[score,numel(part),numel(ownPart),height,robust,radial,gap,localContrast,radius,zz(1),zz(end),z0,ownHeight,peakContrast];
                        for c=1:numel(names), evidence.(names{c})(j)=values(c); end
                    end
                end
            end
        end
    end
end

function seeds=coveringSeeds(xy,maximum,separation)
% Cover all spatial modes, including ones with fewer returns than the clutter.
    seeds=zeros(maximum,2); seeds(1,:)=xy(1,:); distance=inf(size(xy,1),1); count=1;
    for k=2:maximum
        distance=min(distance,sum((xy-seeds(k-1,:)).^2,2)); [far,index]=max(distance);
        if far<separation^2, break; end
        seeds(k,:)=xy(index,:); count=k;
    end
    seeds=seeds(1:count,:);
end

function [center,slope,z0]=balancedAxis(p)
% Equal height measure, rather than return count, prevents a dense end blob
% from dominating a shaft that is sparsely sampled over a longer interval.
    [z,~,group]=unique(p(:,3)); count=accumarray(group,1);
    if isscalar(z), center=mean(p(:,1:2),1); slope=[0 0]; z0=z; return; end
    gaps=diff(z); weight=([gaps;0]+[0;gaps])/2;
    weight=weight(group)./count(group); weight=weight/sum(weight);
    z0=sum(weight.*p(:,3)); center=sum(weight.*p(:,1:2),1);
    dz=p(:,3)-z0; slope=sum(weight.*dz.*(p(:,1:2)-center),1)/max(sum(weight.*dz.^2),eps);
end

function count=heightWindowCount(z,queries,halfHeight)
% Continuous empirical-CDF counts, evaluated at actual support heights.
    if isempty(z), count=zeros(size(queries)); return; end
    [z,~,group]=unique(z); cumulative=cumsum(accumarray(group,1));
    if isscalar(z), count=double(abs(queries-z)<=halfHeight)*cumulative(end); return; end
    lo=queries-halfHeight; hi=queries+halfHeight;
    upper=interp1(z,cumulative,hi,'previous','extrap'); upper(hi<z(1))=0;
    lo=lo-4*eps(max(1,abs(lo))); lower=interp1(z,cumulative,lo,'previous','extrap'); lower(lo<z(1))=0;
    count=upper-lower;
end

function contrast=densityPeakContrast(residual,radius)
% Require a transverse density maximum, not an arbitrary strip of a sheet.
% Probes are only one shaft radius from its axis in the same height interval.
    directions=[1 0;0 1;1 1;1 -1;-1 0;0 -1;-1 -1;-1 1];
    directions=directions./vecnorm(directions,2,2);
    center=nnz(sum(residual.^2,2)<=radius^2); side=0;
    for k=1:size(directions,1)
        side=max(side,nnz(sum((residual-radius*directions(k,:)).^2,2)<=radius^2));
    end
    contrast=center/max(side,1);
end
