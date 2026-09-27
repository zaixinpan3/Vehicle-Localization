function [rows,ownerIds,hypothesisIds]=measurePrecisionFeatures(points,pillarIds,geometry,hypotheses,structural,pointScore,lineScore,cfg)
% measurePrecisionFeatures: Label-free distributions on existing coarse pillars.
% Research feature extractor; no reference labels, frame ID or XY fine grid.
    p=double(points);pillarIds=double(pillarIds);rows={};ownerIds=[];hypothesisIds=[];
    [occupied,~,group]=unique(pillarIds);n=accumarray(group,1);[~,order]=sort(group);offset=cumsum([1;n(1:end-1)]);
    lookup=zeros(geometry.mapSize);lookup(occupied)=1:numel(occupied);
    for a=1:numel(hypotheses)
        h=hypotheses(a);if ~h.geometryAccepted,continue;end
        keep=h.ownerCount>=cfg.minimumOwnerPoints & h.ownerHeight>=cfg.minimumOwnerHeight;
        if ~any(keep),continue;end
        lo=floor((h.axisXY-2-geometry.origin)./geometry.cellSize)+1;
        hi=floor((h.axisXY+2-geometry.origin)./geometry.cellSize)+1;
        rr=max(1,lo(2)):min(geometry.mapSize(1),hi(2));cc=max(1,lo(1)):min(geometry.mapSize(2),hi(1));
        cells=lookup(rr,cc);cells=cells(cells>0);
        pieces=arrayfun(@(j)order(offset(j):offset(j)+n(j)-1),cells(:),'UniformOutput',false);index=vertcat(pieces{:});
        q=p(index,:);owners=pillarIds(index);mask=structural(index);
        f=measurePoleContext(q,mask,h);
        residual=q(:,1:2)-h.axisXY-(q(:,3)-h.axisZ).*h.slopeXY;
        distance=vecnorm(residual,2,2);supported=q(:,3)>=h.minimumZ & q(:,3)<=h.maximumZ;
        core=supported & distance<=.25;xy=residual(core,:);
        f.axisRange=norm(h.axisXY);f.structuralFraction=nnz(core & mask)/max(nnz(core),1);
        [f.circleRadius,f.circleResidual,f.circleCenterOffset,f.circleRelativeResidual]=circleShape(xy);
        f.rangeNormalizedSupport=h.acceptedCount*max(f.axisRange,1)^2/max(h.supportHeight,eps);
        if nnz(core)>=3
            [vectors,values]=eig(cov(xy));[~,largest]=max(diag(values));view=h.axisXY/max(f.axisRange,eps);
            f.transverseViewAlignment=abs(view*vectors(:,largest));
        else
            f.transverseViewAlignment=0;
        end
        for radius=[.30 .40 .60 .75]
            tag=sprintf('%02d',round(100*radius));use=supported & distance<=radius;
            e=sort(max(eig(cov(residual(use,:))),0));
            f.(['std' tag])=sqrt(e(2));f.(['aspect' tag])=sqrt(e(2)/max(e(1),eps));
            f.(['rms' tag])=sqrt(mean(distance(use).^2));f.(['count' tag])=nnz(use);
            f.(['shift' tag])=norm(mean(residual(use,:),1));
        end
        for b=find(keep).'
            owner=h.ownerIds(b);r=f;r.ownerCount=h.ownerCount(b);r.ownerHeight=h.ownerHeight(b);
            r.ownerFraction=r.ownerCount/max(h.acceptedCount,1);
            r.pointScore=double(pointScore(owner));r.lineScore=double(lineScore(owner));
            own=owners==owner;r.ownerTotalCount=nnz(own);r.ownerTotalHeight=max(q(own,3))-min(q(own,3));
            r.ownerMedianRadius=median(distance(own));r.ownerCoreFraction=nnz(own & core)/max(nnz(own),1);
            r.ownerTightFraction=nnz(own & supported & distance<=.10)/max(nnz(own & supported),1);
            r.ownerSupportFraction=r.ownerCount/max(nnz(own),1);
            r.ownerRangeNormalizedSupport=r.ownerCount*max(f.axisRange,1)^2/max(r.ownerHeight,eps);
            [y,x]=ind2sub(geometry.mapSize,owner);lower=geometry.origin+([x y]-1).*geometry.cellSize;
            at=h.axisXY+([h.minimumZ;h.maximumZ]-h.axisZ).*h.slopeXY;
            r.axisOwnerDistance=min(vecnorm(max(max(lower-at,at-lower-geometry.cellSize),0),2,2));
            rows{end+1,1}=r;ownerIds(end+1,1)=owner;hypothesisIds(end+1,1)=a; %#ok<AGROW>
        end
    end
end
function [radius,residual,offset,relative]=circleShape(xy)
    radius=0;residual=0;offset=0;relative=0;if size(xy,1)<4,return;end
    design=[2*xy,ones(size(xy,1),1)];b=pinv(design)*sum(xy.^2,2);
    radius=sqrt(max(0,b(3)+sum(b(1:2).^2)));offset=norm(b(1:2));
    residual=sqrt(mean((vecnorm(xy-b(1:2).',2,2)-radius).^2));
    relative=residual/max(sqrt(mean(sum((xy-mean(xy,1)).^2,2))),1e-6);
end
