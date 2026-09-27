function [features,owners,hypothesisIds]=measurePillarPoleFeatures(points,pillarIds,geometry,hypotheses,structural,pointScore,lineScore,cfg,featureNames)
% measurePillarPoleFeatures: Joint distributions on the existing whole pillars.
% Continuous overlapping radial/height probes are statistics, not an XY grid.
% Only raw current-frame points and their coarse owners enter these features.
    q=double(points);pillarIds=double(pillarIds(:));owners=[];hypothesisIds=[];
    features=zeros(0,numel(featureNames));if isempty(q)||isempty(hypotheses),return;end
    [occupied,~,group]=unique(pillarIds);count=accumarray(group,1);[~,order]=sort(group);offset=cumsum([1;count(1:end-1)]);
    lookup=zeros(geometry.mapSize);lookup(occupied)=1:numel(occupied);heightMap=nan(geometry.mapSize);
    needMoments=any(startsWith(string(featureNames),["moment","core_moment","quantile"]));
    wholeNames=string(featureNames);wholeNames=erase(wholeNames(startsWith(wholeNames,"whole_")),"whole_");
    radii=[.30 .40 .60 .75];needed=false(size(radii));
    for k=1:numel(radii)
        tag=sprintf('%02d',round(100*radii(k)));
        needed(k)=any(ismember(["std","aspect","rms","count","shift"]+tag,string(featureNames)));
    end
    radii=radii(needed);
    featureRows=cell(numel(hypotheses),1);ownerRows=featureRows;hypothesisRows=featureRows;
    for a=1:numel(hypotheses)
        h=hypotheses(a);keep=h.ownerCount>=cfg.minimumOwnerPoints & h.ownerHeight>=cfg.minimumOwnerHeight;
        if ~any(keep),continue;end
        lo=floor((h.axisXY-2-geometry.origin)./geometry.cellSize)+1;
        hi=floor((h.axisXY+2-geometry.origin)./geometry.cellSize)+1;
        rr=max(1,lo(2)):min(geometry.mapSize(1),hi(2));cc=max(1,lo(1)):min(geometry.mapSize(2),hi(1));
        cells=lookup(rr,cc);cells=cells(cells>0);
        pieces=arrayfun(@(j)order(offset(j):offset(j)+count(j)-1),cells(:),'UniformOutput',false);index=vertcat(pieces{:});
        p=q(index,:);localOwners=pillarIds(index);mask=structural(index);
        f=measurePoleAxisContext(p,mask,h,featureNames);
        residual=p(:,1:2)-h.axisXY-(p(:,3)-h.axisZ).*h.slopeXY;
        distance=vecnorm(residual,2,2);supported=p(:,3)>=h.minimumZ & p(:,3)<=h.maximumZ;
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
        for radius=radii
            tag=sprintf('%02d',round(100*radius));use=supported & distance<=radius;
            e=sort(max(eig(cov(residual(use,:))),0));
            f.(['std' tag])=sqrt(e(2));f.(['aspect' tag])=sqrt(e(2)/max(e(1),eps));
            f.(['rms' tag])=sqrt(mean(distance(use).^2));f.(['count' tag])=nnz(use);
            f.(['shift' tag])=norm(mean(residual(use,:),1));
        end
        near=all(abs(p(:,1:2)-h.axisXY)<=2,2);
        local=measurePoleContinuousContext(p(near,:),mask(near),h,true,featureNames);
        whole=measurePoleContinuousContext(p(near,:),mask(near),h,false,wholeNames);
        for key=fieldnames(local).',f.(key{1})=local.(key{1});end
        for key=fieldnames(whole).',f.(['whole_' key{1}])=whole.(key{1});end
        at=find(keep);values=zeros(numel(at),numel(featureNames));
        for j=1:numel(at)
            b=at(j);owner=h.ownerIds(b);r=f;r.ownerCount=h.ownerCount(b);r.ownerHeight=h.ownerHeight(b);
            r.ownerFraction=r.ownerCount/max(h.acceptedCount,1);
            r.pointScore=double(pointScore(owner));r.lineScore=double(lineScore(owner));
            own=localOwners==owner;r.ownerTotalCount=nnz(own);r.ownerTotalHeight=max(p(own,3))-min(p(own,3));
            r.ownerMedianRadius=median(distance(own));r.ownerCoreFraction=nnz(own & core)/max(nnz(own),1);
            r.ownerTightFraction=nnz(own & supported & distance<=.10)/max(nnz(own & supported),1);
            r.ownerSupportFraction=r.ownerCount/max(nnz(own),1);
            r.ownerRangeNormalizedSupport=r.ownerCount*max(f.axisRange,1)^2/max(r.ownerHeight,eps);
            [y,x]=ind2sub(geometry.mapSize,owner);lower=geometry.origin+([x y]-1).*geometry.cellSize;
            ends=h.axisXY+([h.minimumZ;h.maximumZ]-h.axisZ).*h.slopeXY;
            r.axisOwnerDistance=min(vecnorm(max(max(lower-ends,ends-lower-geometry.cellSize),0),2,2));
            xy=(p(own,1:2)-lower)./geometry.cellSize;mu=mean(xy,1);c=cov(xy);axis=(h.axisXY-lower)./geometry.cellSize;
            r.axisOwnerX=axis(1);r.axisOwnerY=axis(2);r.slopeX=h.slopeXY(1);r.slopeY=h.slopeXY(2);
            r.ownerMeanX=mu(1);r.ownerMeanY=mu(2);r.ownerVarianceX=c(1,1);r.ownerVarianceY=c(2,2);r.ownerCovarianceXY=c(1,2);
            r.ownerSkewX=mean((xy(:,1)-mu(1)).^3)/max(c(1,1)^1.5,eps);
            r.ownerSkewY=mean((xy(:,2)-mu(2)).^3)/max(c(2,2)^1.5,eps);
            [cc,rr]=meshgrid(max(1,x-1):min(geometry.mapSize(2),x+1),max(1,y-1):min(geometry.mapSize(1),y+1));
            neighbors=sub2ind(geometry.mapSize,rr(:),cc(:));
            for id=neighbors.'
                if isnan(heightMap(id))
                    row=lookup(id);z=[];
                    if row>0
                        ind=order(offset(row):offset(row)+count(row)-1);z=q(ind(structural(ind)),3);
                    end
                    heightMap(id)=countSupport(z);
                end
            end
            others=heightMap(neighbors(neighbors~=owner));r.ownerContinuousHeight=heightMap(owner);
            r.ownerContextHeightFraction=heightMap(owner)/max(sum(heightMap(neighbors)),eps);
            r.neighborContinuousHeight=sum(others);r.maximumNeighborContinuousHeight=max([0;others]);
            if needMoments
                moment=measurePillarPoleMoments(p(own,:),lower,geometry.cellSize,h);
                for key=fieldnames(moment).',r.(key{1})=moment.(key{1});end
            end
            for k=1:numel(featureNames),values(j,k)=r.(featureNames{k});end
        end
        featureRows{a}=values;ownerRows{a}=h.ownerIds(keep);hypothesisRows{a}=repmat(a,nnz(keep),1);
    end
    nonempty=~cellfun(@isempty,featureRows);
    if any(nonempty)
        features=vertcat(featureRows{nonempty});owners=vertcat(ownerRows{nonempty});hypothesisIds=vertcat(hypothesisRows{nonempty});
    end
end

function [radius,residual,offset,relative]=circleShape(xy)
    radius=0;residual=0;offset=0;relative=0;if size(xy,1)<4,return;end
    design=[2*xy,ones(size(xy,1),1)];b=pinv(design)*sum(xy.^2,2);
    radius=sqrt(max(0,b(3)+sum(b(1:2).^2)));offset=norm(b(1:2));
    residual=sqrt(mean((vecnorm(xy-b(1:2).',2,2)-radius).^2));
    relative=residual/max(sqrt(mean(sum((xy-mean(xy,1)).^2,2))),1e-6);
end

function total=countSupport(z)
    if numel(z)<3,total=0;return;end
    [p,~,g]=unique([z-.25;z+.25]);n=numel(z);count=cumsum(accumarray(g,[ones(n,1);-ones(n,1)]));
    total=sum(diff(p).*(count(1:end-1)>=3));
end
