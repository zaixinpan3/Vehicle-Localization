function [features,names,ids]=measureCurbJointMoments(ground,gc)
% measureCurbJointMoments Joint XY-height moments within complete coarse owners.
% Owner XY is normalized by the existing pillar footprint. Both owner-centered
% heights and neighborhood-plane residuals retain within-pillar edge position.
% No additional spatial partition, fine labels or per-point class is used.
    dims=ground.cellMapSize;ids=find(ground.curbCellMask);points=double(gc.groundPoints);
    [col,row]=ind2sub(dims([2 1]),double(gc.groundCellLinIdx));owners=sub2ind(dims,row,col);
    groups=accumarray(owners,(1:numel(owners)).',[prod(dims) 1],@(i){i},{zeros(0,1)});
    powers=[];names=strings(1,0);
    for i=0:4
        for j=0:4-i
            for k=0:4-i-j
                if i+j+k==0,continue;end
                powers(end+1,:)=[i j k]; %#ok<AGROW>
                names=[names,"joint"+i+j+k,"detrended"+i+j+k]; %#ok<AGROW>
            end
        end
    end
    names=[names,"ownerHeightScale","ownerTerrainSlopeX","ownerTerrainSlopeY"];
    features=zeros(numel(ids),numel(names));
    for a=1:numel(ids)
        id=ids(a);q=points(groups{id},:);if isempty(q),continue;end
        [row,col]=ind2sub(dims,id);lower=ground.cellOrigin+([col row]-1).*ground.cellSize;
        [cc,rr]=meshgrid(max(1,col-1):min(dims(2),col+1),max(1,row-1):min(dims(1),row+1));
        adjacent=sub2ind(dims,rr(:),cc(:));p=points(vertcat(groups{adjacent}),:);mu=mean(p,1);p=p-mu;
        cv=p.'*p/size(p,1);slope=(cv(1:2,1:2)+eye(2)*1e-8)\cv(1:2,3);
        heights=quantile(q(:,3),[.1 .5 .9]);heightScale=max(diff(heights([1 3])),.1);
        v=2*(q(:,1:2)-lower)./ground.cellSize-1;
        z=max(-2,min(2,(q(:,3)-heights(2))/heightScale));
        residual=max(-3,min(3,(q(:,3)-mu(3)-(q(:,1:2)-mu(1:2))*slope)/.1));
        px=ones(size(q,1),5);py=px;pz=px;pr=px;
        for k=1:4
            px(:,k+1)=px(:,k).*v(:,1);py(:,k+1)=py(:,k).*v(:,2);pz(:,k+1)=pz(:,k).*z;pr(:,k+1)=pr(:,k).*residual;
        end
        base=px(:,powers(:,1)+1).*py(:,powers(:,2)+1);
        values=[mean(base.*pz(:,powers(:,3)+1),1);mean(base.*pr(:,powers(:,3)+1),1)];
        features(a,:)=[values(:).' heightScale slope.'];
    end
    features(~isfinite(features))=0;features=floor(features*1e8+.5)/1e8;
end
