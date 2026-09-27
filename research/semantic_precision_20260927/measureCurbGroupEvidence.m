function [x,names]=measureCurbGroupEvidence(ground,support,supportNames)
% measureCurbGroupEvidence: Connected boundary support and competing ridges.
% Uses the final 0.6 m proposal raster and continuous point statistics only.
    mask=ground.curbCellMask;ids=find(mask);components=bwconncomp(mask,8);n=components.NumObjects;
    names=["curbGroup_cells","curbGroup_length","curbGroup_width","curbGroup_anisotropy", ...
        "curbGroup_stepMean","curbGroup_stepStd","curbGroup_middleCells","curbGroup_middleFraction", ...
        "curbGroup_middlePoints","curbGroup_planeQ90Median","curbGroup_nonplanarFraction", ...
        "curbGroup_roadFraction","curbGroup_orientationAgreement","curbGroup_distance", ...
        "curbGroup_strengthFraction","curbGroup_competingRatio","curbGroup_totalGroups"];
    x=zeros(numel(ids),numel(names));if isempty(ids),return;end
    lookup=zeros(size(mask));lookup(ids)=1:numel(ids);group=zeros(numel(ids),1);summary=zeros(n,14);angles=zeros(n,1);center=zeros(n,2);strength=zeros(n,1);
    step=support(:,supportNames=="raw_stepMax");middle=support(:,supportNames=="raw_stepOwnMiddleCount");
    residual=support(:,supportNames=="raw_planeQ90");planeFraction=support(:,supportNames=="raw_planeAbove02");
    road=conv2(double(ground.roadCellMask),ones(3),'same')>0;
    for k=1:n
        cells=components.PixelIdxList{k};at=lookup(cells);group(at)=k;
        [row,col]=ind2sub(size(mask),cells);xy=ground.cellOrigin+([col(:) row(:)]-.5).*ground.cellSize;
        center(k,:)=mean(xy,1);a=xy-center(k,:);covar=(a.'*a)/size(a,1);[vectors,values]=eig(covar);[eigen,order]=sort(diag(values));t=vectors(:,order(end));normal=[-t(2);t(1)];
        along=a*t;length=max(along)-min(along)+.6;width=sqrt(max(eigen(1),0));angles(k)=atan2(t(2),t(1));
        valid=step(at)>=.07 & step(at)<=.35 & middle(at)>0;
        orientation=double(ground.energyMaps.linearityThetaRadians(cells));agreement=mean(abs(cos(orientation-angles(k))));
        summary(k,:)=[numel(cells),length,width,(eigen(2)-eigen(1))/max(sum(eigen),eps), ...
            mean(step(at)),std(step(at),1),nnz(valid),mean(valid),sum(middle(at)),median(residual(at)), ...
            mean(planeFraction(at)>.05),mean(road(cells)),agreement,abs(center(k,:)*normal)];
        strength(k)=length*mean(valid)*max(median(residual(at)),.001);
    end
    context=zeros(n,3);
    for k=1:n
        normal=[-sin(angles(k));cos(angles(k))];delta=center-center(k,:);
        adjacent=abs(cos(angles-angles(k)))>=cosd(30) & abs(delta*normal)<=1.8 & vecnorm(delta,2,2)<=max(summary(k,2),3);
        adjacent(k)=false;competitor=0;if any(adjacent),competitor=max(strength(adjacent));end
        context(k,:)=[strength(k)/max(sum(strength),eps),competitor/max(strength(k),eps),n];
    end
    x=[summary(group,:),context(group,:)];x(~isfinite(x))=0;x=floor(x*1e8+.5)/1e8;
end
