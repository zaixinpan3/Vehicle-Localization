function [x,names]=measureFacadePlacementEvidence(off)
% measureFacadePlacementEvidence: Geometric placement relative to line seeds.
% Sensor bearing and candidate-line coordinates describe current geometry;
% no global position, frame identity or reference label enters these features.
    m=off.columnMaps;ids=find(off.facadeCellMask);[row,col]=ind2sub(m.mapSize,ids);
    xy=m.origin+([col row]-.5).*[m.dx m.dy];r=hypot(xy(:,1),xy(:,2));
    names=["placement_bearingCos","placement_bearingSin","placement_lineNormalX","placement_lineNormalY", ...
        "placement_lineSignedRho","placement_alongFraction","placement_beforeStart","placement_afterEnd", ...
        "placement_signedLineDistance","placement_seedCell","placement_seedDistance", ...
        "placement_ownerSeedFraction","placement_forwardFraction"];
    x=zeros(numel(ids),numel(names));if isempty(ids),return;end
    x(:,1:2)=xy./max(r,eps);facade=off.facade;labels=double(facade.lineMap(ids));
    seeds=facade.detectorMask;distance=bwdist(seeds)*m.dx;x(:,10)=double(seeds(ids));x(:,11)=distance(ids);
    seedFraction=conv2(double(seeds),ones(3),'same')./max(conv2(double(m.occupiedMask),ones(3),'same'),1);x(:,12)=seedFraction(ids);
    for k=1:size(facade.detectedLines,1)
        selected=labels==k;line=facade.detectedLines(k,:);t=line(3:4)-line(1:2);length=norm(t);t=t/max(length,eps);normal=[-t(2),t(1)];
        delta=xy(selected,:)-line(1:2);along=delta*t.';signed=delta*normal.';
        x(selected,3:9)=[repmat([normal,line(1:2)*normal.'],nnz(selected),1), ...
            along/max(length,eps),max(0,-along),max(0,along-length),signed];
        x(selected,13)=mean(xy(selected,1)>0);
    end
    x(~isfinite(x))=0;x=floor(x*1e8+.5)/1e8;
end
