function [assigned,groups]=assignPillarShaftSupport(points,pillarIds,geometry,modes,cfg)
% assignPillarShaftSupport: Associate axes, then verify each owner's own support.
% Neighbor connectivity never deletes an independent shaft. A weak boundary
% owner can use a validated axis only when its own returns support that axis.
% Representative-based association avoids chaining distinct neighboring poles.
    assigned=modes;assigned.found(:)=false;assigned.score(:)=0;
    groups=struct('representativeRows',zeros(0,1),'ownerGroup',zeros(size(modes.pillarIndices)));
    p=double(points);[ids,~,group]=unique(double(pillarIds(:)));
    assert(isequal(ids,modes.pillarIndices),'perception:InvalidShaftModes','Point owners and shaft evidence must agree.');
    if isempty(p) || ~any(modes.found),return;end
    counts=accumarray(group,1);[~,order]=sort(group);first=cumsum([1;counts(1:end-1)]);
    lookup=zeros(geometry.mapSize);lookup(ids)=1:numel(ids);
    [~,ranking]=sortrows([-modes.score,modes.pillarIndices],[1 2]);ranking=ranking(modes.found(ranking));
    representatives=zeros(0,1);fields=fieldnames(modes);fields=setdiff(fields,{'pillarIndices','found','ownCount','ownHeight'});
    for j=ranking.'
        previous=representatives;
        lo=max(modes.minimumZ(previous),modes.minimumZ(j));hi=min(modes.maximumZ(previous),modes.maximumZ(j));
        a0=modes.axisXY(previous,:)+(lo-modes.axisZ(previous)).*modes.slopeXY(previous,:);
        a1=modes.axisXY(previous,:)+(hi-modes.axisZ(previous)).*modes.slopeXY(previous,:);
        b0=modes.axisXY(j,:)+(lo-modes.axisZ(j)).*modes.slopeXY(j,:);
        b1=modes.axisXY(j,:)+(hi-modes.axisZ(j)).*modes.slopeXY(j,:);
        same=hi-lo>=cfg.shaftMergeMinimumOverlap & ...
            max(vecnorm(a0-b0,2,2),vecnorm(a1-b1,2,2))<=cfg.shaftMergeDistance & ...
            vecnorm(modes.slopeXY(previous,:)-modes.slopeXY(j,:),2,2)<=cfg.shaftMergeSlope;
        % Keep the alternative interval for attribution even for a duplicate
        % axis. Otherwise its valid boundary owner could disappear again.
        if any(same),g=find(same,1);else,representatives(end+1,1)=j;g=numel(representatives);end %#ok<AGROW>
        ends=modes.axisXY(j,:)+([modes.minimumZ(j);modes.maximumZ(j)]-modes.axisZ(j)).*modes.slopeXY(j,:);
        lower=floor((min(ends,[],1)-modes.radius(j)-geometry.origin)./geometry.cellSize)+1;
        upper=floor((max(ends,[],1)+modes.radius(j)-geometry.origin)./geometry.cellSize)+1;
        rr=max(1,lower(2)):min(geometry.mapSize(1),upper(2));cc=max(1,lower(1)):min(geometry.mapSize(2),upper(1));
        owners=lookup(rr,cc);owners=owners(owners>0);
        for owner=owners(:).'
            q=p(order(first(owner):first(owner)+counts(owner)-1),:);
            distance=vecnorm(q(:,1:2)-modes.axisXY(j,:)-(q(:,3)-modes.axisZ(j)).*modes.slopeXY(j,:),2,2);
            support=distance<=modes.radius(j) & q(:,3)>=modes.minimumZ(j) & q(:,3)<=modes.maximumZ(j);
            n=nnz(support);if n<cfg.minimumAssignedPoints,continue;end
            h=max(q(support,3))-min(q(support,3));if h<cfg.minimumAssignedHeight,continue;end
            if assigned.found(owner) && assigned.score(owner)>=modes.score(j)-1e-12,continue;end
            for field=fields.',assigned.(field{1})(owner,:)=modes.(field{1})(j,:);end
            assigned.found(owner)=true;assigned.ownCount(owner)=n;assigned.ownHeight(owner)=h;groups.ownerGroup(owner)=g;
        end
    end
    groups.representativeRows=representatives;
end
