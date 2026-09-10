function [selected, evaluated, boundaries] = extendCurbBoundaries(xyz, groundMask, candidateIndices, primaryBoundaries, cfg)
% extendCurbBoundaries: Validate measured continuations outside coarse support.
% Existing fine boundaries supply endpoint search directions, not new labels.
% Unexamined ground returns must pass the same XYZ geometry and thin sampling,
% then form a nearby, directionally consistent connection to the endpoint.
    selected=zeros(0,1);evaluated=zeros(0,1);boundaries={};
    if cfg.curbContinuationLengthMeters<=0,return;end
    missing=groundMask & all(isfinite(xyz),2);missing(candidateIndices)=false;
    for b=1:numel(primaryBoundaries)
        points=xyz(primaryBoundaries{b},:);
        if size(points,1)<3,continue;end
        [~,~,basis]=svd(points(:,1:2)-mean(points(:,1:2)),0);
        along=points(:,1:2)*basis(:,1);
        [along,order]=sort(along);points=points(order,:);
        for side=[-1 1]
            if side<0,origin=points(1,:);local=along<=along(1)+cfg.curbContinuationTangentLengthMeters;
            else,origin=points(end,:);local=along>=along(end)-cfg.curbContinuationTangentLengthMeters;end
            if nnz(local)<3,continue;end
            tail=points(local,1:2);
            [~,~,localBasis]=svd(tail-mean(tail),0);
            direction=localBasis(:,1);
            if dot(direction,side*basis(:,1))<0,direction=-direction;end
            delta=xyz(:,1:2)-origin(1:2);
            forward=delta*direction;
            across=abs(delta*[-direction(2);direction(1)]);
            ids=find(missing & forward>=0 & forward<=cfg.curbContinuationLengthMeters ...
                & across<=cfg.curbContinuationHalfWidthMeters);
            evaluated=union(evaluated,ids);
            [~,detail]=refineCurbGeometry(xyz,ids,groundMask,cfg);
            for j=1:numel(detail.boundaryPointIndices)
                member=detail.boundaryPointIndices{j};p=xyz(member,:);
                distance=vecnorm(p-origin,2,2);
                if min(distance)>cfg.curbContinuationMaximumGapMeters,continue;end
                near=p(distance<=min(distance)+cfg.curbContinuationTangentLengthMeters,1:2);
                if size(near,1)<3,continue;end
                [~,~,tangent]=svd(near-mean(near),0);
                if abs(dot(tangent(:,1),direction))<cosd(cfg.curbContinuationMaximumAngleDegrees),continue;end
                % Keep spacing from previously admitted continuation points.
                if ~isempty(selected)
                    cloud=pointCloud([xyz(selected,1:2),zeros(numel(selected),1)]);
                    keep=true(size(member));
                    for k=1:numel(member)
                        neighbors=findNeighborsInRadius(cloud,[p(k,1:2),0],cfg.curbOutputSpacingMeters);
                        keep(k)=isempty(neighbors);
                    end
                    member=member(keep);
                end
                if isempty(member),continue;end
                selected=union(selected,member);boundaries{end+1}=member; %#ok<AGROW>
            end
        end
    end
end
