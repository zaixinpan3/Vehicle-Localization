function [selected, evaluated, boundaries] = extendCurbBoundaries(xyz, groundMask, candidateIndices, primaryBoundaries, cfg)
% extendCurbBoundaries: Validate measured continuations of fine curb endpoints.
% Existing fine boundaries supply endpoint search directions, not new labels.
% Unexamined returns use ordinary geometry. Rejected endpoint candidates can
% use the observed tangent and longer along-curb support, with unchanged
% transverse geometry gates, anchored overlap and a connected measured chain.
    selected=zeros(0,1);evaluated=zeros(0,1);boundaries={};
    if cfg.curbContinuationLengthMeters<=0,return;end
    finiteGround=groundMask & all(isfinite(xyz),2);
    primaryIndices=unique(vertcat(primaryBoundaries{:}));
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
            ordinaryCount=numel(detail.boundaryPointIndices);
            % Revisit rejected endpoint candidates with the observed tangent.
            % Overlapping support anchors the new model to the existing curb.
            guidedIds=find(finiteGround & forward>=-cfg.curbContinuationTangentLengthMeters ...
                & forward<=cfg.curbContinuationLengthMeters & across<=cfg.curbContinuationHalfWidthMeters);
            evaluated=union(evaluated,guidedIds);
            [~,guided]=refineCurbGeometry(xyz,guidedIds,groundMask,cfg,[-direction(2),direction(1)]);
            [~,traced]=refineCurbGeometry(xyz,guidedIds,groundMask,cfg,[],true);
            detail.boundaryPointIndices=[detail.boundaryPointIndices,guided.boundaryPointIndices,traced.boundaryPointIndices];
            for j=1:numel(detail.boundaryPointIndices)
                member=detail.boundaryPointIndices{j};p=xyz(member,:);
                distance=vecnorm(p-origin,2,2);
                if min(distance)>cfg.curbContinuationMaximumGapMeters,continue;end
                near=p(distance<=min(distance)+cfg.curbContinuationTangentLengthMeters,1:2);
                if size(near,1)<3,continue;end
                [~,~,tangent]=svd(near-mean(near),0);
                if abs(dot(tangent(:,1),direction))<cosd(cfg.curbContinuationMaximumAngleDegrees),continue;end
                if j>ordinaryCount
                    behind=member(forward(member)<=0);
                    if numel(behind)<cfg.curbMinimumOutputSupportCells,continue;end
                    distanceToTail=min(sqrt((xyz(behind,1)-tail(:,1).').^2+ ...
                        (xyz(behind,2)-tail(:,2).').^2),[],2);
                    anchored=behind(distanceToTail<=cfg.curbOutputSpacingMeters);
                    if numel(anchored)<cfg.curbMinimumOutputSupportCells || ...
                            max(forward(anchored))-min(forward(anchored))<cfg.curbMinimumBoundaryLengthMeters/2
                        continue;
                    end
                    member=member(forward(member)>cfg.curbOutputSpacingMeters/2);
                    member=member(~ismember(member,primaryIndices));
                    [~,order]=sort(forward(member));member=member(order);p=xyz(member,:);
                    gaps=vecnorm(diff([origin;p]),2,2);
                    stop=find(gaps>cfg.curbContinuationMaximumGapMeters,1);
                    if ~isempty(stop),member=member(1:stop-1);p=xyz(member,:);end
                    cells=unique(floor(p(:,1:2)/cfg.curbProposalCellSizeMeters),'rows');
                    % The overlap already establishes a boundary; require
                    % output support for its measured continuation segment.
                    if size(cells,1)<cfg.curbMinimumOutputSupportCells || ...
                            max(forward(member))-min(forward(member))<cfg.curbMinimumBoundaryLengthMeters
                        continue;
                    end
                end
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
