function [selected, boundaries] = traceCurbRidges(xyz, indices, score, gradients, seed, planeResidual, cfg)
% traceCurbRidges: Follow locally supported curb bends in unorganized XYZ.
% Suppress neighboring responses across a face, then connect original ridge
% returns only along mutually compatible tangents. A long spatial shortcut
% cannot substitute for a chain of measured, locally supported curb points.
    selected=false(size(indices));boundaries={};
    valid=seed & planeResidual>=cfg.curbMinimumOutputPlaneResidualMeters;
    if ~any(valid),return;end
    normal=gradients./max(vecnorm(gradients,2,2),eps);
    cloud=pointCloud([xyz(indices,1:2),zeros(numel(indices),1)]);
    [~,priority]=sortrows([-score,xyz(indices,:)],[1 2 3 4]);
    suppressed=false(size(indices));nodes=zeros(0,1);
    acrossRadius=cfg.curbProposalCellSizeMeters;
    alongRadius=cfg.curbOutputSpacingMeters/2;
    for k=priority(:).'
        if ~valid(k)||suppressed(k),continue;end
        nodes(end+1,1)=k; %#ok<AGROW>
        rows=findNeighborsInRadius(cloud,[xyz(indices(k),1:2),0],hypot(acrossRadius,alongRadius));
        delta=xyz(indices(rows),1:2)-xyz(indices(k),1:2);
        nearby=abs(delta*normal(k,:).')<acrossRadius ...
            & abs(delta*[-normal(k,2);normal(k,1)])<alongRadius ...
            & normal(rows,:)*normal(k,:).'>cosd(cfg.curbRidgeMaximumNormalAngleDegrees);
        suppressed(rows(nearby))=true;
    end
    points=xyz(indices(nodes),:);unit=normal(nodes,:);edges=zeros(0,2);
    for k=1:numel(nodes)
        delta=points(:,1:2)-points(k,1:2);distance=vecnorm(delta,2,2);
        compatible=distance>0 & distance<=cfg.curbRidgeMaximumGapMeters ...
            & abs(delta*unit(k,:).')<cfg.curbConsensusBandMeters ...
            & abs(sum(delta.*unit,2))<cfg.curbConsensusBandMeters ...
            & unit*unit(k,:).'>cosd(cfg.curbRidgeMaximumNormalAngleDegrees) ...
            & abs(points(:,3)-points(k,3))<=cfg.curbMaximumSurfaceSlope*distance+cfg.curbMinimumReliefMeters;
        rows=find(compatible & (1:numel(nodes)).'>k);
        edges=[edges;repmat(k,numel(rows),1),rows]; %#ok<AGROW>
    end
    components=conncomp(graph(edges(:,1),edges(:,2),[],numel(nodes)));
    for group=unique(components)
        rows=find(components==group);p=points(rows,:);
        cells=unique(floor((p(:,1:2)-min(points(:,1:2),[],1))/cfg.curbProposalCellSizeMeters),'rows');
        if size(cells,1)<cfg.curbMinimumSupportCells || ...
                norm(max(p(:,1:2),[],1)-min(p(:,1:2),[],1))<cfg.curbMinimumBoundaryLengthMeters
            continue;
        end
        selected(nodes(rows))=true;boundaries{end+1}=indices(nodes(rows)); %#ok<AGROW>
    end
end
