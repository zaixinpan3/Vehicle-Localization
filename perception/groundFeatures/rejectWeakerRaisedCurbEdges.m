function [rejected, protected] = rejectWeakerRaisedCurbEdges(xyz, indices, score, gradients, cfg)
% rejectWeakerRaisedCurbEdges: Prefer a supported lower step to a weak upper edge.
% Compare only spatially separated, parallel local height transitions. Several
% independent XY cells must support the stronger edge before suppressing one.
    rejected=false(size(indices));protected=false(size(indices));
    valid=find(score>=cfg.curbMinimumSeedScore);
    if isempty(valid),return;end
    ids=indices(valid);strength=vecnorm(gradients(valid,:),2,2);
    normal=gradients(valid,:)./max(strength,eps);
    cloud=pointCloud([xyz(ids,1:2),zeros(numel(ids),1)]);
    for k=1:numel(ids)
        rows=findNeighborsInRadius(cloud,[xyz(ids(k),1:2),0],cfg.curbCompetingEdgeRadiusMeters);
        delta=xyz(ids(rows),:)-xyz(ids(k),:);
        across=abs(delta(:,1:2)*normal(k,:).');
        along=abs(delta(:,1:2)*[-normal(k,2);normal(k,1)]);
        support=rows(across>cfg.curbCompetingEdgeMinimumSeparationMeters ...
            & along<2*cfg.curbProposalCellSizeMeters ...
            & delta(:,3)<-cfg.curbMinimumReliefMeters ...
            & strength(rows)>cfg.curbCompetingEdgeGradientRatio*strength(k) ...
            & abs(normal(rows,:)*normal(k,:).')>0.8);
        if numel(support)>=cfg.curbMinimumSurfaceNeighbors && ...
                size(unique(floor(xyz(ids(support),1:2)/cfg.curbProposalCellSizeMeters),'rows'),1)>=cfg.curbMinimumOutputSupportCells
            rejected(valid(k))=true;protected(valid(support))=true;
        end
    end
end
