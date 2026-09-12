function [boundaries, residuals] = validateCurbRoadSupport(xyz, groundXYZ, groundXYCloud, boundaries, cfg)
% validateCurbRoadSupport: Require a measured lower road surface beside a curb.
% Fit the two sides independently so rough elevated terrain does not obscure
% a flat lower road. Compare already supported parallel boundaries before
% they can seed further continuation into a raised surface.
    residuals=nan(size(boundaries));
    tangents=zeros(numel(boundaries),2);
    for j=1:numel(boundaries)
        model=xyz(boundaries{j},1:2);
        if size(model,1)<3,continue;end
        [~,~,axes]=svd(model-mean(model,1),0);tangents(j,:)=axes(:,1).';
        localResidual=nan(size(model,1),1);
        for k=1:size(model,1)
            local=model(vecnorm(model-model(k,:),2,2)<=cfg.curbBoundaryTangentRadiusMeters,:);
            if size(local,1)<3,continue;end
            [~,~,axes]=svd(local-mean(local,1),0);normal=axes(:,2);
            rows=findNeighborsInRadius(groundXYCloud,[model(k,:),0],cfg.curbNeighborhoodRadiusMeters);
            p=groundXYZ(rows,:)-xyz(boundaries{j}(k),:);
            across=p(:,1:2)*normal;
            heights=nan(2,1);errors=nan(2,1);
            for side=1:2
                selected=(2*side-3)*across>cfg.curbLocalStripHalfWidthMeters;
                if nnz(selected)<cfg.curbMinimumSurfaceNeighbors,continue;end
                surface=p(selected,:);design=[ones(size(surface,1),1),surface(:,1:2)];
                if rcond(design.'*design)<1e-10,continue;end
                fit=design\surface(:,3);
                heights(side)=fit(1);
                errors(side)=sqrt(mean((surface(:,3)-design*fit).^2));
            end
            if all(isfinite(heights))
                [~,lower]=min(heights);localResidual(k)=errors(lower);
            end
        end
        measured=isfinite(localResidual);
        cells=unique(floor(model(measured,:)/cfg.curbProposalCellSizeMeters),'rows');
        if size(cells,1)>=cfg.curbMinimumOutputSupportCells
            residuals(j)=median(localResidual(measured));
        end
    end
    retained=~(residuals>cfg.curbMaximumRoadSurfaceResidualMeters);
    supported=retained & isfinite(residuals);
    rejected=false(size(boundaries));
    for j=find(supported(:)).'
        p=xyz(boundaries{j},:);tangent=tangents(j,:).';normal=[-tangent(2);tangent(1)];
        conflict=false(size(p,1),1);
        for k=find(supported(:)).'
            if k==j || residuals(j)<=cfg.curbCompetingBoundaryRoadResidualRatio*max(residuals(k),sqrt(eps)) || ...
                    abs(tangents(k,:)*tangent)<cosd(cfg.curbContinuationMaximumAngleDegrees)
                continue;
            end
            lower=xyz(boundaries{k},:);
            for m=1:size(p,1)
                delta=lower-p(m,:);
                witness=vecnorm(delta(:,1:2),2,2)<=cfg.curbBoundaryTangentRadiusMeters & ...
                    abs(delta(:,1:2)*normal)>cfg.curbCompetingEdgeMinimumSeparationMeters & ...
                    abs(delta(:,1:2)*tangent)<2*cfg.curbProposalCellSizeMeters & ...
                    delta(:,3)<-cfg.curbMinimumReliefMeters/2;
                if nnz(witness)>=cfg.curbMinimumOutputSupportCells,conflict(m)=true;end
            end
        end
        cells=unique(floor(p(conflict,1:2)/cfg.curbProposalCellSizeMeters),'rows');
        rejected(j)=nnz(conflict)/size(p,1)>=cfg.curbCompetingBoundaryMinimumFraction && ...
            size(cells,1)>=cfg.curbMinimumOutputSupportCells;
    end
    boundaries=boundaries(retained & ~rejected);
end
