function [consistent, fraction] = hasConsistentCurbNormals(xy, gradients, cfg)
% hasConsistentCurbNormals: Require a height transition across a curb boundary.
% Local XY tangents accommodate bends. Sparse endpoints without enough tangent
% support are inconclusive; only a supported mismatch rejects the boundary.
    aligned=false(size(xy,1),1);evaluated=aligned;
    for k=1:size(xy,1)
        nearby=vecnorm(xy-xy(k,:),2,2)<=cfg.curbBoundaryTangentRadiusMeters;
        local=xy(nearby,:);
        if size(local,1)<3 || norm(max(local,[],1)-min(local,[],1))<cfg.curbProposalCellSizeMeters
            continue;
        end
        [~,~,axes]=svd(local-mean(local),0);
        normal=gradients(k,:)/max(norm(gradients(k,:)),eps);
        aligned(k)=abs(normal*axes(:,1))<=sind(cfg.curbMaximumBoundaryNormalAngleDegrees);
        evaluated(k)=true;
    end
    fraction=nnz(aligned)/max(nnz(evaluated),1);
    consistent=nnz(evaluated)<cfg.curbMinimumOutputSupportCells || ...
        fraction>=cfg.curbMinimumBoundaryNormalFraction;
end
