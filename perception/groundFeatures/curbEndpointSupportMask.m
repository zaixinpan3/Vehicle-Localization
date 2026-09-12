function retained = curbEndpointSupportMask(xy, gradients, cfg)
% curbEndpointSupportMask: Trim unsupported tips of a measured curb boundary.
% Keep sparse inconclusive endpoints. Reject a tip only when its local height
% gradient disagrees with a supported tangent and no measured boundary return
% supports its predicted tangent beyond one proposal cell.
    retained=true(size(xy,1),1);
    if size(xy,1)<3,return;end
    [~,~,basis]=svd(xy-mean(xy,1),0);
    along=xy*basis(:,1);
    tips=along<=min(along)+cfg.curbOutputSpacingMeters | ...
        along>=max(along)-cfg.curbOutputSpacingMeters;
    for k=find(tips).'
        delta=xy-xy(k,:);distance=vecnorm(delta,2,2);
        local=xy(distance<=cfg.curbBoundaryTangentRadiusMeters,:);
        if size(local,1)<3 || norm(max(local,[],1)-min(local,[],1))<cfg.curbProposalCellSizeMeters
            continue;
        end
        [~,~,axes]=svd(local-mean(local,1),0);
        normal=gradients(k,:)/max(norm(gradients(k,:)),eps);
        if abs(normal*axes(:,1))<=sind(cfg.curbMaximumEndpointNormalAngleDegrees),continue;end
        witness=distance>=cfg.curbProposalCellSizeMeters & ...
            distance<=cfg.curbBoundaryTangentRadiusMeters & ...
            abs(delta*normal.')<=cfg.curbConsensusBandMeters;
        retained(k)=any(witness);
    end
end
