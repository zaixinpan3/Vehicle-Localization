function retained = curbEndpointSupportMask(xy, gradients, cfg)
% curbEndpointSupportMask: Trim unsupported terminal runs of a curb boundary.
% Keep sparse inconclusive endpoints. Reject a tip only when its local height
% gradient disagrees with a supported tangent and no measured boundary return
% supports its predicted tangent beyond one proposal cell.
    retained=true(size(xy,1),1);
    if size(xy,1)<3,return;end
    [~,~,basis]=svd(xy-mean(xy,1),0);
    along=xy*basis(:,1);
    unsupported=false(size(retained));
    % Measure against the original boundary so trimming cannot rotate the
    % tangent repeatedly and erode an otherwise supported continuation.
    for k=1:size(xy,1)
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
        unsupported(k)=~any(witness);
    end
    [~,order]=sort(along);
    supported=find(~unsupported(order));
    if isempty(supported),retained(:)=false;return;end
    retained(order(1:supported(1)-1))=false;
    retained(order(supported(end)+1:end))=false;
end
