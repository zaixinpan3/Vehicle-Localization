function keep = checkDowntownCurbSurfaces(candidates,raw,cfg)
% Reject a flat or inverted edge only when both flanks resolve its height.
% Work in the current LiDAR frame. Inconclusive or occluded flanks preserve
% the previous decision. Measurements from one anchor apply to its cell only.
    keep=true(size(candidates,1),1);
    if isempty(candidates) || isempty(raw), return; end
    [~,index,cellId]=unique(floor(candidates(:,1:2)/cfg.anchorSpacing),'rows');
    anchors=double(candidates(index,:));
    if size(anchors,1)<3, return; end
    raw=double(raw);
    cloud=pointCloud([raw(:,1:2),zeros(size(raw,1),1)]);
    accepted=true(size(index));
    for k=1:numel(index)
        edge=anchors(vecnorm(anchors(:,1:2)-anchors(k,1:2),2,2)<=cfg.tangentRadius,1:2);
        if size(edge,1)<3, continue; end
        [~,s,v]=svd(edge-mean(edge,1),0);
        if s(1,1)<2*s(2,2), continue; end
        normal=v(:,2);
        if anchors(k,1:2)*normal<0, normal=-normal; end
        rows=findNeighborsInRadius(cloud,[anchors(k,1:2),0],cfg.patchRadius);
        p=raw(rows,:)-anchors(k,:);
        t=p(:,1:2)*v(:,1); n=p(:,1:2)*normal;
        patch=abs(t)<=cfg.halfLength & abs(n)>=cfg.innerGap & ...
            abs(n)<=cfg.halfWidth & abs(p(:,3))<=cfg.heightWindow;
        inner=surfaceHeight(t,n,p(:,3),patch & n<0,cfg);
        outer=surfaceHeight(t,n,p(:,3),patch & n>0,cfg);
        if isfinite(inner) && isfinite(outer)
            accepted(k)=outer-inner>=cfg.minStep;
        end
    end
    keep=accepted(cellId);
end

function h=surfaceHeight(t,n,z,mask,cfg)
    h=nan;
    if nnz(mask)<cfg.minSidePoints, return; end
    A=[ones(nnz(mask),1),t(mask),n(mask)]; z=z(mask);
    if (max(A(:,3))-min(A(:,3)))<(cfg.halfWidth-cfg.innerGap)/3 || rcond(A.'*A)<1e-5, return; end
    fit=A\z; residual=z-A*fit;
    inlier=abs(residual-median(residual))<=max(cfg.maxSideError,2.5*median(abs(residual-median(residual))));
    A=A(inlier,:); z=z(inlier);
    if numel(z)<cfg.minSidePoints || (max(A(:,3))-min(A(:,3)))<(cfg.halfWidth-cfg.innerGap)/3 || ...
            rcond(A.'*A)<1e-5, return; end
    fit=A\z;
    if rms(z-A*fit)<=cfg.maxSideError, h=fit(1); end
end
