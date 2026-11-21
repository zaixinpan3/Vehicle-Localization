function keep=confirmDowntownBendCurbStep(candidates,raw,cfg)
% Confirm a road-facing step at the measured XY of each bend candidate.
% Compare an inclined road to the same grade with a jump. Multiple strips
% along the edge must see the jump. A single scan may supply all evidence.
    keep=false(size(candidates,1),1);
    if isempty(candidates) || isempty(raw), return; end
    cloud=pointCloud([raw(:,1:2),zeros(size(raw,1),1)]);
    angles=(0:cfg.angleStepDeg:179).';
    tangents=[cosd(angles),sind(angles)];
    for k=1:size(candidates,1)
        anchor=candidates(k,:);
        rows=findNeighborsInRadius(cloud,[anchor(1:2),0],hypot(cfg.halfLength,cfg.halfWidth));
        patch=raw(rows,:)-anchor;
        patch=patch(abs(patch(:,3))<=cfg.heightWindow,:);
        if size(patch,1)<2*cfg.minSideReturns, continue; end
        for j=1:size(tangents,1)
            tangent=tangents(j,:); normal=[-tangent(2);tangent(1)];
            if anchor(1:2)*normal<0, normal=-normal; end
            along=patch(:,1:2)*tangent.'; across=patch(:,1:2)*normal;
            flank=abs(along)<=cfg.halfLength & abs(across)>=cfg.gap & abs(across)<=cfg.halfWidth;
            if measuredStep(along(flank),across(flank),patch(flank,3),cfg)
                keep(k)=true; break;
            end
        end
    end
end

function valid=measuredStep(along,across,z,cfg)
    valid=false;
    outside=across>0;
    if min(nnz(outside),nnz(~outside))<cfg.minSideReturns, return; end
    plane=[ones(size(z)),along,across];
    [coeff,residual]=fitSurface([plane,outside],z);
    jump=coeff(4);
    if ~isfinite(jump) || jump<cfg.minStep || jump>cfg.maxStep || ...
            prctile(abs(residual),80)>cfg.maxResidual, return; end
    [~,flatResidual]=fitSurface(plane,z);
    loss=@(r) mean(min(abs(r),cfg.lossCap));
    if any(~isfinite(flatResidual)) || loss(residual)>=(1-cfg.minLossReduction)*loss(flatResidual), return; end
    level=z-plane(:,2:3)*coeff(2:3);
    strip=discretize(along,linspace(-cfg.halfLength,cfg.halfLength,4));
    supported=0;
    for i=1:3
        near=strip==i & abs(across)<=cfg.nearWidth;
        high=level(near & outside); low=level(near & ~outside);
        if min(numel(high),numel(low))<2, continue; end
        change=median(high)-median(low);
        supported=supported+(change>=cfg.minStep && abs(change-jump)<=max(cfg.maxResidual,jump/2));
    end
    valid=supported>=2;
end

function [coeff,residual]=fitSurface(A,z)
    coeff=nan(size(A,2),1);residual=inf(size(z));
    if rcond(A.'*A)<1e-7, return; end
    estimate=A\z; errors=z-A*estimate; centered=errors-median(errors);
    inlier=abs(centered)<=max(.025,2.8*median(abs(centered)));
    B=A(inlier,:);
    if nnz(inlier)<size(A,2)+2 || rcond(B.'*B)<1e-7, return; end
    coeff=B\z(inlier);residual=z-A*coeff;
end
