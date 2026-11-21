function keep=rejectCurbRoadPatches(candidates,proposals,stats,spacing,cfg)
% Scattered high cells on an inclined road can imitate curb steps. Suppress
% unsupported fragments when candidates spread across the surface and agree
% with the surrounding grade. Preserve measured curb segments and candidates
% reachable along the original proposals, including single-cell breaks.
    keep=candidates;
    anchors=bwareaopen(candidates,cfg.minimumAnchorPillars,8);
    if isequal(candidates,anchors),return;end
    supported=anchors;
    if any(anchors,'all')
        supported=candidates & imreconstruct(anchors,bwmorph(proposals,'bridge',1),8);
    end
    radius=ceil(cfg.supportRadiusMeters/spacing);
    patches=bwconncomp(imdilate(candidates,strel('disk',radius,0)),8);
    occupied=stats.countMap>0 & isfinite(stats.heightMap);
    for k=1:patches.NumObjects
        cells=patches.PixelIdxList{k};
        ids=cells(candidates(cells));
        if numel(ids)<3 || all(supported(ids)),continue;end
        [row,col]=ind2sub(size(candidates),ids);
        transverseVariance=min(eig(cov(spacing*[col(:),row(:)],1)));
        if transverseVariance<cfg.minimumCrosswiseStdMeters^2,continue;end
        observed=cells(occupied(cells));
        if ~all(occupied(ids)),continue;end
        [row,col]=ind2sub(size(candidates),observed);
        xy=spacing*[col(:),row(:)];
        if fitsGradedRoad(xy,double(stats.heightMap(observed)),candidates(observed),cfg)
            keep(ids(~supported(ids)))=false;
        end
    end
end

function fits=fitsGradedRoad(xy,height,candidate,cfg)
% Each occupied pillar counts once. Trim the largest residuals for two
% refits, then evaluate the final plane on all ground and all candidates.
    fits=false;
    height=height(:);
    design=[xy-mean(xy,1),ones(size(height))];
    inliers=true(size(height));
    for iteration=1:3
        samples=design(inliers,:);
        if rank(samples)<3,return;end
        plane=samples\height(inliers);
        residuals=abs(height-design*plane);
        limit=prctile(residuals,cfg.supportPercentile);
        inliers=residuals<=limit;
    end
    fits=norm(plane(1:2))>=cfg.minimumGrade && limit<=cfg.maximumHeightErrorMeters && ...
        prctile(residuals(candidate),cfg.supportPercentile)<=cfg.maximumHeightErrorMeters;
end
