function accepted = validateFacadeCandidatePoints(points, offGround, cfg)
% validateFacadeCandidatePoints: Offline plane tests on candidate members only.
% Each Hough line supplies a vertical-plane seed. Robust re-estimation uses
% only that line's candidate points. Height, extent, verticality, orientation,
% and point-to-plane distance are metric acceptance tests. Failure stays empty.
    accepted = false(size(points,1),1);
    if isempty(points), return; end
    maps = offGround.columnMaps;
    bins = floor((points(:,1:2)-maps.origin)./[maps.dx maps.dy])+1;
    cells = sub2ind(maps.mapSize,bins(:,2),bins(:,1));
    group = double(offGround.facade.lineMap(cells));
    for lineId = unique(group(group>0)).'
        rows = find(group==lineId);
        p = points(rows,:);
        line = offGround.facade.detectedLines(lineId,:);
        tangent = line(3:4)-line(1:2);
        if numel(rows)<cfg.facadeMinimumPoints || norm(tangent)<eps, continue; end
        tangent = tangent/norm(tangent);
        seedNormal = [-tangent(2);tangent(1);0];
        normal = seedNormal;
        center = median(p,1);
        inliers = abs((p-center)*normal)<=cfg.facadeMaximumDistance;
        for iteration = 1:3
            if nnz(inliers)<cfg.facadeMinimumPoints, break; end
            center = mean(p(inliers,:),1);
            [~,~,basis] = svd(p(inliers,:)-center,0);
            normal = basis(:,end);
            residual = abs((p-center)*normal);
            mid = median(residual(inliers));
            sigma = max(cfg.minimumResidualScale,1.4826*median(abs(residual(inliers)-mid)));
            inliers = residual<=min(cfg.facadeMaximumDistance,mid+cfg.robustScale*sigma);
        end
        if nnz(inliers)<cfg.facadeMinimumPoints || ...
                abs(normal(3))>sind(cfg.facadeMaximumNormalAngleDegrees) || ...
                abs(normal.'*seedNormal)<cosd(cfg.facadeMaximumNormalAngleDegrees)
            continue;
        end
        support = p(inliers,:);
        extent = support(:,1:2)*tangent.';
        if (max(support(:,3))-min(support(:,3)))<cfg.facadeMinimumHeight || (max(extent)-min(extent))<cfg.facadeMinimumLength
            continue;
        end
        accepted(rows) = inliers;
    end
end
