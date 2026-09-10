function pillarIds = completeRecoveredPoleShafts(stats, mapSize, seedIds, cfg)
% completeRecoveredPoleShafts: Join a split fine pole candidate across XY cells.
% A neighboring pillar must be slender and upright on its own, overlap the
% seed in height, and fit the seed's full 3D axis. Counts and XYZ covariance
% measure all returns before any trimming; the normal fine validator decides
% the final point labels. This helper is used only in offline fine seeding.
    pillarIds=double(seedIds(:));
    if isempty(pillarIds),return;end
    covariance=stats.covarianceXYZ;
    slope=covariance(:,4:5)./max(covariance(:,6),eps);
    radialSquared=max(0,covariance(:,1)+covariance(:,3) ...
        -sum(covariance(:,4:5).^2,2)./max(covariance(:,6),eps));
    eligible=stats.count>=cfg.poleMinimumPoints ...
        & stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3)>=cfg.poleMinimumSupportedHeight ...
        & vecnorm(slope,2,2)<=tand(cfg.poleRecoveryMaximumTiltDegrees) ...
        & radialSquared<=cfg.poleRecoveryMaximumRadialStd^2;
    [row,col]=ind2sub(mapSize,double(stats.pillarIndices));
    [~,seedRows]=ismember(pillarIds,double(stats.pillarIndices));
    for seed=seedRows(:).'
        if seed==0,continue;end
        overlap=min(stats.maximumXYZ(:,3),stats.maximumXYZ(seed,3)) ...
            -max(stats.minimumXYZ(:,3),stats.minimumXYZ(seed,3));
        nearby=find(eligible & abs(row-row(seed))<=1 & abs(col-col(seed))<=1 ...
            & overlap>=cfg.poleMinimumSupportedHeight);
        delta=stats.meanXYZ(nearby,:)-stats.meanXYZ(seed,:);
        centerError=delta(:,1:2)-delta(:,3)*slope(seed,:);
        c=covariance(nearby,:);
        axisVariance=c(:,1)+c(:,3)-2*sum(c(:,4:5).*slope(seed,:),2) ...
            +c(:,6)*sum(slope(seed,:).^2);
        axisRms=sqrt(max(0,axisVariance+sum(centerError.^2,2)));
        pillarIds=union(pillarIds,double(stats.pillarIndices(nearby(axisRms<=cfg.poleRecoveryMaximumNeighborAxisRms))));
    end
end
