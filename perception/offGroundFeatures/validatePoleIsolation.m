function [isolated, detail] = validatePoleIsolation(points, center, slope, qualified, geometry, cfg)
% validatePoleIsolation: Test shaft separation in a fine raw-point neighborhood.
% Count all nonground returns, including points outside the candidate pillars,
% at the qualified shaft heights. End clutter outside those heights does not
% invalidate an otherwise isolated shaft. This test has no online dependency.
    points=double(points);
    zBin=floor((points(:,3)-geometry.minCorner(3))/geometry.voxelSize(3))+1;
    valid=isfinite(zBin) & zBin>=1 & zBin<=numel(qualified);
    points=points(valid,:);zBin=zBin(valid);
    points=points(qualified(zBin),:);
    radius=vecnorm(points(:,1:2)-center(1:2)-(points(:,3)-center(3))*slope,2,2);
    coreCount=nnz(radius<=cfg.poleIsolationCoreRadius);
    neighborhoodCount=nnz(radius<=cfg.poleIsolationNeighborhoodRadius);
    fraction=coreCount/max(neighborhoodCount,1);
    isolated=fraction>=cfg.poleIsolationMinimumCoreFraction;
    detail=struct('coreCount',coreCount,'neighborhoodCount',neighborhoodCount,'coreFraction',fraction);
end
