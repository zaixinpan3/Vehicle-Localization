function candidates=detectPolePillars(maps,facadeMask,cfg)
% detectPolePillars: Whole-pillar XYZ extent and compact XY context.
% Metric height and raw point support replace all subpillar occupancy gates.
% Candidate footprints join whole neighbors without subdividing any pillar.
    stats=maps.statistics; ids=double(stats.pillarIndices);
    covariance=stats.covarianceXYZ;
    slope=covariance(:,4:5)./max(covariance(:,6),eps);
    radialVariance=max(0,covariance(:,1)+covariance(:,3)- ...
        sum(covariance(:,4:5).^2,2)./max(covariance(:,6),eps));
    eligible=maps.occupiedMask & ~facadeMask;
    core=false(maps.mapSize);
    core(ids)=stats.count>=cfg.minimumPoints & ...
        stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3)>=cfg.minimumHeight & ...
        covariance(:,6)>=cfg.minimumHeightStd^2 & ...
        vecnorm(slope,2,2)<=tand(cfg.maximumTiltDegrees) & ...
        radialVariance<=cfg.maximumRadialStd^2;
    core=core & eligible & maps.lineScore<=cfg.maximumLineScore;
    support=eligible & maps.pillarCounts>=3 & maps.pillarZRange>=0.5;
    [footprint,context,mask,ratio,componentSum,contextSum]=selectPillarFootprints( ...
        core,support,maps.pillarCounts,maps.supportEvidence,cfg,false(maps.mapSize));
    components=bwconncomp(mask,8);
    for cells=components.PixelIdxList
        if mean(double(maps.pointScore(cells{1})))<cfg.minimumPointScore
            mask(cells{1})=false;
        end
    end
    candidates=struct('eligibleMask',eligible,'coreMask',core,'supportMask',support, ...
        'footprintCandidateMask',footprint,'contextCandidateMask',context, ...
        'contextFraction',ratio,'componentEvidence',componentSum,'contextEvidence',contextSum,'candidateMask',mask);
end
