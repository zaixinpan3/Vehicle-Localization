function candidates=detectPolePillars(maps,facadeMask,cfg)
% detectPolePillars: Select supported poles on the shared pillar lattice.
% The experimental subset mode requires one compact, continuous shaft and
% does not veto it using whole-pillar scatter, core fraction, or isolation.
% The default pillar mode retains the existing whole-pillar/context gates.
% Both modes join whole neighbors without creating a finer pillar lattice.
    stats=maps.statistics; ids=double(stats.pillarIndices);
    if isfield(cfg,'detector') && cfg.detector=="subset"
        candidates=detectSupportedSubsets(maps,cfg,ids); return;
    end
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
        radialVariance<=cfg.maximumRadialStd^2 & ...
        maps.coreFraction(ids)>=cfg.minimumCoreFraction & ...
        maps.coreHeight(ids)>=cfg.minimumCoreHeight & ...
        maps.coreIsolation(ids)>=cfg.minimumCoreIsolation & ...
        maps.corePointCount(ids)>=cfg.minimumCorePoints;
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
    mask=completeSplitShafts(mask,core,maps,cfg);
    candidates=struct('eligibleMask',eligible,'coreMask',core,'supportMask',support, ...
        'footprintCandidateMask',footprint,'contextCandidateMask',context, ...
        'contextFraction',ratio,'componentEvidence',componentSum,'contextEvidence',contextSum,'candidateMask',mask);
end

function mask=completeSplitShafts(mask,core,maps,cfg)
% completeSplitShafts: Rejoin a shaft split over a cell boundary.
% A lone accepted pillar adopts the edge-adjacent core pillar whose density
% peak lies within shaftCompletionDistance of its own peak: both pillars hold
% the same shaft, and the footprint stays an edge pair. A distance of zero
% leaves footprints as selected.
    distance=double(cfg.shaftCompletionDistance);
    if distance<=0 || ~any(mask(:)), return; end
    mapSize=size(mask); components=bwconncomp(mask,8);
    single=components.PixelIdxList(cellfun(@numel,components.PixelIdxList)==1);
    if isempty(single), return; end
    ids=[single{:}].'; [rows,cols]=ind2sub(mapSize,ids);
    rowOffset=[-1 1 0 0]; colOffset=[0 0 -1 1];
    best=zeros(size(ids)); bestDistance=inf(size(ids));
    for direction=1:4
        nr=rows+rowOffset(direction); nc=cols+colOffset(direction);
        valid=nr>=1 & nr<=mapSize(1) & nc>=1 & nc<=mapSize(2);
        neighbour=zeros(size(ids)); neighbour(valid)=sub2ind(mapSize,nr(valid),nc(valid));
        candidate=valid; candidate(valid)=core(neighbour(valid)) & ~mask(neighbour(valid));
        gap=inf(size(ids));
        gap(candidate)=hypot(maps.corePeakX(ids(candidate))-maps.corePeakX(neighbour(candidate)), ...
            maps.corePeakY(ids(candidate))-maps.corePeakY(neighbour(candidate)));
        closer=candidate & gap<=distance & gap<bestDistance;
        best(closer)=neighbour(closer); bestDistance(closer)=gap(closer);
    end
    mask(best(best>0))=true;
end

function candidates=detectSupportedSubsets(maps,cfg,ids)
% A supported point bundle is sufficient even when the whole pillar is broad,
% dense with clutter, or also overlaps a facade. Only supported owner cells
% participate in footprint selection; a neighboring shaft cannot lend a label.
    eligible=maps.occupiedMask; core=false(maps.mapSize);
    core(ids)=maps.poleSubset.found;
    evidence=zeros(maps.mapSize); evidence(ids)=maps.poleSubset.ownCount.*maps.poleSubset.score;
    [footprint,context,mask,ratio,componentSum,contextSum]=selectPillarFootprints( ...
        core,core,evidence,evidence,cfg,false(maps.mapSize));
    mask=completeSplitShafts(mask,core,maps,cfg);
    candidates=struct('eligibleMask',eligible,'coreMask',core,'supportMask',core, ...
        'footprintCandidateMask',footprint,'contextCandidateMask',context, ...
        'contextFraction',ratio,'componentEvidence',componentSum,'contextEvidence',contextSum,'candidateMask',mask);
end
