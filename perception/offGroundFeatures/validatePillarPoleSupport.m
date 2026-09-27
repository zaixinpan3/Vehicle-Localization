function [evidence,hypotheses]=validatePillarPoleSupport(points,pillarIds,geometry,modes,cfg,structuralMask,pointScores)
% validatePillarPoleSupport: Accept full shaft support before assigning owners.
% Intensity affects density qualification only. Every return can contribute to
% the fitted distribution and ownership. Scores rank geometric evidence; they
% are not calibrated probabilities. Reference labels never enter this path.
    ids=double(modes.pillarIndices(:));n=numel(ids);
    evidence=struct('pillarIndices',ids,'found',false(n,1),'score',zeros(n,1), ...
        'ownCount',zeros(n,1),'height',zeros(n,1),'radialRms',zeros(n,1), ...
        'isolation',zeros(n,1));
    hypotheses=measurePillarPoleSupport(points,pillarIds,geometry,modes,cfg,structuralMask);
    for k=1:numel(hypotheses)
        h=hypotheses(k);short=h.supportHeight<=cfg.shortSupportHeight;
        maximumStd=cfg.maximumStd;if short,maximumStd=cfg.maximumShortStd;end
        wide=h.maximumStd>maximumStd && h.aspect>cfg.maximumAspect;
        dense=h.coreCount>=cfg.denseMinimumPoints && h.densityContrast>=cfg.denseMinimumContrast && ...
            h.tilt<=cfg.denseMaximumTilt && h.radialRms<=cfg.denseMaximumRms;
        accepted=h.supportHeight>=cfg.minimumSupportHeight-1e-10 && h.meanRatio>=cfg.minimumMeanRatio && ...
            h.massRatio>=cfg.minimumMassRatio && h.tilt<=cfg.maximumTilt && ...
            h.radialRms<=cfg.maximumRms && ~wide && h.isolation>=cfg.minimumIsolation && ...
            h.acceptedCount>=cfg.poleMinimumPoints;
        if short
            accepted=accepted && h.longestSupportedHeight>=cfg.minimumShortRun && ...
                h.tilt<=cfg.maximumShortTilt && h.radialRms<=cfg.maximumShortRms;
        end
        if h.meanRatio<cfg.strongMeanRatio || short
            accepted=accepted && (h.isolation>=cfg.weakMinimumIsolation || dense);
        end
        if ~accepted,continue;end
        [~,rows]=ismember(h.ownerIds,ids);
        supported=h.ownerCount>=cfg.minimumOwnerPoints & h.ownerHeight>=cfg.minimumOwnerHeight & ...
            pointScores(rows)>=cfg.minimumPointScore;
        rows=rows(supported);ownCount=h.ownerCount(supported);
        score=min(1,h.supportHeight/2)*h.meanRatio*h.isolation/(1+(h.radialRms/.10)^2);
        update=score>evidence.score(rows);rows=rows(update);ownCount=ownCount(update);
        evidence.found(rows)=true;evidence.score(rows)=score;evidence.ownCount(rows)=ownCount;
        evidence.height(rows)=h.supportHeight;evidence.radialRms(rows)=h.radialRms;
        evidence.isolation(rows)=h.isolation;
    end
end
