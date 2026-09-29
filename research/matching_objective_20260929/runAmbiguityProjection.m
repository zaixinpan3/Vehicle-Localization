function runAmbiguityProjection()
% runAmbiguityProjection Do not anchor directions dominated by association spread.
    setupVehicleLocalization();file='output/root_cause_matching_20260929/finalSurface_sources.mat';cfg=distributionRegistrationConfig();
    cfg.pyramid.mapMergeRadius=0;cfg.pyramid.sourceMergeRadius=0;
    cfg.softPointAssociation.hardWithUnmergedAnchors=1e6;
    replayStudy(file,"unmergedSoftControl",cfg);
    for fraction=[.15 .35 .55]
        cfg.softPointAssociation.ambiguityVarianceFraction=fraction;
        replayStudy(file,"projectedSoftAmbiguity"+round(100*fraction),cfg);
    end
end
