function runLocalDirectionVariants()
% runLocalDirectionVariants Downweight extrapolated map tangent directions.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for scale=[.5 1 2 4]
        cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";cfg.pyramid.trustRadius=.1;
        cfg.pyramid.minimumUnmergedAnchors=2;cfg.pyramid.unmergedAnchorTrustRadius=.15;
        cfg.lineDirection.scatterScale=.25;cfg.lineDirection.localityScale=scale;
        row=replayCloudVariant(source,"localDirection"+round(100*scale),cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'local_direction_screen.csv'));
    end
end
