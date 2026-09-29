function runAnchoredVariants()
% runAnchoredVariants Require compact pole support before trusting a merged pose.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for scale=[0 1 2 4]
        cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";cfg.lineDirection.scatterScale=scale;
        row=replayCloudVariant(source,"anchored_scatter"+scale,cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'anchored_screen.csv'));
    end
end
