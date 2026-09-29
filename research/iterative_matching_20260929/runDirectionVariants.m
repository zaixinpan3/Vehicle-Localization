function runDirectionVariants()
% runDirectionVariants Account for observed line scatter in angular influence.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for scale=[0 1 2 4]
        cfg=baselineMatchingConfig();cfg.lineDirection.scatterScale=scale;
        row=replayCloudVariant(source,"scatter"+scale,cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'direction_screen.csv'));
    end
end
