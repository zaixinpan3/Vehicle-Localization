function runPriorVariants()
% runPriorVariants Reduce density-prior preference without changing map moments.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for power=[0 .25 .5 .75]
        cfg=baselineMatchingConfig();cfg.mapPriorExponent=power;
        row=replayCloudVariant(source,"prior"+round(100*power),cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'prior_screen.csv'));
    end
end
