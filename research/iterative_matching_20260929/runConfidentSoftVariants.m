function runConfidentSoftVariants()
% runConfidentSoftVariants Preserve strong priors and independent anchor corrections.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for temperature=[.5 1 2 4]
        cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";
        cfg.softPointAssociation=struct('temperature',temperature,'radius',1.5, ...
            'minimumPosterior',.9,'preservePrior',true,'hardWithUnmergedAnchors',2);
        row=replayCloudVariant(source,"confidentSoft"+round(100*temperature),cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'confident_soft_screen.csv'));
    end
end
