function runSoftVariants()
% runSoftVariants Moment-match plausible association alternatives during solving.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for temperature=[.25 .5 1 2]
        cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";
        cfg.softPointAssociation=struct('temperature',temperature,'radius',1.5);
        row=replayCloudVariant(source,"soft"+round(100*temperature),cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'soft_screen.csv'));
    end
end
