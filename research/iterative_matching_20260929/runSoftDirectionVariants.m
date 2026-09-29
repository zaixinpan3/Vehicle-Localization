function runSoftDirectionVariants()
% runSoftDirectionVariants Audit the remaining maximum after ambiguity handling.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    saved=load('output/iterative_matching_20260929/confidentSoft200.mat','cfg');
    for scale=[.25 .5 1]
        cfg=saved.cfg;cfg.lineDirection.scatterScale=scale;
        row=replayCloudVariant(source,"softDirection"+round(100*scale),cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'soft_direction_screen.csv'));
    end
end
