function runCurrentDirectionVariants()
% runCurrentDirectionVariants Estimate direction from confirmed current means.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for only=[false true]
        for sigma=[1 2]
            cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";cfg.pyramid.trustRadius=.1;
            cfg.lineDirection.currentAcquisition=true;cfg.lineDirection.currentOnly=only;cfg.lineDirection.standardDeviation=deg2rad(sigma);
            label="currentDirection_o"+double(only)+"_s"+sigma;
            row=replayCloudVariant(source,label,cfg);summaries=[summaries;row]; %#ok<AGROW>
            writetable(summaries,fullfile(dest,'current_direction_screen.csv'));
        end
    end
end
