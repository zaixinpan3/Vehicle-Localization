function runCombinedVariants()
% runCombinedVariants Combine retained mechanisms after isolated comparisons.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for power=[1 0]
        for scale=[.25 .5 1]
            cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";cfg.pyramid.trustRadius=.1;cfg.mapPriorExponent=power;cfg.lineDirection.scatterScale=scale;
            label="combine_p"+power+"_s"+round(100*scale);row=replayCloudVariant(source,label,cfg);summaries=[summaries;row]; %#ok<AGROW>
            writetable(summaries,fullfile(dest,'combined_screen.csv'));
        end
    end
end
