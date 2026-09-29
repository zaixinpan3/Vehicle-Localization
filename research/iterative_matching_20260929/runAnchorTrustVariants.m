function runAnchorTrustVariants()
% runAnchorTrustVariants Test compact-landmark trust scale and prior tempering.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for power=[1 0]
        for radius=[.05 .1 .15]
            cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";cfg.pyramid.trustRadius=radius;cfg.mapPriorExponent=power;
            label="anchor_p"+power+"_t"+round(100*radius);row=replayCloudVariant(source,label,cfg);summaries=[summaries;row]; %#ok<AGROW>
            writetable(summaries,fullfile(dest,'anchor_trust_screen.csv'));
        end
    end
end
