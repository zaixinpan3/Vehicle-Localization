function runIndependentAnchorVariants()
% runIndependentAnchorVariants Allow larger refinement with independent map anchors.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    for radius=[.05 .1]
        for scatter=[0 .25 .5]
            cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";cfg.pyramid.trustRadius=radius;
            cfg.pyramid.minimumUnmergedAnchors=2;cfg.pyramid.unmergedAnchorTrustRadius=.15;cfg.lineDirection.scatterScale=scatter;
            label="independent_t"+round(100*radius)+"_s"+round(100*scatter);row=replayCloudVariant(source,label,cfg);summaries=[summaries;row]; %#ok<AGROW>
            writetable(summaries,fullfile(dest,'independent_anchor_screen.csv'));
        end
    end
end
