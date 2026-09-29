function screenConsensusSign()
% screenConsensusSign Preserve center information from coherent sign patches.
    setupVehicleLocalization;
    source='output/root_cause_matching_20260929/finalSurface_sources.mat';map='output/mississippi_mapping_calibrated/view_conditioned_cloud.mat';
    for radius=[.1 .15 .2 .25]
        cfg=distributionRegistrationConfig();cfg.partialSign=struct('enabled',true,'minimumAnisotropy',5,'minimumVariance',.01,'consensusRadius',radius,'maximumNormalDifference',.20);
        replayPartialSign(source,"consensus"+round(100*radius),cfg,map);
    end
end
