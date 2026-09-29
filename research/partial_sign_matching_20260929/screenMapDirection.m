function screenMapDirection()
% screenMapDirection Include target-line scatter in direction compatibility.
    setupVehicleLocalization;
    assert(contains(fileread(which('prepareSemanticRegistrationGeometry')),'mapScatterScale'),'Apply rejected_direction.patch in an isolated checkout.');
    source='output/root_cause_matching_20260929/finalSurface_sources.mat';map='output/mississippi_mapping_calibrated/view_conditioned_cloud.mat';
    for scale=[.15 .25 .5 1]
        cfg=distributionRegistrationConfig();cfg.partialSign=struct('enabled',true,'minimumAnisotropy',5,'minimumVariance',.01,'consensusRadius',.15,'maximumNormalDifference',.20);
        cfg.lineDirection.mapScatterScale=scale;
        replayPartialSign(source,"mapDirection"+round(100*scale),cfg,map);
    end
end
