function screenPartialSign()
% screenPartialSign Compare partial-support geometry on the full causal route.
    setupVehicleLocalization;
    assert(contains(fileread(which('prepareSemanticRegistrationGeometry')),'lineSource(selected)=group.line | f.surfaceEligible'),'Apply rejected_unconditional_surface.patch in an isolated checkout.');
    source='output/root_cause_matching_20260929/finalSurface_sources.mat';map='output/mississippi_mapping_calibrated/view_conditioned_cloud.mat';
    for ratio=[2 3 5 9]
        cfg=distributionRegistrationConfig();cfg.partialSign=struct('enabled',true,'minimumAnisotropy',ratio,'minimumVariance',.01,'consensusRadius',.15,'maximumNormalDifference',.2);
        replayPartialSign(source,"intrinsicSurface"+ratio,cfg,map);
    end
end
