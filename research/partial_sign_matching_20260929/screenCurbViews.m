function screenCurbViews()
% screenCurbViews Evaluate conditional curb directions with fixed detector output.
    setupVehicleLocalization;buildCurbViewPrototype;
    source='output/root_cause_matching_20260929/finalSurface_sources.mat';map='output/partial_sign_matching_20260929/curb_view_map.mat';
    for mode=["normal","mean"]
        for bandwidth=[3 5 8]
            cfg=distributionRegistrationConfig();cfg.partialSign=struct('enabled',true,'minimumAnisotropy',5,'minimumVariance',.01,'consensusRadius',.15,'maximumNormalDifference',.20);
            cfg.curbPrototype=struct('bandwidth',bandwidth,'mode',mode);
            replayPartialSign(source,"curbView"+mode+bandwidth,cfg,map);
        end
    end
end
