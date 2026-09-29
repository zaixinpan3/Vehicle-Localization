function screenShapeMatching()
% screenShapeMatching Compare causal shape gates on the complete recording.
    setupVehicleLocalization();cfg=localizationSourceWindowConfig();cfg.maximumShapeDistance=Inf;
    replayShapeMatching("baseline",cfg);
    for threshold=[.25 .5 .75 1]
        cfg=localizationSourceWindowConfig();cfg.maximumShapeDistance=threshold;
        replayShapeMatching("shape"+round(100*threshold),cfg);
    end
    cfg=localizationSourceWindowConfig();matching=distributionRegistrationConfig();matching.partialSign.enabled=false;
    replayShapeMatching("shape50_originalSignResidual",cfg,matching);
end
