function runBoundedSigns()
% runBoundedSigns Redescend only photometric sign constraints, retaining anchors.
    setupVehicleLocalization();cfg=distributionRegistrationConfig();cfg.geometric.robustLoss="switchable";cfg.geometric.classGateNormalization="global";cfg.geometric.boundedClasses="trafficSign";
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"boundedSignsBaseline",cfg);
    rootCauseReplay('output/root_cause_matching_20260929/curbSurface_sources.mat',"boundedSignsSurface",cfg);
    rootCauseReplay('output/root_cause_matching_20260929/deskew50_sources.mat',"boundedSignsDeskew50",cfg,'output/root_cause_matching_20260929/deskew50_map.mat');
    rootCauseReplay('output/root_cause_matching_20260929/deskew100_sources.mat',"boundedSignsDeskew100",cfg,'output/root_cause_matching_20260929/deskew100_map.mat');
end
