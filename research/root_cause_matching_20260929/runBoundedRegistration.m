function runBoundedRegistration()
% runBoundedRegistration Redescending map constraints for conflicting landmarks.
    setupVehicleLocalization();cfg=distributionRegistrationConfig();cfg.geometric.robustLoss="switchable";
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"boundedBaseline",cfg);
    rootCauseReplay('output/root_cause_matching_20260929/curbSurface_sources.mat',"boundedCurbSurface",cfg);
    rootCauseReplay('output/root_cause_matching_20260929/deskew50_sources.mat',"boundedDeskew50",cfg,'output/root_cause_matching_20260929/deskew50_map.mat');
end
