function runBoundedConsensus()
% runBoundedConsensus Keep a suppressed class from vetoing supported geometry.
    setupVehicleLocalization();cfg=distributionRegistrationConfig();cfg.geometric.robustLoss="switchable";cfg.geometric.classGateNormalization="global";
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"boundedConsensus",cfg);
    rootCauseReplay('output/root_cause_matching_20260929/curbSurface_sources.mat',"boundedSurfaceConsensus",cfg);
    rootCauseReplay('output/root_cause_matching_20260929/deskew50_sources.mat',"boundedDeskew50Consensus",cfg,'output/root_cause_matching_20260929/deskew50_map.mat');
end
