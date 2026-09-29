function runClassAssociation()
% runClassAssociation Separate compact pole and extended sign association.
    setupVehicleLocalization();assert(contains(fileread(which('prepareSemanticRegistrationGeometry')),'association.semanticNames'),'VehicleLocalization:ResearchPrototypeRequired','Apply rejected_association_prototypes.patch in an isolated baseline checkout first.');cfg=distributionRegistrationConfig();
    cfg.softPointAssociation.semanticNames="trafficSign";
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"hardPoleSoftSign",cfg);
    rootCauseReplay('output/root_cause_matching_20260929/curbStep2_sources.mat',"curb2HardPoleSoftSign",cfg);
    cfg=distributionRegistrationConfig();cfg.softPointAssociation.semanticNames="pole";
    rootCauseReplay('output/root_cause_matching_20260929/curbStep2_sources.mat',"curb2SoftPoleHardSign",cfg);
end
