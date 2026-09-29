function runSurfaceMatching()
% runSurfaceMatching Test actual extended-landmark normal constraints.
    setupVehicleLocalization();assert(contains(fileread(which('prepareSemanticRegistrationGeometry')),'surfaceEligible'),'VehicleLocalization:ResearchPrototypeRequired','Apply rejected_association_prototypes.patch in an isolated baseline checkout first.');cfg=distributionRegistrationConfig();cfg.pointSurfaceClasses="trafficSign";
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"signSurface",cfg);
end
