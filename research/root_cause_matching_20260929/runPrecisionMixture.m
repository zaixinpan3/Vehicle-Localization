function runPrecisionMixture()
% runPrecisionMixture Evaluate information-weighted soft assignment updates.
    setupVehicleLocalization();assert(contains(fileread(which('softPointAssociationTarget')),'precisionMixture'),'VehicleLocalization:ResearchPrototypeRequired','Apply rejected_association_prototypes.patch in an isolated baseline checkout first.');cfg=distributionRegistrationConfig();
    cfg.softPointAssociation.method="precisionMixture";
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"precisionMixture",cfg);
    cfg.softPointAssociation.temperature=1;
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"precisionMixtureT1",cfg);
end
