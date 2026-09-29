function runHeightAssociation()
% runHeightAssociation Test whether nearby map modes are separated by height.
    setupVehicleLocalization();cfg=distributionRegistrationConfig();cfg.relativeHeight.enabled=true;
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"relativeHeightMarginal",cfg);
    cfg.relativeHeight.candidateModel="conditional";
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"relativeHeightConditional",cfg);
end
