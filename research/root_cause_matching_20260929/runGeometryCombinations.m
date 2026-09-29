function runGeometryCombinations()
% runGeometryCombinations Separate curb localization from map association.
    setupVehicleLocalization();assert(contains(fileread(which('softPointAssociationTarget')),'precisionMixture') && contains(fileread(which('prepareSemanticRegistrationGeometry')),'surfaceEligible'),'VehicleLocalization:ResearchPrototypeRequired','Apply rejected_association_prototypes.patch in an isolated baseline checkout first.');out='output/root_cause_matching_20260929';
    for mode=2:4
        file=fullfile(out,"curbStep"+mode+"_sources.mat");cfg=distributionRegistrationConfig();
        cfg.softPointAssociation.method="precisionMixture";
        rootCauseReplay(file,"curb"+mode+"Precision",cfg);
    end
    file=fullfile(out,'curbStep2_sources.mat');cfg=distributionRegistrationConfig();
    cfg.pointSurfaceClasses="trafficSign";rootCauseReplay(file,"curb2Surface",cfg);
    cfg=distributionRegistrationConfig();rootCauseReplay(file,"curb2PointMap",cfg,fullfile(out,'pointObjectMap_cloud.mat'));
end
