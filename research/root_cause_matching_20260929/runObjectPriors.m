function runObjectPriors(sourceFile,prefix)
% runObjectPriors Test landmark association priors without point-density mass.
% Map intensity mass and landmark identity likelihood answer different queries.
    setupVehicleLocalization();assert(contains(fileread(which('prepareSemanticRegistrationGeometry')),'association.semanticNames'),'VehicleLocalization:ResearchPrototypeRequired','Apply rejected_association_prototypes.patch in an isolated baseline checkout first.');out='output/root_cause_matching_20260929';
    if nargin<1,sourceFile='output/pole_boundary_recovery_20260929/replay.mat';prefix="";end
    mc=featureMapBuildConfig();a=load(mc.probabilityCloudPath,'cloud');
    for mode=1:3
        cloud=a.cloud;c=cloud.components;prior=ones(c.numComponents,1);
        if mode>=2,prior=c.repeatability;end
        classes=unique(c.semanticName);if mode==3,classes="pole";end
        for name=classes.'
            ids=c.semanticName==name & c.mixtureWeight>0;
            c.mixtureWeight(ids)=sum(c.mixtureWeight(ids))*prior(ids)/sum(prior(ids));
        end
        c.mass=c.mixtureWeight*cloud.totalMass;
        for name=unique(c.semanticName).'
            ids=c.semanticName==name;c.classMixtureWeight(ids)=c.mass(ids)/sum(c.mass(ids));
        end
        cloud.components=c;cloud.weightSemantics="registrationIdentityPriorCounterfactual";
        file=fullfile(out,"objectPrior"+mode+"_cloud.mat");save(file,'cloud');cfg=distributionRegistrationConfig();
        rootCauseReplay(sourceFile,prefix+"objectPrior"+mode,cfg,file);
        if mode>=2
            cfg.softPointAssociation.semanticNames="trafficSign";
            rootCauseReplay(sourceFile,prefix+"objectPrior"+mode+"HardPole",cfg,file);
        end
    end
end
