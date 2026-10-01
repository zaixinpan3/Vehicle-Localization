function inspectFrames()
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    a=load('output/source_shape_matching_20260929/shape50.mat','sources');
    map=load(featureMapBuildConfig().probabilityCloudPath,'cloud');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');
    baseline=readtable('research/source_shape_matching_20260929/final_raw.csv');
    new=readtable(fullfile(dest,'full_covariance.csv'));cfg=anisotropicRegistrationConfig();
    for k=[178 894 932]
        seed=baseline{k,{'x','y','psi'}};
        r=matchLocalProbabilityCloud(map.cloud,a.sources{k},seed,cfg);
        ref=old.report.calls{k,{'referenceX','referenceY','referencePsi'}};
        conditioned=conditionSemanticMapOnView(selectLocalProbabilityCloud(map.cloud,seed,cfg.localMapRadius),seed);
        model=prepareSemanticRegistrationGeometry(conditioned,a.sources{k},seed,cfg);
        sr=model.linearize([ref(1:2)-seed(1:2),ref(3)],ones(3,1));
        sf=model.linearize([r.poseXYTheta(1:2)-seed(1:2),r.poseXYTheta(3)],ones(3,1));
        fprintf('Frame %d baseline %.9f recursive %.9f isolated %.9f costRef %.9f costFit %.9f\n',k,baseline.errorM(k),new.errorM(k),norm(r.poseXYTheta(1:2)-ref(1:2)),sr.cost,sf.cost);
        disp(r.classDiagnostics);
        pairs=r.correspondences;mov=a.sources{k}.components.covariance;
        pairs.sourceCovarianceXY=[reshape(mov(1,1,pairs.source),[],1),reshape(mov(1,2,pairs.source),[],1),reshape(mov(2,2,pairs.source),[],1)];
        writetable(pairs,fullfile(dest,sprintf('initial_frame%d_pairs.csv',k)));
    end
end
