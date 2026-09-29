function finishRecovery()
% finishRecovery Independently compare full replay and class moments.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    a=load('output/current_matching_20260929/replay.mat');b=load('output/pole_boundary_recovery_20260929/replay.mat');
    changed=[];
    for k=1:numel(a.currentClouds)
        u=a.currentClouds{k}.components;v=b.currentClouds{k}.components;
        if ~isequaln(u,v),changed(end+1)=k;end %#ok<AGROW>
        for name=["curb","trafficSign"]
            i=u.semanticName==name;j=v.semanticName==name;
            for field=["cellLinIdx","cellSub","count","mean","meanXYZ","semanticProbability","occupancyProbability","unnormalizedWeight"]
                assert(isequaln(u.(field)(i,:),v.(field)(j,:)));
            end
            for field=["covariance","covarianceXYZ","invCovariance"]
                assert(isequaln(u.(field)(:,:,i),v.(field)(:,:,j)));
            end
        end
    end
    assert(isequal(changed,[685 820 856]));
    prior=load('output/line_direction_matching_20260928/production/report.mat','report');calls=prior.report.calls;
    errors=vecnorm(b.replay{:,{'x','y'}}-calls{:,{'referenceX','referenceY'}},2,2);
    assert(max(abs(errors-b.replay.errorM))<1e-10);
    s=load('output/line_direction_matching_20260928/sources.mat','motion');k=856;
    before=b.replay{k-1,{'x','y','psi'}};motion0=s.motion(k-1,:);motion1=s.motion(k,:);
    R=@(psi)[cos(psi) -sin(psi);sin(psi) cos(psi)];
    relative=(motion1(1:2)-motion0(1:2))*R(motion0(3));
    predicted=[before(1:2)+relative*R(before(3)).',before(3)+motion1(3)-motion0(3)];
    mc=featureMapBuildConfig();map=load(mc.probabilityCloudPath,'cloud');fixed=registrationSupport.projectSemanticProbabilityCloud(map.cloud,2);
    local=selectLocalProbabilityCloud(fixed,predicted,b.reg.localMapRadius);
    result=registerSemanticProbabilityCloud(local,b.sources{k},predicted,b.reg);
    event=registrationSupport.registrationPoseMeasurement(result,calls.timeSeconds(k));assert(~isempty(event));
    assert(max(abs(event.pose-b.replay{k,{'x','y','psi'}}))<1e-9);
    target=struct('frame',k,'previousErrorM',a.replay.errorM(k),'currentErrorM',b.replay.errorM(k), ...
        'sourcePoles',nnz(b.sources{k}.components.semanticName=="pole"), ...
        'matchedPoles',nnz(result.correspondences.semanticName=="pole"), ...
        'accepted',result.accepted,'reason',result.reason,'fineShiftM',result.pyramid.refinementShiftM);
    validation=struct('rawCloudsChangedFrames',changed,'allNonPoleMeansCovariancesEvidenceUnchanged',true, ...
        'independentErrorCheck',true,'frame856',target);
    fid=fopen(fullfile(dest,'validation.json'),'w');fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));fclose(fid);disp(validation);disp(target);
    save('output/pole_boundary_recovery_20260929/frame856_matching.mat','result','predicted','target');
end
