function replaySpecialist()
% replaySpecialist Validate raw pole selections and recursive route matching.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/line_direction_matching_20260928/sources.mat','fixed','motion');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    cfg=perceptionConfig('Mississippi');reg=distributionRegistrationConfig();wc=localizationSourceWindowConfig();
    model=loadPillarPoleModel('output/pole_selective_recovery_20260928/model.json');
    cfg.offGroundFeatures.pole.distributionValidation.modelFile=fullfile(root,'output/pole_selective_recovery_20260928/model.json');
    cfg.offGroundFeatures.pole.distributionValidation.minimumScore=model.decisionThreshold;
    addpath('research/pillar_fine_alignment_20260926');
    frozen=load('output/pole_precision_20260927/mississippi.mat','frames','references');
    prediction=readtable(fullfile(dest,'specialist_expected.csv'));
    mc=featureMapBuildConfig();n=height(calls);poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:n);
    store=matfile(fullfile(root,'data',mc.pointCloudMatPath));history=[];frames=[];first=0;
    state=calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(n,13);metrics=cell(n,1);sources=cell(n,1);currentClouds=sources;maximum=[];
    for k=1:n
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(n,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        timer=tic;p=perceiveFrame(frames(k-first+1),cfg);msPerception=1000*toc(timer);current=p.probabilityCloud;
        ids=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
        expected=prediction.pillar(prediction.frame==k & prediction.score>=model.decisionThreshold);
        assert(isequal(sort(ids(:)),sort(expected(:))),'Raw selection mismatch frame %d',k);
        m=measureFinePoleAlignment(frames(k-first+1),frozen.references{frozen.frames==k}.pointIndices,ids,p.candidates.geometry);
        m.frame=k;m.perceptionMs=msPerception;metrics{k}=m;currentClouds{k}=current;
        [source,history]=updateLocalizationSourceWindow(current,calls.timeSeconds(k),s.motion(k,:),history,wc);
        sources{k}=source;
        predicted=state;if k>1,predicted=compose(state,relative(s.motion(k-1,:),s.motion(k,:)));end
        local=selectLocalProbabilityCloud(s.fixed,predicted,reg.localMapRadius);t=tic;
        r=registerSemanticProbabilityCloud(local,source,predicted,reg);ms=toc(t)*1000;
        event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
        ref=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-ref;
        rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),ms,nnz(source.components.semanticName=="pole"),nnz(source.components.semanticName=="curb"),nnz(source.components.semanticName=="trafficSign")};
        if k>1 && (isempty(maximum)||norm(e(1:2))>maximum.errorM)
            maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'source',source,'current',current,'predicted',predicted,'reference',ref);
        end
        if mod(k,100)==0||k==n,fprintf('Revised causal replay %d/%d\n',k,n);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','poleComponents','curbComponents','signComponents'});
    writetable(replay,fullfile(dest,'full_route.csv'));
    metrics=struct2table(vertcat(metrics{:}));writetable(metrics,fullfile(dest,'raw_perception.csv'));
    disp(replay(857,:));disp(maximum.frame);disp(maximum.errorM);
    [~,order]=sort(replay.errorM,'descend');writetable(replay(order(1:20),:),fullfile(dest,'largest_errors.csv'));
    save('output/pole_selective_recovery_20260928/replay.mat','replay','metrics','maximum','cfg','reg','wc','sources','currentClouds','-v7.3');
    fprintf('SPECIALIST_REPLAY_COMPLETED\n');
end
function p=compose(a,b)
    R=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[a(1:2)+b(1:2)*R.' wrap(a(3)+b(3))];
end
function b=relative(a,c)
    R=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];b=[(c(1:2)-a(1:2))*R wrap(c(3)-a(3))];
end
function a=wrap(a)
    a=atan2(sin(a),cos(a));
end
