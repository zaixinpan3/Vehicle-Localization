function replayCurrentRoute()
% replayCurrentRoute Recompute current production perception and causal registration.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/line_direction_matching_20260928/sources.mat','fixed','motion');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    cfg=perceptionConfig('Mississippi');reg=distributionRegistrationConfig();wc=localizationSourceWindowConfig();
    mc=featureMapBuildConfig();loaded=load(mc.probabilityCloudPath,'cloud');s.fixed=registrationSupport.projectSemanticProbabilityCloud(loaded.cloud,2);n=height(calls);poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:n);
    store=matfile(fullfile(root,'data',mc.pointCloudMatPath));history=[];frames=[];first=0;
    state=calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(n,14);sources=cell(n,1);currentClouds=sources;maximum=[];
    for k=1:n
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(n,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        timer=tic;p=perceiveFrame(frames(k-first+1),cfg);msPerception=1000*toc(timer);current=p.probabilityCloud;
        currentClouds{k}=current;
        [source,history]=updateLocalizationSourceWindow(current,calls.timeSeconds(k),s.motion(k,:),history,wc);
        sources{k}=source;
        predicted=state;if k>1,predicted=compose(state,relative(s.motion(k-1,:),s.motion(k,:)));end
        local=selectLocalProbabilityCloud(s.fixed,predicted,reg.localMapRadius);t=tic;
        r=registerSemanticProbabilityCloud(local,source,predicted,reg);ms=toc(t)*1000;
        event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
        ref=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-ref;
        rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),ms,nnz(source.components.semanticName=="pole"),nnz(source.components.semanticName=="curb"),nnz(source.components.semanticName=="trafficSign"),msPerception};
        if k>1 && (isempty(maximum)||norm(e(1:2))>maximum.errorM)
            maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'source',source,'current',current,'predicted',predicted,'reference',ref);
        end
        if mod(k,100)==0||k==n,fprintf('Revised causal replay %d/%d\n',k,n);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','poleComponents','curbComponents','signComponents','perceptionMs'});
    writetable(replay,fullfile(dest,'full_route.csv'));
    disp(replay(maximum.frame,:));
    [~,order]=sort(replay.errorM,'descend');writetable(replay(order(1:20),:),fullfile(dest,'largest_errors.csv'));
    save('output/current_matching_20260929/replay.mat','replay','maximum','cfg','reg','wc','sources','currentClouds','-v7.3');
    summary=struct('frames',n,'maximumFrameAfterInitialization',maximum.frame,'maximumErrorM',maximum.errorM, ...
        'initializationErrorM',replay.errorM(1),'rmseM',rms(replay.errorM),'p95M',prctile(replay.errorM,95), ...
        'fullUpdates',nnz(replay.accepted),'directionalUpdates',nnz(replay.directional));
    fid=fopen(fullfile(dest,'replay_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    disp(summary);fprintf('CURRENT_ROUTE_REPLAY_COMPLETED\n');
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
