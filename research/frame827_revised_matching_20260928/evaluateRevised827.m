function evaluateRevised827()
% evaluateRevised827 Replay the causal prefix with current raw perception.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/line_direction_matching_20260928/sources.mat','fixed','motion','sources');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    cfg=perceptionConfig('Mississippi');reg=distributionRegistrationConfig();wc=localizationSourceWindowConfig();
    mc=featureMapBuildConfig();n=827;poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:n);
    store=matfile(fullfile(root,'data',mc.pointCloudMatPath));history=[];frames=[];first=0;
    state=calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(n,13);
    for k=1:n
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(n,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        current=perceiveCoarseProbabilityCloud(frames(k-first+1),cfg);
        [source,history]=updateLocalizationSourceWindow(current,calls.timeSeconds(k),s.motion(k,:),history,wc);
        predicted=state;if k>1,predicted=compose(state,relative(s.motion(k-1,:),s.motion(k,:)));end
        local=selectLocalProbabilityCloud(s.fixed,predicted,reg.localMapRadius);t=tic;
        r=registerSemanticProbabilityCloud(local,source,predicted,reg);ms=toc(t)*1000;
        event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
        ref=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-ref;
        rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),ms,nnz(source.components.semanticName=="pole"),nnz(source.components.semanticName=="curb"),nnz(source.components.semanticName=="trafficSign")};
        if mod(k,100)==0||k==n,fprintf('Revised causal replay %d/%d\n',k,n);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','poleComponents','curbComponents','signComponents'});
    writetable(replay,fullfile(dest,'causal_prefix.csv'));disp(replay(end,:));
    seed=calls{n,{'predictedX','predictedY','predictedPsi'}};local=selectLocalProbabilityCloud(s.fixed,seed,reg.localMapRadius);
    baseline=registerSemanticProbabilityCloud(local,s.sources{n},seed,reg);
    assert(max(abs(baseline.poseXYTheta-calls{n,{'x','y','psi'}}))<1e-7);
    sameSeed=registerSemanticProbabilityCloud(local,source,seed,reg);
    results={baseline,sameSeed,r};names=["previous","revised_same_seed","revised_causal"];
    controls=cell(3,8);
    for j=1:3
        a=results{j};e=a.poseXYTheta-ref;
        controls(j,:)={names(j),norm(e(1:2)),rad2deg(wrap(e(3))),a.accepted,a.reason,a.observableRank,a.pyramid.coarseRetained,a.pyramid.refinementShiftM};
    end
    comparison=cell2table(controls,VariableNames={'variant','errorM','yawErrorDeg','accepted','reason','rank','coarseRetained','refinementShiftM'});disp(comparison);
    writetable(comparison,fullfile(dest,'comparison.csv'));writetable(r.classDiagnostics,fullfile(dest,'class_diagnostics.csv'));
    save('output/frame827_revised_matching_20260928/results.mat','replay','comparison','results','source','current','predicted','ref','cfg','reg','wc');
    fprintf('REVISED_827_COMPLETED\n');
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
