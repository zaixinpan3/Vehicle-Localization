function replayRawPartial()
% replayRawPartial Recompute every raw scan and run the adopted production path.
% The fixed independent motion and original initialization are unchanged.
% References score outputs only; the existing INS tilt remains a sensor input.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/partial_sign_matching_20260929';
    mc=featureMapBuildConfig();mapFile=mc.probabilityCloudPath;map=load(mapFile,'cloud');cfg=distributionRegistrationConfig();
    pcfg=perceptionConfig('Mississippi');pcfg.coarseProbabilityCloud.storeDiagnostics=true;wc=localizationSourceWindowConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    expected=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds','sources');
    cached=readtable(fullfile(dest,'production.csv'));state=calls{1,{'predictedX','predictedY','predictedPsi'}};
    history=[];frames=[];first=0;currentClouds=cell(1170,1);sources=currentClouds;rows=cell(1170,13);audit=cell(1170,5);maximum=[];
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        raw=frames(k-first+1);[~,tilt]=poseRowToPlanarPose(poses(k,:));pcfg.coarseProbabilityCloud.projectionRotation=tilt;
        timer=tic;perception=perceiveFrame(raw,pcfg);currentClouds{k}=perception.probabilityCloud;perceptionMs=1000*toc(timer);
        assert(isequaln(currentClouds{k},expected.currentClouds{k}),'VehicleLocalization:PerceptionRegression','Raw current cloud changed at frame %d.',k);
        timer=tic;[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),motion(k,:),history,wc);windowMs=1000*toc(timer);
        assert(isequaln(sources{k},expected.sources{k}),'VehicleLocalization:SourceRegression','Temporal cloud changed at frame %d.',k);
        predicted=state;if k>1,predicted=compose(state,relative(motion(k-1,:),motion(k,:)));end
        r=matchLocalProbabilityCloud(map.cloud,sources{k},predicted,cfg);
        event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
        reference=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-reference;
        difference=max(abs(state-cached{k,{'x','y','psi'}}));assert(difference<1e-7);
        rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),1000*r.matchingSeconds,perceptionMs,windowMs,r.mapViewConditioning.effectiveSupport};
        audit(k,:)={k,true,true,difference,currentClouds{k}.components.numComponents};
        if k>1&&(isempty(maximum)||norm(e(1:2))>maximum.errorM)
            maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'source',sources{k},'current',currentClouds{k},'predicted',predicted,'reference',reference);
        end
        if mod(k,100)==0,fprintf('Final production raw replay %d/1170\n',k);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','perceptionMs','windowMs','viewSupport'});
    verification=cell2table(audit,VariableNames={'frame','identicalCurrentCloud','identicalWindowCloud','maximumPoseDifference','components'});
    writetable(replay,fullfile(dest,'final_raw.csv'));writetable(verification,fullfile(dest,'final_raw_verification.csv'));
    save(fullfile(out,'final_raw.mat'),'replay','maximum','cfg','pcfg','wc','mapFile','verification','-v7.3');
    summary=table("final_raw",maximum.frame,maximum.errorM,rms(replay.errorM),prctile(replay.errorM,95),nnz(replay.accepted),nnz(replay.directional),median(replay.matchingMs), ...
        VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','full','directional','medianMs'});disp(summary);writetable(summary,fullfile(dest,'final_raw_summary.csv'));
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
