function replayRawShape()
% replayRawShape Verify adopted association on every original Mississippi scan.
% Current perception must equal the previous cache; temporal clouds must equal
% an independent cached-current-cloud replay of the new association algorithm.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/source_shape_matching_20260929';
    mc=featureMapBuildConfig();mapFile=mc.probabilityCloudPath;map=load(mapFile,'cloud');cfg=distributionRegistrationConfig();wc=localizationSourceWindowConfig();
    pcfg=perceptionConfig('Mississippi');pcfg.coarseProbabilityCloud.storeDiagnostics=true;
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    original=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds');
    expected=load(fullfile(out,'shape50.mat'),'sources','windowCfg');assert(isequaln(wc,expected.windowCfg));
    cached=readtable(fullfile(dest,'shape50.csv'));state=calls{1,{'predictedX','predictedY','predictedPsi'}};
    history=[];frames=[];first=0;rows=cell(1170,13);audit=cell(1170,5);maximum=[];
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        raw=frames(k-first+1);[~,tilt]=poseRowToPlanarPose(poses(k,:));pcfg.coarseProbabilityCloud.projectionRotation=tilt;
        timer=tic;p=perceiveFrame(raw,pcfg);current=p.probabilityCloud;perceptionMs=1000*toc(timer);
        assert(isequaln(current,original.currentClouds{k}),'VehicleLocalization:PerceptionRegression','Current perception changed at frame %d.',k);
        timer=tic;[source,history,d]=updateLocalizationSourceWindow(current,calls.timeSeconds(k),motion(k,:),history,wc);windowMs=1000*toc(timer);
        assert(isequaln(source,expected.sources{k}),'VehicleLocalization:SourceRegression','Raw/cached new source mismatch at frame %d.',k);
        predicted=state;if k>1,predicted=compose(state,relative(motion(k-1,:),motion(k,:)));end
        r=matchLocalProbabilityCloud(map.cloud,source,predicted,cfg);
        event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
        reference=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-reference;
        difference=max(abs(state-cached{k,{'x','y','psi'}}));assert(difference<1e-7);
        rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),1000*r.matchingSeconds,perceptionMs,windowMs,d.shapeRejectedPairs};
        audit(k,:)={k,true,true,difference,source.components.numComponents};
        if k>1&&(isempty(maximum)||norm(e(1:2))>maximum.errorM)
            maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'source',source,'current',current,'predicted',predicted,'reference',reference);
        end
        if mod(k,100)==0,fprintf('Shape raw replay %d/1170\n',k);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','perceptionMs','windowMs','shapeRejectedPairs'});
    verification=cell2table(audit,VariableNames={'frame','identicalOriginalCurrentCloud','identicalAdoptedWindowCloud','maximumPoseDifference','components'});
    writetable(replay,fullfile(dest,'final_raw.csv'));writetable(verification,fullfile(dest,'final_raw_verification.csv'));
    save(fullfile(out,'final_raw.mat'),'replay','maximum','cfg','pcfg','wc','mapFile','verification','-v7.3');
    summary=table("final_raw",maximum.frame,maximum.errorM,rms(replay.errorM),prctile(replay.errorM,95),nnz(replay.accepted),nnz(replay.directional),median(replay.windowMs),median(replay.matchingMs),sum(replay.shapeRejectedPairs),replay.errorM(178), ...
        VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','full','directional','medianWindowMs','medianMatchingMs','shapeRejectedPairs','frame178ErrorM'});
    disp(summary);writetable(summary,fullfile(dest,'final_raw_summary.csv'));
end
function p=compose(a,b)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[a(1:2)+b(1:2)*r.' wrap(a(3)+b(3))];
end
function b=relative(a,c)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];b=[(c(1:2)-a(1:2))*r wrap(c(3)-a(3))];
end
function a=wrap(a)
    a=atan2(sin(a),cos(a));
end
