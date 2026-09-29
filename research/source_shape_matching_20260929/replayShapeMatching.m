function summary=replayShapeMatching(label,windowCfg,matchingCfg)
% replayShapeMatching Rebuild causal tracks and recursively match every scan.
% Original current clouds, independent motion and initialization are frozen.
% Reference XY/yaw score outputs only; no query labels enter association.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/source_shape_matching_20260929';
    if nargin<2,windowCfg=localizationSourceWindowConfig();end
    if nargin<3,matchingCfg=distributionRegistrationConfig();end
    data=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds','sources');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    mc=featureMapBuildConfig();mapFile=mc.probabilityCloudPath;map=load(mapFile,'cloud');
    baseline=readtable('research/partial_sign_matching_20260929/final_raw.csv');
    history=[];state=calls{1,{'predictedX','predictedY','predictedPsi'}};n=height(calls);sources=cell(n,1);
    rows=cell(n,16);maximum=[];audit=cell(n,7);
    for k=1:n
        timer=tic;[sources{k},history,d]=updateLocalizationSourceWindow(data.currentClouds{k},calls.timeSeconds(k),motion(k,:),history,windowCfg);windowMs=1000*toc(timer);
        predicted=state;if k>1,predicted=compose(state,relative(motion(k-1,:),motion(k,:)));end
        r=matchLocalProbabilityCloud(map.cloud,sources{k},predicted,matchingCfg);
        event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
        reference=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-reference;
        names=sources{k}.components.semanticName;original=data.sources{k}.components.semanticName;
        identical=isequaln(sources{k},data.sources{k});
        if isinf(windowCfg.maximumShapeDistance),assert(identical);assert(max(abs(state-baseline{k,{'x','y','psi'}}))<1e-7);end
        rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),1000*r.matchingSeconds,windowMs, ...
            numel(names),nnz(names=="pole"),nnz(names=="trafficSign"),nnz(names=="curb"),d.shapeRejectedPairs};
        audit(k,:)={k,identical,numel(original),numel(names),d.positionCompatiblePairs,d.shapeRejectedPairs,d.trackCount};
        if k>1&&(isempty(maximum)||norm(e(1:2))>maximum.errorM)
            maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'source',sources{k},'current',data.currentClouds{k},'predicted',predicted,'reference',reference);
        end
        if mod(k,200)==0,fprintf('%s shape replay %d/%d\n',label,k,n);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','windowMs','components','poles','signs','curbs','shapeRejectedPairs'});
    audit=cell2table(audit,VariableNames={'frame','identicalBaselineSources','baselineComponents','components','positionCompatiblePairs','shapeRejectedPairs','tracks'});
    writetable(replay,fullfile(dest,label+".csv"));writetable(audit,fullfile(dest,label+"_audit.csv"));
    save(fullfile(out,label+".mat"),'replay','audit','maximum','sources','windowCfg','matchingCfg','mapFile','-v7.3');
    summary=table(string(label),maximum.frame,maximum.errorM,rms(replay.errorM),prctile(replay.errorM,95),nnz(replay.accepted),nnz(replay.directional),median(replay.windowMs),median(replay.matchingMs),sum(replay.shapeRejectedPairs),sum(replay.components),replay.errorM(178), ...
        VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','full','directional','medianWindowMs','medianMatchingMs','shapeRejectedPairs','totalComponents','frame178ErrorM'});
    disp(summary);writetable(summary,fullfile(dest,label+"_summary.csv"));
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
