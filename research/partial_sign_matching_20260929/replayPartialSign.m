function summary=replayPartialSign(sourceFile,label,cfg,mapFile)
% replayPartialSign Replay an explicitly frozen source set without pose resets.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/partial_sign_matching_20260929';
    data=load(sourceFile,'sources','currentClouds');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    if nargin<4,mapFile='output/mississippi_mapping_calibrated/probability_cloud.mat';end;assert(string(mapFile)~=string(fullfile(out,label+".mat")),'VehicleLocalization:ExperimentOutputCollision','Map and replay outputs must use distinct paths.');map=load(mapFile,'cloud');fixed=registrationSupport.projectSemanticProbabilityCloud(map.cloud,2);
    n=height(calls);state=calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(n,10);maximum=[];timer=tic;
    for k=1:n
    predicted=state;if k>1,predicted=compose(state,relative(motion(k-1,:),motion(k,:)));end
    queryMap=fixed;if isfield(map.cloud,'curbViews'),queryMap.curbViews=map.cloud.curbViews;queryMap=conditionCurbPrototype(queryMap,predicted,cfg.curbPrototype.bandwidth,cfg.curbPrototype.mode);end
    local=selectLocalProbabilityCloud(queryMap,predicted,cfg.localMapRadius);t=tic;
    r=registerSemanticProbabilityCloud(local,data.sources{k},predicted,cfg);ms=1000*toc(t);
    event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
    ref=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-ref;
    rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),ms};
    if k>1 && (isempty(maximum)||norm(e(1:2))>maximum.errorM)
    maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'source',data.sources{k},'current',data.currentClouds{k},'predicted',predicted,'reference',ref);
    end
    if mod(k,200)==0,fprintf('%s: %d/1170\n',label,k);end
    end
    replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs'});
    writetable(replay,fullfile(dest,label+".csv"));save(fullfile(out,label+".mat"),'replay','maximum','cfg','mapFile','-v7.3');
    summary=table(string(label),maximum.frame,maximum.errorM,rms(replay.errorM),prctile(replay.errorM,95),nnz(replay.accepted),nnz(replay.directional),median(replay.matchingMs),toc(timer), ...
        VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','full','directional','medianMs','elapsedSeconds'});
    writetable(summary,fullfile(dest,label+"_summary.csv"));disp(summary);
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
