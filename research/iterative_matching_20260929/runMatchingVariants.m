function runMatchingVariants(batch)
% runMatchingVariants Causal full-route comparisons on fixed production sources.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/iterative_matching_20260929';
    data=load('output/pole_boundary_recovery_20260929/replay.mat','sources','currentClouds');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    mc=featureMapBuildConfig();map=load(mc.probabilityCloudPath,'cloud');fixed=registrationSupport.projectSemanticProbabilityCloud(map.cloud,2);
    base=baselineMatchingConfig();
    names=["baseline","merge0","merge025","merge05","merge075","merge1","trust03","trust05","fine_unlimited","source0","pole_merge_only","sign_merge_only"];
    if nargin<1,batch=1:numel(names);end
    summaries=cell(0,9);
    for variant=batch
        cfg=base;
        switch names(variant)
            case "merge0",cfg.pyramid.mapMergeRadius=0;
            case "merge025",cfg.pyramid.mapMergeRadius=.25;
            case "merge05",cfg.pyramid.mapMergeRadius=.5;
            case "merge075",cfg.pyramid.mapMergeRadius=.75;
            case "merge1",cfg.pyramid.mapMergeRadius=1;
            case "trust03",cfg.pyramid.trustRadius=.3;
            case "trust05",cfg.pyramid.trustRadius=.5;
            case "fine_unlimited",cfg.pyramid.trustRadius=1e6;
            case "source0",cfg.pyramid.sourceMergeRadius=0;
            case "pole_merge_only",cfg.pyramid.pointClasses="pole";
            case "sign_merge_only",cfg.pyramid.pointClasses="trafficSign";
        end
        n=height(calls);state=calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(n,10);maximum=[];timer=tic;
        for k=1:n
            predicted=state;if k>1,predicted=compose(state,relative(motion(k-1,:),motion(k,:)));end
            local=selectLocalProbabilityCloud(fixed,predicted,cfg.localMapRadius);t=tic;
            r=registerSemanticProbabilityCloud(local,data.sources{k},predicted,cfg);ms=1000*toc(t);
            event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
            ref=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-ref;
            rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),ms};
            if k>1 && (isempty(maximum)||norm(e(1:2))>maximum.errorM)
                maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'source',data.sources{k},'current',data.currentClouds{k},'predicted',predicted,'reference',ref);
            end
        end
        replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs'});
        label=names(variant);writetable(replay,fullfile(dest,label+".csv"));save(fullfile(out,label+".mat"),'replay','maximum','cfg','-v7.3');
        summaries(end+1,:)={label,maximum.frame,maximum.errorM,rms(replay.errorM),prctile(replay.errorM,95),nnz(replay.accepted),nnz(replay.directional),median(replay.matchingMs),toc(timer)}; %#ok<AGROW>
        summary=cell2table(summaries,VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','full','directional','medianMs','elapsedSeconds'});
        writetable(summary,fullfile(dest,"screen_"+batch(1)+".csv"));disp(summary(end,:));
    end
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
