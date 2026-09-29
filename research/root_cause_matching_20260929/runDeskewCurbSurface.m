function runDeskewCurbSurface(phase)
% runCurbStepReplay Evaluate continuous curb-edge geometry on all raw scans.
    if nargin<1,phase=.1;end
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');
    label="deskew"+round(1000*phase)+"Surface";baseline=load(fullfile(out,"deskew"+round(1000*phase)+"_sources.mat"),'currentClouds');timing=load(fullfile(out,'point_timing.mat'));motion=odom.motion;currentClouds=cell(1170,1);sources=currentClouds;wc=localizationSourceWindowConfig();history=[];frames=[];first=0;rows=cell(1170,4);
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;twist=zeros(1,3);
        if k>1
            dt=calls.timeSeconds(k)-calls.timeSeconds(k-1);yaw=atan2(sin(motion(k,3)-motion(k-1,3)),cos(motion(k,3)-motion(k-1,3)));r=[cos(motion(k-1,3)) -sin(motion(k-1,3));sin(motion(k-1,3)) cos(motion(k-1,3))];translation=(motion(k,1:2)-motion(k-1,1:2))*r;
            V=eye(2);if abs(yaw)>1e-8,a=sin(yaw)/yaw;b=2*sin(yaw/2)^2/yaw;V=[a -b;b a];end
            twist=[(V\translation.').'/dt,yaw/dt];
        end
        raw=deskewStoredFrame(frames(k-first+1),timing.pointTimes(:,:,k),twist,phase,cfg.frameCalibration,tilt);p=perceiveFrame(raw,cfg);
        assert(isequaln(p.probabilityCloud,baseline.currentClouds{k}));
        g=p.diagnostics.ground;timer=tic;[g.moments,detail]=estimateCurbSurfaceMoments(g,p.diagnostics.groundPointContext,p.probabilityCloud.projectionRotation,p.probabilityCloud.projectionTranslation);ms=1000*toc(timer);
        cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;cc.projectionRotation=p.probabilityCloud.projectionRotation;cc.projectionTranslation=p.probabilityCloud.projectionTranslation;cc.frameCalibration=p.probabilityCloud.frameCalibration;
        currentClouds{k}=buildCoarseSemanticProbabilityCloud(g,p.diagnostics.offGround,cc);
        [sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),history,wc);
        rows(k,:)={k,nnz(g.curbCellMask),nnz(detail.accepted),ms};
        if mod(k,100)==0,fprintf('Curb surface %d/1170\n',k);end
    end
    file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','cfg','-v7.3');
    writetable(cell2table(rows,VariableNames={'frame','curbPillars','localizedSteps','extraMs'}),fullfile(dest,label+"_metrics.csv"));
    mapFile=fullfile(out,"deskew"+round(1000*phase)+"_map.mat");reg=distributionRegistrationConfig();
    rootCauseReplay(file,label,reg,mapFile);
    reg.geometric.robustLoss="switchable";reg.geometric.classGateNormalization="global";reg.geometric.boundedClasses="trafficSign";
    rootCauseReplay(file,label+"BoundedSigns",reg,mapFile);
end
