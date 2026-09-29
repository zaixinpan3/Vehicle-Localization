function runDeskewExperiment(phase)
% runDeskewExperiment Rebuild map geometry and replay raw coarse scans coherently.
% Original fine point identities are offline map annotations only. Independent
% wheel/gyro increments supply scan compensation and source-window alignment.
    if nargin<1,phase=.05;end
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';label="deskew"+round(1000*phase);
    if isfile(fullfile(dest,label+"_summary.csv")),fprintf('Completed phase %.3f already exists.\n',phase);return;end
    claimPath=fullfile(out,label+"_running.lock");claim=java.io.File(char(claimPath));
    if ~claim.createNewFile(),fprintf('Phase %.3f already running.\n',phase);return;end
    cleanup=onCleanup(@()delete(claimPath)); %#ok<NASGU>
    cfg=rootCausePerceptionConfig();mc=featureMapBuildConfig();timing=load(fullfile(out,'point_timing.mat'));
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');featureData=a.featureData;names=string(featureData.featureNames);
    identities=load(fullfile(out,'map_point_indices.mat'),'indices');refs=identities.indices.';
    poses=featureData.framePoseTable;store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;o=load('output/line_direction_matching_20260928/sources.mat','motion');motion=o.motion;
    currentClouds=cell(1170,1);sources=currentClouds;history=[];wc=localizationSourceWindowConfig();frames=[];first=0;rows=cell(1170,4);
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        [ref,tilt,height]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;twist=zeros(1,3);
        if k>1
            dt=calls.timeSeconds(k)-calls.timeSeconds(k-1);yaw=wrap(motion(k,3)-motion(k-1,3));translation=(motion(k,1:2)-motion(k-1,1:2))*rotation(motion(k-1,3));
            V=eye(2);if abs(yaw)>1e-8,a=sin(yaw)/yaw;b=2*sin(yaw/2)^2/yaw;V=[a -b;b a];end
            twist=[(V\translation.').'/dt,yaw/dt];
        end
        timer=tic;raw=deskewStoredFrame(frames(k-first+1),timing.pointTimes(:,:,k),twist,phase,cfg.frameCalibration,tilt);deskewMs=1000*toc(timer);
        timer=tic;p=perceiveFrame(raw,cfg);perceptionMs=1000*toc(timer);currentClouds{k}=p.probabilityCloud;
        [sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),motion(k,:),history,wc);
        for c=1:numel(names)
            ids=refs{k,c};xyz=double([raw.x(ids),raw.y(ids),raw.z(ids)])*p.probabilityCloud.projectionRotation.'+p.probabilityCloud.projectionTranslation;
            xyz(:,1:2)=xyz(:,1:2)*rotation(ref(3)).'+ref(1:2);xyz(:,3)=xyz(:,3)+height;
            assert(size(xyz,1)==size(featureData.pointsByFeatureFrame{c,k},1),'Frame %d class %s reference points %d map points %d',k,names(c),size(xyz,1),size(featureData.pointsByFeatureFrame{c,k},1));featureData.pointsByFeatureFrame{c,k}=xyz;
        end
        rows(k,:)={k,deskewMs,perceptionMs,currentClouds{k}.components.numComponents};
        if mod(k,100)==0,fprintf('Deskew %.3f frame %d/1170\n',phase,k);end
    end
    sourceFile=fullfile(out,label+"_sources.mat");save(sourceFile,'sources','currentClouds','phase','cfg','-v7.3');
    writetable(cell2table(rows,VariableNames={'frame','deskewMs','perceptionMs','components'}),fullfile(dest,label+"_timing.csv"));
    save(fullfile(out,label+"_features.mat"),'featureData','phase','-v7.3');
    mc.logEnabled=true;mc.temporalMap.frameCalibration=featureData.frameCalibration;map=buildSlidingWindowMap(featureData,mc);cloud=temporalMapToProbabilityCloud(map);
    cloud.motionCompensation=struct('model',"constantPlanarBodyTwist",'phaseSeconds',phase,'absolutePhaseCalibrated',false,'motionSource',"independent causal wheel/gyro increments");
    mapFile=fullfile(out,label+"_map.mat");save(mapFile,'cloud','map','phase','mc','-v7.3');
    rootCauseReplay(sourceFile,label,distributionRegistrationConfig(),mapFile);
end
function R=rotation(x)
    R=[cos(x) -sin(x);sin(x) cos(x)];
end
function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
