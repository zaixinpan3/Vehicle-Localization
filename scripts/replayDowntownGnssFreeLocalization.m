function report=replayDowntownGnssFreeLocalization(datasetFolder,mapFile,outputFolder,lateralDesign)
% replayDowntownGnssFreeLocalization Actual synchronous localization cascade.
% GNSS absent from initialization through the last frame. Runtime reads only
% measured clouds/motion and a frozen map; evaluation reference is a separate
% scorer. Fixed approximate local-origin prior is shared across recordings.
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    protocol=downtownReferenceReplayConfig();metadata=jsondecode(fileread(fullfile(datasetFolder,'metadata.json')));
    assert(metadata.measurementOnly && ~metadata.gnssExported && ~metadata.clock.referencePoseUsed);
    inputs=prepareDowntownMotionInputs(datasetFolder,lateralDesign);save(fullfile(outputFolder,'motion_inputs.mat'),'inputs','-v7.3');
    source=load(mapFile,'cloud');map=registrationSupport.projectSemanticProbabilityCloud(source.cloud,2);
    frames=readtable(fullfile(datasetFolder,'frames.csv'));
    valid=frames.available==1 & mod(frames.frame_index,2)==protocol.queryFrameParity & ...
        frames.native_time_sec>=inputs.nativeOriginSeconds & ...
        frames.native_time_sec<=inputs.nativeOriginSeconds+inputs.highRate.time(end)-.1002;
    ids=frames.frame_index(valid);native=frames.native_time_sec(valid);t=native-inputs.nativeOriginSeconds;n=numel(t);
    assert(n>=2 && isempty(intersect(ids,source.cloud.trainingFrameIndices)),'VehicleLocalization:EvaluationLeakage','Query acquisitions must not enter the map.');
    h=struct('time',t);
    for field=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"]
        h.(field)=interp1(inputs.highRate.time,inputs.highRate.(field),t);
    end
    lat=struct('time',t,'lateralVelocity',interp1(inputs.lateral.time,inputs.lateral.lateralVelocity,t), ...
        'sideSlipAngleRate',interp1(inputs.lateral.time,inputs.lateral.sideSlipAngleRate,t));
    motion=interp1(inputs.highRate.time,inputs.motion,t);
    cfg=protocol.observer;prior=protocol.initialPose;R=[cos(prior(3)),-sin(prior(3));sin(prior(3)),cos(prior(3))];v=R*[h.longitudinalSpeed(1);lat.lateralVelocity(1)];
    cfg.initialState=[prior(1);v(1);0;prior(2);v(2);0;prior(3)];
    Rmotion=[cos(motion(1,3)),-sin(motion(1,3));sin(motion(1,3)),cos(motion(1,3))];
    relativeXY=(motion(:,1:2)-motion(1,1:2))*Rmotion;
    deadReckoning=[prior(1:2)+relativeXY*R.',atan2(sin(prior(3)+motion(:,3)-motion(1,3)),cos(prior(3)+motion(:,3)-motion(1,3)))];
    data=struct('highRate',h,'lidarMatcher',@matchFrame);history=[];timings=zeros(n,3);accepted=false(n,1);directional=false(n,1);candidates=nan(n,3);
    p=perceptionConfig("Downtown");p.featureNames=protocol.featureNames;
    matching=struct('perception',p,'registration',protocol.registration,'sourceWindow',protocol.sourceWindow);
    timer=tic;estimate=runFullLocalizationObserver(data,lateralDesign,cfg,LateralInputs=lat);runtime=toc(timer);
    assert(estimate.diagnostics.packetCounts.gnssValid==0 && all(ismember(estimate.diagnostics.mode,[0,2])));
    trajectory=array2table([ids,native,t,estimate.pose,candidates,deadReckoning,accepted,directional,timings*1000], ...
        'VariableNames',{'frame_index','native_time_sec','elapsed_sec','x_m','y_m','yaw_rad','match_x_m','match_y_m','match_yaw_rad','dead_reckoning_x_m','dead_reckoning_y_m','dead_reckoning_yaw_rad','accepted','directional_accepted','perception_ms','registration_ms','total_ms'});
    writetable(trajectory,fullfile(outputFolder,'trajectory.csv'));
    if isfield(estimate,'matchingResults'),estimate=rmfield(estimate,'matchingResults');end
    report=struct('frames',n,'durationSeconds',t(end)-t(1),'accepted',nnz(accepted),'directionalAccepted',nnz(directional), ...
        'gnssValidPackets',estimate.diagnostics.packetCounts.gnssValid,'gnssInitializationUsed',false, ...
        'referenceUsedForRuntime',false,'initialPose',prior,'initialization',"Fixed approximate local-map-origin prior; no per-frame reference reset or global-relocalization claim", ...
        'featureNames',protocol.featureNames,'runtimeSeconds',runtime,'medianProcessingMs',median(timings(:,3))*1000, ...
        'continuousLmiVerified',estimate.diagnostics.continuousLmiVerified,'sampledSystemCertified',false, ...
        'mapFile',mapFile,'datasetFolder',datasetFolder,'trainingQueryOverlap',0,'evaluation',protocol.evaluation, ...
        'deskew',"Measured wheel/gyro/lateral planar motion; no reference trajectory",'tiltMode',inputs.tiltMode, ...
        'motionReconstruction',inputs.reconstruction,'pointAlignment',inputs.pointAlignment, ...
        'referencePoint',"front LiDAR origin; unsurveyed point transport",'informationQualifiedFrames',nnz(estimate.diagnostics.continuousInformationQualified));
    save(fullfile(outputFolder,'estimate.mat'),'estimate','cfg','report','-v7.3');
    f=fopen(fullfile(outputFolder,'summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);disp(report);
    function result=matchFrame(k,seed,aid)
        assert(~aid.valid,'VehicleLocalization:UnexpectedGnss','GNSS must remain absent.');
        timerFrame=tic;frame=loadPointCloudFrame(fullfile(datasetFolder,'pointClouds.mat'),ids(k));frame=deskewDowntownMotionFrame(frame,inputs);
        local=selectLocalProbabilityCloud(map,seed,matching.registration.localMapRadius);
        [~,result,history]=localizeLidarFrame(frame,local,seed,t(k),matching,history,motion(k,:),aid);
        timings(k,:)=[result.perceptionSeconds,result.registrationSeconds,toc(timerFrame)];accepted(k)=result.accepted;directional(k)=result.directionalAccepted;candidates(k,:)=result.poseXYTheta;
        result=rmfield(result,intersect(fieldnames(result),{'currentProbabilityCloud','probabilityCloud'}));
        if mod(k,25)==0 || k==n,fprintf('localization %s %d/%d accepted=%d\n',datasetFolder,k,n,result.accepted);end
    end
end
