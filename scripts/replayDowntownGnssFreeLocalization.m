function report=replayDowntownGnssFreeLocalization(datasetFolder,mapFile,outputFolder,lateralDesign,options)
% replayDowntownGnssFreeLocalization Actual synchronous localization cascade.
% GNSS absent from initialization through the last frame. Runtime reads only
% measured clouds/motion and a frozen map; evaluation reference is a separate
% scorer. One externally supplied reference pose initializes the first query.
% No reference trajectory is read here, including when initializing velocity.
% The current LiDAR channel is the
% correspondence-free overlap gradient (overlapGradientConfig) against the same
% projected map; its source horizons are the matcher's (same deskew,
% perception, source window and odometry), built causally before the run.
    arguments
        datasetFolder
        mapFile
        outputFolder
        lateralDesign
        options.InitialPosePacket (1,1) struct=struct()
        options.LidarChannel (1,1) string="overlapGradient"
    end
    setupVehicleLocalization();options.LidarChannel=activeLocalizationChannel(options.LidarChannel);
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    protocol=downtownReferenceReplayConfig();metadata=jsondecode(fileread(fullfile(datasetFolder,'metadata.json')));
    assert(metadata.measurementOnly && ~metadata.gnssExported && ~metadata.clock.referencePoseUsed);
    inputs=prepareDowntownMotionInputs(datasetFolder,lateralDesign);save(fullfile(outputFolder,'motion_inputs.mat'),'inputs','-v7.3');
    source=load(mapFile,'cloud');map=registrationSupport.projectSemanticProbabilityCloud(source.cloud,2);
    [ids,native,t]=downtownReplayFrameSchedule(datasetFolder,inputs);n=numel(t);
    assert(n>=2 && isempty(intersect(ids,source.cloud.trainingFrameIndices)),'VehicleLocalization:EvaluationLeakage','Query acquisitions must not enter the map.');
    h=struct('time',t);
    for field=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"]
        h.(field)=interp1(inputs.highRate.time,inputs.highRate.(field),t);
    end
    lat=struct('time',t,'lateralVelocity',interp1(inputs.lateral.time,inputs.lateral.lateralVelocity,t), ...
        'sideSlipAngle',interp1(inputs.lateral.time,inputs.lateral.sideSlipAngle,t), ...
        'sideSlipAngleRate',interp1(inputs.lateral.time,inputs.lateral.sideSlipAngleRate,t), ...
        'velocityValid',interp1(inputs.lateral.time,double(inputs.lateral.velocityValid),t)==1);
    motion=interp1(inputs.highRate.time,inputs.motion,t);
    motionSegment=recordedMotionSegmentAt(inputs.highRate.time,inputs.motionSegment,t);
    cfg=protocol.observer;prior=validateDowntownInitialPose(options.InitialPosePacket,ids(1),native(1));
    R=[cos(prior(3)),-sin(prior(3));sin(prior(3)),cos(prior(3))];v=R*[h.longitudinalSpeed(1);lat.lateralVelocity(1)];
    cfg.initialState=[prior(1);v(1);0;prior(2);v(2);0;prior(3)];
    Rmotion=[cos(motion(1,3)),-sin(motion(1,3));sin(motion(1,3)),cos(motion(1,3))];
    relativeXY=(motion(:,1:2)-motion(1,1:2))*Rmotion;
    deadReckoning=[prior(1:2)+relativeXY*R.',atan2(sin(prior(3)+motion(:,3)-motion(1,3)),cos(prior(3)+motion(:,3)-motion(1,3)))];
    data=struct('highRate',h);history=[];timings=zeros(n,3);accepted=false(n,1);directional=false(n,1);candidates=nan(n,3);
    p=perceptionConfig("Downtown");p.featureNames=protocol.featureNames;
    matching=struct('perception',p,'sourceWindow',protocol.sourceWindow);
    sourcePreparationSeconds=0;
    sources=cell(n,1);deskewApplied=false(n,1);
    for k=1:n
        timerFrame=tic;frame=loadPointCloudFrame(fullfile(datasetFolder,'pointClouds.mat'),ids(k));[frame,deskewApplied(k)]=deskewDowntownMotionFrame(frame,inputs);
        current=perceiveCoarseProbabilityCloud(frame,matching.perception);
        [sources{k},history]=updateLocalizationSourceWindow(current,t(k),motion(k,:),history,matching.sourceWindow,motionSegment(k));
        timings(k,1)=toc(timerFrame);
        if mod(k,25)==0 || k==n,fprintf('overlap sources %s %d/%d\n',datasetFolder,k,n);end
    end
    sourcePreparationSeconds=sum(timings(:,1));
    data.lidarOverlap=struct('map',map,'sources',{sources},'valid',true(n,1),'config',overlapGradientConfig());
    [~,cfg]=synthesizeMncavInvariantDesign(cfg);
    timer=tic;estimate=runFullLocalizationObserver(data,lateralDesign,cfg,LateralInputs=lat);runtime=toc(timer);
    accepted=estimate.diagnostics.lidarOverlap.available;overlapSeconds=estimate.diagnostics.lidarOverlap.seconds;overlapSeconds(~isfinite(overlapSeconds))=0;
    timings(:,2)=overlapSeconds;timings(:,3)=timings(:,1)+overlapSeconds;
    assert(estimate.diagnostics.packetCounts.gnssValid==0 && all(ismember(estimate.diagnostics.mode,[0,2])));
    estimate.diagnostics.referenceUsed=true; % One initialization pose, separately audited.
    estimate.diagnostics.referenceUsedForInitialization=true;
    estimate.diagnostics.referenceInitializationCount=1;
    estimate.diagnostics.referenceInitializationFrameIndex=ids(1);
    estimate.diagnostics.referenceUsedAfterInitialization=false;
    trajectory=array2table([ids,native,t,estimate.pose,candidates,deadReckoning,accepted,directional,timings*1000], ...
        'VariableNames',{'frame_index','native_time_sec','elapsed_sec','x_m','y_m','yaw_rad','match_x_m','match_y_m','match_yaw_rad','dead_reckoning_x_m','dead_reckoning_y_m','dead_reckoning_yaw_rad','accepted','directional_accepted','perception_ms','registration_ms','total_ms'});
    writetable(trajectory,fullfile(outputFolder,'trajectory.csv'));
    if isfield(estimate,'matchingResults'),estimate=rmfield(estimate,'matchingResults');end
    report=struct('frames',n,'lidarChannel',options.LidarChannel,'sourcePreparationSeconds',sourcePreparationSeconds, ...
        'durationSeconds',t(end)-t(1),'accepted',nnz(accepted),'directionalAccepted',nnz(directional), ...
        'gnssValidPackets',estimate.diagnostics.packetCounts.gnssValid,'gnssInitializationUsed',false, ...
        'referenceUsedForInitialization',true,'referenceInitializationCount',1,'referenceUsedAfterInitialization',false, ...
        'initialPose',prior,'initialPosePacket',options.InitialPosePacket,'initializationMode',protocol.initializationMode, ...
        'initialization',"Exact first-query pseudo-reference pose once; velocities measured; no later reference or GNSS input", ...
        'featureNames',protocol.featureNames,'runtimeSeconds',runtime,'medianProcessingMs',median(timings(:,3))*1000, ...
        'continuousLmiVerified',estimate.diagnostics.continuousLmiVerified,'sampledSystemCertified',false, ...
        'mapFile',mapFile,'datasetFolder',datasetFolder,'trainingQueryOverlap',0,'evaluation',protocol.evaluation, ...
        'deskew',"Measured wheel/gyro/lateral planar motion within valid segments; otherwise acquired scan unchanged", ...
        'deskewAppliedFrames',nnz(deskewApplied),'currentScanOnlyFrames',nnz(motionSegment==0),'tiltMode',inputs.tiltMode, ...
        'motionReconstruction',inputs.reconstruction,'pointAlignment',inputs.pointAlignment, ...
        'referencePoint',"front LiDAR origin; unsurveyed point transport",'informationQualifiedFrames',nnz(estimate.diagnostics.continuousInformationQualified));
    save(fullfile(outputFolder,'estimate.mat'),'estimate','cfg','report','-v7.3');
    f=fopen(fullfile(outputFolder,'summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);disp(report);
end
