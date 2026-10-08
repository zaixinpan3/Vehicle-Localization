function report=runCarlaLocalization(datasetFolder,mapFile,outputFolder,options)
% runCarlaLocalization Recursive LiDAR map matching on a prepared CARLA drive.
% Every sweep runs the online coarse perception of perceptionConfig("Carla"),
% the causal source window and geometric D2D against the fixed map through
% localizeLidarFrame, as replayMississippiLocalization does in recursive mode.
% The prediction integrates the drive's 50 Hz wheel speed and gyro yaw rate
% (prepareCarlaDataset.py) with zero lateral velocity at the kinematic
% reference point. An accepted full-pose event replaces the state; a rejected
% or directional-only event keeps the prediction. The first prediction is the
% ground-truth pose plus InitialOffset; afterwards ground truth is used only
% for scoring. Known tilt is not applied (identity projection rotation).
    arguments
        datasetFolder (1,1) string
        mapFile (1,1) string
        outputFolder (1,1) string
        options.FrameIndices (1,:) double=[]
        options.InitialOffset (1,3) double=[0.5 -0.4 deg2rad(2)]
        options.MapCropRadius (1,1) double=100
        options.PerceptionConfig (1,1) struct=struct()
    end
    setupVehicleLocalization();
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    poseTable=readtable(fullfile(datasetFolder,'poses.csv'),'TextType','string');
    frames=options.FrameIndices;if isempty(frames),frames=1:height(poseTable);end
    poses=readFramePoseTable(fullfile(datasetFolder,'poses.csv'),frames);
    n=numel(frames);reference=zeros(n,3);
    for k=1:n,reference(k,:)=poseSupport.poseRowToPlanarPose(poses(k,:));end
    scanTime=poses.lidar_stamp_sec;
    motionTable=readtable(fullfile(datasetFolder,'motion.csv'));
    twist=[motionTable.wheel_speed_mps,zeros(height(motionTable),1),motionTable.gyro_yaw_rate_radps];
    motion=integrateRecordedPlanarMotion(motionTable.time_s,twist,scanTime);
    loaded=load(mapFile,'cloud');map=registrationSupport.projectSemanticProbabilityCloud(loaded.cloud,2);
    perception=options.PerceptionConfig;
    if isempty(fieldnames(perception)),perception=perceptionConfig("Carla");end
    cfg=struct('perception',perception,'registration',distributionRegistrationConfig(), ...
        'sourceWindow',localizationSourceWindowConfig());
    cfg.registration.heightMode="xy";
    store=matfile(fullfile(datasetFolder,'pointClouds.mat'));
    state=reference(1,:)+options.InitialOffset;history=[];rows=cell(n,1);
    deadReckoning=zeros(n,3);
    for k=1:n
        deadReckoning(k,:)=compose(reference(1,:)+options.InitialOffset,motion(k,:));
        if k>1,state=compose(state,relative(motion(k-1,:),motion(k,:)));end
        predicted=state;
        frame=store.pointClouds(1,frames(k));
        timer=tic;
        local=registrationSupport.selectLocalProbabilityCloud(map,predicted,options.MapCropRadius);
        [event,result,history]=localizeLidarFrame(frame,local,predicted,scanTime(k),cfg,history,motion(k,:));
        elapsed=toc(timer);
        if ~isempty(event),state=event.pose;end
        d=state-reference(k,:);d(3)=wrap(d(3));heading=reference(k,3);
        counts=classCounts(result.currentProbabilityCloud);
        rows{k}=struct('frame',frames(k),'timeSeconds',scanTime(k)-scanTime(1), ...
            'referenceX',reference(k,1),'referenceY',reference(k,2),'referencePsi',reference(k,3), ...
            'predictedX',predicted(1),'predictedY',predicted(2),'predictedPsi',predicted(3), ...
            'x',state(1),'y',state(2),'psi',state(3),'accepted',result.accepted, ...
            'directionalAccepted',result.directionalAccepted,'reason',string(result.reason), ...
            'positionErrorM',hypot(d(1),d(2)),'alongErrorM',d(1)*cos(heading)+d(2)*sin(heading), ...
            'lateralErrorM',-d(1)*sin(heading)+d(2)*cos(heading),'yawErrorDeg',rad2deg(d(3)), ...
            'similarity',result.similarity,'rank',result.observableRank,'curbComponents',counts(1), ...
            'poleComponents',counts(2),'facadeComponents',counts(3), ...
            'windowComponents',result.probabilityCloud.components.numComponents, ...
            'totalMs',1000*elapsed,'perceptionMs',1000*result.perceptionSeconds, ...
            'registrationMs',1000*result.registrationSeconds);
        if mod(k,100)==0 || k==n
            fprintf('sweep %d/%d: accepted %d, error %.3f m, %.2f deg\n',k,n,rows{k}.accepted, ...
                rows{k}.positionErrorM,rows{k}.yawErrorDeg);
        end
    end
    calls=struct2table(vertcat(rows{:}));
    writetable(calls,fullfile(outputFolder,'calls.csv'));
    dr=hypot(deadReckoning(:,1)-reference(:,1),deadReckoning(:,2)-reference(:,2));
    a=logical(calls.accepted);
    report=struct('frames',n,'accepted',nnz(a),'directionalAccepted',nnz(calls.directionalAccepted), ...
        'acceptedPositionRmseM',sqrt(mean(calls.positionErrorM(a).^2)), ...
        'allFramePositionRmseM',sqrt(mean(calls.positionErrorM.^2)), ...
        'positionMaximumM',max(calls.positionErrorM),'lateralRmseM',sqrt(mean(calls.lateralErrorM.^2)), ...
        'alongRmseM',sqrt(mean(calls.alongErrorM.^2)),'yawRmseDeg',sqrt(mean(calls.yawErrorDeg.^2)), ...
        'deadReckoningFinalErrorM',dr(end),'deadReckoningMaximumErrorM',max(dr), ...
        'perceptionMedianMs',median(calls.perceptionMs),'totalMedianMs',median(calls.totalMs), ...
        'mapFile',mapFile,'datasetFolder',datasetFolder,'initialOffset',options.InitialOffset, ...
        'motion',"wheel speed + gyro yaw rate, zero lateral velocity at the reference point", ...
        'perceptionFeatures',cfg.perception.featureNames);
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
    writetable(array2table([frames(:),deadReckoning],'VariableNames',{'frame','x','y','psi'}), ...
        fullfile(outputFolder,'dead_reckoning.csv'));
end

function counts=classCounts(cloud)
    names=cloud.components.semanticName;
    counts=[nnz(names=="curb"),nnz(names=="pole"),nnz(names=="facade")];
end

function pose=compose(a,b)
    r=[cos(a(3)),-sin(a(3));sin(a(3)),cos(a(3))];
    pose=[a(1:2)+b(1:2)*r.',wrap(a(3)+b(3))];
end

function increment=relative(a,b)
    r=[cos(a(3)),-sin(a(3));sin(a(3)),cos(a(3))];
    increment=[(b(1:2)-a(1:2))*r,wrap(b(3)-a(3))];
end

function value=wrap(value)
    value=atan2(sin(value),cos(value));
end
