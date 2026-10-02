function report=runMncavVdbLocalization(datasetFolder,mapFile,interfaceFile,outputFolder,options)
% runMncavVdbLocalization Validate the full sensor-to-pose observer cascade.
% Truth is read only for a common initial pose and independent error scoring.
% Every scenario starts fresh matching history and uses fused observer seeds.
% Logical acquisition-time replay is not a real-time latency experiment.
    arguments
        datasetFolder (1,1) string
        mapFile (1,1) string
        interfaceFile (1,1) string
        outputFolder (1,1) string
        options.Scenarios (1,:) string=["fused","gnss_outage","lidar_only","dead_reckoning"]
        options.MaximumFrames (1,1) double=Inf
    end
    setupVehicleLocalization();maxNumCompThreads(2);
    metadata=jsondecode(fileread(fullfile(datasetFolder,'metadata.json')));
    current=jsondecode(jsonencode(mncavVdbConfig()));
    assert(metadata.completed && ~metadata.carlaPhysicsEnabled && ...
        isequaln(metadata.parameters.config,current), ...
        'VehicleLocalization:StaleVdbCapture','Require a complete capture with the current VDB configuration and disabled CARLA physics.');
    point= lateralObserverConfig();
    assert(metadata.outputPointForwardOffsetM==point.outputPoint.forwardOffsetM, ...
        'VehicleLocalization:StaleVdbCapture','Capture and observer output points differ.');
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    cache=fullfile(outputFolder,'processed_inputs.mat');
    if isfile(cache)
        a=load(cache,'inputs');inputs=a.inputs;assertLateralVehicleMatches(inputs.lateralDesign,lateralObserverConfig());
        assert(isfield(inputs,'signature') && inputs.signature==mncavVdbInputSignature(datasetFolder,interfaceFile), ...
            'VehicleLocalization:StaleVdbInputs','Input cache is stale. Regenerate processed_inputs.mat.');
    else
        inputs=prepareMncavVdbInputs(datasetFolder,interfaceFile);save(cache,'inputs','-v7.3');
    end
    poses=readtable(fullfile(datasetFolder,'poses.csv'),'TextType','string');
    frames=find(poses.lidar_stamp_sec>=3);frames=frames(1:min(numel(frames),options.MaximumFrames));
    t=poses.lidar_stamp_sec(frames);n=numel(t);ref=zeros(n,3);
    for k=1:n,ref(k,:)=poseRowToPlanarPose(poses(frames(k),:));end
    [distance,idx]=min(abs(inputs.highRate.time-t.'),[],1);assert(max(distance)<1e-7);idx=idx(:);
    h=struct();for name=string(fieldnames(inputs.highRate)).',h.(name)=inputs.highRate.(name)(idx);end
    lateral=struct('time',t,'lateralVelocity',inputs.lateral.lateralVelocity(idx), ...
        'sideSlipAngleRate',inputs.lateral.sideSlipAngleRate(idx));
    motion=integrateRecordedPlanarMotion(inputs.highRate.time, ...
        [inputs.highRate.longitudinalSpeed,inputs.lateral.lateralVelocity,inputs.highRate.yawRate],t);
    source=load(mapFile,'cloud');map=registrationSupport.projectSemanticProbabilityCloud(source.cloud,2);
    p=perceptionConfig("Carla");cal=jsondecode(fileread(fullfile(datasetFolder,'calibration.json')));
    physicalCalibration=validateLidarFrameCalibration(cal.calibration);
    p.frameCalibration=validateLidarFrameCalibration(map.frameCalibration);
    matcher=struct('perception',p,'registration',distributionRegistrationConfig(),'sourceWindow',localizationSourceWindowConfig());
    matcher.registration.heightMode="xy";
    store=matfile(fullfile(datasetFolder,'pointClouds.mat'));
    base=mncavFullObserverConfig();base.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    initial=ref(1,:)+[.5,-.4,deg2rad(2)];base.initialState=[initial(1);0;0;initial(2);0;0;initial(3)];
    s=inputs.sensorTable;history=[];timings=zeros(n,3);accepted=false(n,1);directional=accepted;
    summaries=cell(numel(options.Scenarios),1);
    for scenarioIndex=1:numel(options.Scenarios)
        scenario=options.Scenarios(scenarioIndex);history=[];timings(:)=0;accepted(:)=false;directional(:)=false;
        data=struct('highRate',h);
        if ismember(scenario,["fused","gnss_outage"])
            valid=true(n,1);if scenario=="gnss_outage",valid(t>=35 & t<65)=false;end
            data.gnss=struct('time',t,'position',[s.gnssX(idx),s.gnssY(idx)], ...
                'information',repmat(eye(2)/s.gnssVariance(1),1,1,n),'valid',valid,'delay',0);
        else
            assert(ismember(scenario,["lidar_only","dead_reckoning"]),'Unknown scenario.');
        end
        if scenario~="dead_reckoning",data.lidarMatcher=@matchFrame;end
        runTimer=tic;estimate=runFullLocalizationObserver(data,inputs.lateralDesign,base,LateralInputs=lateral);runtime=toc(runTimer);
        error=estimate.pose-ref;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
        e=hypot(error(:,1),error(:,2));steady=t>=t(1)+2;
        row=struct('scenario',scenario,'frames',n,'durationSeconds',t(end)-t(1), ...
            'positionRmseM',sqrt(mean(e.^2)),'positionP95M',prctile(e,95),'positionMaximumM',max(e), ...
            'after2sRmseM',sqrt(mean(e(steady).^2)),'after2sMaximumM',max(e(steady)), ...
            'yawRmseDeg',rad2deg(sqrt(mean(error(:,3).^2))),'finalPositionErrorM',e(end), ...
            'accepted',nnz(accepted),'directionalAccepted',nnz(directional), ...
            'medianPerceptionMs',median(timings(:,1))*1000,'medianMatchingMs',median(timings(:,2))*1000, ...
            'medianFrameMs',median(timings(:,3))*1000,'runtimeSeconds',runtime, ...
            'summedFrameProcessingSeconds',sum(timings(:,3)),'sampledSystemCertified',false);
        assert(runtime>=sum(timings(:,3)),'VehicleLocalization:InvalidRuntimeClock', ...
            'The full-run clock must include every matching callback.');
        summaries{scenarioIndex}=row;disp(row);
        trajectory=array2table([t,ref,estimate.pose,e,rad2deg(error(:,3)),accepted,directional,timings*1000], ...
            'VariableNames',{'time','refX','refY','refYaw','x','y','yaw','positionErrorM','yawErrorDeg', ...
            'accepted','directionalAccepted','perceptionMs','registrationMs','totalMs'});
        writetable(trajectory,fullfile(outputFolder,scenario+"_trajectory.csv"));
        if isfield(estimate,'matchingResults'),estimate=rmfield(estimate,'matchingResults');end
        save(fullfile(outputFolder,scenario+"_estimate.mat"),'estimate','row','base','-v7.3');
        f=fopen(fullfile(outputFolder,scenario+"_summary.json"),'w');fprintf(f,'%s\n',jsonencode(row,PrettyPrint=true));fclose(f);
    end
    report=struct('scenarios',vertcat(summaries{:}),'mapFile',mapFile,'datasetFolder',datasetFolder, ...
        'initialOffset',[.5,-.4,deg2rad(2)],'truthUsage',"Common initial pose and scoring only", ...
        'timing',"Zero-delay synchronous acquisition-time replay; measured processing time is reported separately");
    report.scanCoordinateAdapter=struct('physical',physicalCalibration,'virtual',p.frameCalibration, ...
        'description',"Virtual stored scan coordinates preserve the same body points; fixed map geometry is unchanged");
    f=fopen(fullfile(outputFolder,'summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
    function result=matchFrame(k,seed,aid)
        frameTimer=tic;cfg=matcher;cfg.perception.coarseProbabilityCloud.projectionRotation=inputs.tilt(:,:,idx(k));
        local=selectLocalProbabilityCloud(map,seed,100);frame=store.pointClouds(1,frames(k));
        frame=reframeLidarForCalibration(frame,physicalCalibration,cfg.perception.frameCalibration);
        [~,result,history]=localizeLidarFrame(frame,local,seed,t(k),cfg,history,motion(k,:),aid);
        timings(k,:)=[result.perceptionSeconds,result.registrationSeconds,toc(frameTimer)];
        accepted(k)=result.accepted;directional(k)=result.directionalAccepted;
        large=intersect(fieldnames(result),{'currentProbabilityCloud','probabilityCloud'});result=rmfield(result,large);
        if mod(k,100)==0 || k==n,fprintf('%s frame %d/%d accepted=%d\n',scenario,k,n,result.accepted);end
    end
end
