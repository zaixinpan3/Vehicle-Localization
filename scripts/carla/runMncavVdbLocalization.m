function report=runMncavVdbLocalization(datasetFolder,mapFile,interfaceFile,outputFolder,options)
% runMncavVdbLocalization Run a measurement-only sensor-to-pose cascade.
% No reference pose, true velocity or semantic label is read by this runtime.
% Use scoreMncavVdbLocalization in a separate step after saving estimates.
% Every scenario starts fresh matching history and uses fused observer seeds.
% Logical acquisition-time replay is not a real-time latency experiment.
% Prepare the dataset with prepareMncavVdbMeasurements.py followed by
% buildMncavVdbMeasurementClouds. The default t=8 s start follows stationary
% IMU alignment and a three-second measured GNSS course bootstrap. Scenarios
% named lidar_only/dead_reckoning withdraw GNSS only after that bootstrap.
% gnss_only keeps GNSS and withdraws LiDAR: the same sensors as a GNSS and
% vehicle-motion observer, for algorithm-level comparisons.
% The current LiDAR channel is the
% correspondence-free overlap gradient (overlapGradientConfig) against the same
% projected map. Its source horizons are the matcher's: the same perception,
% tilt, frame adapter, source window and odometry, built causally once before
% the scenarios, because they do not depend on the pose estimate.
    arguments
        datasetFolder (1,1) string
        mapFile (1,1) string
        interfaceFile (1,1) string
        outputFolder (1,1) string
        options.Scenarios (1,:) string=["fused","gnss_outage","lidar_only","dead_reckoning"]
        options.MaximumFrames (1,1) double=Inf
        options.StartTime (1,1) double=8
        options.LidarChannel (1,1) string="overlapGradient"
    end
    setupVehicleLocalization();options.LidarChannel=activeLocalizationChannel(options.LidarChannel);maxNumCompThreads(2);
    metadata=jsondecode(fileread(fullfile(datasetFolder,'metadata.json')));
    assert(isfield(metadata,'schemaVersion') && metadata.schemaVersion==2 && metadata.sensorOnly, ...
        'VehicleLocalization:MeasurementOnlyRequired','Prepare a measurement-only dataset before localization.');
    % Captures made before the envelope option record no envelope; they are nominal.
    envelope="nominal";recordedEnvelope=isfield(metadata.parameters.config.simulation,'envelope');
    if recordedEnvelope,envelope=string(metadata.parameters.config.simulation.envelope);end
    current=jsondecode(jsonencode(mncavVdbConfig(Envelope=envelope)));
    if ~recordedEnvelope,current.simulation=rmfield(current.simulation,'envelope');end
    assert(metadata.completed && ~metadata.carlaPhysicsEnabled && ...
        isequaln(metadata.parameters.config,current), ...
        'VehicleLocalization:StaleVdbCapture','Require a complete capture with the current VDB configuration and disabled CARLA physics.');
    point= lateralObserverConfig();
    assert(metadata.outputPointForwardOffsetM==point.outputPoint.forwardOffsetM, ...
        'VehicleLocalization:StaleVdbCapture','Capture and observer output points differ.');
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    cache=fullfile(outputFolder,'processed_inputs.mat');
    if isfile(cache)
        a=load(cache,'inputs');inputs=a.inputs;lateralObserverSupport.assertLateralVehicleMatches(inputs.lateralDesign,lateralObserverConfig());
        assert(isfield(inputs,'signature') && inputs.signature==mncavVdbInputSignature(datasetFolder,interfaceFile), ...
            'VehicleLocalization:StaleVdbInputs','Input cache is stale. Regenerate processed_inputs.mat.');
    else
        inputs=prepareMncavVdbInputs(datasetFolder,interfaceFile);save(cache,'inputs','-v7.3');
    end
    frameTable=readtable(fullfile(datasetFolder,'frames.csv'));
    assert(isequal(frameTable.Properties.VariableNames,{'frame_index','lidar_stamp_sec','points'}), ...
        'VehicleLocalization:InvalidMeasurementFrames','Only acquisition metadata belongs in the frame table.');
    frames=find(frameTable.lidar_stamp_sec>=options.StartTime);
    frames=frames(1:min(numel(frames),options.MaximumFrames));
    t=frameTable.lidar_stamp_sec(frames);n=numel(t);
    [distance,idx]=min(abs(inputs.highRate.time-t.'),[],1);assert(max(distance)<1e-7);idx=idx(:);
    h=struct();for name=string(fieldnames(inputs.highRate)).',h.(name)=inputs.highRate.(name)(idx);end
    lateral=struct('time',t,'lateralVelocity',inputs.lateral.lateralVelocity(idx), ...
        'sideSlipAngle',inputs.lateral.sideSlipAngle(idx), ...
        'sideSlipAngleRate',inputs.lateral.sideSlipAngleRate(idx), ...
        'velocityValid',inputs.lateral.velocityValid(idx));
    motion=integrateRecordedPlanarMotion(inputs.highRate.time, ...
        [inputs.highRate.longitudinalSpeed,inputs.lateral.lateralVelocity,inputs.highRate.yawRate],t);
    source=load(mapFile,'cloud');map=registrationSupport.projectSemanticProbabilityCloud(source.cloud,2);
    p=perceptionConfig("Carla");cal=jsondecode(fileread(fullfile(datasetFolder,'calibration.json')));
    physicalCalibration=validateLidarFrameCalibration(cal.calibration);
    p.frameCalibration=validateLidarFrameCalibration(map.frameCalibration);
    sourceConfig=struct('perception',p,'sourceWindow',localizationSourceWindowConfig());
    store=matfile(fullfile(datasetFolder,'pointClouds.mat'));
    base=mncavFullObserverConfig();base.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    [~,base]=synthesizeMncavInvariantDesign(base);
    initial=initializeMncavVdbFromGnss(inputs.sensorTable,inputs,t(1));base.initialState=initial.state;
    s=inputs.sensorTable;timings=zeros(n,3);accepted=false(n,1);directional=accepted;
    summaries=cell(numel(options.Scenarios),1);
    sources={};sourceTimings=zeros(n,1);
    if any(~ismember(options.Scenarios,["dead_reckoning","gnss_only"])),[sources,sourceTimings]=buildSources();end
    for scenarioIndex=1:numel(options.Scenarios)
        scenario=options.Scenarios(scenarioIndex);timings(:)=0;accepted(:)=false;directional(:)=false;
        data=struct('highRate',h);
        if ismember(scenario,["fused","gnss_outage","gnss_only"])
            valid=logical(s.gnssValid(idx));if scenario=="gnss_outage",valid(t>=35 & t<65)=false;end
            data.gnss=struct('time',t,'position',[s.gnssX(idx),s.gnssY(idx)], ...
                'information',eye(2).*reshape(1./s.gnssVariance(idx),1,1,n),'valid',valid,'delay',0);
        else
            assert(ismember(scenario,["lidar_only","dead_reckoning"]),'Unknown scenario.');
        end
        if ~ismember(scenario,["dead_reckoning","gnss_only"])
            data.lidarOverlap=struct('map',map,'sources',{sources},'valid',true(n,1),'config',overlapGradientConfig());
        end
        runTimer=tic;estimate=runFullLocalizationObserver(data,inputs.lateralDesign,base,LateralInputs=lateral);runtime=toc(runTimer);
        if ~ismember(scenario,["dead_reckoning","gnss_only"])
            accepted=estimate.diagnostics.lidarOverlap.available;directional(:)=false;
            timings=[sourceTimings,estimate.diagnostics.lidarOverlap.seconds,sourceTimings+estimate.diagnostics.lidarOverlap.seconds];
            timings(~isfinite(timings))=0;
        end
        row=struct('scenario',scenario,'lidarChannel',options.LidarChannel,'frames',n,'durationSeconds',t(end)-t(1), ...
            'referenceUsed',false, ...
            'accepted',nnz(accepted),'directionalAccepted',nnz(directional), ...
            'medianPerceptionMs',median(timings(:,1))*1000,'medianMatchingMs',median(timings(:,2))*1000, ...
            'medianFrameMs',median(timings(:,3))*1000,'runtimeSeconds',runtime, ...
            'summedFrameProcessingSeconds',sum(timings(:,3)),'sampledSystemCertified',false);
        row.sourcePreparationSeconds=sum(sourceTimings);
        summaries{scenarioIndex}=row;disp(row);
        trajectory=array2table([frames,t,estimate.pose,accepted,directional,timings*1000], ...
            'VariableNames',{'frame','time','x','y','yaw','accepted','directionalAccepted', ...
            'perceptionMs','registrationMs','totalMs'});
        writetable(trajectory,fullfile(outputFolder,scenario+"_trajectory.csv"));
        if isfield(estimate,'matchingResults'),estimate=rmfield(estimate,'matchingResults');end
        save(fullfile(outputFolder,scenario+"_estimate.mat"),'estimate','row','base','-v7.3');
        f=fopen(fullfile(outputFolder,scenario+"_summary.json"),'w');fprintf(f,'%s\n',jsonencode(row,PrettyPrint=true));fclose(f);
    end
    report=struct('scenarios',vertcat(summaries{:}),'lidarChannel',options.LidarChannel,'envelope',envelope,'mapFile',mapFile,'datasetFolder',datasetFolder, ...
        'initialization',initial,'truthUsage',"None: scoring is a separate process; initial pose comes from measured GNSS", ...
        'timing',"Zero-delay synchronous acquisition-time replay; measured processing time is reported separately");
    report.scanCoordinateAdapter=struct('physical',physicalCalibration,'virtual',p.frameCalibration, ...
        'description',"Virtual stored scan coordinates preserve the same body points; fixed map geometry is unchanged");
    f=fopen(fullfile(outputFolder,'summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
    function [clouds,seconds]=buildSources()
        clouds=cell(n,1);seconds=zeros(n,1);window=[];
        for k=1:n
            frameTimer=tic;cfg=sourceConfig;cfg.perception.coarseProbabilityCloud.projectionRotation=inputs.tilt(:,:,idx(k));
            frame=store.pointClouds(1,frames(k));frame=struct('x',frame.x,'y',frame.y,'z',frame.z);
            frame=reframeLidarForCalibration(frame,physicalCalibration,cfg.perception.frameCalibration);
            current=perceiveCoarseProbabilityCloud(frame,cfg.perception);
            [clouds{k},window]=updateLocalizationSourceWindow(current,t(k),motion(k,:),window,cfg.sourceWindow);
            seconds(k)=toc(frameTimer);
            if mod(k,100)==0 || k==n,fprintf('overlap sources %d/%d\n',k,n);end
        end
    end
end
