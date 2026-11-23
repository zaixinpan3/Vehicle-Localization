function report=runMncavVdbLocalization(datasetFolder,mapFile,interfaceFile,outputFolder,options)
% runMncavVdbLocalization Run a measurement-only sensor-to-pose cascade.
% No reference pose, true velocity or semantic label is read by this runtime.
% Use scoreMncavVdbLocalization in a separate step after saving estimates.
% Replay the exported GNSS fixes, optional receiver heading, motion and scans
% at their acquisition times. The default fused run uses all generated channels.
% Optional CARLA scenarios change tracking inputs after the same GNSS initialization.
% gnss_outage withholds receiver packets for simulator time 35 <= t < 65 s.
    arguments
        datasetFolder (1,1) string
        mapFile (1,1) string
        interfaceFile (1,1) string
        outputFolder (1,1) string
        options.Scenarios (1,:) string {mustBeNonempty,mustBeMember(options.Scenarios, ...
            ["fused","gnss_outage","gnss_only","lidar_only","dead_reckoning"])}="fused"
        options.MaximumFrames (1,1) double=Inf
        options.StartTime (1,1) double=8
    end
    setupVehicleLocalization();maxNumCompThreads(2);
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
    % Synthesize lateral gains once for this recording.
    fprintf('Fresh CARLA lateral gain synthesis and sensor preparation\n');
    inputs=prepareMncavVdbInputs(datasetFolder,interfaceFile);
    frameTable=readtable(fullfile(datasetFolder,'frames.csv'));
    assert(isequal(frameTable.Properties.VariableNames,{'frame_index','lidar_stamp_sec','points'}), ...
        'VehicleLocalization:InvalidMeasurementFrames','Only acquisition metadata belongs in the frame table.');
    frames=find(frameTable.lidar_stamp_sec>=options.StartTime);
    frames=frames(1:min(numel(frames),options.MaximumFrames));
    t=frameTable.lidar_stamp_sec(frames);n=numel(t);
    [distance,idx]=min(abs(inputs.highRate.time-t.'),[],1);assert(max(distance)<1e-7);idx=idx(:);
    h=struct();for name=string(fieldnames(inputs.highRate)).',h.(name)=inputs.highRate.(name)(idx);end
    m=inputs.globalMeasurements;h.longitudinalSpeed=m.longitudinalSpeed(idx);
    lateral=struct('time',t,'yawRate',m.yawRate(idx),'lateralVelocity',m.lateralVelocity(idx), ...
        'yawRateValid',m.yawRateValid(idx),'velocityValid',m.velocityValid(idx));
    [motion,motionSegment]=integrateRecordedPlanarMotion(inputs.highRate.time, ...
        [inputs.highRate.longitudinalSpeed,inputs.lateral.lateralVelocity,inputs.highRate.yawRate],t,inputs.lateral.velocityValid);
    source=load(mapFile,'cloud');map=localizationSupport.projectSemanticProbabilityCloud(source.cloud,2);
    p=perceptionConfig("Carla");cal=jsondecode(fileread(fullfile(datasetFolder,'calibration.json')));
    physicalCalibration=validateLidarFrameCalibration(cal.calibration);
    p.frameCalibration=validateLidarFrameCalibration(map.frameCalibration);
    sourceConfig=struct('perception',p,'sourceWindow',localizationSourceWindowConfig());
    store=matfile(fullfile(datasetFolder,'pointClouds.mat'));
    base=mncavFullObserverConfig();base.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    base.gnss.positionStdM=current.sensors.gnssNoiseStdM;
    if ismember('gnssVariance',inputs.sensorTable.Properties.VariableNames)
        base.gnss.positionStdM=sqrt(inputs.sensorTable.gnssVariance(idx(1)));
    end
    [~,base]=synthesizeMncavInvariantDesign(base);
    initial=initializeMncavVdbFromGnss(inputs.sensorTable,inputs,t(1));base.initialState=initial.state;
    s=inputs.sensorTable;
    scenarios=unique(options.Scenarios,'stable');
    useLidar=ismember(scenarios,["fused","gnss_outage","lidar_only"]);
    sources={};sourceTimings=zeros(n,1);
    if any(useLidar),[sources,sourceTimings]=buildSources();end
    summaries=cell(size(scenarios));
    outageInterval=[35,65];
    for j=1:numel(scenarios)
        scenario=scenarios(j);data=struct('highRate',h);
        gnssOutage=[];
        if ismember(scenario,["fused","gnss_outage","gnss_only"])
            valid=logical(s.gnssValid(idx));
            if scenario=="gnss_outage"
                gnssOutage=outageInterval;
                valid(t>=gnssOutage(1) & t<gnssOutage(2))=false;
            end
            data.gnss=struct('time',t,'position',[s.gnssX(idx),s.gnssY(idx)],'valid',valid,'delay',0);
            if ismember('gnssHeadingRad',s.Properties.VariableNames)
                data.gnss.heading=s.gnssHeadingRad(idx);
            end
        end
        if useLidar(j)
            data.lidarOverlap=struct('map',map,'sources',{sources},'valid',true(n,1),'config',overlapGradientConfig());
        end
        runTimer=tic;estimate=runFullLocalizationObserver(data,base,LateralInputs=lateral);runtime=toc(runTimer);
        accepted=false(n,1);timings=zeros(n,3);lidarChannel="none";
        if useLidar(j)
            lidarChannel="overlapGradient";accepted=estimate.diagnostics.lidarOverlap.available;
            overlapSeconds=estimate.diagnostics.lidarOverlap.seconds;
            timings=[sourceTimings,overlapSeconds,sourceTimings+overlapSeconds];
            timings(~isfinite(timings))=0;
        end
        row=struct('scenario',scenario,'lidarChannel',lidarChannel,'gnssOutageIntervalS',gnssOutage,'frames',n,'durationSeconds',t(end)-t(1), ...
            'referenceUsed',false,'completeGnssPosePackets',nnz(estimate.diagnostics.gnssPoseValid), ...
            'accepted',nnz(accepted), ...
            'medianPerceptionMs',median(timings(:,1))*1000,'medianOverlapMs',median(timings(:,2))*1000, ...
            'medianFrameMs',median(timings(:,3))*1000,'runtimeSeconds',runtime, ...
            'summedFrameProcessingSeconds',sum(timings(:,3)),'sampledSystemCertified',false);
        row.sourcePreparationSeconds=sum(timings(:,1));
        summaries{j}=row;disp(row);
        trajectory=array2table([frames,t,estimate.pose,accepted,timings*1000], ...
            'VariableNames',{'frame','time','x','y','yaw','accepted', ...
            'perceptionMs','overlapMs','totalMs'});
        writetable(trajectory,fullfile(outputFolder,scenario+"_trajectory.csv"));
        save(fullfile(outputFolder,scenario+"_estimate.mat"),'estimate','row','base','-v7.3');
        f=fopen(fullfile(outputFolder,scenario+"_summary.json"),'w');fprintf(f,'%s\n',jsonencode(row,PrettyPrint=true));fclose(f);
    end
    report=struct('scenarios',vertcat(summaries{:}),'lidarChannel',"overlapGradient",'envelope',envelope,'mapFile',mapFile,'datasetFolder',datasetFolder, ...
        'initialization',initial,'truthUsage',"None: scoring is a separate process", ...
        'timing',"Zero-delay synchronous acquisition-time replay; measured processing time is reported separately", ...
        'currentScanOnlyFrames',nnz(motionSegment==0), ...
        'lateralSolverSolved',inputs.lateralDesign.solverSolved,'gainSource',"fresh-lmi; no persisted gains loaded");
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
            [clouds{k},window]=localizationSupport.updateLocalizationSourceWindow(current,t(k),motion(k,:),window,cfg.sourceWindow,motionSegment(k));
            seconds(k)=toc(frameTimer);
            if mod(k,100)==0 || k==n,fprintf('overlap sources %d/%d\n',k,n);end
        end
    end
end
