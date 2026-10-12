function cache=prepareMississippiLocalizationClouds(matchingFolder,options)
% prepareMississippiLocalizationClouds Recompute coarse horizons on recorded motion.
% Preparation is reusable across estimator controls; each horizon uses only
% current and preceding scans. Raw LiDAR IMU supplies causal tilt; reference
% only one reference pose is read for the declared initial state; later pose
% values are excluded from horizon construction. Installation compensation is fixed.
    arguments
        matchingFolder (1,1) string=""
        options.SensorFolder (1,1) string="output/mncav_wheel_only_20260916/sensors"
        options.TiltConfiguration (1,1) struct=lidarImuTiltConfig()
        options.MapFile (1,1) string=""
        options.MotionInputs (1,1) struct=struct()
        options.FrameCalibration (1,1) struct=struct()
    end
    root=setupVehicleLocalization();
    if strlength(options.MapFile)==0,mapDefaults=featureMapBuildConfig();options.MapFile=mapDefaults.probabilityCloudPath;end
    if strlength(matchingFolder)>0
        % Historical acquisition/motion cache only; no registration is executed.
        saved=load(fullfile(matchingFolder,'report.mat'),'cfg','report');
        cfg=saved.cfg;calls=saved.report.calls;
        motion=saved.report.deadReckoning{:,{'x','y','psi'}};
        mapFile=saved.report.metadata.sourceMap;clockModelId=saved.report.metadata.clockModelId;
        initialPose=calls{1,{'predictedX','predictedY','predictedPsi'}};
    else
        mapCfg=featureMapBuildConfig();posePath=fullfile(root,'data',mapCfg.poseMatchCsvPath);
        imported=detectImportOptions(posePath);
        imported.SelectedVariableNames={'frame_index','lidar_stamp_sec','receiver_time_sec','clock_model_id'};
        epochs=readtable(posePath,imported);
        calls=table(epochs.frame_index,epochs.receiver_time_sec,epochs.lidar_stamp_sec, ...
            'VariableNames',{'frame','timeSeconds','rosStamp'});
        supplied=options.MotionInputs;
        assert(all(isfield(supplied,{'time','longitudinalSpeed','lateralVelocity','yawRate','clockModelId'})), ...
            'VehicleLocalization:MissingMotionInputs','Supply current measured motion when preparing fresh horizons.');
        clockModelId=string(supplied.clockModelId);
        assert(all(string(epochs.clock_model_id)==clockModelId),'VehicleLocalization:ClockMismatch','Motion and scan clocks differ.');
        values=[supplied.longitudinalSpeed,supplied.lateralVelocity,supplied.yawRate];motionTime=supplied.time;
        endHold=max(0,calls.timeSeconds(end)-motionTime(end));
        assert(endHold<=.02,'VehicleLocalization:MotionCoverage','Motion ends more than 20 ms before the last scan.');
        if endHold>0,motionTime(end+1)=calls.timeSeconds(end);values(end+1,:)=values(end,:);end
        motion=integrateRecordedPlanarMotion(motionTime,values,calls.timeSeconds);
        cfg=struct('perception',perceptionConfig("Mississippi"));mapFile=options.MapFile;
        if ~isempty(fieldnames(options.FrameCalibration)),cfg.perception.frameCalibration=validateLidarFrameCalibration(options.FrameCalibration);end
        % Preserve the declared one-pose initialization, never later reference poses.
        firstPose=readFramePoseTable(posePath,calls.frame(1));
        initialPose=poseSupport.poseRowToPlanarPose(firstPose(1,:))+[.5,-.4,deg2rad(2)];
    end
    cfg.sourceWindow=localizationSourceWindowConfig();n=height(calls);sources=cell(n,1);
    currentSources=cell(n,1);
    mapCfg=featureMapBuildConfig();history=[];seconds=zeros(n,1);counts=zeros(n,1);
    tilt=prepareMississippiLidarTilt(calls.rosStamp,options.SensorFolder,Configuration=options.TiltConfiguration);
    store=matfile(fullfile(root,'data',mapCfg.pointCloudMatPath));
    for first=1:50:n
        ids=first:min(n,first+49);block=store.pointClouds(1,calls.frame(ids));
        for k=ids
            timer=tic;
            cfg.perception.coarseProbabilityCloud.projectionRotation=tilt.rotation(:,:,k);
            raw=perceiveCoarseProbabilityCloud(block(k-first+1),cfg.perception);
            [sources{k},history,~,currentSources{k}]=updateLocalizationSourceWindow(raw,calls.timeSeconds(k),motion(k,:),history,cfg.sourceWindow);
            seconds(k)=toc(timer);counts(k)=sources{k}.components.numComponents;
        end
        fprintf('Coarse source cache %d/%d\n',ids(end),n);
    end
    map=load(mapFile,'cloud');
    assert(isfield(map.cloud,'clockModelId') && string(map.cloud.clockModelId)==string(clockModelId), ...
        'VehicleLocalization:ClockMismatch','Map and source clocks differ.');
    % Keep map XYZ side information while retaining its exact XY geometry.
    fixed=map.cloud;
    cache=struct('sources',{sources},'currentSources',{currentSources},'fixed',fixed,'calls',calls,'cfg',cfg, ...
        'seconds',seconds,'counts',counts,'clockModelId',clockModelId,'tilt',tilt,'initialPose',initialPose, ...
        'sourceMap',mapFile,'preparation',"Measured-motion coarse horizons; no registration");
end
