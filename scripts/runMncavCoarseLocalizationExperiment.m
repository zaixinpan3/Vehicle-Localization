function report=runMncavCoarseLocalizationExperiment(outputFolder,options)
% runMncavCoarseLocalizationExperiment Replay raw scans and fuse coarse poses.
% The existing offline semantic map is fixed. Every online scan is classified
% only as whole XY pillars, then matched by geometric D2D. Recursive matching
% uses wheel/gyro/lateral motion and accepted matches after its first seed.
% The observer consumes full-pose measurements with their information matrices;
% rejected or directional-only matches do not become full-pose measurements.
    arguments
        outputFolder (1,1) string="output/mncav_coarse_localization"
        options.MapFile (1,1) string=""
        options.FrameCalibration (1,1) struct=struct()
    end
    setupVehicleLocalization();
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    mapCfg=featureMapBuildConfig();mapFile=mapCfg.probabilityCloudPath;
    if strlength(options.MapFile)>0,mapFile=options.MapFile;end
    sensorFolder="output/mncav_wheel_only_20260916/sensors";
    parameterFile="output/mncav_interface_audit_20260916/vehicle_parameters.json";
    [prepared,~,~]=prepareMncavObserverReplay(sensorFolder,parameterFile,table(),0,IncludeOdom=false);
    saved=load('tests/reference/mncavLateralObserverDesign.mat','design');
    lateral=runLateralVelocityObserver(prepared.highRate,saved.design,lateralObserverConfig("mncav"));
    h=prepared.highRate;
    motion=struct('time',h.time,'longitudinalSpeed',h.longitudinalSpeed, ...
        'lateralVelocity',lateral.lateralVelocity,'yawRate',h.yawRate, ...
        'longitudinalVelocitySource',"four_wheel",'clockModelId',prepared.clockModelId);
    % Profile a separate representative call to verify the actual call path.
    % Profiling and disk loading are excluded from full-sequence timing.
    map=load(mapFile,'cloud');mapCfg=featureMapBuildConfig();
    root=fileparts(fileparts(mfilename('fullpath')));
    frame=loadPointCloudFrame(fullfile(root,'data',mapCfg.pointCloudMatPath),1);
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),1);
    [pose,tilt]=poseRowToPlanarPose(poses(1,:));
    cfg=struct('perception',perceptionConfig("Mississippi"),'registration',distributionRegistrationConfig());
    if ~isempty(fieldnames(options.FrameCalibration)),cfg.perception.frameCalibration=validateLidarFrameCalibration(options.FrameCalibration);end
    cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
    profile clear;profile on;
    profileCleanup=onCleanup(@()profile('off'));
    localizeLidarFrame(frame,map.cloud,pose,0,cfg);
    profile off;trace=profile('info');clear profileCleanup;
    names=string({trace.FunctionTable.FunctionName});
    assert(any(contains(names,'perceiveCoarseProbabilityCloud')) && any(contains(names,'perceiveFrame')), ...
        'Expected the coarse perception entry in the executed call trace.');
    forbidden=["refinePerceptionCandidates","buildFineFeatureMasks","buildSavedFeatureProbabilityCloud"];
    assert(~any(contains(names,forbidden)),'Fine or saved point-feature processing was executed.');
    audit=struct('coarseEntryExecuted',true,'fineRefinementCalls',0, ...
        'scope',"One independent frame-1 localizeLidarFrame call; same entry used on every replay scan", ...
        'functionNames',names);
    save(fullfile(outputFolder,'call_path_audit.mat'),'trace','audit');
    writeJson(fullfile(outputFolder,'call_path_audit.json'),audit);
    profile clear;clear trace map frame;
    matchingFolder=fullfile(outputFolder,'matching');
    matching=replayMississippiLocalization(mapFile,sensorFolder,matchingFolder,"recursive",[],MotionInputs=motion, ...
        FrameCalibration=cfg.perception.frameCalibration);
    observer=runMncavFullObserverExperiment(fullfile(outputFolder,'observer'),MatchingFolder=matchingFolder);
    report=struct('matching',matching.summary,'observer',observer,'callPathAudit',rmfield(audit,'functionNames'));
    writeJson(fullfile(outputFolder,'summary.json'),report);
end

function writeJson(path,value)
    fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
