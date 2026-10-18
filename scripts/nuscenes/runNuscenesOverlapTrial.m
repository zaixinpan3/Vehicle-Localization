function [reports,estimates,referenceAudit]=runNuscenesOverlapTrial(datasetFolder,outputFolder,options)
% runNuscenesOverlapTrial Known-map LiDAR/CAN development trial on nuScenes.
% With one-based frame numbers, map all odd sweeps with reference poses and
% query disjoint even sweeps at about 10 Hz. Reference is allowed for mapping, one initial pose/
% tilt and scoring only. Live inputs are LiDAR, rear-wheel RPM, steering,
% IMU force/gyro and IMU tilt. No pose.json or navigation velocity is loaded.
% Fresh global and vehicle-specific lateral LMIs are solved for each call.
% Keep only small summaries/figures; all feature/state arrays stay in memory.
% Optional estimates output permits paired reference-isolation checks without
% persisting state arrays or exposing scoring references to the estimator.
% An optional audit dataset shares this call's fresh in-memory gains and RNG
% state, isolating reference changes from independent numerical gain solves.
    arguments
        datasetFolder (1,1) string
        outputFolder (1,1) string
        options.Scenes (1,:) string=["scene-0061","scene-0103"]
        options.CalibrationScenes (1,:) string=["scene-0553","scene-0655","scene-0757", ...
            "scene-0796","scene-0916","scene-1077","scene-1094","scene-1100"]
        options.RandomSeed (1,1) double {mustBeInteger,mustBeNonnegative}=20261014
        options.ReferenceAuditDatasetFolder (1,1) string=""
        options.ReferenceAuditOutputFolder (1,1) string=""
    end
    assert((strlength(options.ReferenceAuditDatasetFolder)>0)==(strlength(options.ReferenceAuditOutputFolder)>0), ...
        'VehicleLocalization:IncompleteReferenceAudit','Supply both reference-audit dataset and output folders.');
    assert(isempty(intersect(options.Scenes,options.CalibrationScenes)), ...
        'VehicleLocalization:NuscenesCalibrationLeakage','Steering calibration/evaluation scenes must be disjoint.');
    root=setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    previousRng=rng;restoreRng=onCleanup(@()rng(previousRng));rng(options.RandomSeed,'twister');
    calibration=calibrateNuscenesSteering(datasetFolder,options.CalibrationScenes);
    trial=nuscenesTrialConfig(calibration.steeringRatio);
    fprintf('Fresh nuScenes global gain synthesis\n');[globalDesign,trial.global]=synthesizeMncavInvariantDesign(trial.global);
    fprintf('Fresh assumed-Zoe lateral gain synthesis\n');lateralDesign=designLateralObserverGains(trial.lateral);
    writeJson(fullfile(outputFolder,'gain_and_vehicle_summary.json'),struct( ...
        'steeringCalibration',calibration,'vehicle',trial.vehicle,'vehicleProvenance',trial.provenance, ...
        'globalInputs',invariantObserverSupport.inputs(trial.global),'globalDesign',globalDesign, ...
        'lateralVertexGains',lateralDesign.vertexGains,'lateralSolverSolved',lateralDesign.solverSolved, ...
        'lateralIssGain',lateralDesign.issGain, ...
        'lateralVehicleIdentified',false,'gainSource',"fresh-lmi; no persisted gains loaded"));
    tables=loadTables(datasetFolder);reports=cell(numel(options.Scenes),1);estimates=cell(numel(options.Scenes),1);
    sceneRng=rng;
    for j=1:numel(options.Scenes)
        [reports{j},estimates{j}]=runScene(datasetFolder,outputFolder,options.Scenes(j),tables,trial,lateralDesign);
    end
    [status,commit]=system(sprintf('git -C "%s" rev-parse HEAD',root));
    assert(status==0,'VehicleLocalization:MissingTrialVersion','Cannot identify executed source version.');
    writeJson(fullfile(outputFolder,'summary.json'),struct('scenes',{reports}, ...
        'sourceBaseCommit',strtrim(commit),'randomSeed',options.RandomSeed,'developmentTrial',true,'officialBenchmark',false, ...
        'reference',"nuScenes ego_pose fused localization; not independent survey truth", ...
        'vehicleIdentified',false,'independentGnssUsed',false,'sampledSystemCertified',false));
    referenceAudit=struct();
    if strlength(options.ReferenceAuditDatasetFolder)>0
        rng(sceneRng);
        tables=loadTables(options.ReferenceAuditDatasetFolder);
        auditReports=cell(size(reports));auditEstimates=cell(size(estimates));
        for j=1:numel(options.Scenes)
            [auditReports{j},auditEstimates{j}]=runScene(options.ReferenceAuditDatasetFolder, ...
                options.ReferenceAuditOutputFolder,options.Scenes(j),tables,trial,lateralDesign);
        end
        referenceAudit=struct('reports',{auditReports},'estimates',{auditEstimates});
    end
end

function [report,estimate]=runScene(folder,out,sceneName,tables,trial,lateralDesign)
    scene=tables.scenes(strcmp({tables.scenes.name},sceneName));
    assert(isscalar(scene),'VehicleLocalization:MissingNuscenesScene','Scene not found: %s.',sceneName);
    sampleTokens=string({tables.samples(strcmp({tables.samples.scene_token},scene.token)).token});
    frames=tables.frames(contains(string({tables.frames.filename}),'LIDAR_TOP') & ...
        ismember(string({tables.frames.sample_token}),sampleTokens));
    first=tables.samples(strcmp({tables.samples.token},scene.first_sample_token)).timestamp;
    last=tables.samples(strcmp({tables.samples.token},scene.last_sample_token)).timestamp;
    frames=frames([frames.timestamp]>=first & [frames.timestamp]<=last);
    [~,order]=sort([frames.timestamp]);frames=frames(order);stamp=double([frames.timestamp].');
    prefix=fullfile(folder,'can_bus',sceneName);
    imu=jsondecode(fileread(prefix+"_ms_imu.json"));wheels=jsondecode(fileread(prefix+"_zoe_veh_info.json"));
    steering=jsondecode(fileread(prefix+"_steeranglefeedback.json"));
    [motion,aux]=prepareNuscenesMotion(imu,wheels,steering,trial);
    lateral=runLateralVelocityObserver(motion,lateralDesign,trial.lateral);
    ids=(0:numel(frames)-1).';training=find(mod(ids,2)==0);
    query=find(mod(ids,2)==1 & stamp>=aux.utime(1) & stamp<=aux.utime(end));
    assert(numel(query)>=10,'VehicleLocalization:MissingNuscenesCoverage','Too few query sweeps.');
    [~,poseIndex]=ismember(string({frames.ego_pose_token}),string({tables.poses.token}));
    reference=zeros(numel(frames),4);tilt=zeros(numel(frames),2);
    for k=1:numel(frames)
        pose=tables.poses(poseIndex(k));angles=euler(pose.rotation);
        reference(k,:)=[pose.translation(1:2).',angles(3),pose.translation(3)];tilt(k,:)=angles(1:2);
    end
    reference(:,1:2)=reference(:,1:2)-reference(1,1:2);
    names=["curb","pole","facade"];p=profile("offline",names);
    mapCfg=featureMapBuildConfig();mapCfg.frameCalibration=p.frameCalibration;mapCfg.featureNames=names;
    mapCfg.temporalMap.frameCalibration=p.frameCalibration;mapCfg.logEnabled=false;
    observations=struct('featureNames',names,'frameIndices',ids(training).', ...
        'frameCalibration',p.frameCalibration,'pointsByFeatureFrame',{cell(numel(names),numel(training))});
    counts=zeros(numel(training),numel(names));timer=tic;
    for j=1:numel(training)
        k=training(j);xyz=readScan(folder,frames(k),tables.calibrations);level=xyz*tiltRotation(tilt(k,:)).';
        perception=perceiveFrame(asFrame(level),p);R=rotation(reference(k,3));
        globalXYZ=[level(:,1:2)*R.'+reference(k,1:2),level(:,3)+reference(k,4)];
        for c=1:numel(names)
            points=globalXYZ(logical(perception.featureMasks.(names(c))),:);counts(j,c)=size(points,1);
            if ~isempty(points)
                resolution=mapCfg.temporalMap.defaultParams.representativeResolution;
                [~,keep]=unique(floor(points(:,1:2)/resolution),'rows','stable');points=points(keep,:);
            end
            observations.pointsByFeatureFrame{c,j}=points;
        end
        if mod(j,10)==0 || j==numel(training),fprintf('%s mapping %d/%d\n',sceneName,j,numel(training));end
    end
    fittedMap=buildSlidingWindowMap(observations,mapCfg);
    stability=mapStabilitySummary(fittedMap);
    map=registrationSupport.projectSemanticProbabilityCloud(temporalMapToProbabilityCloud(fittedMap),2);
    mappingSeconds=toc(timer);clear observations fittedMap;
    scoreReference=reference(query,1:3);initial=struct('pose',scoreReference(1,:)+[.5,-.4,deg2rad(2)],'tilt',tilt(query(1),:));
    % Remove all frame-reference arrays before constructing the runtime inputs.
    % The retained scoreReference crosses only the evaluation boundary below.
    clear reference tilt poseIndex;
    tables=rmfield(tables,{'poses','samples','scenes'});
    queryStamp=stamp(query);nativeIndex=interp1(aux.utime,(1:numel(aux.utime)).',queryStamp,'previous');
    nativeIndex=round(nativeIndex);t=(queryStamp-queryStamp(1))/1e6;
    acceleration=aux.rearAxleAcceleration(nativeIndex,:);
    h=struct('time',t,'longitudinalSpeed',motion.longitudinalSpeed(nativeIndex), ...
        'longitudinalAcceleration',acceleration(:,1),'lateralAcceleration',acceleration(:,2), ...
        'yawRate',motion.yawRate(nativeIndex));
    l=struct('time',t,'lateralVelocity',lateral.lateralVelocity(nativeIndex), ...
        'sideSlipAngle',lateral.sideSlipAngle(nativeIndex),'sideSlipAngleRate',lateral.sideSlipAngleRate(nativeIndex), ...
        'velocityValid',lateral.velocityValid(nativeIndex));
    queryTilt=aux.tilt(nativeIndex,:);queryTilt=queryTilt-queryTilt(1,:)+initial.tilt;
    p=profile("coarseProbabilityCloud",names);sources=cell(numel(query),1);timer=tic;
    for j=1:numel(query)
        xyz=readScan(folder,frames(query(j)),tables.calibrations);
        sources{j}=perceiveCoarseProbabilityCloud(asFrame(xyz*tiltRotation(queryTilt(j,:)).'),p);
        if mod(j,40)==0 || j==numel(query),fprintf('%s queries %d/%d\n',sceneName,j,numel(query));end
    end
    perceptionSeconds=toc(timer);cfg=trial.global;
    v=rotation(initial.pose(3))*[h.longitudinalSpeed(1);l.lateralVelocity(1)];
    cfg.initialState=[initial.pose(1);v(1);0;initial.pose(2);v(2);0;initial.pose(3)];
    data=struct('highRate',h,'lidarOverlap',struct('map',map,'sources',{sources}, ...
        'valid',true(numel(query),1),'config',overlapGradientConfig()));
    timer=tic;estimate=runFullLocalizationObserver(data,struct(),cfg,LateralInputs=l);observerSeconds=toc(timer);
    assert(all(isfinite(estimate.z),'all'),'VehicleLocalization:NuscenesDivergence','Nonfinite localization state.');
    errors=estimate.pose-scoreReference;errors(:,3)=atan2(sin(errors(:,3)),cos(errors(:,3)));
    distance=hypot(errors(:,1),errors(:,2));tracking=t>=2;
    metrics=table(["all";"after_2s"],[numel(t);nnz(tracking)], ...
        [sqrt(mean(distance.^2));sqrt(mean(distance(tracking).^2))], ...
        [median(distance);median(distance(tracking))],[prctile(distance,95);prctile(distance(tracking),95)], ...
        [max(distance);max(distance(tracking))],rad2deg([sqrt(mean(errors(:,3).^2));sqrt(mean(errors(tracking,3).^2))]), ...
        'VariableNames',{'window','frames','position_rmse_m','position_median_m','position_p95_m','position_max_m','heading_rmse_deg'});
    sceneOut=fullfile(out,sceneName);if ~isfolder(sceneOut),mkdir(sceneOut);end
    writetable(metrics,fullfile(sceneOut,'metrics.csv'));disp(sceneName);disp(metrics);
    % Scoring-only diagnostics: none of these reference-derived values return
    % to the observer. Retain a small table to expose domain-transition errors.
    support=estimate.diagnostics.lidarPoseSupport;
    forwardError=cos(scoreReference(:,3)).*errors(:,1)+sin(scoreReference(:,3)).*errors(:,2);
    lateralError=-sin(scoreReference(:,3)).*errors(:,1)+cos(scoreReference(:,3)).*errors(:,2);
    bodyVx=cos(estimate.pose(:,3)).*estimate.velocity(:,1)+sin(estimate.pose(:,3)).*estimate.velocity(:,2);
    diagnostics=table(t,h.longitudinalSpeed,l.velocityValid,bodyVx,distance,forwardError,lateralError, ...
        support.rank,support.minimumStrength,support.headingStrengthAfterPosition,support.sectorQualified, ...
        'VariableNames',{'time_s','wheel_speed_mps','lateral_valid','estimated_body_vx_mps', ...
        'position_error_m','forward_error_m','lateral_error_m','lidar_rank', ...
        'lidar_minimum_strength','heading_strength_after_position','sector_qualified'});
    writetable(diagnostics,fullfile(sceneOut,'frame_diagnostics.csv'));
    report=struct('scene',sceneName,'description',string(scene.description),'durationS',t(end), ...
        'mappingFrames',numel(training),'queryFrames',numel(query),'trainingQueryOverlap',numel(intersect(training,query)), ...
        'mappingFrameTokens',string({frames(training).token}),'queryFrameTokens',string({frames(query).token}), ...
        'frameNumberConvention',"One-based, chronological within the scene; odd mapping, even localization", ...
        'mappingFrameNumbers',training.','queryFrameNumbers',query.', ...
        'mappingFeatureCounts',sum(counts,1),'mapComponents',map.components.numComponents, ...
        'mapStability',stability, ...
        'lidarAvailable',nnz(estimate.diagnostics.lidarOverlap.available), ...
        'wheelSpeedRangeMps',[min(h.longitudinalSpeed),max(h.longitudinalSpeed)], ...
        'roadWheelRangeDeg',rad2deg([min(motion.steeringAngle),max(motion.steeringAngle)]), ...
        'lateralVelocityRangeMps',[min(l.lateralVelocity),max(l.lateralVelocity)], ...
        'sideSlipRangeDeg',rad2deg([min(l.sideSlipAngle),max(l.sideSlipAngle)]), ...
        'dynamicLateralSamples',nnz(lateral.diagnostics.dynamicModelEvaluated), ...
        'nativeMotionSamples',numel(motion.time),'maximumCanAgeS',max(aux.packetAgeS,[],'all'), ...
        'maximumImuAgeAtQueryS',max((queryStamp-aux.utime(nativeIndex))/1e6), ...
        'bodyVelocityValidPackets',estimate.diagnostics.packetCounts.bodyVelocityValid, ...
        'mappingSeconds',mappingSeconds,'perceptionSeconds',perceptionSeconds,'observerSeconds',observerSeconds, ...
        'runtimeInputKeys',string(fieldnames(data)),'poseCanFileRead',false,'navigationVelocityUsed',false, ...
        'referenceUsedAfterInitialization',false,'referenceMapUsed',true,'independentGnssUsed',false, ...
        'lateralObserverUsed',true,'wheelSpeedUsed',true,'steeringUsed',true,'intrinsicPredictionUsesImu',false, ...
        'queryTilt',"IMU tilt changes aligned to a single reference initial tilt", ...
        'deskew',"Disabled for map and query; no truth-based deskew", ...
        'protocol',"Same-pass odd-frame map and disjoint even-frame localization (one-based); not an independent mapping traversal", ...
        'actualOverlapOperatorLmiVerified',false,'vehicleIdentified',false,'sampledSystemCertified',false, ...
        'developmentTrial',true,'initialization',initial,'metrics',table2struct(metrics));
    writeJson(fullfile(sceneOut,'summary.json'),report);
    fig=figure('Visible','off','Color','white','Position',[100,100,900,700]);tiledlayout(2,1,TileSpacing="compact");nexttile;
    plot(scoreReference(:,1),scoreReference(:,2),'k-',estimate.position(:,1),estimate.position(:,2),'b-');axis equal;grid on;
    legend('nuScenes reference','Overlap observer');xlabel('Map x (m)');ylabel('Map y (m)');title(sceneName);
    nexttile;plot(t,distance);grid on;xlabel('Elapsed time (s)');ylabel('Position difference (m)');
    set(findall(fig,'Type','axes'),'Color','white','XColor','black','YColor','black','GridColor',[.7,.7,.7]);
    set(findall(fig,'Type','text'),'Color','black');set(findall(fig,'Type','legend'),'Color','white','TextColor','black');
    exportgraphics(fig,fullfile(sceneOut,'trajectory.png'),BackgroundColor="white");close(fig);
end

function summary=mapStabilitySummary(map)
% Summarize actual repeated-frame inference; never scale weights by frame count.
    layers=map.canonicalMap.layers;classes=cell(numel(layers),1);
    for k=1:numel(layers)
        layer=layers(k);components=layer.components;published=layer.componentPublished;
        count=reshape([components.observedBlockCount],[],1);
        classes{k}=struct('feature',layer.classLabel,'inputPoints',layer.pointCount, ...
            'observedFrameBlocks',numel(layer.provenance.uniqueBlockIds), ...
            'fittedComponents',numel(components),'publishedComponents',nnz(published), ...
            'observedBlockCountMinMedianP95Max',distribution(count), ...
            'repeatabilityMinMedianP95Max',distribution(layer.componentRepeatability), ...
            'publishedRepeatabilityMinMedianP95Max',distribution(layer.componentRepeatability(published)), ...
            'referenceMass',layer.referenceMass,'publishedMass',layer.totalMass);
    end
    summary=struct('ingestedFrames',map.sourceFrameCount,'sourcePoints',map.canonicalMap.sourcePointCount, ...
        'observationBlockSize',map.config.temporalMap.observationBlockSize, ...
        'weightSemantics',"Reference mass times inferred repeatability; separate frame/block IDs retained", ...
        'classes',{classes});
end

function values=distribution(x)
    if isempty(x),values=[];else,values=[min(x),median(x),prctile(x,95),max(x)];end
end

function tables=loadTables(folder)
    names={'scene','sample','sample_data','ego_pose','calibrated_sensor'};
    fields={'scenes','samples','frames','poses','calibrations'};tables=struct();
    for k=1:numel(names),tables.(fields{k})=jsondecode(fileread(fullfile(folder,'v1.0-mini',names{k}+".json")));end
end

function p=profile(mode,names)
    p=perceptionConfig("Downtown",mode);p.featureNames=names;p.frameCalibration=lidarFrameCalibrationConfig();
    % Points are already transformed to the rear axle, whose road is near z=0.
    p.groundSegmentation.groundSeedMaxZ=0;p.groundSegmentation.slopeGridPriorHeight=0;
end

function xyz=readScan(folder,frame,calibrations)
    calibration=calibrations(strcmp({calibrations.token},frame.calibrated_sensor_token));
    path=fullfile(folder,frame.filename);f=fopen(path,'rb');
    assert(f>=0,'VehicleLocalization:MissingNuscenesScan','Missing scan %s.',path);cleanup=onCleanup(@()fclose(f));
    raw=fread(f,[5,Inf],'single=>double').';R=quaternionRotation(calibration.rotation);
    xyz=raw(:,1:3)*R.'+calibration.translation(:).';
end

function frame=asFrame(xyz)
    frame=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3));
end

function angles=euler(q)
    R=quaternionRotation(q);angles=[atan2(R(3,2),R(3,3)),asin(max(-1,min(1,-R(3,1)))),atan2(R(2,1),R(1,1))];
end

function R=quaternionRotation(q)
    q=q(:)/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);
    R=[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y);2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x); ...
        2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)];
end

function R=rotation(yaw)
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end

function R=tiltRotation(angles)
    phi=angles(1);theta=angles(2);Rx=[1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)];
    Ry=[cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];R=Ry*Rx;
end

function writeJson(path,value)
    f=fopen(path,'w');assert(f>=0,'VehicleLocalization:CannotWriteTrial','Cannot write %s.',path);
    cleanup=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(value,PrettyPrint=true));
end
