function report = replayMississippiLocalization(mapFile, sensorFolder, outputFolder, mode, frameIndices,options)
% replayMississippiLocalization Run every raw scan against one frozen map.
% recursive uses four-wheel speed and accepted D2D poses after one
% GNSS initialization. referenceSeed resets the initial guess on every scan
% and is a local-registration diagnostic, not an autonomous trajectory.
% Recorded GNSS/INS supplies known tilt and the evaluation reference. No
% ground-truth position/yaw enters recursive predictions after initialization.
% A receiver-time bridge corrects the recorded ROS/GPS clock-rate mismatch.
% MotionInputs can supply time relative to the first selected scan, recorded
% wheel-derived speed/corrected gyro, and actual lateral-observer velocity.
% Supplied motion must explicitly declare longitudinalVelocitySource="four_wheel".
% Its final value
% may be held for at most 20 ms to cover a fractional final grid interval;
% metadata reports the actual extension.
    arguments
        mapFile (1,1) string
        sensorFolder (1,1) string
        outputFolder (1,1) string
        mode (1,1) string {mustBeMember(mode,["recursive","referenceSeed"])} = "recursive"
        frameIndices (1,:) double {mustBeInteger,mustBePositive} = []
        options.FrameBlockSize (1,1) double {mustBeInteger,mustBePositive} = 50
        options.MotionInputs (1,1) struct = struct()
        options.ParameterFile (1,1) string = "output/mncav_interface_audit_20260916/vehicle_parameters.json"
        options.RegistrationConfig (1,1) struct = distributionRegistrationConfig()
        options.SourceWindowConfig (1,1) struct = localizationSourceWindowConfig()
        options.FrameCalibration (1,1) struct = struct()
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    mapCfg=featureMapBuildConfig();
    pcfg=perceptionConfig("Mississippi");
    if ~isempty(fieldnames(options.FrameCalibration)),pcfg.frameCalibration=validateLidarFrameCalibration(options.FrameCalibration);end
    cfg=struct('perception',pcfg,'registration',options.RegistrationConfig,'sourceWindow',options.SourceWindowConfig);
    matPath=fullfile(root,'data',mapCfg.pointCloudMatPath);
    posePath=fullfile(root,'data',mapCfg.poseMatchCsvPath);
    [~,frameCount]=loadPointCloudFrame(matPath,1);
    if isempty(frameIndices), frameIndices=1:frameCount; end
    assert(all(diff(frameIndices)>0) && frameIndices(end)<=frameCount,'Invalid frame subset.');
    n=numel(frameIndices);
    poses=readFramePoseTable(posePath,frameIndices);
    if ismember('pose_source',poses.Properties.VariableNames)
        poseReferenceSource=strjoin(unique(string(poses.pose_source)),', ')+" interpolated pose at the declared LiDAR header epoch";
        referenceTimeOffset=0;
    else
        poseReferenceSource="nearest matched NovAtel ODOM pose";
        referenceTimeOffset=max(abs(poses.odom_dt_sec));
    end
    poseReference=zeros(n,3); tilt=zeros(3,3,n);
    for k=1:n
        [poseReference(k,:),tilt(:,:,k)]=poseRowToPlanarPose(poses(k,:));
    end
    gnssFolder=fileparts(posePath); stem="raw_data_2024-06-07-12-09-31_0";
    clock=loadReceiverClock(fullfile(gnssFolder,stem+"_inspva.csv"));
    assert(ismember('clock_model_id',poses.Properties.VariableNames) && ...
        all(string(poses.clock_model_id)==string(clock.modelId)), ...
        'VehicleLocalization:ClockMismatch','Regenerate poses with the current receiver clock.');
    scanTime=receiverClockTime(clock,poses.lidar_stamp_sec);
    assert(max(abs((scanTime-scanTime(1))-(poses.receiver_time_sec-poses.receiver_time_sec(1))))<1e-7, ...
        'VehicleLocalization:ClockMismatch','Pose table receiver epochs differ from the shared clock.');
    supplied=options.MotionInputs;
    if isempty(fieldnames(supplied))
        [prepared,~,~]=prepareMncavObserverReplay(sensorFolder,options.ParameterFile);
        lateralCfg=lateralObserverConfig("mncav");lateralDesign=designLateralObserverGains(lateralCfg);
        lateral=runLateralVelocityObserver(prepared.highRate,lateralDesign,lateralCfg);
        firstPose=readFramePoseTable(posePath,1);
        firstTime=receiverClockTime(clock,firstPose.lidar_stamp_sec);
        supplied=struct('time',prepared.highRate.time+firstTime-scanTime(1), ...
            'longitudinalSpeed',prepared.highRate.longitudinalSpeed,'lateralVelocity',lateral.lateralVelocity, ...
            'yawRate',prepared.highRate.yawRate,'longitudinalVelocitySource',"four_wheel",'clockModelId',string(clock.modelId));
    end
    assert(isfield(supplied,'longitudinalVelocitySource') && string(supplied.longitudinalVelocitySource)=="four_wheel", ...
        'VehicleLocalization:WheelSpeedSourceRequired','Supplied replay motion must come from four-wheel estimation.');
    assert(isfield(supplied,'clockModelId') && string(supplied.clockModelId)==string(clock.modelId), ...
        'VehicleLocalization:ClockMismatch','Motion inputs use a different or undeclared receiver clock.');
    suppliedTime=supplied.time+scanTime(1);
    suppliedValues=[supplied.longitudinalSpeed,supplied.lateralVelocity,supplied.yawRate];
    motionSource="four-wheel speed and corrected gyro with actual lateral-observer velocity; causal zero-order hold";
    motionEndHoldSeconds=max(0,scanTime(end)-suppliedTime(end));
    assert(motionEndHoldSeconds<=.02,'VehicleLocalization:MotionCoverage', ...
            'Supplied motion ends more than 20 ms before the final scan.');
    if motionEndHoldSeconds>0
        suppliedTime(end+1)=scanTime(end);suppliedValues(end+1,:)=suppliedValues(end,:);
    end
    motion=integrateRecordedPlanarMotion(suppliedTime,suppliedValues,scanTime);
    timer=tic; loaded=load(mapFile);
    if isfield(loaded,'probabilityCloud')
        fullCloud=loaded.probabilityCloud;
    elseif isfield(loaded,'cloud')
        fullCloud=loaded.cloud;
    else
        fullCloud=temporalMapToProbabilityCloud(loaded.probabilityCloudMap);
    end
    assert(isfield(fullCloud,'clockModelId') && string(fullCloud.clockModelId)==string(clock.modelId), ...
        'VehicleLocalization:ClockMismatch','Rebuild the map with synchronized poses before replay.');
    mapOverlap="same sequence used for mapping and query; includes query observations";
    if isfield(loaded,'probabilityCloudMap') && isfield(loaded.probabilityCloudMap,'evaluationTrainingFrames')
        assert(isempty(intersect(frameIndices,loaded.probabilityCloudMap.evaluationTrainingFrames)), ...
            'VehicleLocalization:EvaluationLeakage','Query frames overlap the declared training set.');
        mapOverlap="disjoint training/query frames from the same drive; neighboring scans remain correlated";
    end
    cloud=registrationSupport.projectSemanticProbabilityCloud(fullCloud,2);
    mapPreparationSeconds=toc(timer);
    clear loaded fullCloud
    offset=[.5,-.4,deg2rad(2)]; state=poseReference(1,:)+offset;
    deadReckoning=zeros(n,3);
    for k=1:n, deadReckoning(k,:)=compose(state,motion(k,:)); end
    cfg.registration.heightMode="xy";
    radius=100; rows=cell(n,32); maxTimestampDifference=0;
    store=matfile(matPath); frameBlock=[]; firstInBlock=0;sourceHistory=[];
    diskBlocks=zeros(0,3);candidatePoses=zeros(n,3);windowDetails=zeros(n,3);
    for k=1:n
        if isempty(frameBlock) || k>=firstInBlock+numel(frameBlock)
            firstInBlock=k; lastInBlock=min(n,k+options.FrameBlockSize-1);
            loadTimer=tic;
            frameBlock=store.pointClouds(1,frameIndices(k:lastInBlock));
            blockLoadSeconds=toc(loadTimer);
            diskBlocks(end+1,:)=[k,lastInBlock,blockLoadSeconds]; %#ok<AGROW>
        end
        frame=frameBlock(k-firstInBlock+1);
        loadSeconds=blockLoadSeconds/numel(frameBlock);
        maxTimestampDifference=max(maxTimestampDifference,abs(double(frame.timestamp)-poses.lidar_stamp_sec(k)));
        if mode=="referenceSeed"
            predicted=poseReference(k,:)+offset;
        elseif k==1
            predicted=state;
        else
            predicted=compose(state,relative(motion(k-1,:),motion(k,:)));
        end
        cfg.perception.coarseProbabilityCloud.projectionRotation=tilt(:,:,k);
        timer=tic; selectionTimer=tic;
        local=selectMap(cloud,predicted,radius);
        selectionSeconds=toc(selectionTimer);
        [event,result,sourceHistory]=localizeLidarFrame(frame,local,predicted,scanTime(k),cfg,sourceHistory,motion(k,:));
        candidatePoses(k,:)=result.poseXYTheta;
        windowDetails(k,:)=[result.sourceWindow.frameCount,result.sourceWindow.spanSeconds,result.sourceWindow.componentCount];
        elapsed=toc(timer);
        assert((result.accepted || result.directionalAccepted)==~isempty(event),'Acceptance/event mismatch.');
        state=predicted;
        information=result.information;
        if ~isempty(event)
            state=event.pose;
            information=event.information;
        end
        difference=state-poseReference(k,:); difference(3)=wrap(difference(3));
        pairCount=0;
        if isfield(result,'correspondences'), pairCount=height(result.correspondences); end
        rows(k,:)={frameIndices(k),scanTime(k)-scanTime(1),poses.lidar_stamp_sec(k), ...
            poseReference(k,1),poseReference(k,2),poseReference(k,3), ...
            predicted(1),predicted(2),predicted(3),state(1),state(2),state(3), ...
            result.accepted,string(result.reason),norm(difference(1:2)),rad2deg(difference(3)), ...
            result.similarity,result.observableRank,result.iterations,pairCount, ...
            1000*elapsed,1000*result.perceptionSeconds,1000*result.registrationSeconds, ...
            1000*selectionSeconds,1000*loadSeconds, ...
            information(1,1),information(1,2),information(1,3), ...
            information(2,2),information(2,3),information(3,3),result.directionalAccepted};
        if mod(k,50)==0 || k==n
            partial=callTable(rows(1:k,:));
            writetable(partial,fullfile(outputFolder,'calls.csv'));
            fprintf('%s %d/%d: accepted %d, current error %.3f m, %.3f deg, %.1f ms.\n', ...
                mode,k,n,nnz(partial.accepted),partial.positionErrorM(end), ...
                partial.yawErrorDeg(end),partial.totalMs(end));
        end
    end
    report.calls=callTable(rows);
    report.calls.clockModelId=repmat(string(clock.modelId),n,1);
    report.candidatePoses=array2table([frameIndices(:),candidatePoses],VariableNames={'frame','x','y','psi'});
    report.sourceWindows=array2table([frameIndices(:),windowDetails],VariableNames={'frame','scans','spanSeconds','components'});
    report.diskBlocks=array2table(diskBlocks,'VariableNames',{'firstQuery','lastQuery','seconds'});
    report.deadReckoning=array2table([frameIndices(:),scanTime-scanTime(1),deadReckoning], ...
        'VariableNames',{'frame','timeSeconds','x','y','psi'});
    report.metadata=struct('mode',mode,'frameCount',n,'sourceMap',mapFile, ...
        'perceptionMode',"coarseProbabilityCloud",'perceptionRerun',true,'finePerceptionUsed',false, ...
        'initialOffset',offset,'mapCropRadiusM',radius,'mapComponents',cloud.components.numComponents, ...
        'mapPreparationSeconds',mapPreparationSeconds,'features',pcfg.featureNames, ...
        'heightMode',"xy",'registrationMethod',cfg.registration.method, ...
        'sourceWindow',cfg.sourceWindow, ...
        'frameCalibration',cfg.perception.frameCalibration, ...
        'poseReferencePoint',"recorded_INS_output_point", ...
        'sourceWindowMotion',"Causal relative wheel/gyro/lateral odometry; no matching pose is reused in source geometry", ...
        'poseState',"X,Y,psi",'motionSource',motionSource, ...
        'motionEndHoldSeconds',motionEndHoldSeconds, ...
        'clock',clock,'clockModelId',string(clock.modelId), ...
        'rosDurationSeconds',poses.lidar_stamp_sec(end)-poses.lidar_stamp_sec(1), ...
        'receiverDurationSeconds',scanTime(end)-scanTime(1), ...
        'reference',poseReferenceSource, ...
        'knownTilt',"recorded GNSS/INS roll/pitch through quaternion yaw/tilt decomposition", ...
        'maximumReferenceTimeOffsetSeconds',referenceTimeOffset, ...
        'maximumMatTimestampDifferenceSeconds',maxTimestampDifference, ...
        'timingScope',"map crop, fresh coarse perception, D2D and pose event; excludes disk read and offline preparation", ...
        'diskLoading',"blocks loaded before processing; diskLoadMs is block time divided by block frame count", ...
        'frameBlockSize',options.FrameBlockSize, ...
        'mapOverlap',mapOverlap, ...
        'matlabVersion',version,'computationalThreads',maxNumCompThreads);
    report.summary=summarize(report.calls,deadReckoning,poseReference);
    writetable(report.calls,fullfile(outputFolder,'calls.csv'));
    writetable(report.candidatePoses,fullfile(outputFolder,'candidate_poses.csv'));
    writetable(report.sourceWindows,fullfile(outputFolder,'source_windows.csv'));
    writetable(report.diskBlocks,fullfile(outputFolder,'disk_blocks.csv'));
    writetable(report.deadReckoning,fullfile(outputFolder,'dead_reckoning.csv'));
    writeJson(fullfile(outputFolder,'metadata.json'),report.metadata);
    writeJson(fullfile(outputFolder,'summary.json'),report.summary);
    save(fullfile(outputFolder,'report.mat'),'report','cfg');
    plotReplay(report,outputFolder);
    disp(report.summary);
end

function cloud=selectMap(full,pose,radius)
    cloud=full; c=full.components;
    keep=sum((c.mean-pose(1:2)).^2,2)<=radius^2;
    cloud.components.mean=c.mean(keep,:);
    cloud.components.covariance=c.covariance(:,:,keep);
    cloud.components.numComponents=nnz(keep);
    for field=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,field), cloud.components.(field)=c.(field)(keep,:); end
    end
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

function value=callTable(rows)
    value=cell2table(rows,'VariableNames',{'frame','timeSeconds','rosStamp', ...
        'referenceX','referenceY','referencePsi','predictedX','predictedY','predictedPsi', ...
        'x','y','psi','accepted','reason','positionErrorM','yawErrorDeg', ...
        'similarity','rank','iterations','matches','totalMs','perceptionMs', ...
        'registrationMs','mapSelectionMs','diskLoadMs', ...
        'informationXX','informationXY','informationXPsi','informationYY', ...
        'informationYPsi','informationPsiPsi','directionalAccepted'});
end

function summary=summarize(calls,deadReckoning,reference)
    error=hypot(deadReckoning(:,1)-reference(:,1),deadReckoning(:,2)-reference(:,2));
    summary=struct('frames',height(calls),'accepted',nnz(calls.accepted), ...
        'directionalAccepted',nnz(calls.directionalAccepted), ...
        'measurementEvents',nnz(calls.accepted | calls.directionalAccepted), ...
        'acceptanceFraction',mean(calls.accepted),'allFramePositionRmseM',sqrt(mean(calls.positionErrorM.^2)), ...
        'acceptedPositionRmseM',sqrt(mean(calls.positionErrorM(calls.accepted).^2)), ...
        'positionMedianM',median(calls.positionErrorM),'positionP95M',quantileLinear(calls.positionErrorM,.95), ...
        'positionMaximumM',max(calls.positionErrorM),'yawRmseDeg',sqrt(mean(calls.yawErrorDeg.^2)), ...
        'yawMaximumAbsDeg',max(abs(calls.yawErrorDeg)), ...
        'deadReckoningPositionRmseM',sqrt(mean(error.^2)),'deadReckoningFinalErrorM',error(end), ...
        'totalMedianMs',median(calls.totalMs),'totalP95Ms',quantileLinear(calls.totalMs,.95), ...
        'totalP99Ms',quantileLinear(calls.totalMs,.99),'totalMaximumMs',max(calls.totalMs), ...
        'callsOver100ms',nnz(calls.totalMs>100),'perceptionMedianMs',median(calls.perceptionMs), ...
        'registrationMedianMs',median(calls.registrationMs));
    reasons=unique(calls.reason); summary.reasons=struct();
    for k=1:numel(reasons)
        summary.reasons.(matlab.lang.makeValidName(reasons(k)))=nnz(calls.reason==reasons(k));
    end
end

function value=quantileLinear(values,fraction)
    values=sort(values(:)); coordinate=1+(numel(values)-1)*fraction;
    low=floor(coordinate); high=ceil(coordinate);
    value=values(low)+(coordinate-low)*(values(high)-values(low));
end

function writeJson(path,value)
    fid=fopen(path,'w'); assert(fid>=0,'Cannot write result.');
    cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end

function plotReplay(report,folder)
    t=report.calls; origin=[t.referenceX(1),t.referenceY(1)];
    fig=figure('Name',"Mississippi "+report.metadata.mode,'Color','w','Position',[80 80 1350 800]);
    tiledlayout(fig,2,2,'TileSpacing','compact');
    nexttile; plot(t.referenceX-origin(1),t.referenceY-origin(2),'k-','LineWidth',1.5); hold on;
    plot(t.x-origin(1),t.y-origin(2),'Color',[0 .4 .8]);
    plot(report.deadReckoning.x-origin(1),report.deadReckoning.y-origin(2),'--','Color',[.7 .4 .1]);
    scatter(t.x(~t.accepted)-origin(1),t.y(~t.accepted)-origin(2),8,[.8 .2 .2],'filled');
    axis equal; grid on; xlabel('Easting from start (m)'); ylabel('Northing from start (m)');
    legend('GNSS/INS reference','Replay estimate','Vehicle-motion only','Rejected D2D','Location','best');
    title("Trajectory: "+report.metadata.mode,'Interpreter','none');
    nexttile; plot(t.timeSeconds,t.positionErrorM); grid on;
    xlabel('Receiver elapsed time (s)'); ylabel('Position error (m)'); title('All frames, including prediction on rejection');
    nexttile; plot(t.timeSeconds,t.yawErrorDeg); grid on;
    xlabel('Receiver elapsed time (s)'); ylabel('Heading error (deg)'); title('Wrapped heading discrepancy');
    nexttile; plot(t.timeSeconds,t.totalMs); hold on; yline(100,'r--'); grid on;
    xlabel('Receiver elapsed time (s)'); ylabel('Pipeline computation (ms)'); title('No warmup calls removed');
    exportgraphics(fig,fullfile(folder,'replay.png'),'Resolution',180);
    exportgraphics(fig,fullfile(folder,'replay.pdf'),'ContentType','vector');
end
