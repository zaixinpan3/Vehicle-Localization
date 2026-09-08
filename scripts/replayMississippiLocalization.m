function report = replayMississippiLocalization(mapFile, sensorFolder, outputFolder, mode, frameIndices,options)
% replayMississippiLocalization Run every raw scan against one frozen map.
% recursive uses recorded vehicle twist and accepted D2D poses after one
% GNSS initialization. referenceSeed resets the initial guess on every scan
% and is a local-registration diagnostic, not an autonomous trajectory.
% Recorded GNSS/INS supplies known tilt and the evaluation reference. No
% ground-truth position/yaw enters recursive predictions after initialization.
% A receiver-time bridge corrects the recorded ROS/GPS clock-rate mismatch.
    arguments
        mapFile (1,1) string
        sensorFolder (1,1) string
        outputFolder (1,1) string
        mode (1,1) string {mustBeMember(mode,["recursive","referenceSeed"])} = "recursive"
        frameIndices (1,:) double {mustBeInteger,mustBePositive} = []
        options.FrameBlockSize (1,1) double {mustBeInteger,mustBePositive} = 50
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    mapCfg=featureMapBuildConfig();
    pcfg=perceptionConfig("Mississippi");
    cfg=struct('perception',pcfg,'registration',distributionRegistrationConfig());
    matPath=fullfile(root,'data',mapCfg.pointCloudMatPath);
    posePath=fullfile(root,'data',mapCfg.poseMatchCsvPath);
    [~,frameCount]=loadPointCloudFrame(matPath,1);
    if isempty(frameIndices), frameIndices=1:frameCount; end
    assert(all(diff(frameIndices)>0) && frameIndices(end)<=frameCount,'Invalid frame subset.');
    n=numel(frameIndices);
    poses=readFramePoseTable(posePath,frameIndices);
    poseReference=zeros(n,3); tilt=zeros(3,3,n);
    for k=1:n
        [poseReference(k,:),tilt(:,:,k)]=poseRowToPlanarPose(poses(k,:));
    end
    gnssFolder=fileparts(posePath); stem="raw_data_2024-06-07-12-09-31_0";
    ins=readtable(fullfile(gnssFolder,stem+"_inspva.csv"));
    twist=readtable(fullfile(sensorFolder,'twist.csv'));
    % Receiver seconds are a clock source only, never a motion/pose input.
    rosOrigin=ins.stamp_sec(1); receiverOrigin=ins.gps_seconds(1);
    receiverTime=ins.gps_seconds-receiverOrigin;
    assert(all(diff(receiverTime)>0) && all(diff(ins.stamp_sec)>0),'Invalid receiver clock.');
    scanTime=interp1(ins.stamp_sec-rosOrigin,receiverTime,poses.lidar_stamp_sec-rosOrigin,'linear','extrap');
    motionTime=interp1(ins.stamp_sec-rosOrigin,receiverTime,twist.stamp_sec-rosOrigin,'linear','extrap');
    motion=integrateRecordedPlanarMotion(motionTime, ...
        [twist.linear_x_mps,twist.linear_y_mps,twist.angular_z_radps],scanTime);
    timer=tic; loaded=load(mapFile);
    if isfield(loaded,'probabilityCloud')
        fullCloud=loaded.probabilityCloud;
    else
        fullCloud=temporalMapToProbabilityCloud(loaded.probabilityCloudMap);
    end
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
    store=matfile(matPath); frameBlock=[]; firstInBlock=0;
    diskBlocks=zeros(0,3);
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
        [event,result]=localizeLidarFrame(frame,local,predicted,scanTime(k),cfg);
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
    report.diskBlocks=array2table(diskBlocks,'VariableNames',{'firstQuery','lastQuery','seconds'});
    report.deadReckoning=array2table([frameIndices(:),scanTime-scanTime(1),deadReckoning], ...
        'VariableNames',{'frame','timeSeconds','x','y','psi'});
    report.metadata=struct('mode',mode,'frameCount',n,'sourceMap',mapFile, ...
        'initialOffset',offset,'mapCropRadiusM',radius,'mapComponents',cloud.components.numComponents, ...
        'mapPreparationSeconds',mapPreparationSeconds,'features',pcfg.featureNames, ...
        'heightMode',"xy",'poseState',"X,Y,psi",'motionSource',"recorded /vehicle/twist with causal zero-order hold", ...
        'clockSource',"INSPVA receiver GPS seconds interpolated on ROS stamp; edge extrapolation only", ...
        'rosDurationSeconds',poses.lidar_stamp_sec(end)-poses.lidar_stamp_sec(1), ...
        'receiverDurationSeconds',scanTime(end)-scanTime(1), ...
        'reference',"nearest matched NovAtel odom base_link GNSS/INS pose, also used for mapping", ...
        'knownTilt',"recorded GNSS/INS roll/pitch through quaternion yaw/tilt decomposition", ...
        'maximumReferenceTimeOffsetSeconds',max(abs(poses.odom_dt_sec)), ...
        'maximumMatTimestampDifferenceSeconds',maxTimestampDifference, ...
        'timingScope',"map crop, fresh coarse perception, D2D and pose event; excludes disk read and offline preparation", ...
        'diskLoading',"blocks loaded before processing; diskLoadMs is block time divided by block frame count", ...
        'frameBlockSize',options.FrameBlockSize, ...
        'mapOverlap',mapOverlap, ...
        'matlabVersion',version,'computationalThreads',maxNumCompThreads);
    report.summary=summarize(report.calls,deadReckoning,poseReference);
    writetable(report.calls,fullfile(outputFolder,'calls.csv'));
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
