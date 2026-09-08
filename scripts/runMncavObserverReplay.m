function report = runMncavObserverReplay(replayFolder,designFile,parameterFile,outputFolder,scenario,fixedLidarDelay,options)
% runMncavObserverReplay Evaluate the actual lateral/global observer cascade.
% D2D events come from recursive vehicle-motion-seeded matching and do not use
% reference-seeded guesses. This feed-forward experiment does not feed global
% observer predictions back to matching. Replay here means dataset playback;
% the observer processes delayed measurements once and never revises states.
    arguments
        replayFolder (1,1) string
        designFile (1,1) string
        parameterFile (1,1) string
        outputFolder (1,1) string
        scenario (1,1) string {mustBeMember(scenario,["fusion","gpsOnly","positionOutage","outageNoLidar","lidarOnly"])} = "fusion"
        fixedLidarDelay (1,1) double {mustBeNonnegative} = .15
        options.SensorFolder (1,1) string = ""
    end
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    sequenceFolder=fileparts(replayFolder);
    calls=table();
    if isfile(fullfile(replayFolder,'calls.csv'))
        calls=readtable(fullfile(replayFolder,'calls.csv'),'TextType','string');
    else
        assert(any(scenario==["gpsOnly","outageNoLidar"]),'The D2D replay must finish before fusion.');
    end
    if any(scenario==["fusion","positionOutage","lidarOnly"])
        assert(height(calls)==1170,'The complete 1170-frame replay is required.');
    else
        calls=table();
    end
    sensorFolder=options.SensorFolder;
    if strlength(sensorFolder)==0, sensorFolder=fullfile(sequenceFolder,'sensors'); end
    [sensorData,reference,metadata]=prepareMncavObserverReplay( ...
        sensorFolder,parameterFile,calls,fixedLidarDelay);
    designs=load(designFile,'observerDesign','observerCfg','lateralDesign');
    cfg=designs.observerCfg;
    cfg.measurement.inputInterpolation="zoh";
    cfg.measurement.timestampTolerance=0;
    cfg.measurement.fixedLidarDelay=fixedLidarDelay;
    cfg.measurement.inputHistoryDuration=max(1,fixedLidarDelay+.01);
    if isfield(cfg.measurement,'replayBufferDuration')
        cfg.measurement=rmfield(cfg.measurement,'replayBufferDuration');
    end
    % Recheck saved matrices with the current verifier; old MAT snapshots may
    % predate fields in the provenance contract. This is reference validation.
    designs.observerDesign.verification=verifyImprovedObserverDesign(designs.observerDesign,cfg);
    designs.observerDesign.certified=designs.observerDesign.verification.certified;
    cfg.lidar.missingInformationTranslationWeight=0;
    cfg.lidar.missingInformationHeadingWeight=cfg.lidar.minimumHeadingWeight;
    initial=[reference.x(1)+.5;0;0;reference.y(1)-.4;0;0;reference.psi(1)+deg2rad(2)];
    cfg.observer.initialState=initial;
    if any(scenario==["positionOutage","outageNoLidar"])
        keep=~(sensorData.gps.timestamp>=40 & sensorData.gps.timestamp<60);
        sensorData.gps.timestamp=sensorData.gps.timestamp(keep);
        sensorData.gps.arrivalTime=sensorData.gps.arrivalTime(keep);
        sensorData.gps.pose=sensorData.gps.pose(keep,:);
    end
    if any(scenario==["gpsOnly","outageNoLidar"]), sensorData.lidar=struct(); end
    if scenario=="lidarOnly", sensorData.gps=struct(); end
    timer=tic;
    failure=struct('completed',true,'message',"",'firstFailingSampleTime',NaN, ...
        'diagnosticSeconds',0);
    try
        estimate=runImprovedVehicleObserver(sensorData,designs.lateralDesign,designs.observerDesign,cfg);
        elapsed=toc(timer);
    catch exception
        elapsed=toc(timer);
        failure.completed=false;
        failure.message=string(getReport(exception,'extended','hyperlinks','off'));
        writeJson(fullfile(outputFolder,'failure.json'),failure);
        diagnosticTimer=tic;
        % Diagnostic reruns locate the finite prefix without modifying the
        % production observer or replacing a failed state with a fallback.
        low=2; high=numel(sensorData.highRate.time);
        while high-low>1
            middle=floor((low+high)/2);
            prefix=prefixInputs(sensorData,middle);
            try
                runImprovedVehicleObserver(prefix,designs.lateralDesign,designs.observerDesign,cfg);
                low=middle;
            catch
                high=middle;
            end
        end
        estimate=runImprovedVehicleObserver(prefixInputs(sensorData,low), ...
            designs.lateralDesign,designs.observerDesign,cfg);
        failure.firstFailingSampleTime=sensorData.highRate.time(high);
        failure.diagnosticSeconds=toc(diagnosticTimer);
        reference=reference(1:low,:);
        writeJson(fullfile(outputFolder,'failure.json'),failure);
    end
    error=estimate.onlineZ(:,[1 4 7])-[reference.x,reference.y,reference.psi];
    positionError=hypot(error(:,1),error(:,2));
    yawError=rad2deg(atan2(sin(error(:,3)),cos(error(:,3))));
    online=array2table([reference.time,estimate.onlineZ(:,[1 4 7]),reference.x,reference.y, ...
        reference.psi,positionError,yawError,estimate.lateral.lateralVelocity,estimate.lateral.sideSlipAngle], ...
        'VariableNames',{'time','x','y','psi','referenceX','referenceY','referencePsi', ...
        'positionErrorM','yawErrorDeg','lateralVelocityMps','sideSlipAngleRad'});
    outage=online.time>=40 & online.time<60;
    summary=struct('scenario',scenario,'samples',height(online), ...
        'completed',failure.completed,'firstFailingSampleTime',failure.firstFailingSampleTime, ...
        'positionRmseM',sqrt(mean(positionError.^2)),'positionMedianM',median(positionError), ...
        'positionP95M',prctile(positionError,95),'positionMaximumM',max(positionError), ...
        'yawRmseDeg',sqrt(mean(yawError.^2)),'yawMaximumAbsDeg',max(abs(yawError)), ...
        'outageWindowPositionRmseM',sqrt(mean(positionError(outage).^2)), ...
        'outageWindowPositionMaximumM',max(positionError(outage)), ...
        'computationSeconds',elapsed,'amortizedComputationMsPerSample',elapsed/height(online)*1000, ...
        'acceptedLidarEvents',nnz(estimate.diagnostics.acceptedLidar), ...
        'acceptedGpsEvents',nnz(estimate.diagnostics.acceptedGps), ...
        'outsideTrackRateEnvelope',nnz(estimate.diagnostics.outsideTrackRateEnvelope), ...
        'velocityOutsideEnvelope',nnz(any(abs(estimate.onlineZ(:,[2 5]))>cfg.operating.maximumSpeed,2)), ...
        'accelerationOutsideEnvelope',nnz(any(abs(estimate.onlineZ(:,[3 6]))>cfg.operating.maximumAcceleration,2)), ...
        'stateHistoryRecomputed',estimate.diagnostics.stateHistoryRecomputed, ...
        'maximumInputHistorySegments',estimate.diagnostics.maximumInputHistorySegments, ...
        'maximumAbsLateralVelocityMps',max(abs(estimate.lateral.lateralVelocity)), ...
        'maximumAbsSideSlipDeg',rad2deg(max(abs(estimate.lateral.sideSlipAngle))), ...
        'meanTranslationWeight',mean(tracePages(estimate.diagnostics.translationWeight)/2));
    if ~failure.completed, summary.amortizedComputationMsPerSample=NaN; end
    metadata.scenario=scenario;
    metadata.fixedDelayIsSimulationSetting=true;
    if ~isempty(calls)
        metadata.matchingCallsExceedingFixedDelay=nnz(calls.totalMs>1000*fixedLidarDelay);
    end
    metadata.scoring="causal onlineZ; all past states immutable, no observer replay";
    metadata.architecture="recorded CAN -> lateral observer; recursive D2D events -> seven-state global observer; no global-observer-to-D2D feedback";
    metadata.outageScope="synthetic GNSS XY input outage [40,60) only; D2D retains recorded known roll/pitch, so this is not a complete GNSS-device failure test";
    metadata.certificateScope="reference current-pose timer inequalities retained; not a certificate for fixed-delay transport; matching, cascade and numerical budgets uncalibrated";
    conditions=estimate.diagnostics.certificateConditions;
    summary.maximumQualifiedPoseGapSeconds=conditions.maximumQualifiedPoseGapSeconds;
    summary.shortQualifiedIntervalCount=conditions.shortIntervalCount;
    summary.longQualifiedIntervalCount=conditions.longIntervalCount;
    summary.fixedDelayMatchesConfiguration=conditions.fixedDelayMatchesConfiguration;
    summary.timingWithinCertificate=conditions.timingWithinCertificate;
    summary.flowCertificateVerified=estimate.observer.certificateVerified;
    summary.referenceCertificateVerified=estimate.observer.referenceCertificateVerified;
    summary.timingWithinReferenceSchedule=conditions.timingWithinReferenceSchedule;
    summary.informationWithinReferenceSector=conditions.informationWithinReferenceSector;
    summary.unconditionalStabilityClaimed=false;
    metadata.gpsFusion="Full-matrix information fusion of separate LiDAR and GPS residuals during pose pulses; GPS-only continuation remains outside the full-pose certificate";
    summary.informationWithinCertificate=conditions.informationWithinCertificate;
    summary.weightSectorViolationCount=conditions.weightSectorViolationCount;
    weights=conditions.minimumLidarWeightEigenvalues;
    if isempty(weights),summary.minimumLidarWeight=NaN;summary.medianMinimumLidarWeight=NaN;
    else,summary.minimumLidarWeight=min(weights);summary.medianMinimumLidarWeight=median(weights);end
    summary.gainInformationScale=cfg.lidar.gainInformationScale;
    if scenario=="lidarOnly"
        metadata.outageScope="no GNSS XY events after one reference-based biased initialization; recorded known roll/pitch retained for D2D, not complete GNSS/INS device loss";
    end
    summary.invariantExtensionSamples=nnz(estimate.diagnostics.invariantExtensionActive);
    metadata.matlabVersion=version;
    metadata.computationalThreads=maxNumCompThreads;
    report=struct('summary',summary,'metadata',metadata,'online',online,'estimate',estimate,'failure',failure);
    writetable(online,fullfile(outputFolder,'online.csv'));
    writeJson(fullfile(outputFolder,'summary.json'),summary);
    writeJson(fullfile(outputFolder,'metadata.json'),metadata);
    save(fullfile(outputFolder,'report.mat'),'report','cfg','-v7.3');
    fig=figure('Visible','off','Color','w','Position',[100 100 1300 750]);
    tiledlayout(fig,2,2,'TileSpacing','compact');
    nexttile; plot(reference.x-reference.x(1),reference.y-reference.y(1),'k'); hold on;
    plot(online.x-reference.x(1),online.y-reference.y(1)); axis equal; grid on;
    xlabel('Easting from start (m)'); ylabel('Northing from start (m)');
    legend('GNSS/INS reference','Causal observer','Location','best'); title(scenario);
    nexttile; plot(online.time,positionError); grid on; xlabel('Time (s)'); ylabel('Position error (m)');
    nexttile; plot(online.time,yawError); grid on; xlabel('Time (s)'); ylabel('Yaw error (deg)');
    nexttile; plot(online.time,rad2deg(online.sideSlipAngleRad)); grid on; xlabel('Time (s)'); ylabel('Lateral-observer slip angle (deg)');
    exportgraphics(fig,fullfile(outputFolder,'observer.png'),'Resolution',160);
    exportgraphics(fig,fullfile(outputFolder,'observer.pdf'),'ContentType','vector'); close(fig);
    disp(summary);
end

function data=prefixInputs(data,count)
    for name=string(fieldnames(data.highRate)).'
        data.highRate.(name)=data.highRate.(name)(1:count);
    end
end

function values=tracePages(matrix)
    values=reshape(matrix(1,1,:)+matrix(2,2,:),[],1);
end

function writeJson(path,value)
    fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
