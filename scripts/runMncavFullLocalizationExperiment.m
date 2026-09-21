function report=runMncavFullLocalizationExperiment(outputFolder,options)
% runMncavFullLocalizationExperiment Evaluate the complete recorded cascade.
% Re-synthesize MnCAV lateral/global gains, run the actual lateral observer,
% recursively match raw LiDAR to the frozen map, and evaluate the global DDE.
% The continuous input is an explicitly offline reconstruction of accepted
% matching poses and their actual information. No real-time claim follows.
    arguments
        outputFolder (1,1) string="output/mncav_full_localization"
        options.MapFile (1,1) string="output/mississippi_mapping_synchronized/probability_cloud_map.mat"
        options.MatchingFolder (1,1) string=""
        options.InformationScale (1,1) double {mustBeFinite,mustBePositive}=.001
        options.MaximumOfflineGap (1,1) double {mustBeFinite,mustBePositive}=1
    end
    root=setupVehicleLocalization();
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    sensorFolder=fullfile(root,'output','mississippi_20240607_120931_20260907','sensors');
    parameterFile=fullfile(fileparts(sensorFolder),'vehicle_parameters.json');
    lateralCfg=lateralObserverConfig("mncav");
    timer=tic;lateralDesign=designLateralObserverGains(lateralCfg);lateralSynthesisSeconds=toc(timer);
    [raw,reference,metadata]=prepareMncavObserverReplay(sensorFolder,parameterFile);
    assert(isequal(lateralCfg.vehicle,metadata.parameters.vehicle), ...
        'VehicleLocalization:VehicleParameterMismatch','Input preparation and lateral model must use the same nominal vehicle.');
    timer=tic;lateral=runLateralVelocityObserver(raw.highRate,lateralDesign,lateralCfg);lateralSeconds=toc(timer);
    cfg=improvedObserverConfig("lidar","mncav");
    cfg.lidar.gainInformationScale=options.InformationScale;
    timer=tic;design=designImprovedObserverGains(cfg);globalSynthesisSeconds=toc(timer);
    if strlength(options.MatchingFolder)==0
        matchingFolder=fullfile(outputFolder,'matching');
        motion=struct('time',raw.highRate.time,'longitudinalSpeed',raw.highRate.longitudinalSpeed, ...
            'lateralVelocity',lateral.lateralVelocity,'yawRate',raw.highRate.yawRate, ...
            'longitudinalVelocitySource',"four_wheel",'clockModelId',raw.clockModelId);
        matching=replayMississippiLocalization(options.MapFile,sensorFolder,matchingFolder, ...
            "recursive",[],MotionInputs=motion);
        matchingReused=false;
    else
        matchingFolder=options.MatchingFolder;stored=load(fullfile(matchingFolder,'report.mat'),'report');
        matching=stored.report;matchingReused=true;
        assert(string(matching.metadata.sourceMap)==options.MapFile ...
            && string(matching.metadata.mode)=="recursive" ...
            && contains(string(matching.metadata.motionSource),'actual lateral-observer') ...
            && contains(string(matching.metadata.motionSource),'four-wheel'), ...
            'VehicleLocalization:MatchingProvenance','Matching must use the declared map and actual lateral-observer motion.');
    end
    assert(isfield(matching.metadata,'clockModelId') && ...
        string(matching.metadata.clockModelId)==raw.clockModelId, ...
        'VehicleLocalization:ClockMismatch','Cached matching and motion clocks differ.');
    calls=matching.calls;accepted=calls.accepted==1;
    assert(nnz(accepted)>=2,'VehicleLocalization:InsufficientMatches','Need at least two full-pose matches.');
    poseTime=calls.timeSeconds(accepted);pose=[calls.x(accepted),calls.y(accepted),calls.psi(accepted)];
    sourceCalls=calls(accepted,:);information=zeros(3,3,height(sourceCalls));minimumEigenvalue=Inf;
    for k=1:height(sourceCalls)
        c=sourceCalls(k,:);
        information(:,:,k)=[c.informationXX,c.informationXY,c.informationXPsi; ...
            c.informationXY,c.informationYY,c.informationYPsi; ...
            c.informationXPsi,c.informationYPsi,c.informationPsiPsi];
        minimumEigenvalue=min(minimumEigenvalue,min(eig(information(:,:,k))));
    end
    strictCfg=improvedObserverConfig("lidar","mncav");strictOutcome="completed";
    try
        reconstructContinuousObserverSignals(raw.highRate,poseTime,pose,information,strictCfg);
    catch exception
        strictOutcome=string(exception.identifier)+": "+string(exception.message);
    end
    [data,reconstruction]=reconstructContinuousObserverSignals(raw.highRate,poseTime,pose,information,cfg, ...
        MaximumGap=options.MaximumOfflineGap);
    t=data.highRate.time;[covered,index]=ismember(t,lateral.time);assert(all(covered));
    lateralInput=struct('time',t,'lateralVelocity',lateral.lateralVelocity(index), ...
        'sideSlipAngle',lateral.sideSlipAngle(index),'sideSlipAngleRate',lateral.sideSlipAngleRate(index));
    % One biased reference initialization shared with recursive matching.
    % Propagate it to the global start using only recorded motion and lateral output.
    p0=[calls.referenceX(1),calls.referenceY(1),calls.referencePsi(1)]+matching.metadata.initialOffset;
    initialMotion=integrateRecordedPlanarMotion(raw.highRate.time, ...
        [raw.highRate.longitudinalSpeed,lateral.lateralVelocity,raw.highRate.yawRate],[raw.highRate.time(1);t(1)]);
    R=[cos(p0(3)),-sin(p0(3));sin(p0(3)),cos(p0(3))];
    p0=[p0(1:2)+initialMotion(end,1:2)*R.',p0(3)+initialMotion(end,3)];
    R=[cos(p0(3)),-sin(p0(3));sin(p0(3)),cos(p0(3))];
    velocity=R*[data.highRate.longitudinalSpeed(1);lateralInput.lateralVelocity(1)];
    initial=[p0(1);velocity(1);0;p0(2);velocity(2);0;p0(3)];cfg.observer.initialState=initial;
    timer=tic;estimate=runImprovedVehicleObserver(data,struct(),design,cfg, ...
        LateralInputs=lateralInput,InitialHistory=@(~) initial);globalSeconds=toc(timer);
    referencePose=interp1(reference.time,[reference.x,reference.y,reference.psi],t,'linear');
    error=estimate.pose-referencePose;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
    observerMetrics=poseMetrics(error);
    afterStartup=poseMetrics(error(t>=t(1)+5,:));
    interpolatedPose=interp1(poseTime,[pose(:,1:2),unwrap(pose(:,3))],t,'linear');
    reconstructionError=interpolatedPose-referencePose;
    reconstructionError(:,3)=atan2(sin(reconstructionError(:,3)),cos(reconstructionError(:,3)));
    common=isfinite(reconstructionError(:,1));reconstructionMetrics=poseMetrics(reconstructionError(common,:));
    nextPose=interp1(poseTime,poseTime,t-cfg.measurement.fixedLidarDelay,'next');
    reconstruction.maximumLookaheadBeyondCurrentTimeSeconds=max(0,max(nextPose-t));
    reconstruction.samplesRequiringFutureCaptureEvenWithZeroProcessing=nnz(nextPose>t+1e-12);
    reconstruction.strictDefaultOutcome=strictOutcome;
    summary=struct('completed',true,'samples',numel(t),'startSeconds',t(1),'endSeconds',t(end), ...
        'observer',observerMetrics,'observerAfterFiveSeconds',afterStartup, ...
        'offlineMatchedPoseInterpolation',reconstructionMetrics,'matching',matching.summary, ...
        'lateralSynthesisSeconds',lateralSynthesisSeconds,'globalSynthesisSeconds',globalSynthesisSeconds, ...
        'lateralRunSeconds',lateralSeconds,'globalRunSeconds',globalSeconds, ...
        'minimumActualInformationEigenvalue',minimumEigenvalue, ...
        'informationScale',cfg.lidar.gainInformationScale, ...
        'minimumActualPoseWeight',minimumEigenvalue/(minimumEigenvalue+cfg.lidar.gainInformationScale), ...
        'lateralDesignGridPassed',lateralDesign.certified,'globalMatrixCertificatePassed',design.certified, ...
        'globalUniformMargin',design.verification.uniformMargin, ...
        'maximumSuppliedCourseRate',max(abs(raw.highRate.yawRate+lateral.sideSlipAngleRate)), ...
        'outsideCourseRateEnvelope',estimate.diagnostics.anyStageOutsideTrackRateEnvelope, ...
        'matchingCallsExceedingAssumedDelay',nnz(calls.totalMs>1000*cfg.measurement.fixedLidarDelay), ...
        'matchingReusedFromThisExperiment',matchingReused,'directionalMatchesExcluded',nnz(calls.directionalAccepted), ...
        'referenceUsedForMap',true,'onlineCausalityClaimed',false,'physicalCascadeStabilityClaimed',false);
    metadata.matching=matching.metadata;metadata.reconstruction=reconstruction;
    metadata.lateralInput="Actual nominal MnCAV hybrid lateral observer, run from sequence start; cropped outputs passed without reset.";
    metadata.initialization="One biased reference initialization; motion-only propagation to global start; constant estimated history.";
    metadata.information="Original matcher information retained; lambda="+string(cfg.lidar.gainInformationScale)+ ...
        " is an explicitly selected gain-shaping parameter, not covariance calibration.";
    metadata.experiment="Offline feed-forward cascade: lateral -> recursive map matching -> continuous global observer. Global estimates do not feed back into matching.";
    report=struct('summary',summary,'metadata',metadata);
    save(fullfile(outputFolder,'full_experiment.mat'),'report','lateralCfg','lateralDesign','cfg','design', ...
        'raw','lateral','data','estimate','referencePose','interpolatedPose','error','-v7.3');
    writetable(array2table([t,estimate.pose,referencePose,error,interpolatedPose], ...
        'VariableNames',{'time','x','y','psi','referenceX','referenceY','referencePsi', ...
        'errorX','errorY','errorPsi','interpolatedMatchX','interpolatedMatchY','interpolatedMatchPsi'}),fullfile(outputFolder,'trajectory.csv'));
    writetable(array2table([lateral.time,lateral.lateralVelocity,lateral.sideSlipAngle,lateral.sideSlipAngleRate, ...
        lateral.diagnostics.dynamicParticipation,lateral.gainNorm], ...
        'VariableNames',{'time','lateralVelocity','sideSlipAngle','sideSlipAngleRate','dynamicParticipation','gainNorm'}),fullfile(outputFolder,'lateral.csv'));
    gains=struct('lateralSpeedGrid',lateralDesign.grid.speeds,'lateralAccelerationGrid',lateralDesign.grid.accelerations, ...
        'lateralGains',lateralDesign.gains,'lateralMaxCertificateMargin',lateralDesign.maxCertificateMargin, ...
        'theta',cfg.observer.theta,'K',design.K,'N',design.N,'P',design.P,'Q',design.Q,'R',design.R,'g',design.g, ...
        'globalVerification',design.verification);
    writeJson(fullfile(outputFolder,'gains.json'),gains);writeJson(fullfile(outputFolder,'summary.json'),summary);
    writeJson(fullfile(outputFolder,'metadata.json'),metadata);
    plotResults(t,error,estimate.pose,referencePose,lateral,outputFolder);
    disp(summary);
end

function metrics=poseMetrics(error)
    position=vecnorm(error(:,1:2),2,2);heading=rad2deg(error(:,3));
    metrics=struct('positionRmseM',rms(position),'positionMedianM',median(position), ...
        'positionP95M',prctile(position,95),'positionMaximumM',max(position), ...
        'headingRmseDeg',rms(heading),'headingMaximumAbsDeg',max(abs(heading)));
end

function writeJson(path,value)
    fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end

function plotResults(t,error,pose,reference,lateral,folder)
    fig=figure('Name','MnCAV full recorded localization','Color','w','Position',[100,80,1350,800]);
    tiledlayout(fig,2,2,'TileSpacing','compact');origin=reference(1,1:2);
    nexttile;plot(reference(:,1)-origin(1),reference(:,2)-origin(2),'k-');hold on;
    plot(pose(:,1)-origin(1),pose(:,2)-origin(2));axis equal;grid on;
    xlabel('Easting from start (m)');ylabel('Northing from start (m)');legend('Mapping reference','Global observer');
    title('Same-drive map: offline cascade');
    nexttile;plot(t,vecnorm(error(:,1:2),2,2));grid on;xlabel('Receiver time (s)');ylabel('Position error (m)');
    nexttile;plot(t,rad2deg(error(:,3)));grid on;xlabel('Receiver time (s)');ylabel('Heading error (deg)');
    nexttile;plot(lateral.time,lateral.lateralVelocity);grid on;xlabel('Receiver time (s)');ylabel('Estimated lateral velocity (m/s)');
    exportgraphics(fig,fullfile(folder,'localization.png'),'Resolution',180);
    exportgraphics(fig,fullfile(folder,'localization.pdf'),'ContentType','vector');
end
