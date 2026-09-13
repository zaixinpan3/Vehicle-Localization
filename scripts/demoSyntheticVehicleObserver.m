function report = demoSyntheticVehicleObserver(outputFolder)
% demoSyntheticVehicleObserver Run both real observer stages on synthetic data.
% From the project root: setupVehicleLocalization; demoSyntheticVehicleObserver
% No bag, map, registration, optimizer, or identified vehicle is needed.
% Four 40 s experiments compare analytic steady-turn truth with the existing
% GNSS and delayed LiDAR observers, with and without bounded sensor errors.
% The plant and observer share nominal bicycle parameters (no mismatch test).
    arguments
        outputFolder (1,1) string = "output/synthetic_vehicle_observer"
    end
    root = setupVehicleLocalization;
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    stored = load(fullfile(root,'tests','reference','lateralObserverDesign.mat'),'design');
    lateralDesign = stored.design;
    current = lateralObserverConfig;
    lateralDesign.cfg.hybrid = current.hybrid;
    results = cell(4,1);
    rows = cell(4,1);
    index = 0;
    for mode = ["gnss","lidar"]
        for noisy = [false,true]
            index = index + 1;
            fprintf('Running %s, noisy=%d ...\n',mode,noisy);
            results{index} = runScenario(mode,noisy,lateralDesign);
            rows{index} = results{index}.metrics;
            drawScenario(results{index},outputFolder);
        end
    end
    report.metrics = struct2table([rows{:}]);
    report.vehicle = lateralDesign.cfg.vehicle;
    report.scope = "Nominal synthetic cascade; continuous reconstructed sensors; no real-data or general robustness claim.";
    report.allPassed = all(report.metrics.passed);
    % Repeat the noisy cases with half the global integration step; the same
    % lateral stage, noise functions and initial history are retained.
    refinement = cell(2,1);
    for k = 1:2
        result = results{2*k};
        refinedCfg = result.cfg;
        refinedCfg.measurement.maximumIntegrationStep = .0025;
        lateralDesign.cfg = result.lateralCfg;
        refined = runImprovedVehicleObserver(result.sensorData,lateralDesign, ...
            improvedObserverReferenceDesign(refinedCfg),refinedCfg, ...
            InitialHistory=result.initialHistory);
        refinement{k} = struct('mode',result.mode, ...
            'maximumAbsoluteStateDifference',max(abs(refined.z-result.estimate.z),[],1));
    end
    report.refinement = [refinement{:}];
    writetable(report.metrics,fullfile(outputFolder,'metrics.csv'));
    save(fullfile(outputFolder,'traces.mat'),'results','report','-v7.3');
    fid = fopen(fullfile(outputFolder,'summary.json'),'w');
    assert(fid>=0,'Cannot write summary.');
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(report.metrics);
end

function result = runScenario(mode,noisy,lateralDesign)
    cfg = improvedObserverConfig(mode);
    t = (0:.01:40).';
    vx = 8;
    q = .04;
    if mode=="lidar", q = .001; end
    % Solve A*[vy;q]+B*delta=0 for a physically consistent steady turn.
    A = evaluateLateralModel(lateralDesign.model,[vx;1/vx]);
    equilibrium = [A(:,1),lateralDesign.model.B]\(-A(:,2)*q);
    vy = equilibrium(1);
    steering = equilibrium(2);
    beta = atan2(vy,vx);
    speed = hypot(vx,vy);
    truth = steadyTruth(t,speed,q,beta);
    truthAt = @(time) steadyTruth(time,speed,q,beta).';
    amplitude = double(noisy);
    high = struct('time',t,'steeringAngle',steering+amplitude*deg2rad(.005)*sin(.8*t), ...
        'longitudinalSpeed',vx+amplitude*.02*sin(1.1*t), ...
        'longitudinalAcceleration',-vy*q+amplitude*.02*sin(1.7*t), ...
        'lateralAcceleration',vx*q+amplitude*.02*cos(1.4*t), ...
        'yawRate',q+amplitude*.0002*sin(.6*t));
    positionAmplitude = amplitude*.05;
    if mode=="lidar", positionAmplitude = amplitude*.01; end
    headingAmplitude = amplitude*deg2rad(.1);
    data.highRate = high;
    if mode=="gnss"
        data.gnss.evaluate = @(time) positionSignal(time,truthAt,positionAmplitude);
    else
        data.lidar = struct('delay',cfg.measurement.fixedLidarDelay, ...
            'headingConvention',"unwrapped",'evaluate', ...
            @(time) lidarSignal(time,truthAt,cfg,positionAmplitude,headingAmplitude));
    end
    initialError = [1;.3;.1;-1;-.2;-.1;deg2rad(10)];
    cfg.observer.initialState = truth(1,:).' + initialError;
    % The history is an imperfect prior, never an observer correction input.
    history = @(time) truthAt(time)+initialError;
    lateralDesign.cfg.observer.initialState = [vy+.3;q+.01];
    lateralDesign.cfg.hybrid.initialMasterState = [vy+.3;0];
    lateralDesign.cfg.hybrid.sideSlip.initialState = [beta;0];
    design = improvedObserverReferenceDesign(cfg);
    estimate = runImprovedVehicleObserver(data,lateralDesign,design,cfg,InitialHistory=history);
    e = estimate.z-truth;
    settled = t>=20;
    positionError = vecnorm(e(:,[1,4]),2,2);
    velocityError = vecnorm(e(:,[2,5]),2,2);
    accelerationError = vecnorm(e(:,[3,6]),2,2);
    headingError = rad2deg(abs(e(:,7)));
    lateralError = estimate.lateral.lateralVelocity-vy;
    % Practical acceptance limits fixed before execution, applied over 20-40 s.
    limits = struct('positionRmseM',.15,'velocityRmseMps',.15, ...
        'accelerationRmseMps2',.15,'headingRmseDeg',1,'lateralVelocityRmseMps',.05);
    metrics = struct('mode',mode,'noisy',noisy, ...
        'positionRmseM',rms(positionError(settled)), ...
        'velocityRmseMps',rms(velocityError(settled)), ...
        'accelerationRmseMps2',rms(accelerationError(settled)), ...
        'headingRmseDeg',rms(headingError(settled)), ...
        'lateralVelocityRmseMps',rms(lateralError(settled)), ...
        'initialPositionErrorM',positionError(1),'finalPositionErrorM',positionError(end), ...
        'initialHeadingErrorDeg',headingError(1),'finalHeadingErrorDeg',headingError(end), ...
        'peakPositionErrorM',max(positionError),'peakVelocityErrorMps',max(velocityError), ...
        'peakAccelerationErrorMps2',max(accelerationError), ...
        'peakHeadingErrorDeg',max(headingError), ...
        'matrixCertificateVerified',estimate.observer.certificateVerified, ...
        'anyRateOutsideCertificate',estimate.diagnostics.anyStageOutsideTrackRateEnvelope, ...
        'passed',all(isfinite(estimate.z),'all'));
    for name = string(fieldnames(limits)).'
        metrics.passed = metrics.passed && metrics.(name)<=limits.(name);
    end
    within = positionError<=limits.positionRmseM & velocityError<=limits.velocityRmseMps ...
        & accelerationError<=limits.accelerationRmseMps2 & headingError<=limits.headingRmseDeg ...
        & abs(lateralError)<=limits.lateralVelocityRmseMps;
    lastOutside = find(~within,1,'last');
    metrics.settlingTimeSec = NaN;
    if isempty(lastOutside)
        metrics.settlingTimeSec = t(1);
    elseif lastOutside<numel(t)
        metrics.settlingTimeSec = t(lastOutside+1);
    end
    result = struct('mode',mode,'noisy',noisy,'time',t,'truth',truth, ...
        'trueLateralVelocity',vy,'sensorData',data,'estimate',estimate, ...
        'metrics',metrics,'limits',limits,'cfg',cfg,'lateralCfg',lateralDesign.cfg, ...
        'initialHistory',history, ...
        'parameters',struct('longitudinalSpeedMps',vx,'yawRateRadps',q, ...
        'steeringRad',steering,'sideSlipRad',beta,'positionNoiseAmplitudeM',positionAmplitude, ...
        'headingNoiseAmplitudeRad',headingAmplitude,'noiseKind',"Deterministic bounded sinusoids; no RNG"));
end

function z = steadyTruth(t,speed,q,beta)
    yaw = .2+q*t(:);
    course = yaw+beta;
    course0 = .2+beta;
    x = speed/q*(sin(course)-sin(course0));
    y = speed/q*(cos(course0)-cos(course));
    z = [x,speed*cos(course),-speed*q*sin(course), ...
         y,speed*sin(course),speed*q*cos(course),yaw];
end

function position = positionSignal(t,truthAt,amplitude)
    z = truthAt(t);
    position = z([1,4])+amplitude*[sin(1.3*t);cos(.9*t)];
end

function measurement = lidarSignal(t,truthAt,cfg,positionAmplitude,headingAmplitude)
    z = truthAt(t-cfg.measurement.fixedLidarDelay);
    measurement = struct('pose',z([1,4,7])+[positionAmplitude*sin(1.3*t); ...
        positionAmplitude*cos(.9*t);headingAmplitude*sin(.7*t)],'information',1e6*eye(3));
end

function drawScenario(result,outputFolder)
    label = result.mode+"_clean";
    if result.noisy, label = result.mode+"_noisy"; end
    t = result.time;
    z = result.truth;
    estimate = result.estimate;
    fig = figure('Name',char(label),'Color','w','Position',[80,80,1250,820]);
    tiledlayout(fig,3,3,'TileSpacing','compact');
    nexttile; plot(z(:,1),z(:,4),'k-',estimate.z(:,1),estimate.z(:,4),'r--');
    axis equal; grid on; xlabel('X (m)'); ylabel('Y (m)'); title('Trajectory');
    legend('Truth','Estimate','Location','best');
    labels = ["X (m)","V_X (m/s)","A_X (m/s^2)","Y (m)", ...
        "V_Y (m/s)","A_Y (m/s^2)","Yaw (deg)"];
    for k = 1:7
        nexttile;
        scale = 1;
        if k==7, scale = 180/pi; end
        plot(t,scale*z(:,k),'k-',t,scale*estimate.z(:,k),'r--');
        xlabel('Time (s)'); ylabel(labels(k)); grid on;
    end
    nexttile;
    plot(t,result.trueLateralVelocity*ones(size(t)),'k-',t,estimate.lateral.lateralVelocity,'r--');
    xlabel('Time (s)'); ylabel('Body v_y (m/s)'); grid on;
    sgtitle(strrep(label,'_',' ')+' | actual lateral + global observer');
    exportgraphics(fig,fullfile(outputFolder,label+'.png'),'Resolution',140);
    exportgraphics(fig,fullfile(outputFolder,label+'.pdf'),'ContentType','vector');
end
