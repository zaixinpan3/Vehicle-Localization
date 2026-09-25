function report = evaluateLateralPerformance()
% evaluateLateralPerformance Reproducible current-working-tree evaluation.
% Synthetic truth is scored at the configured output point. Plant parameter
% perturbations change truth only; observer gains and parameters stay fixed.
% Raw INSPVA velocity is used only for scoring the two recorded drives.
    root = setupVehicleLocalization();
    destination = fileparts(mfilename('fullpath'));
    traceFolder = fullfile(root,'output','lateral_performance_20260925');
    if ~isfolder(traceFolder), mkdir(traceFolder); end
    previousRng = rng;
    cleanup = onCleanup(@() rng(previousRng));
    rows = struct([]); certificates = struct([]); traces = {};
    for profile = ["reference","mncav"]
        cfg = lateralObserverConfig(profile);
        filename = 'lateralObserverDesign.mat';
        if profile == "mncav", filename = 'mncavLateralObserverDesign.mat'; end
        stored = load(fullfile(root,'tests','reference',filename),'design');
        design = stored.design;
        assert(isequal(design.model.vehicle,cfg.vehicle));
        certificates = [certificates; denseCertificate(design,profile)]; %#ok<AGROW>
        cases = ["noiseless","nominal","noise5x","large_initial", ...
            "tires07","tires13","mass12","ay_bias02","yaw_bias001", ...
            "dynamic_dropout","speed5p1","speed29p9","stop_go","overspeed"];
        for name = cases
            seeds = 2026;
            if name == "nominal", seeds = 2026:2045; end
            if name == "noise5x", seeds = 2026:2035; end
            for seed = seeds
                c = cfg; c.simulation.randomSeed = seed; truthDesign = design;
                switch name
                    case "noiseless"
                        c.simulation.lateralAccelerationNoiseStd = 0;
                        c.simulation.yawRateNoiseStd = 0;
                    case "noise5x"
                        c.simulation.lateralAccelerationNoiseStd = .25;
                        c.simulation.yawRateNoiseStd = .01;
                    case "large_initial", c.simulation.initialStateError = [3;.3];
                    case {"tires07","tires13","mass12"}
                        v = cfg.vehicle;
                        if name == "mass12"
                            v.mass = 1.2*v.mass;
                        else
                            factor = .7;
                            if name == "tires13", factor = 1.3; end
                            v.frontCorneringStiffness = factor*v.frontCorneringStiffness;
                            v.rearCorneringStiffness = factor*v.rearCorneringStiffness;
                        end
                        truthDesign.model = lateralBicycleModel(v);
                    case {"speed5p1","speed29p9"}
                        c.simulation.accelerationAmplitude = 0;
                        c.simulation.initialSpeed = 5.1;
                        if name == "speed29p9", c.simulation.initialSpeed = 29.9; end
                end
                simulated = simulateLateralObserverScenario(truthDesign,c);
                truth = simulated.truth; u = simulated.measurements;
                if name == "ay_bias02", u.lateralAcceleration = u.lateralAcceleration+.2; end
                if name == "yaw_bias001", u.yawRate = u.yawRate+.01; end
                if name == "dynamic_dropout", u.dynamicValid = ~(u.time>=8 & u.time<16); end
                if name == "stop_go" || name == "overspeed"
                    [u,truth] = straightScenario(c,name);
                end
                c.observer.initialState = truth.state(1,:).'+c.simulation.initialStateError;
                timer = tic;
                estimate = runLateralVelocityObserver(u,design,c);
                elapsed = toc(timer);
                row = scoreRun(profile,name,seed,truth,estimate,c,elapsed);
                rows = [rows; row]; %#ok<AGROW>
                if seed == 2026
                    traces{end+1} = struct('profile',profile,'name',name, ...
                        'truth',truth,'estimate',estimate,'cfg',c); %#ok<AGROW>
                end
            end
        end
        fprintf('%s: completed synthetic campaign\n',profile);
    end
    synthetic = struct2table(rows); certificate = struct2table(certificates);
    writetable(synthetic,fullfile(destination,'synthetic_metrics.csv'));
    writetable(certificate,fullfile(destination,'dense_certificate.csv'));
    [recorded,recordedTraces] = recordedEvaluation(root);
    writetable(recorded,fullfile(destination,'recorded_metrics.csv'));
    [integration,integrationTraces] = integrationEvaluation();
    writetable(integration,fullfile(destination,'integration_metrics.csv'));
    report = struct('matlab',version,'syntheticRuns',height(synthetic), ...
        'syntheticFinite',all(synthetic.finite),'recordedRows',height(recorded), ...
        'seedRange',[2026 2045],'certificate',certificate, ...
        'scope',"Finite empirical evaluation; dense grid is not a continuous-domain proof; LPV certificate does not certify the hybrid wrapper");
    save(fullfile(traceFolder,'experiment.mat'),'report','traces','recordedTraces', ...
        'integrationTraces','synthetic','recorded','certificate','integration','-v7.3');
    plotResults(traces,recordedTraces,destination);
    disp(certificate); disp(recorded); disp(integration);
end

function row = denseCertificate(d,profile)
    margins = []; frozen = []; noise = []; minP = Inf;
    for v = linspace(5,30,1001)
        [L,P] = scheduleLateralObserverGain(d,v);
        [A,C] = evaluateLateralModel(d.model,[v;1/v]); F = A-L*C;
        frozen(end+1) = max(real(eig(F))); %#ok<AGROW>
        minP = min(minP,min(eig((P+P')/2)));
        for a = [-3 0 3]
            [~,rate] = schedulingCoordinates(d.polytope,v,a);
            Pdot = sum(d.lyapunovBasis.*reshape(rate,1,1,3),3);
            Q = P*F+F'*P+Pdot+2*d.decayRate*P+d.tau*(P*P) ...
                +(d.lipschitzConstant^2/d.tau)*eye(2);
            margins(end+1) = max(eig((Q+Q')/2)); %#ok<AGROW>
            N = [Q -P*L;-L'*P -d.issGain^2*eye(2)];
            noise(end+1) = max(eig((N+N')/2)); %#ok<AGROW>
        end
    end
    row = struct('profile',profile,'gridPoints',numel(margins),'minimumP',minP, ...
        'maxPsiEigenvalue',max(margins),'positivePsiPoints',nnz(margins>=0), ...
        'maxNoiseBlockEigenvalue',max(noise),'positiveNoisePoints',nnz(noise>1e-9), ...
        'maxFrozenEigenvalue',max(frozen),'decayRate',d.decayRate,'issGain',d.issGain);
end

function [u,t] = straightScenario(c,name)
    time = (0:.01:24)'; n = numel(time); speed = zeros(n,1); acceleration = speed;
    if name == "overspeed"
        speed(:) = 32;
    else
        rising = time>=3 & time<9; cruise = time>=9 & time<15; falling = time>=15 & time<21;
        speed(rising) = 4*(1-cos(pi*(time(rising)-3)/6));
        acceleration(rising) = 4*pi/6*sin(pi*(time(rising)-3)/6);
        speed(cruise) = 8;
        speed(falling) = 4*(1+cos(pi*(time(falling)-15)/6));
        acceleration(falling) = -4*pi/6*sin(pi*(time(falling)-15)/6);
    end
    rng(c.simulation.randomSeed,'twister'); z = zeros(n,1);
    u = struct('time',time,'longitudinalSpeed',speed,'longitudinalAcceleration', ...
        acceleration,'steeringAngle',z,'yawRate',.002*randn(n,1), ...
        'lateralAcceleration',.1+.05*randn(n,1));
    t = struct('time',time,'longitudinalSpeed',speed,'lateralVelocity',z, ...
        'yawRate',z,'state',zeros(n,2));
end

function row = scoreRun(profile,name,seed,t,e,c,elapsed)
    vy = t.lateralVelocity-c.outputPoint.forwardOffsetM*t.yawRate;
    error = e.lateralVelocity-vy; post = t.time>=6;
    beta = atan2(vy,t.longitudinalSpeed); validBeta = post & t.longitudinalSpeed>=6;
    firstSettled = NaN;
    % Settling means |vy error| <= 0.05 m/s for the entire remaining record.
    lastOutside = find(abs(error)>.05,1,'last');
    if isempty(lastOutside)
        firstSettled = 0;
    elseif lastOutside<numel(error)
        firstSettled = t.time(lastOutside+1);
    end
    row = struct('profile',profile,'scenario',name,'seed',seed,'samples',numel(error), ...
        'rmseAllMps',rms(error),'rmsePost6Mps',rms(error(post)), ...
        'biasPost6Mps',mean(error(post)),'p95Post6Mps',prctile(abs(error(post)),95), ...
        'maxPost6Mps',max(abs(error(post))),'peakAllMps',max(abs(error)), ...
        'settling005Seconds',firstSettled, ...
        'yawRmsePost6Radps',rms(e.yawRate(post)-t.yawRate(post)), ...
        'betaRmsePost6Deg',rad2deg(rms(e.sideSlipAngle(validBeta)-beta(validBeta))), ...
        'dynamicParticipationMean',mean(e.diagnostics.dynamicParticipation), ...
        'finite',all(isfinite([e.state,e.dynamicState,e.sideSlipAngle,e.sideSlipAngleRate]),'all'), ...
        'runtimeSeconds',elapsed,'meanMicrosecondsPerSample',1e6*elapsed/numel(error));
end

function [metrics,traces] = recordedEvaluation(root)
    saved = load(fullfile(root,'tests','reference','mncavLateralObserverDesign.mat'),'design');
    cfg = lateralObserverConfig("mncav");
    parameters = jsondecode(fileread(fullfile(root,'output','mncav_interface_audit_20260916','vehicle_parameters.json')));
    drives = {"12-11-24","output/mncav_wheel_only_20260916/calibration_sensors", ...
        "output/mncav_wheel_only_20260916/calibration_sensors/inspva.csv"; ...
        "12-09-31","output/mncav_wheel_only_20260916/sensors", ...
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv"};
    rows = struct([]); traces = cell(2,1);
    for j = 1:2
        clock = loadReceiverClock(fullfile(root,drives{j,3}));
        ins = readtable(fullfile(root,drives{j,2},'inspva.csv'));
        insTime = receiverClockTime(clock,ins.stamp_sec);
        start = insTime(1)+.5; stop = insTime(end)-.5;
        high = prepareWheelMotionInputs(fullfile(root,drives{j,2}),parameters,clock,start,stop);
        timer = tic; e = runLateralVelocityObserver(high,saved.design,cfg); elapsed = toc(timer);
        yaw = deg2rad(90-ins.azimuth);
        reference = interp1(insTime-start, ...
            [ins.east_velocity.*cos(yaw)+ins.north_velocity.*sin(yaw), ...
            -ins.east_velocity.*sin(yaw)+ins.north_velocity.*cos(yaw)],high.time,'linear');
        assert(all(isfinite(reference),'all'));
        masks = {true(size(high.time)),reference(:,1)>1, ...
            high.longitudinalSpeed>=5 & abs(high.yawRate)<.03, ...
            high.longitudinalSpeed>=5 & abs(high.yawRate)>=.03,high.time>40};
        names = ["all","moving_gt1","straight_ge5","turning_ge5","after40"];
        for k = 1:numel(names)
            use = masks{k}; error = e.lateralVelocity(use)-reference(use,2);
            demeaned = error-mean(error);
            betaValid = use & reference(:,1)>=6;
            betaError = e.sideSlipAngle(betaValid)-atan2(reference(betaValid,2),reference(betaValid,1));
            row = struct('drive',drives{j,1},'population',names(k), ...
                'samples',nnz(use),'rmseMps',rms(error),'biasMps',mean(error), ...
                'demeanedRmseMps',rms(demeaned),'p95Mps',prctile(abs(error),95), ...
                'maximumMps',max(abs(error)),'betaRmseDeg',rad2deg(rms(betaError)), ...
                'zeroVyBaselineRmseMps',rms(reference(use,2)), ...
                'observerPointRmseMps',rms(e.observerPointLateralVelocity(use)-reference(use,2)), ...
                'meanDynamicParticipation',mean(e.diagnostics.dynamicParticipation(use)), ...
                'runtimeSeconds',elapsed,'microsecondsPerSample',1e6*elapsed/numel(high.time));
            rows = [rows; row]; %#ok<AGROW>
        end
        traces{j} = struct('drive',drives{j,1},'estimate',e,'reference',reference,'high',high);
    end
    metrics = struct2table(rows);
end

function [metrics,traces] = integrationEvaluation()
    saved = load('tests/reference/lateralObserverDesign.mat','design');
    cfg = lateralObserverConfig(); cfg.simulation.sampleTime = .001;
    cfg.simulation.lateralAccelerationNoiseStd = 0; cfg.simulation.yawRateNoiseStd = 0;
    fine = simulateLateralObserverScenario(saved.design,cfg);
    rows = struct([]); traces = {};
    for dt = [.005 .01 .02 .05 .1 .2]
        use = 1:round(dt/.001):numel(fine.truth.time);
        fields = fieldnames(fine.measurements); u = struct();
        for k = 1:numel(fields), u.(fields{k}) = fine.measurements.(fields{k})(use); end
        c = cfg; c.observer.initialState = cfg.simulation.initialStateError;
        status = "completed"; message = ""; err = NaN; peak = NaN;
        try
            e = runLateralVelocityObserver(u,saved.design,c);
            error = e.lateralVelocity-fine.truth.lateralVelocity(use);
            err = rms(error(u.time>=6)); peak = max(abs(error));
            traces{end+1} = struct('dt',dt,'estimate',e); %#ok<AGROW>
        catch failure
            status = "error"; message = string(failure.message);
        end
        row = struct('sampleTimeSeconds',dt,'status',status, ...
            'rmsePost6Mps',err,'peakAllMps',peak,'message',message);
        rows = [rows; row]; %#ok<AGROW>
    end
    metrics = struct2table(rows);
end

function plotResults(traces,recorded,destination)
    fig = figure('Visible','off','Position',[100 100 1100 800]);
    cleanup = onCleanup(@() close(fig));
    tiledlayout(2,2);
    nexttile; hold on;
    for k = 1:numel(traces)
        s = traces{k};
        if s.profile=="reference" && any(s.name==["noiseless","nominal","noise5x"])
            plot(s.truth.time,s.estimate.lateralVelocity-s.truth.lateralVelocity,'DisplayName',s.name);
        end
    end
    xlabel('Time (s)'); ylabel('Lateral velocity error (m/s)'); title('Synthetic convergence'); legend; grid on;
    nexttile; hold on;
    for k = 1:numel(traces)
        s = traces{k};
        if s.profile=="reference" && any(s.name==["tires07","tires13","yaw_bias001","dynamic_dropout"])
            plot(s.truth.time,s.estimate.lateralVelocity-s.truth.lateralVelocity,'DisplayName',s.name);
        end
    end
    xlabel('Time (s)'); ylabel('Lateral velocity error (m/s)'); title('Mismatch and channel dropout'); legend; grid on;
    for j = 1:2
        nexttile; s = recorded{j};
        plot(s.high.time,s.reference(:,2),s.high.time,s.estimate.lateralVelocity);
        xlabel('Receiver-relative time (s)'); ylabel('Lateral velocity (m/s)');
        title("Recorded drive "+s.drive); legend('INSPVA reference','Current observer'); grid on;
    end
    exportgraphics(fig,fullfile(destination,'performance.png'),'Resolution',150);
end
