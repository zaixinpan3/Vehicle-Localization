function report = evaluateLateralObserverPerformance()
% evaluateLateralObserverPerformance Simulation and recorded-drive evaluation
% of the current MnCAV lateral-velocity observer. The design is synthesized
% from the current configuration; nothing is tuned, refitted, or saved back.
%
% Synthetic truth comes from an evaluator-local bicycle plant, not from the
% observer's own model code, so plant parameters, tire saturation, the CG
% location, and every sensor error can differ from what the observer
% assumes. Truth is scored at the configured output point. The recorded
% drives are scored against the INSPVA body-frame lateral velocity, which
% never enters the observer.
%
% Input:
%   none
%
% Output:
%   report: struct with the design summary and every metric table. Compact
%       CSV/PNG exports are written beside this file; full traces go to
%       output/lateral_observer_performance_20261001/experiment.mat.
    root = setupVehicleLocalization();
    destination = fileparts(mfilename('fullpath'));
    traceFolder = fullfile(root,'output','lateral_observer_performance_20261001');
    if ~isfolder(traceFolder), mkdir(traceFolder); end
    previousRng = rng;
    restoreRng = onCleanup(@() rng(previousRng));

    cfg = lateralObserverConfig();
    design = designLateralObserverGains(cfg);
    designTable = designSummary(design);
    gainSchedule = gainScheduleTable(design);
    harness = harnessCheck(design,cfg);
    fprintf('design and harness checks completed\n');

    [synthetic,syntheticTraces] = syntheticCampaign(design,cfg);
    summary = summarizeSynthetic(synthetic);
    fprintf('synthetic campaign completed: %d runs\n',height(synthetic));
    [convergence,convergenceTraces] = convergenceStudy(design,cfg);
    integration = integrationStudy(design,cfg);
    [recorded,diagnostics,windows,recordedTraces] = recordedEvaluation(root,design,cfg);
    sensitivity = decayRateStudy(cfg,recordedTraces);
    hashes = inputHashes(root);

    writetable(sensitivity,fullfile(destination,'decay_rate_sensitivity.csv'));
    writetable(designTable,fullfile(destination,'design_summary.csv'));
    writetable(gainSchedule,fullfile(destination,'gain_schedule.csv'));
    writetable(harness,fullfile(destination,'harness_check.csv'));
    writetable(synthetic,fullfile(destination,'synthetic_metrics.csv'));
    writetable(summary,fullfile(destination,'synthetic_summary.csv'));
    writetable(convergence,fullfile(destination,'convergence.csv'));
    writetable(integration,fullfile(destination,'integration_metrics.csv'));
    writetable(recorded,fullfile(destination,'recorded_metrics.csv'));
    writetable(diagnostics,fullfile(destination,'recorded_diagnostics.csv'));
    writetable(windows,fullfile(destination,'recorded_windows.csv'));
    writetable(hashes,fullfile(destination,'input_hashes.csv'));

    report = struct('matlab',version,'design',designTable,'gainSchedule',gainSchedule, ...
        'harness',harness,'synthetic',synthetic,'syntheticSummary',summary, ...
        'convergence',convergence,'integration',integration,'recorded',recorded, ...
        'recordedDiagnostics',diagnostics,'recordedWindows',windows, ...
        'decayRateSensitivity',sensitivity, ...
        'scope',"Finite empirical evaluation of the working-tree observer; the dense certificate grid is not a continuous-domain proof and does not certify the hybrid wrapper.");
    save(fullfile(traceFolder,'experiment.mat'),'report','cfg','design','syntheticTraces', ...
        'convergenceTraces','recordedTraces','hashes','-v7.3');
    plotSimulation(syntheticTraces,convergenceTraces,summary,design,destination);
    plotRecorded(recordedTraces,destination);
    disp(designTable); disp(summary); disp(convergence); disp(integration);
    disp(recorded); disp(diagnostics); disp(windows); disp(sensitivity);
end

%% Design and certificate

function summary = designSummary(design)
% designSummary Report the synthesis result and an independent dense check
% of the original certificate on 1,001 speeds and three accelerations.
    margins = []; noise = []; frozen = []; minimumP = Inf;
    speedRange = design.polytope.speedRange;
    for speed = linspace(speedRange(1),speedRange(2),1001)
        [L,P] = scheduleLateralObserverGain(design,speed);
        [A,C] = evaluateLateralModel(design.model,[speed;1/speed]);
        closedLoop = A-L*C;
        frozen(end+1) = max(real(eig(closedLoop))); %#ok<AGROW>
        minimumP = min(minimumP,min(eig((P+P.')/2)));
        for acceleration = [-3 0 3]
            [~,alphaRate] = schedulingCoordinates(design.polytope,speed,acceleration);
            PRate = sum(design.lyapunovBasis.*reshape(alphaRate,1,1,3),3);
            Psi = P*closedLoop+closedLoop.'*P+PRate+2*design.decayRate*P+design.tau*(P*P)+ ...
                (design.lipschitzConstant^2/design.tau)*eye(2);
            margins(end+1) = max(eig((Psi+Psi.')/2)); %#ok<AGROW>
            block = [Psi,-P*L;-(P*L).',-design.issGain^2*eye(2)];
            noise(end+1) = max(eig((block+block.')/2)); %#ok<AGROW>
        end
    end
    summary = table(design.tau,design.slackScale,design.decayRate,design.lipschitzConstant, ...
        design.gainBound,design.maxVertexGainNorm,design.issGain,design.maxCertificateMargin, ...
        numel(margins),max(margins),nnz(margins>=0),max(noise),nnz(noise>1e-9),minimumP,max(frozen), ...
        'VariableNames',{'tau','slackScale','decayRatePerSecond','lipschitzConstant','gainBound', ...
        'maxVertexGainNorm','issGain','designGridMaxPsiEigenvalue','denseGridPoints', ...
        'denseMaxPsiEigenvalue','densePositivePsiPoints','denseMaxNoiseBlockEigenvalue', ...
        'densePositiveNoisePoints','denseMinimumPEigenvalue','denseMaxFrozenPoleRealPart'});
end

function schedule = gainScheduleTable(design)
% gainScheduleTable Tabulate the scheduled gain and frozen poles by speed.
    speeds = [5 7.5 10 15 20 25 30].';
    rows = zeros(numel(speeds),8);
    for k = 1:numel(speeds)
        L = scheduleLateralObserverGain(design,speeds(k));
        [A,C] = evaluateLateralModel(design.model,[speeds(k);1/speeds(k)]);
        poles = sort(real(eig(A-L*C)),'descend');
        openLoop = sort(real(eig(A)),'descend');
        rows(k,:) = [L(1,1),L(1,2),L(2,1),L(2,2),norm(L),poles(1),poles(2),openLoop(1)];
    end
    schedule = array2table([speeds,rows],'VariableNames',{'speedMps','L11','L12','L21','L22', ...
        'gainNorm','slowFrozenPoleRealPart','fastFrozenPoleRealPart','slowOpenLoopPoleRealPart'});
end

function check = harnessCheck(design,cfg)
% harnessCheck Prove that the evaluator-local plant, measurement builder and
% scoring reproduce the production synthetic scenario for the nominal case.
    production = simulateLateralObserverScenario(design,cfg);
    s = scenario("harness","default_production_equivalent");
    p = buildProfile(s,cfg);
    truth = simulatePlant(p,cfg.vehicle,Inf,0,cfg.outputPoint.forwardOffsetM,1);
    u = buildMeasurements(truth,s,cfg.simulation.randomSeed,cfg);
    c = cfg; c.observer.initialState = truth.observerState(1,:).'+cfg.simulation.initialStateError(:);
    estimate = runLateralVelocityObserver(u,design,c);
    refined = simulatePlant(p,cfg.vehicle,Inf,0,cfg.outputPoint.forwardOffsetM,10);
    quantity = ["truthState";"truthLateralAcceleration";"truthOutputLateralVelocity"; ...
        "truthSideSlipAngle";"measuredLateralAcceleration";"measuredYawRate"; ...
        "estimatedLateralVelocity";"tenSubstepPlantVersusOneStep"];
    maximumDifference = [max(abs(truth.state-production.truth.state),[],'all'); ...
        max(abs(truth.imuLateralAcceleration-production.truth.lateralAcceleration)); ...
        max(abs(truth.outputLateralVelocity-production.truth.lateralVelocity)); ...
        max(abs(truth.sideSlipAngle-production.truth.sideSlipAngle)); ...
        max(abs(u.lateralAcceleration-production.measurements.lateralAcceleration)); ...
        max(abs(u.yawRate-production.measurements.yawRate)); ...
        max(abs(estimate.lateralVelocity-production.estimate.lateralVelocity)); ...
        max(abs(refined.state-truth.state),[],'all')];
    tolerance = [1e-10*ones(7,1);1e-5];
    check = table(quantity,maximumDifference,tolerance,maximumDifference<=tolerance, ...
        'VariableNames',{'quantity','maximumAbsoluteDifference','tolerance','passed'});
    assert(all(check.passed),'The evaluator plant does not reproduce the production scenario.');
end

%% Synthetic campaign

function [metrics,traces] = syntheticCampaign(design,cfg)
% syntheticCampaign Run every declared scenario and keep seed-2026 traces.
    scenarios = scenarioList();
    rows = cell(0,1); traces = cell(0,1);
    for s = scenarios.'
        for seed = s.seeds
            [row,trace] = runScenario(s,seed,design,cfg);
            rows{end+1,1} = row; %#ok<AGROW>
            if seed == s.seeds(1), traces{end+1,1} = trace; end %#ok<AGROW>
        end
    end
    metrics = struct2table(vertcat(rows{:}));
end

function scenarios = scenarioList()
% scenarioList Declare the matched, envelope, parameter, sensor and
% nonlinear-tire scenarios. Stress magnitudes follow the sensitivity entries
% of mncavVehicleParameters.json and mncavSensorParameters.json.
    few = 2026:2030;
    scenarios = [ ...
        scenario("matched","noiseless",'noiseScale',0)
        scenario("matched","nominal_noise",'seeds',2026:2045)
        scenario("matched","noise_2x",'noiseScale',2,'seeds',2026:2035)
        scenario("matched","noise_5x",'noiseScale',5,'seeds',2026:2035)
        scenario("matched","large_initial_error",'initialError',[3;0.3])
        scenario("envelope","urban_8_12_mps",'profile',"urban",'seeds',few)
        scenario("envelope","highway_25_29_mps",'profile',"highway",'seeds',few)
        scenario("envelope","constant_5p1_mps",'profile',"constant",'constantSpeed',5.1,'seeds',few)
        scenario("envelope","constant_29p9_mps",'profile',"constant",'constantSpeed',29.9,'seeds',few)
        scenario("envelope","launch_turn_stop",'profile',"launch",'seeds',few)
        scenario("envelope","overspeed_32_mps_ay_bias_0p1",'profile',"straight",'constantSpeed',32,'ayBias',0.1,'seeds',few)
        scenario("parameter","tires_x0p7",'frontFactor',0.7,'rearFactor',0.7)
        scenario("parameter","tires_x1p3",'frontFactor',1.3,'rearFactor',1.3)
        scenario("parameter","front_tires_x0p7",'frontFactor',0.7)
        scenario("parameter","front_tires_x1p3",'frontFactor',1.3)
        scenario("parameter","rear_tires_x0p7",'rearFactor',0.7)
        scenario("parameter","rear_tires_x1p3",'rearFactor',1.3)
        scenario("parameter","mass_2473_kg",'massFactor',2473/2273)
        scenario("parameter","mass_2673_kg",'massFactor',2673/2273)
        scenario("parameter","mass_2858_kg",'massFactor',2858/2273)
        scenario("parameter","yaw_inertia_x0p7",'inertiaFactor',0.7)
        scenario("parameter","yaw_inertia_x1p3",'inertiaFactor',1.3)
        scenario("parameter","cg_rearward_0p15_m",'cgShift',0.15)
        scenario("parameter","cg_forward_0p15_m",'cgShift',-0.15)
        scenario("parameter","urban_tires_x0p7",'profile',"urban",'frontFactor',0.7,'rearFactor',0.7)
        scenario("parameter","urban_tires_x1p3",'profile',"urban",'frontFactor',1.3,'rearFactor',1.3)
        scenario("sensor","ay_bias_plus_0p2",'ayBias',0.2)
        scenario("sensor","ay_bias_minus_0p2",'ayBias',-0.2)
        scenario("sensor","yaw_bias_plus_0p01",'yawBias',0.01)
        scenario("sensor","yaw_bias_minus_0p01",'yawBias',-0.01)
        scenario("sensor","speed_scale_0p98",'speedScale',0.98)
        scenario("sensor","speed_scale_1p02",'speedScale',1.02)
        scenario("sensor","steering_offset_plus_0p15_deg",'steeringOffset',deg2rad(0.15))
        scenario("sensor","steering_offset_minus_0p15_deg",'steeringOffset',deg2rad(-0.15))
        scenario("sensor","imu_delay_20_ms",'imuDelay',0.02)
        scenario("sensor","imu_delay_50_ms",'imuDelay',0.05)
        scenario("sensor","imu_delay_100_ms",'imuDelay',0.10)
        scenario("sensor","dynamic_validity_dropout_8_16_s",'dropout',[8 16])
        scenario("nonlinear_tire","sine_1p8_deg_linear",'profile',"sine",'steeringAmplitudeDeg',1.8)
        scenario("nonlinear_tire","sine_1p8_deg_mu_0p9",'profile',"sine",'steeringAmplitudeDeg',1.8,'friction',0.9)
        scenario("nonlinear_tire","sine_3p6_deg_mu_0p9",'profile',"sine",'steeringAmplitudeDeg',3.6,'friction',0.9)
        scenario("nonlinear_tire","sine_5p0_deg_mu_0p9",'profile',"sine",'steeringAmplitudeDeg',5.0,'friction',0.9)
        scenario("nonlinear_tire","sine_1p8_deg_mu_0p4",'profile',"sine",'steeringAmplitudeDeg',1.8,'friction',0.4)
        scenario("nonlinear_tire","sine_3p6_deg_mu_0p4",'profile',"sine",'steeringAmplitudeDeg',3.6,'friction',0.4)];
end

function s = scenario(group,name,varargin)
% scenario Build one scenario record; unspecified fields are nominal.
    s = struct('group',string(group),'name',string(name),'profile',"default", ...
        'seeds',2026,'noiseScale',1,'initialError',[0.5;0.05],'constantSpeed',NaN, ...
        'steeringAmplitudeDeg',0,'massFactor',1,'inertiaFactor',1,'frontFactor',1, ...
        'rearFactor',1,'cgShift',0,'friction',Inf,'ayBias',0,'yawBias',0, ...
        'speedScale',1,'steeringOffset',0,'imuDelay',0,'dropout',zeros(1,0));
    for k = 1:2:numel(varargin)
        assert(isfield(s,varargin{k}),'Unknown scenario field %s.',varargin{k});
        s.(varargin{k}) = varargin{k+1};
    end
end

function [row,trace] = runScenario(s,seed,design,cfg)
% runScenario Simulate the truth, corrupt the measurements, run the nominal
% observer, and score it at the output point.
    p = buildProfile(s,cfg);
    vehicle = cfg.vehicle;
    vehicle.mass = s.massFactor*vehicle.mass;
    vehicle.yawInertia = s.inertiaFactor*vehicle.yawInertia;
    vehicle.frontCorneringStiffness = s.frontFactor*vehicle.frontCorneringStiffness;
    vehicle.rearCorneringStiffness = s.rearFactor*vehicle.rearCorneringStiffness;
    vehicle.lf = vehicle.lf+s.cgShift; vehicle.lr = vehicle.lr-s.cgShift;
    truth = simulatePlant(p,vehicle,s.friction,s.cgShift,cfg.outputPoint.forwardOffsetM,10);
    u = buildMeasurements(truth,s,seed,cfg);
    c = cfg; c.observer.initialState = truth.observerState(1,:).'+s.initialError(:);
    timer = tic;
    estimate = runLateralVelocityObserver(u,design,c);
    elapsed = toc(timer);
    row = scoreRun(s,seed,truth,estimate,elapsed);
    trace = struct('scenario',s,'seed',seed,'truth',truth,'measurements',u,'estimate',estimate);
end

function p = buildProfile(s,cfg)
% buildProfile Return time, speed, speed derivative and road-wheel steering.
    defaultSteering = @(time) raisedCosine(time,cfg.simulation.steeringWaypointTime, ...
        deg2rad(cfg.simulation.steeringWaypointDeg));
    switch s.profile
        case "default"
            sampleTime = cfg.simulation.sampleTime;
            time = (0:round(cfg.simulation.tFinal/sampleTime)).'*sampleTime;
            rate = 2*pi/cfg.simulation.accelerationPeriod;
            amplitude = cfg.simulation.accelerationAmplitude;
            acceleration = amplitude*sin(rate*time);
            speed = cfg.simulation.initialSpeed+(amplitude/rate)*(1-cos(rate*time));
            steering = defaultSteering(time);
        case "constant"
            time = (0:0.01:24).';
            speed = s.constantSpeed*ones(size(time)); acceleration = zeros(size(time));
            steering = defaultSteering(time);
        case "straight"
            time = (0:0.01:24).';
            speed = s.constantSpeed*ones(size(time)); acceleration = zeros(size(time));
            steering = zeros(size(time));
        case "urban"
            time = (0:0.01:40).'; rate = 2*pi/20;
            speed = 10+2*sin(rate*time); acceleration = 2*rate*cos(rate*time);
            steering = raisedCosine(time,[0 4 7 12 15 18 23 26 30 34 40], ...
                deg2rad([0 0 5 5 0 -4 -4 0 3 -3 0]));
        case "highway"
            time = (0:0.01:30).'; rate = 2*pi/15;
            speed = 27+1.5*sin(rate*time); acceleration = 1.5*rate*cos(rate*time);
            steering = raisedCosine(time,[0 3 6 9 14 17 20 23 30], ...
                deg2rad([0 0.6 -0.6 0 0 -0.5 0.5 0 0]));
        case "sine"
            time = (0:0.01:24).';
            speed = 20*ones(size(time)); acceleration = zeros(size(time));
            active = time>=2 & time<=22;
            steering = zeros(size(time));
            steering(active) = deg2rad(s.steeringAmplitudeDeg)*sin(2*pi*(time(active)-2)/4);
        case "launch"
            time = (0:0.01:34).';
            speed = zeros(size(time)); acceleration = zeros(size(time));
            rising = time>=3 & time<11; cruise = time>=11 & time<20; falling = time>=20 & time<28;
            speed(rising) = 4*(1-cos(pi*(time(rising)-3)/8));
            acceleration(rising) = (pi/2)*sin(pi*(time(rising)-3)/8);
            speed(cruise) = 8;
            speed(falling) = 4*(1+cos(pi*(time(falling)-20)/8));
            acceleration(falling) = -(pi/2)*sin(pi*(time(falling)-20)/8);
            steering = raisedCosine(time,[0 4 7 13 16 18 21 24 34],deg2rad([0 0 6 6 0 -4 -4 0 0]));
        otherwise
            error('Unknown profile %s.',s.profile);
    end
    p = struct('time',time,'speed',speed,'acceleration',acceleration,'steering',steering);
end

function value = raisedCosine(time,waypointTime,waypointValue)
% raisedCosine Blend consecutive waypoints with half-cosine transitions.
    waypointTime = double(waypointTime(:)); waypointValue = double(waypointValue(:));
    value = waypointValue(1)*ones(size(time));
    for k = 1:numel(waypointTime)-1
        use = time>=waypointTime(k) & time<=waypointTime(k+1);
        fraction = (time(use)-waypointTime(k))/(waypointTime(k+1)-waypointTime(k));
        value(use) = waypointValue(k)+(0.5-0.5*cos(pi*fraction))*(waypointValue(k+1)-waypointValue(k));
    end
    value(time>=waypointTime(end)) = waypointValue(end);
end

function truth = simulatePlant(p,vehicle,friction,cgShift,outputOffset,substeps)
% simulatePlant Integrate a planar bicycle at its true center of gravity.
% Slip angles use the same small-angle form as lateralBicycleModel, so with
% linear tires (friction = Inf) the plant is that model exactly. A finite
% friction saturates each axle force as Fmax*tanh(C*alpha/Fmax) with the
% static axle load. Below standstillSpeed the reciprocal-speed dynamics are
% replaced by the no-slip kinematic state. cgShift moves the true CG
% rearward of the observer (IMU) point, which stays at the nominal CG.
    standstillSpeed = 0.3;
    count = numel(p.time); state = zeros(count,2);
    if p.speed(1) < standstillSpeed
        state(1,:) = kinematicState(p.speed(1),p.steering(1),vehicle);
    end
    for k = 1:count-1
        if p.speed(k+1) < standstillSpeed
            state(k+1,:) = kinematicState(p.speed(k+1),p.steering(k+1),vehicle);
            continue;
        end
        step = (p.time(k+1)-p.time(k))/substeps; x = state(k,:).';
        for j = 0:substeps-1
            fraction = (j+[0 0.5 1])/substeps;
            speed = max(p.speed(k)+fraction*(p.speed(k+1)-p.speed(k)),standstillSpeed);
            steering = p.steering(k)+fraction*(p.steering(k+1)-p.steering(k));
            k1 = plantDerivative(x,speed(1),steering(1),vehicle,friction);
            k2 = plantDerivative(x+0.5*step*k1,speed(2),steering(2),vehicle,friction);
            k3 = plantDerivative(x+0.5*step*k2,speed(2),steering(2),vehicle,friction);
            k4 = plantDerivative(x+step*k3,speed(3),steering(3),vehicle,friction);
            x = x+(step/6)*(k1+2*k2+2*k3+k4);
        end
        state(k+1,:) = x.';
    end
    rate = zeros(count,2); lateralAcceleration = zeros(count,1);
    numericalRate = [gradient(state(:,1),p.time),gradient(state(:,2),p.time)];
    for k = 1:count
        if p.speed(k) < standstillSpeed
            rate(k,:) = numericalRate(k,:);
            lateralAcceleration(k) = rate(k,1)+p.speed(k)*state(k,2);
        else
            [derivative,lateralAcceleration(k)] = plantDerivative(state(k,:).', ...
                p.speed(k),p.steering(k),vehicle,friction);
            rate(k,:) = derivative.';
        end
    end
    yawRate = state(:,2);
    observerVy = state(:,1)+cgShift*yawRate;
    outputVy = state(:,1)+(cgShift-outputOffset)*yawRate;
    outputVyRate = rate(:,1)+(cgShift-outputOffset)*rate(:,2);
    truth = struct('time',p.time,'longitudinalSpeed',p.speed, ...
        'longitudinalAcceleration',p.acceleration,'steeringAngle',p.steering, ...
        'state',state,'yawRate',yawRate,'yawRateRate',rate(:,2), ...
        'observerState',[observerVy,yawRate], ...
        'cgLateralAcceleration',lateralAcceleration, ...
        'imuLateralAcceleration',lateralAcceleration+cgShift*rate(:,2), ...
        'longitudinalSpecificForce',p.acceleration-observerVy.*yawRate, ...
        'outputLateralVelocity',outputVy,'outputLateralVelocityRate',outputVyRate, ...
        'sideSlipAngle',atan2(outputVy,p.speed));
end

function [derivative,lateralAcceleration] = plantDerivative(x,speed,steering,vehicle,friction)
% plantDerivative Bicycle dynamics at the CG with optional force saturation.
    frontSlip = steering-(x(1)+vehicle.lf*x(2))/speed;
    rearSlip = -(x(1)-vehicle.lr*x(2))/speed;
    frontForce = vehicle.frontCorneringStiffness*frontSlip;
    rearForce = vehicle.rearCorneringStiffness*rearSlip;
    if isfinite(friction)
        wheelbase = vehicle.lf+vehicle.lr;
        frontLimit = friction*vehicle.mass*9.81*vehicle.lr/wheelbase;
        rearLimit = friction*vehicle.mass*9.81*vehicle.lf/wheelbase;
        frontForce = frontLimit*tanh(frontForce/frontLimit);
        rearForce = rearLimit*tanh(rearForce/rearLimit);
    end
    lateralAcceleration = (frontForce+rearForce)/vehicle.mass;
    derivative = [lateralAcceleration-speed*x(2); ...
        (vehicle.lf*frontForce-vehicle.lr*rearForce)/vehicle.yawInertia];
end

function state = kinematicState(speed,steering,vehicle)
% kinematicState No-slip bicycle state [vy; r] at the CG.
    yawRate = speed*tan(steering)/(vehicle.lf+vehicle.lr);
    state = [vehicle.lr*yawRate,yawRate];
end

function u = buildMeasurements(truth,s,seed,cfg)
% buildMeasurements Corrupt the IMU-point truth with noise, bias, delay,
% speed scale, steering offset and an optional validity dropout. The noise
% draw order matches simulateLateralObserverScenario.
    rng(seed,'twister');
    count = numel(truth.time);
    ayNoise = s.noiseScale*cfg.simulation.lateralAccelerationNoiseStd*randn(count,1);
    yawNoise = s.noiseScale*cfg.simulation.yawRateNoiseStd*randn(count,1);
    imu = [truth.imuLateralAcceleration,truth.yawRate,truth.longitudinalSpecificForce];
    if s.imuDelay > 0
        imu = interp1(truth.time,imu,max(truth.time-s.imuDelay,truth.time(1)),'linear');
    end
    u = struct();
    u.time = truth.time;
    u.steeringAngle = truth.steeringAngle+s.steeringOffset;
    u.longitudinalSpeed = s.speedScale*truth.longitudinalSpeed;
    u.longitudinalAcceleration = imu(:,3);
    u.lateralAcceleration = imu(:,1)+s.ayBias+ayNoise;
    u.yawRate = imu(:,2)+s.yawBias+yawNoise;
    if ~isempty(s.dropout)
        u.dynamicValid = ~(truth.time>=s.dropout(1) & truth.time<s.dropout(2));
    end
end

function row = scoreRun(s,seed,truth,estimate,elapsed)
% scoreRun Score one run at the output point. Settled metrics use t >= 6 s;
% settling means |error| <= 0.05 m/s for the entire remaining record.
    reference = truth.outputLateralVelocity;
    error = estimate.lateralVelocity-reference;
    settled = truth.time>=6;
    betaValid = settled & truth.longitudinalSpeed>=6;
    settlingTime = NaN;
    lastOutside = find(abs(error)>0.05,1,'last');
    if isempty(lastOutside)
        settlingTime = 0;
    elseif lastOutside < numel(error)
        settlingTime = truth.time(lastOutside+1);
    end
    betaRmse = NaN;
    if any(betaValid)
        betaRmse = rad2deg(rms(estimate.sideSlipAngle(betaValid)-truth.sideSlipAngle(betaValid)));
    end
    correlation = NaN;
    if std(reference(settled)) > 0 && std(estimate.lateralVelocity(settled)) > 0
        matrix = corrcoef(estimate.lateralVelocity(settled),reference(settled));
        correlation = matrix(1,2);
    end
    referenceRms = rms(reference(settled));
    row = struct('group',s.group,'scenario',s.name,'seed',seed,'samples',numel(error), ...
        'rmseAllMps',rms(error),'rmseSettledMps',rms(error(settled)), ...
        'biasSettledMps',mean(error(settled)),'p95SettledMps',prctile(abs(error(settled)),95), ...
        'maxSettledMps',max(abs(error(settled))),'peakAllMps',max(abs(error)), ...
        'settling005Seconds',settlingTime,'truthRmsSettledMps',referenceRms, ...
        'relativeRmse',rms(error(settled))/max(referenceRms,eps), ...
        'correlation',correlation,'peakTruthLateralAccelerationMps2',max(abs(truth.cgLateralAcceleration)), ...
        'yawRateRmseSettledRadps',rms(estimate.yawRate(settled)-truth.yawRate(settled)), ...
        'sideSlipRmseSettledDeg',betaRmse, ...
        'meanDynamicParticipation',mean(estimate.diagnostics.dynamicParticipation), ...
        'stationaryFraction',mean(estimate.mode=="stationary"), ...
        'crawlFraction',mean(estimate.mode=="crawl"), ...
        'dynamicFraction',mean(estimate.mode=="dynamic"), ...
        'fallbackFraction',mean(estimate.mode=="kinematicFallback"), ...
        'finite',all(isfinite([estimate.state,estimate.dynamicState,estimate.sideSlipAngle, ...
        estimate.sideSlipAngleRate]),'all'), ...
        'microsecondsPerSample',1e6*elapsed/numel(error));
end

function summary = summarizeSynthetic(metrics)
% summarizeSynthetic Average per-run metrics over the seeds of a scenario.
    [groups,group,name] = findgroups(metrics.group,metrics.scenario);
    order = splitapply(@min,(1:height(metrics)).',groups);
    summary = table(group,name,splitapply(@numel,metrics.seed,groups), ...
        splitapply(@mean,metrics.rmseSettledMps,groups),splitapply(@max,metrics.rmseSettledMps,groups), ...
        splitapply(@mean,metrics.biasSettledMps,groups),splitapply(@mean,metrics.p95SettledMps,groups), ...
        splitapply(@max,metrics.maxSettledMps,groups),splitapply(@mean,metrics.truthRmsSettledMps,groups), ...
        splitapply(@mean,metrics.relativeRmse,groups),splitapply(@mean,metrics.sideSlipRmseSettledDeg,groups), ...
        splitapply(@max,metrics.settling005Seconds,groups), ...
        splitapply(@mean,metrics.peakTruthLateralAccelerationMps2,groups), ...
        splitapply(@mean,metrics.meanDynamicParticipation,groups),splitapply(@all,metrics.finite,groups), ...
        'VariableNames',{'group','scenario','runs','meanRmseSettledMps','worstRmseSettledMps', ...
        'meanBiasSettledMps','meanP95SettledMps','worstMaxSettledMps','meanTruthRmsSettledMps', ...
        'meanRelativeRmse','meanSideSlipRmseSettledDeg','worstSettling005Seconds', ...
        'peakTruthLateralAccelerationMps2','meanDynamicParticipation','allFinite'});
    [~,index] = sort(order); summary = summary(index,:);
end

function [metrics,traces] = convergenceStudy(design,cfg)
% convergenceStudy Noise-free straight driving at constant speed from a
% lateral-velocity initial error, shared by the hidden LPV state and the
% master state. The truth is zero, so the estimate is the error.
    time = (0:0.01:8).'; count = numel(time); zero = zeros(count,1);
    rows = cell(0,1); traces = cell(0,1);
    for speed = [6 10 15 20 28]
        for initialError = [0.5 1 3 -1]
            u = struct('time',time,'steeringAngle',zero,'longitudinalSpeed',speed*ones(count,1), ...
                'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero);
            c = cfg; c.observer.initialState = [initialError;0];
            estimate = runLateralVelocityObserver(u,design,c);
            error = estimate.lateralVelocity; magnitude = abs(error);
            L = scheduleLateralObserverGain(design,speed);
            [A,C] = evaluateLateralModel(design.model,[speed;1/speed]);
            row = struct('speedMps',speed,'initialErrorMps',initialError, ...
                'timeTo37PercentSeconds',firstStayingBelow(time,magnitude,abs(initialError)*exp(-1)), ...
                'timeTo5PercentSeconds',firstStayingBelow(time,magnitude,0.05*abs(initialError)), ...
                'timeTo005MpsSeconds',firstStayingBelow(time,magnitude,0.05), ...
                'overshootMps',max(0,max(-sign(initialError)*error)), ...
                'certifiedBound5PercentSeconds',-log(0.05)/design.decayRate, ...
                'slowFrozenPoleRealPart',max(real(eig(A-L*C))), ...
                'finalErrorMps',error(end));
            rows{end+1,1} = row; %#ok<AGROW>
            traces{end+1,1} = struct('speed',speed,'initialError',initialError, ...
                'time',time,'error',error); %#ok<AGROW>
        end
    end
    metrics = struct2table(vertcat(rows{:}));
end

function value = firstStayingBelow(time,magnitude,threshold)
% firstStayingBelow First time after which magnitude stays within threshold.
    lastOutside = find(magnitude>threshold,1,'last');
    if isempty(lastOutside)
        value = 0;
    elseif lastOutside < numel(time)
        value = time(lastOutside+1);
    else
        value = NaN;
    end
end

function metrics = integrationStudy(design,cfg)
% integrationStudy Replay one noiseless 1 ms truth at coarser sample times.
    fineCfg = cfg; fineCfg.simulation.sampleTime = 0.001;
    s = scenario("integration","default",'noiseScale',0);
    p = buildProfile(s,fineCfg);
    truth = simulatePlant(p,cfg.vehicle,Inf,0,cfg.outputPoint.forwardOffsetM,1);
    full = buildMeasurements(truth,s,2026,cfg);
    sampleTime = [0.005 0.01 0.02 0.05 0.1 0.2].';
    rmseSettled = NaN(size(sampleTime)); peak = rmseSettled; status = strings(size(sampleTime));
    for k = 1:numel(sampleTime)
        use = (1:round(sampleTime(k)/0.001):numel(truth.time)).';
        u = struct();
        for field = string(fieldnames(full)).', u.(field) = full.(field)(use); end
        c = cfg; c.observer.initialState = truth.observerState(1,:).'+s.initialError;
        try
            estimate = runLateralVelocityObserver(u,design,c);
            error = estimate.lateralVelocity-truth.outputLateralVelocity(use);
            rmseSettled(k) = rms(error(u.time>=6)); peak(k) = max(abs(error));
            status(k) = "completed";
        catch failure
            status(k) = "error: "+string(failure.message);
        end
    end
    metrics = table(sampleTime,status,rmseSettled,peak,'VariableNames', ...
        {'sampleTimeSeconds','status','rmseSettledMps','peakAllMps'});
end

%% Recorded drives

function [metrics,diagnostics,windows,traces] = recordedEvaluation(root,design,cfg)
% recordedEvaluation Replay both June 7, 2024 drives from four-wheel speed,
% corrected DBW IMU and steering, and score against INSPVA body velocity.
% The open-loop bicycle model driven by the same steering and speed is the
% no-IMU baseline; always-zero lateral velocity is the trivial baseline.
% Ten-second window means expose where the reference and the estimate part.
% The INSPVAX reported azimuth standard deviation is read where exported.
    parameters = mncavReplayConfig();
    drives = {"12-11-24","output/mncav_wheel_only_20260916/calibration_sensors", ...
        "output/mncav_wheel_only_20260916/calibration_sensors/inspva.csv",""; ...
        "12-09-31","output/mncav_wheel_only_20260916/sensors", ...
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv", ...
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspvax.csv"};
    rows = cell(0,1); diagnosticRows = cell(0,1); windowRows = cell(0,1);
    traces = cell(size(drives,1),1);
    for j = 1:size(drives,1)
        clock = loadReceiverClock(fullfile(root,drives{j,3}));
        ins = readtable(fullfile(root,drives{j,2},'inspva.csv'));
        insTime = receiverClockTime(clock,ins.stamp_sec);
        start = insTime(1)+0.5; stop = insTime(end)-0.5;
        high = prepareWheelMotionInputs(fullfile(root,drives{j,2}),parameters,clock,start,stop);
        timer = tic;
        estimate = runLateralVelocityObserver(high,design,cfg);
        elapsed = toc(timer);
        yaw = deg2rad(90-ins.azimuth);
        reference = interp1(insTime-start, ...
            [ins.east_velocity.*cos(yaw)+ins.north_velocity.*sin(yaw), ...
            -ins.east_velocity.*sin(yaw)+ins.north_velocity.*cos(yaw)],high.time,'linear');
        assert(all(isfinite(reference),'all'),'INSPVA does not cover the replay.');
        profile = struct('time',high.time,'speed',high.longitudinalSpeed, ...
            'acceleration',zeros(size(high.time)),'steering',high.steeringAngle);
        model = simulatePlant(profile,cfg.vehicle,Inf,0,cfg.outputPoint.forwardOffsetM,10);

        fast = high.longitudinalSpeed>=5;
        names = ["all","moving_reference_vx_gt_1","straight_vx_ge_5","turning_vx_ge_5"];
        masks = {true(size(high.time)),reference(:,1)>1, ...
            fast & abs(high.yawRate)<0.03,fast & abs(high.yawRate)>=0.03};
        if drives{j,1} == "12-11-24"
            names = [names,"output_point_fit_1_40_s","held_out_after_40_s"]; %#ok<AGROW>
            masks = [masks,{high.time>=1 & high.time<=40,high.time>40}]; %#ok<AGROW>
        end
        for k = 1:numel(names)
            use = masks{k};
            error = estimate.lateralVelocity(use)-reference(use,2);
            modelError = model.outputLateralVelocity(use)-reference(use,2);
            fit = [ones(nnz(use),1),reference(use,2)]\estimate.lateralVelocity(use);
            matrix = corrcoef(estimate.lateralVelocity(use),reference(use,2));
            betaUse = use & reference(:,1)>=6;
            betaError = estimate.sideSlipAngle(betaUse)-atan2(reference(betaUse,2),reference(betaUse,1));
            rows{end+1,1} = struct('drive',drives{j,1},'population',names(k),'samples',nnz(use), ...
                'rmseMps',rms(error),'biasMps',mean(error),'demeanedRmseMps',rms(error-mean(error)), ...
                'p95Mps',prctile(abs(error),95),'maximumMps',max(abs(error)), ...
                'referenceRmsMps',rms(reference(use,2)),'referenceMeanMps',mean(reference(use,2)), ...
                'modelOnlyRmseMps',rms(modelError),'modelOnlyBiasMps',mean(modelError), ...
                'observerPointRmseMps',rms(estimate.observerPointLateralVelocity(use)-reference(use,2)), ...
                'correlation',matrix(1,2),'slopeEstimatePerReference',fit(2), ...
                'sideSlipRmseDeg',rad2deg(rms(betaError)), ...
                'meanDynamicParticipation',mean(estimate.diagnostics.dynamicParticipation(use))); %#ok<AGROW>
        end

        for windowStart = 0:10:floor(high.time(end))-10
            use = high.time>=windowStart & high.time<windowStart+10;
            windowRows{end+1,1} = struct('drive',drives{j,1},'windowStartSeconds',windowStart, ...
                'meanSpeedMps',mean(reference(use,1)),'meanAbsYawRateRadps',mean(abs(high.yawRate(use))), ...
                'meanMeasuredLateralAccelerationMps2',mean(high.lateralAcceleration(use)), ...
                'meanReferenceLateralVelocityMps',mean(reference(use,2)), ...
                'meanObserverLateralVelocityMps',mean(estimate.lateralVelocity(use)), ...
                'meanModelOnlyLateralVelocityMps',mean(model.outputLateralVelocity(use)), ...
                'referenceCrabAngleDeg',atan2d(mean(reference(use,2)),mean(reference(use,1))), ...
                'observerCrabAngleDeg',atan2d(mean(estimate.lateralVelocity(use)),mean(reference(use,1)))); %#ok<AGROW>
        end
        azimuthStdMaximumDeg = NaN;
        if strlength(drives{j,4}) > 0
            extended = readtable(fullfile(root,drives{j,4}));
            azimuthStdMaximumDeg = max(extended.azimuth_stdev_deg);
        end

        % Descriptive error structure on the dynamic-speed samples only.
        error = estimate.lateralVelocity-reference(:,2);
        regressors = [ones(nnz(fast),1),high.longitudinalSpeed(fast),high.yawRate(fast)];
        coefficients = regressors\error(fast);
        residual = error(fast)-regressors*coefficients;
        evaluated = estimate.diagnostics.dynamicModelEvaluated;
        diagnosticRows{end+1,1} = struct('drive',drives{j,1},'durationSeconds',high.time(end), ...
            'samples',numel(high.time),'speedMinimumMps',min(high.longitudinalSpeed), ...
            'speedMaximumMps',max(high.longitudinalSpeed),'peakAbsYawRateRadps',max(abs(high.yawRate)), ...
            'peakAbsLateralAccelerationMps2',max(abs(high.lateralAcceleration)), ...
            'stationaryFraction',mean(estimate.mode=="stationary"), ...
            'crawlFraction',mean(estimate.mode=="crawl"), ...
            'dynamicFraction',mean(estimate.mode=="dynamic"), ...
            'fallbackFraction',mean(estimate.mode=="kinematicFallback"), ...
            'errorInterceptMps',coefficients(1),'errorPerSpeedMpsPerMps',coefficients(2), ...
            'errorPerYawRateM',coefficients(3),'equivalentHeadingOffsetDeg',rad2deg(coefficients(2)), ...
            'regressionResidualRmsMps',rms(residual), ...
            'regressionExplainedFraction',1-sum(residual.^2)/sum((error(fast)-mean(error(fast))).^2), ...
            'lateralInnovationMeanMps2',mean(estimate.innovation(evaluated,1)), ...
            'lateralInnovationRmsMps2',rms(estimate.innovation(evaluated,1)), ...
            'yawInnovationMeanRadps',mean(estimate.innovation(evaluated,2)), ...
            'yawInnovationRmsRadps',rms(estimate.innovation(evaluated,2)), ...
            'finalAccelerometerBiasMps2',estimate.lateralAccelerationBias(end), ...
            'reportedAzimuthStdMaximumDeg',azimuthStdMaximumDeg, ...
            'runtimeSeconds',elapsed,'microsecondsPerSample',1e6*elapsed/numel(high.time)); %#ok<AGROW>
        traces{j} = struct('drive',drives{j,1},'high',high,'estimate',estimate, ...
            'reference',reference,'modelOnly',model.outputLateralVelocity);
    end
    metrics = struct2table(vertcat(rows{:}));
    diagnostics = struct2table(vertcat(diagnosticRows{:}));
    windows = struct2table(vertcat(windowRows{:}));
end

%% Decay-rate sensitivity

function sensitivity = decayRateStudy(cfg,recordedTraces)
% decayRateStudy Re-synthesize the gains at other certified decay rates and
% rerun a fixed subset of cases. This is an evaluation-only diagnostic of
% the synthesis knob; the configured decay rate is not changed.
    cases = [ ...
        scenario("matched","nominal_noise")
        scenario("matched","noise_5x",'noiseScale',5)
        scenario("parameter","tires_x0p7",'frontFactor',0.7,'rearFactor',0.7)
        scenario("parameter","rear_tires_x0p7",'rearFactor',0.7)
        scenario("sensor","ay_bias_plus_0p2",'ayBias',0.2)
        scenario("sensor","yaw_bias_plus_0p01",'yawBias',0.01)
        scenario("sensor","imu_delay_50_ms",'imuDelay',0.05)
        scenario("nonlinear_tire","sine_3p6_deg_mu_0p9",'profile',"sine",'steeringAmplitudeDeg',3.6,'friction',0.9)];
    rows = cell(0,1);
    for decayRate = [2 3 4 5 6 8 10 15]
        c = cfg; c.synthesis.decayRate = decayRate;
        base = struct('decayRatePerSecond',decayRate,'certified',false,'tau',NaN, ...
            'slackScale',NaN,'maxVertexGainNorm',NaN,'issGain',NaN, ...
            'slowFrozenPoleRealPart',NaN,'case',"synthesis",'rmseMps',NaN,'biasMps',NaN);
        try
            design = designLateralObserverGains(c);
        catch
            rows{end+1,1} = base; %#ok<AGROW>
            continue;
        end
        base.certified = design.certified; base.tau = design.tau;
        base.slackScale = design.slackScale; base.maxVertexGainNorm = design.maxVertexGainNorm;
        base.issGain = design.issGain; base.slowFrozenPoleRealPart = design.maxErrorEigenvalueRealPart;
        for s = cases.'
            result = runScenario(s,2026,design,c);
            row = base; row.case = "synthetic_"+s.name;
            row.rmseMps = result.rmseSettledMps; row.biasMps = result.biasSettledMps;
            rows{end+1,1} = row; %#ok<AGROW>
        end
        for j = 1:numel(recordedTraces)
            trace = recordedTraces{j};
            estimate = runLateralVelocityObserver(trace.high,design,c);
            error = estimate.lateralVelocity-trace.reference(:,2);
            row = base; row.case = "recorded_"+trace.drive;
            row.rmseMps = rms(error); row.biasMps = mean(error);
            rows{end+1,1} = row; %#ok<AGROW>
        end
    end
    sensitivity = struct2table(vertcat(rows{:}));
end

%% Provenance

function hashes = inputHashes(root)
% inputHashes SHA-256 of the evaluated sources, configuration and inputs.
    files = [ ...
        "localization/lateralObserver/runLateralVelocityObserver.m"
        "localization/lateralObserver/designLateralObserverGains.m"
        "localization/lateralObserver/scheduleLateralObserverGain.m"
        "localization/lateralObserver/schedulingCoordinates.m"
        "localization/lateralObserver/buildSchedulingPolytope.m"
        "localization/lateralObserver/lateralBicycleModel.m"
        "localization/lateralObserver/evaluateLateralModel.m"
        "localization/lateralObserver/simulateLateralObserverScenario.m"
        "localization/lateralObserver/assertLateralVehicleMatches.m"
        "localization/estimateWheelLongitudinalSpeed.m"
        "scripts/prepareWheelMotionInputs.m"
        "scripts/loadReceiverClock.m"
        "scripts/receiverClockTime.m"
        "config/lateralObserverConfig.m"
        "config/mncavVehicleParameters.json"
        "config/mncavSensorParameters.json"
        "config/mncavMotionOutputPoint.json"
        "config/mncavReplayInterface.json"
        "config/mncavInputCorrections.json"
        "config/mncavWheelSpeedCalibration.json"
        "config/wheelSpeedObserverConfig.m"
        "tests/lateralObserverTest.m"
        "research/lateral_observer_performance_20261001/evaluateLateralObserverPerformance.m"
        "output/mncav_wheel_only_20260916/calibration_sensors/imu.csv"
        "output/mncav_wheel_only_20260916/calibration_sensors/steering.csv"
        "output/mncav_wheel_only_20260916/calibration_sensors/wheel_speed_report.csv"
        "output/mncav_wheel_only_20260916/calibration_sensors/inspva.csv"
        "output/mncav_wheel_only_20260916/calibration_sensors/inspva.clock.json"
        "output/mncav_wheel_only_20260916/sensors/imu.csv"
        "output/mncav_wheel_only_20260916/sensors/steering.csv"
        "output/mncav_wheel_only_20260916/sensors/wheel_speed_report.csv"
        "output/mncav_wheel_only_20260916/sensors/inspva.csv"
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv"
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.clock.json"
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspvax.csv"];
    sha256 = strings(size(files));
    for k = 1:numel(files)
        fid = fopen(fullfile(root,files(k)),'rb');
        assert(fid>=0,'Missing evaluation input %s.',files(k));
        bytes = fread(fid,Inf,'*uint8'); fclose(fid);
        digest = java.security.MessageDigest.getInstance('SHA-256');
        digest.update(bytes);
        sha256(k) = lower(string(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[])));
    end
    hashes = table(files,sha256,'VariableNames',{'path','sha256'});
end

%% Figures

function plotSimulation(traces,convergenceTraces,summary,design,destination)
% plotSimulation Six-panel overview of the synthetic results.
    ink = "#52514e"; blue = "#2a78d6"; orange = "#eb6834"; aqua = "#1baf7a";
    fig = figure('Visible','off','Color','w','Position',[100 100 1500 860]);
    theme(fig,'light');
    closeFigure = onCleanup(@() close(fig));
    layout = tiledlayout(fig,2,3,'TileSpacing','compact','Padding','compact');
    title(layout,'Lateral observer: synthetic evaluation (truth and estimate at the output point)');

    nexttile; hold on; s = findTrace(traces,"nominal_noise");
    plot(s.truth.time,s.truth.outputLateralVelocity,'Color',ink,'LineWidth',1.6,'DisplayName','Truth');
    plot(s.truth.time,s.estimate.lateralVelocity,'Color',blue,'LineWidth',1.0,'DisplayName','Observer');
    styleAxes('Time (s)','Lateral velocity (m/s)','Matched model, nominal noise');

    nexttile; hold on; names = ["noise_5x","nominal_noise","noiseless"];
    labels = ["5x noise","Nominal noise","Noiseless"]; colors = [aqua,orange,blue];
    for k = 1:3
        s = findTrace(traces,names(k));
        plot(s.truth.time,s.estimate.lateralVelocity-s.truth.outputLateralVelocity, ...
            'Color',colors(k),'LineWidth',1.0,'DisplayName',labels(k));
    end
    ylim([-0.15 0.15]);
    styleAxes('Time (s)','Lateral velocity error (m/s)','Matched model: error by noise level');

    nexttile; hold on; speeds = [6 15 28]; colors = [blue,orange,aqua];
    for k = 1:3
        for j = 1:numel(convergenceTraces)
            c = convergenceTraces{j};
            if c.speed == speeds(k) && c.initialError == 1
                plot(c.time,max(abs(c.error),1e-6),'Color',colors(k),'LineWidth',1.4, ...
                    'DisplayName',sprintf('%g m/s',speeds(k)));
            end
        end
    end
    time = linspace(0,4,200);
    plot(time,exp(-design.decayRate*time),'--','Color',ink,'LineWidth',1.2, ...
        'DisplayName',sprintf('exp(-%gt) reference',design.decayRate));
    set(gca,'YScale','log'); xlim([0 4]); ylim([1e-4 2]);
    styleAxes('Time (s)','|error| (m/s)','Convergence from a 1 m/s initial error');

    nexttile;
    use = ismember(summary.group,["parameter","sensor"]) & ~startsWith(summary.scenario,"urban");
    selected = sortrows(summary(use,:),'meanRmseSettledMps','ascend');
    barh(selected.meanRmseSettledMps,0.6,'FaceColor',blue,'EdgeColor','none');
    set(gca,'YTick',1:height(selected),'YTickLabel',strrep(selected.scenario,'_',' '), ...
        'TickLabelInterpreter','none','FontSize',7);
    styleAxes('Settled RMSE (m/s)','','Model and sensor mismatch (default scenario)');
    legend off;

    nexttile; hold on; s = findTrace(traces,"sine_3p6_deg_mu_0p9");
    plot(s.truth.time,s.truth.outputLateralVelocity,'Color',ink,'LineWidth',1.6,'DisplayName','Truth');
    plot(s.truth.time,s.estimate.lateralVelocity,'Color',blue,'LineWidth',1.0,'DisplayName','Observer');
    styleAxes('Time (s)','Lateral velocity (m/s)', ...
        sprintf('Saturating tires, peak |a_y| %.1f m/s^2',max(abs(s.truth.cgLateralAcceleration))));

    nexttile; hold on; s = findTrace(traces,"launch_turn_stop");
    plot(s.truth.time,s.truth.outputLateralVelocity,'Color',ink,'LineWidth',1.6,'DisplayName','Truth');
    plot(s.truth.time,s.estimate.lateralVelocity,'Color',blue,'LineWidth',1.0,'DisplayName','Observer');
    styleAxes('Time (s)','Lateral velocity (m/s)','Launch, turn and stop (0-8-0 m/s)');

    exportgraphics(fig,fullfile(destination,'simulation_performance.png'),'Resolution',150);
end

function plotRecorded(traces,destination)
% plotRecorded Estimate, baselines and error for both recorded drives.
    ink = "#52514e"; blue = "#2a78d6"; orange = "#eb6834";
    fig = figure('Visible','off','Color','w','Position',[100 100 1500 800]);
    theme(fig,'light');
    closeFigure = onCleanup(@() close(fig));
    layout = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
    title(layout,'Lateral observer: recorded MnCAV drives of June 7, 2024 against INSPVA');
    for j = 1:numel(traces)
        s = traces{j};
        nexttile(j); hold on;
        plot(s.high.time,s.reference(:,2),'Color',ink,'LineWidth',1.6,'DisplayName','INSPVA reference');
        plot(s.high.time,s.modelOnly,'Color',orange,'LineWidth',1.0,'DisplayName','Open-loop bicycle model');
        plot(s.high.time,s.estimate.lateralVelocity,'Color',blue,'LineWidth',1.0,'DisplayName','Observer');
        styleAxes('Receiver-relative time (s)','Lateral velocity (m/s)',"Drive "+s.drive);
        nexttile(j+2); hold on;
        plot(s.high.time,s.estimate.lateralVelocity-s.reference(:,2),'Color',blue,'LineWidth',1.0, ...
            'DisplayName','Observer minus reference');
        yline(0,'Color',ink,'LineWidth',0.8,'HandleVisibility','off');
        ylim([-0.6 0.6]);
        styleAxes('Receiver-relative time (s)','Lateral velocity error (m/s)',"Drive "+s.drive+": error");
    end
    exportgraphics(fig,fullfile(destination,'recorded_performance.png'),'Resolution',150);
end

function trace = findTrace(traces,name)
% findTrace Return the retained trace of one scenario.
    for k = 1:numel(traces)
        if traces{k}.scenario.name == name
            trace = traces{k};
            return;
        end
    end
    error('No retained trace for scenario %s.',name);
end

function styleAxes(xLabel,yLabel,panelTitle)
% styleAxes Recessive grid and frame, text in neutral ink, legend on.
    axesHandle = gca;
    grid(axesHandle,'on'); box(axesHandle,'off');
    axesHandle.GridColor = [0.5 0.5 0.5]; axesHandle.GridAlpha = 0.18;
    axesHandle.XColor = [0.32 0.32 0.31]; axesHandle.YColor = [0.32 0.32 0.31];
    xlabel(xLabel); ylabel(yLabel); title(panelTitle,'FontWeight','normal');
    legend('Location','best','Box','off');
end
