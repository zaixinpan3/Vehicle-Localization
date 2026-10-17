function estimate = runLateralVelocityObserver(measurements, design, cfg)
% runLateralVelocityObserver Export the LPV lateral-velocity estimate directly.
% The only velocity state is the certified bicycle observer [vy; yawRate].
% Inside its positive-speed domain it uses the scheduled LPV innovation.
% Outside that domain, or when dynamicValid is false, its state is held and
% velocityValid is false. Holding is not a low-speed estimation guarantee.
% There is no inertial master, bias estimate, correction filter or shadow model.
%
% The output-point transport is algebraic. A separate two-state angle filter
% still exports beta and betaDot from this LPV velocity; it does not modify vy.
% Consumers must honor velocityValid for velocity, angle and angle-rate aiding.
%
% Input:
%   measurements: aligned time, steeringAngle, longitudinalSpeed,
%       longitudinalAcceleration, lateralAcceleration, yawRate, and optional
%       dynamicValid series; design: current designLateralObserverGains result;
%       cfg: current lateralObserverConfig (optional).
% Output:
%   estimate: LPV state, output-point velocity, angle interface and validity.
    if nargin < 3 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(isfield(cfg,"sideSlip") && ~isfield(cfg,"hybrid"), ...
        "VehicleLocalization:ObsoleteLateralConfiguration", ...
        "Use the current direct-LPV lateralObserverConfig.");
    assert(isfield(cfg,"outputPoint") && isfield(cfg.outputPoint,"forwardOffsetM") && ...
        isscalar(cfg.outputPoint.forwardOffsetM) && isfinite(double(cfg.outputPoint.forwardOffsetM)), ...
        "VehicleLocalization:MissingOutputPointConfiguration", ...
        "Use the current lateralObserverConfig with an outputPoint.forwardOffsetM entry.");
    forwardOffset = double(cfg.outputPoint.forwardOffsetM);
    lateralObserverSupport.assertLateralVehicleMatches(design,cfg);
    measurements = normalizeMeasurements(measurements);
    settings = validateRuntimeConfiguration(cfg, design);
    sampleCount = numel(measurements.time);
    state = double(cfg.observer.initialState(:));
    firstSample = sampleAt(measurements, 1, 0.0);
    angleState = double(cfg.sideSlip.initialState(:));
    if isempty(angleState)
        angleState = [sideSlipCommandAt(state, firstSample, cfg, settings); 0.0];
    end
    state = [state; angleState];
    history = zeros(sampleCount, 4);
    innovationHistory = zeros(sampleCount, 2);
    gainNorm = zeros(sampleCount, 1);
    lateralVelocityRate = zeros(sampleCount, 1);
    sideSlipCommand = zeros(sampleCount, 1);
    velocityValid = false(sampleCount, 1);
    integrationMethod = lower(strtrim(string(cfg.observer.integrationMethod)));
    for sampleIdx = 1:sampleCount
        sample = sampleAt(measurements, sampleIdx, 0.0);
        [derivative, innovation, gain, command, valid] = ...
            observerDerivative(state, sample, design, cfg, settings);
        history(sampleIdx, :) = state.';
        innovationHistory(sampleIdx, :) = innovation.';
        gainNorm(sampleIdx) = norm(gain);
        lateralVelocityRate(sampleIdx) = derivative(1);
        sideSlipCommand(sampleIdx) = command;
        velocityValid(sampleIdx) = valid;
        if sampleIdx == sampleCount
            break;
        end
        dt = measurements.time(sampleIdx + 1) - measurements.time(sampleIdx);
        if integrationMethod == "euler"
            state = state + dt .* derivative;
        else
            midSample = sampleAt(measurements, sampleIdx, 0.5);
            nextSample = sampleAt(measurements, sampleIdx, 1.0);
            k1 = derivative;
            k2 = observerDerivative(state + 0.5 .* dt .* k1, midSample, design, cfg, settings);
            k3 = observerDerivative(state + 0.5 .* dt .* k2, midSample, design, cfg, settings);
            k4 = observerDerivative(state + dt .* k3, nextSample, design, cfg, settings);
            state = state + (dt ./ 6.0) .* (k1 + 2.0 .* k2 + 2.0 .* k3 + k4);
        end
        assert(all(isfinite(state)), "The lateral observer diverged in interval %d.", sampleIdx);
    end
    mode = repmat("unavailable", sampleCount, 1);
    mode(velocityValid) = "dynamic";
    modeChanged = [false; diff(velocityValid) ~= 0];
    estimate = struct();
    estimate.time = measurements.time;
    estimate.state = history(:, 1:2);
    estimate.stateNames = ["lateralVelocity", "yawRate"];
    estimate.observerPointLateralVelocity = history(:, 1);
    estimate.lateralVelocity = history(:, 1) - forwardOffset .* measurements.yawRate;
    estimate.outputPoint = cfg.outputPoint;
    estimate.yawRate = history(:, 2);
    estimate.lateralVelocityRate = lateralVelocityRate;
    estimate.longitudinalSpeedRate = measurements.longitudinalAcceleration + ...
        history(:, 1) .* measurements.yawRate;
    estimate.sideSlipAngle = history(:, 3);
    estimate.sideSlipAngleRate = history(:, 4);
    estimate.sideSlipCommand = sideSlipCommand;
    estimate.velocityValid = velocityValid;
    estimate.innovation = innovationHistory;
    estimate.gainNorm = gainNorm;
    estimate.scheduledSpeed = measurements.longitudinalSpeed;
    estimate.mode = mode;
    estimate.modeChanged = modeChanged;
    estimate.diagnostics = struct("dynamicModelEvaluated", velocityValid, ...
        "sideSlipCommandValid", velocityValid, "modeChanged", modeChanged);
    estimate.measurements = measurements;
end

function [derivative, innovation, gain, command, valid] = observerDerivative( ...
        state, sample, design, cfg, settings)
% observerDerivative Integrate the LPV state and its separate angle interface.
    [command, valid] = sideSlipCommandAt(state(1:2), sample, cfg, settings);
    if valid
        [lpvDerivative, innovation, gain] = ...
            dynamicObserverDerivative(state(1:2), sample, design, cfg.observer.nonlinearity);
    else
        lpvDerivative = zeros(2, 1);
        innovation = zeros(2, 1);
        gain = zeros(2, 2);
    end
    frequency = double(cfg.sideSlip.naturalFrequency);
    damping = double(cfg.sideSlip.dampingRatio);
    angleError = wrapAngleDifference(command - state(3));
    derivative = [lpvDerivative; state(4); ...
        frequency.^2 .* angleError - 2.0 .* damping .* frequency .* state(4)];
end

function [command, valid] = sideSlipCommandAt(state, sample, cfg, settings)
% sideSlipCommandAt Admit the LPV domain before forming a velocity direction.
    valid = sample.dynamicValid && sample.longitudinalSpeed >= settings.speedRange(1) && ...
        sample.longitudinalSpeed <= settings.speedRange(2);
    command = 0.0;
    if valid
        vy = state(1) - double(cfg.outputPoint.forwardOffsetM) .* sample.yawRate;
        command = atan2(vy, sample.longitudinalSpeed);
    end
end

function [derivative, innovation, gain] = dynamicObserverDerivative( ...
        state, sample, design, nonlinearity)
% dynamicObserverDerivative Evaluate the certified reciprocal-speed branch.
    rho = [sample.longitudinalSpeed; 1.0 ./ sample.longitudinalSpeed];
    [A, C] = lateralObserverSupport.evaluateLateralModel(design.model, rho);
    gain = lateralObserverSupport.scheduleLateralObserverGain(design, sample.longitudinalSpeed);
    measuredOutput = [sample.lateralAcceleration; sample.yawRate];
    predictedOutput = (C * state) + (design.model.D .* sample.steeringAngle);
    innovation = measuredOutput - predictedOutput;
    nonlinearTerm = double(nonlinearity(state));
    nonlinearTerm = nonlinearTerm(:);
    assert(numel(nonlinearTerm) == 2 && all(isfinite(nonlinearTerm)), ...
        "cfg.observer.nonlinearity must return a finite [2 x 1] vector.");
    derivative = (A * state) + (design.model.B .* sample.steeringAngle) + ...
        nonlinearTerm + (gain * innovation);
end

function sample = sampleAt(measurements, sampleIdx, stepFraction)
% sampleAt Interpolate physical measurements without flooring speed.
    numericFields = ["steeringAngle", "longitudinalSpeed", ...
        "longitudinalAcceleration", "lateralAcceleration", "yawRate"];
    nextIdx = min(sampleIdx + 1, numel(measurements.time));
    sample = struct();
    for fieldName = numericFields
        series = measurements.(fieldName);
        sample.(fieldName) = ((1.0 - stepFraction) .* series(sampleIdx)) + ...
            (stepFraction .* series(nextIdx));
    end
    if stepFraction <= 0.0
        sample.dynamicValid = measurements.dynamicValid(sampleIdx);
    elseif stepFraction >= 1.0
        sample.dynamicValid = measurements.dynamicValid(nextIdx);
    else
        sample.dynamicValid = measurements.dynamicValid(sampleIdx) && ...
            measurements.dynamicValid(nextIdx);
    end
end

function measurements = normalizeMeasurements(measurements)
% normalizeMeasurements Validate and align all measurement series.
    requiredFields = ["time", "steeringAngle", "longitudinalSpeed", ...
        "longitudinalAcceleration", "lateralAcceleration", "yawRate"];
    assert(isstruct(measurements), "measurements must be a struct of measurement series.");
    for fieldName = requiredFields
        assert(isfield(measurements, fieldName), ...
            "measurements is missing required series: %s.", fieldName);
    end
    sampleCount = numel(measurements.time);
    assert(sampleCount >= 2, "measurements must contain at least two samples.");
    for fieldName = requiredFields
        series = double(measurements.(fieldName)(:));
        assert(numel(series) == sampleCount && all(isfinite(series)), ...
            "measurements.%s must be a finite series aligned with measurements.time.", fieldName);
        measurements.(fieldName) = series;
    end
    if isfield(measurements, "dynamicValid")
        validity = measurements.dynamicValid(:);
        assert((islogical(validity) || isnumeric(validity)) && numel(validity) == sampleCount && ...
            all(isfinite(double(validity))), ...
            "measurements.dynamicValid must be a finite logical series aligned with time.");
        measurements.dynamicValid = logical(validity);
    else
        measurements.dynamicValid = true(sampleCount, 1);
    end
    assert(all(diff(measurements.time) > 0), "measurements.time must be strictly increasing.");
    assert(all(measurements.longitudinalSpeed >= 0), ...
        "measurements.longitudinalSpeed must be nonnegative.");
end

function settings = validateRuntimeConfiguration(cfg, design)
% validateRuntimeConfiguration Check the LPV domain and angle-interface configuration.
    assert(isstruct(design) && isfield(design, "model") && isfield(design, "polytope") && ...
        isfield(design, "vertexGains") && isfield(design, "lyapunovBasis"), ...
        "design must come from designLateralObserverGains and include model, polytope, vertexGains, " + ...
        "and lyapunovBasis.");
    speedRange = double(design.polytope.speedRange(:).');
    assert(numel(speedRange) == 2 && all(isfinite(speedRange)) && speedRange(1) > 0.0 && ...
        speedRange(2) > speedRange(1), ...
        "design.polytope.speedRange must define a positive nonempty speed interval.");
    integrationMethod = lower(strtrim(string(cfg.observer.integrationMethod)));
    assert(integrationMethod == "rk4" || integrationMethod == "euler", ...
        "cfg.observer.integrationMethod must be rk4 or euler.");
    assert(isa(cfg.observer.nonlinearity, "function_handle"), ...
        "cfg.observer.nonlinearity must be a function handle.");
    assertFiniteVector(cfg.observer.initialState, 2, "cfg.observer.initialState");
    if ~isempty(cfg.sideSlip.initialState)
        assertFiniteVector(cfg.sideSlip.initialState, 2, "cfg.sideSlip.initialState");
    end
    assertPositiveScalar(cfg.sideSlip.naturalFrequency, "cfg.sideSlip.naturalFrequency");
    assertPositiveScalar(cfg.sideSlip.dampingRatio, "cfg.sideSlip.dampingRatio");
    settings = struct("speedRange", speedRange);
end

function assertFiniteVector(value, expectedLength, name)
% assertFiniteVector Check a fixed-size finite vector.
    vector = double(value(:));
    assert(numel(vector) == expectedLength && all(isfinite(vector)), ...
        "%s must be a finite [%d x 1] vector.", name, expectedLength);
end

function assertPositiveScalar(value, name)
% assertPositiveScalar Check a positive finite scalar.
    numericValue = double(value);
    assert(isscalar(numericValue) && isfinite(numericValue) && numericValue > 0.0, ...
        "%s must be a positive finite scalar.", name);
end

function wrapped = wrapAngleDifference(angle)
% wrapAngleDifference Wrap an angular error to [-pi, pi).
    wrapped = mod(angle + pi, 2.0 .* pi) - pi;
end
