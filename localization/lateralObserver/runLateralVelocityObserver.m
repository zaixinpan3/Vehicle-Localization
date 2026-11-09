function estimate = runLateralVelocityObserver(measurements, design, cfg, outputVelocityMap)
% runLateralVelocityObserver Export the LPV lateral-velocity estimate directly.
% The only velocity state is the certified bicycle observer [vy; yawRate].
% Inside its positive-speed domain it uses y = Vx*ay and subtracts the
% known steering feedthrough, together with the measured yaw-rate innovation.
% Outside that domain, or when dynamicValid is false, its state is held and
% velocityValid is false. Holding is not a low-speed estimation guarantee.
% There is no inertial master, bias estimate, correction filter or shadow model.
%
% Output-point transport and sideslip are algebraic; there are no angle states.
% betaDot uses the LPV derivative and the chain rule through the output map.
% Longitudinal speed rate is reconstructed as ax + vyObserver*estimatedYawRate.
% Point transport uses estimated yaw rate and its LPV derivative. Affine-map
% derivatives use backward secants on the
% received clock (zero at initialization). No future sample or angle difference
% is used for the exported rate. These are derivative estimates, not a claim
% of exact differentiation of noisy measurements.
% Consumers must honor velocityValid for velocity, angle and angle-rate aiding.
% The default exponential-midpoint implementation freezes LPV coefficients
% at the interval midpoint and exponentiates that linear flow. A midpoint
% stage handles the copied nonlinearity. Speed
% boundary crossings split an interval; unavailable LPV flow remains held.
% Inputs are linearly interpolated between the two received endpoints,
% as for RK4. This is not exact integration of varying/nonlinear dynamics.
%
% Input:
%   measurements: aligned time, steeringAngle, longitudinalSpeed,
%       longitudinalAcceleration, lateralAcceleration, yawRate, and optional
%       dynamicValid series; design: current designLateralObserverGains result;
%       cfg: current lateralObserverConfig (optional).
% Output:
%   estimate: LPV state, output-point velocity, angle interface and validity.
% Optional outputVelocityMap is n-by-6: [a11 a12 a21 a22 b1 b2], applying
% vOutput=A*[vx;vyOutputPoint]+b before computing angle and rate. It changes
% only output coordinates, never the LPV dynamics, scheduling or innovation.
    if nargin < 3 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(~isfield(cfg,"sideSlip") && ~isfield(cfg,"hybrid"), ...
        "VehicleLocalization:ObsoleteLateralConfiguration", ...
        "Use the current algebraic-sideslip lateralObserverConfig; angle-filter settings are obsolete.");
    assert(isfield(cfg,"outputPoint") && isfield(cfg.outputPoint,"forwardOffsetM") && ...
        isscalar(cfg.outputPoint.forwardOffsetM) && isfinite(double(cfg.outputPoint.forwardOffsetM)), ...
        "VehicleLocalization:MissingOutputPointConfiguration", ...
        "Use the current lateralObserverConfig with an outputPoint.forwardOffsetM entry.");
    forwardOffset = double(cfg.outputPoint.forwardOffsetM);
    lateralObserverSupport.assertLateralVehicleMatches(design,cfg);
    measurements = normalizeMeasurements(measurements);
    if nargin<4,outputVelocityMap=repmat([1,0,0,1,0,0],numel(measurements.time),1);end
    assert(isequal(size(outputVelocityMap),[numel(measurements.time),6]) && all(isfinite(outputVelocityMap),'all'), ...
        'VehicleLocalization:InvalidOutputVelocityMap','Require a finite n-by-6 affine velocity map.');
    measurements.outputVelocityMap=double(outputVelocityMap);
    settings = validateRuntimeConfiguration(cfg, design);
    sampleCount = numel(measurements.time);
    state = double(cfg.observer.initialState(:));
    history = zeros(sampleCount, 2);
    innovationHistory = zeros(sampleCount, 2);
    gainNorm = zeros(sampleCount, 1);
    stateRate = zeros(sampleCount, 2);
    velocityValid = false(sampleCount, 1);
    integrationMethod = lower(strtrim(string(cfg.observer.integrationMethod)));
    for sampleIdx = 1:sampleCount
        sample = sampleAt(measurements, sampleIdx, 0.0);
        [derivative, innovation, gain, valid] = ...
            observerDerivative(state, sample, design, cfg, settings);
        history(sampleIdx, :) = state.';
        innovationHistory(sampleIdx, :) = innovation.';
        gainNorm(sampleIdx) = norm(gain);
        stateRate(sampleIdx,:) = derivative.';
        velocityValid(sampleIdx) = valid;
        if sampleIdx == sampleCount
            break;
        end
        dt = measurements.time(sampleIdx + 1) - measurements.time(sampleIdx);
        if integrationMethod == "exponentialmidpoint"
            state = exponentialInterval(state, measurements, sampleIdx, dt, design, cfg, settings);
        elseif integrationMethod == "euler"
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
    % Differentiate the same point transport and affine map used for velocity.
    observerValid = velocityValid;
    r = history(:,2);
    rDot = stateRate(:,2);
    vx = measurements.longitudinalSpeed;
    vy = history(:,1) - forwardOffset .* r;
    vxDot = measurements.longitudinalAcceleration + history(:,1) .* r;
    vyDot = stateRate(:,1) - forwardOffset .* rDot;
    map = measurements.outputVelocityMap;
    mapDot = backwardRate(map, measurements.time);
    outputVx = map(:,1).*vx + map(:,2).*vy + map(:,5);
    outputVy = map(:,3).*vx + map(:,4).*vy + map(:,6);
    outputVxDot = map(:,1).*vxDot + map(:,2).*vyDot + ...
        mapDot(:,1).*vx + mapDot(:,2).*vy + mapDot(:,5);
    outputVyDot = map(:,3).*vxDot + map(:,4).*vyDot + ...
        mapDot(:,3).*vx + mapDot(:,4).*vy + mapDot(:,6);
    % A map can cancel an otherwise admissible velocity. Withdraw all aiding
    % when its direction is poorly defined, without disabling the LPV flow.
    outputSpeed = hypot(outputVx, outputVy);
    velocityValid = observerValid & outputSpeed >= settings.speedRange(1);
    beta = zeros(sampleCount,1);
    betaDot = zeros(sampleCount,1);
    beta(velocityValid) = atan2(outputVy(velocityValid),outputVx(velocityValid));
    betaDot(velocityValid) = (outputVx(velocityValid).*outputVyDot(velocityValid) - ...
        outputVy(velocityValid).*outputVxDot(velocityValid))./outputSpeed(velocityValid).^2;
    mode = repmat("unavailable", sampleCount, 1);
    mode(velocityValid) = "dynamic";
    modeChanged = [false; diff(velocityValid) ~= 0];
    estimate = struct();
    estimate.time = measurements.time;
    estimate.state = history;
    estimate.stateNames = ["lateralVelocity", "yawRate"];
    estimate.observerPointLateralVelocity = history(:, 1);
    estimate.observerPointLateralVelocityRate = stateRate(:,1);
    estimate.yawRateRate = stateRate(:,2);
    estimate.longitudinalSpeed = outputVx;
    estimate.lateralVelocity = outputVy;
    estimate.outputPoint = cfg.outputPoint;
    estimate.yawRate = history(:, 2);
    estimate.lateralVelocityRate = outputVyDot;
    estimate.longitudinalSpeedRate = outputVxDot;
    estimate.sideSlipAngle = beta;
    estimate.sideSlipAngleRate = betaDot;
    estimate.velocityValid = velocityValid;
    estimate.innovation = innovationHistory;
    estimate.gainNorm = gainNorm;
    estimate.scheduledSpeed = measurements.longitudinalSpeed;
    estimate.scheduledSpeedRate = backwardRate(measurements.longitudinalSpeed, measurements.time);
    % Causal secants test the received interpolant, not the unknown physical
    % derivative between samples. Startup has no measured prior interval.
    rateKnown = [false; true(sampleCount - 1, 1)];
    rateWithinBounds = rateKnown & estimate.scheduledSpeedRate >= settings.speedRateRange(1) & ...
        estimate.scheduledSpeedRate <= settings.speedRateRange(2);
    estimate.mode = mode;
    estimate.modeChanged = modeChanged;
    estimate.diagnostics = struct("dynamicModelEvaluated", observerValid, ...
        "sideSlipValid", velocityValid, "modeChanged", modeChanged, ...
        "integrationMethod", integrationMethod, ...
        "schedulingRateKnown", rateKnown, "schedulingRateWithinBounds", rateWithinBounds);
    estimate.measurements = measurements;
end

function [derivative, innovation, gain, valid] = observerDerivative( ...
        state, sample, design, cfg, settings)
% observerDerivative Evaluate only the two-state LPV observer.
    valid = modelAvailable(sample, settings);
    if valid
        [lpvDerivative, innovation, gain] = ...
            dynamicObserverDerivative(state(1:2), sample, design, cfg.observer.nonlinearity);
    else
        lpvDerivative = zeros(2, 1);
        innovation = zeros(2, 1);
        gain = zeros(2);
    end
    derivative = lpvDerivative;
end

function state = exponentialInterval(state, measurements, sampleIdx, dt, design, cfg, settings)
% exponentialInterval Resolve speed-domain boundaries before midpoint flow.
% Boolean validity is conservative on the open interval: both endpoints
% must be available. An isolated valid endpoint has zero integration time.
    speed = measurements.longitudinalSpeed(sampleIdx:sampleIdx+1);
    fractions = [0.0, 1.0];
    if speed(2) ~= speed(1)
        crossings = (settings.speedRange - speed(1)) ./ (speed(2) - speed(1));
        fractions = sort([fractions, crossings(crossings > 0 & crossings < 1)]);
    end
    for segmentIdx = 1:numel(fractions)-1
        left = fractions(segmentIdx);
        right = fractions(segmentIdx+1);
        sample = sampleAt(measurements, sampleIdx, (left+right)/2);
        h = dt*(right-left);
        valid = modelAvailable(sample, settings);
        if valid
            rho = [sample.longitudinalSpeed; 1/sample.longitudinalSpeed];
            [A, C, D] = lateralObserverSupport.evaluateLateralModel(design.model, rho);
            gain = lateralObserverSupport.scheduleLateralObserverGain(design, sample.longitudinalSpeed);
            linear = A-gain*C;
            forcing = design.model.B*sample.steeringAngle + ...
                gain*([sample.longitudinalSpeed*sample.lateralAcceleration; sample.yawRate] - D*sample.steeringAngle);
            nonlinearMap = eye(2) - sample.longitudinalSpeed*gain(:, 1)*[1, 0];
            % The augmented exponential gives Phi and the input integral
            % without inverting linear, including a singular linear part.
            transition = expm((h/2)*[linear, eye(2); zeros(2,4)]);
            phi = transition(1:2,1:2);
            integral = transition(1:2,3:4);
            midpoint = phi*state(1:2) + integral*(forcing + ...
                nonlinearMap*evaluateNonlinearity(cfg.observer.nonlinearity, state(1:2)));
            midpointForcing = forcing + nonlinearMap*evaluateNonlinearity(cfg.observer.nonlinearity, midpoint);
            state(1:2) = phi*(phi*state(1:2) + integral*midpointForcing) + ...
                integral*midpointForcing;
        end
    end
end

function valid = modelAvailable(sample, settings)
% modelAvailable Gate reciprocal-speed dynamics independently of output geometry.
    valid = sample.dynamicValid && sample.longitudinalSpeed >= settings.speedRange(1) && ...
        sample.longitudinalSpeed <= settings.speedRange(2);
end

function rate = backwardRate(series, time)
% backwardRate Causal secants for exogenous inputs, with held-input startup.
    rate = [zeros(1,size(series,2)); diff(series,1,1)./diff(time)];
end

function [derivative, innovation, gain] = dynamicObserverDerivative( ...
        state, sample, design, nonlinearity)
% dynamicObserverDerivative Evaluate the certified reciprocal-speed branch.
    rho = [sample.longitudinalSpeed; 1.0 ./ sample.longitudinalSpeed];
    [A, C, D] = lateralObserverSupport.evaluateLateralModel(design.model, rho);
    gain = lateralObserverSupport.scheduleLateralObserverGain(design, sample.longitudinalSpeed);
    nonlinearTerm = evaluateNonlinearity(nonlinearity, state);
    innovation = [sample.longitudinalSpeed*sample.lateralAcceleration; sample.yawRate] - C*state - ...
        D*sample.steeringAngle - [sample.longitudinalSpeed*nonlinearTerm(1); 0];
    derivative = (A * state) + (design.model.B .* sample.steeringAngle) + ...
        nonlinearTerm + (gain * innovation);
end

function value = evaluateNonlinearity(nonlinearity, state)
% evaluateNonlinearity Validate every explicit nonlinear stage.
    value = double(nonlinearity(state));
    value = value(:);
    assert(numel(value) == 2 && all(isfinite(value)), ...
        "cfg.observer.nonlinearity must return a finite [2 x 1] vector.");
end

function sample = sampleAt(measurements, sampleIdx, stepFraction)
% sampleAt Interpolate physical measurements without flooring speed.
    numericFields = ["steeringAngle", "longitudinalSpeed", ...
        "longitudinalAcceleration", "lateralAcceleration", "yawRate"];
    nextIdx = min(sampleIdx + 1, numel(measurements.time));
    sample = struct();
    for fieldName = numericFields
        series = measurements.(fieldName);
        sample.(fieldName) = ((1.0 - stepFraction) .* series(sampleIdx,:)) + ...
            (stepFraction .* series(nextIdx,:));
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
            "VehicleLocalization:MissingLateralMeasurement", ...
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
% validateRuntimeConfiguration Check the LPV domain and integration configuration.
    assert(isstruct(design) && isfield(design, "model") && isfield(design, "polytope"), ...
        "design must come from designLateralObserverGains.");
    assert(isfield(design, "certificateMethod") && ...
        string(design.certificateMethod) == "virtual-ay-yaw-scheduled-metric-iss-v2" && ...
        isfield(design, "vertexLyapunovMatrices") && isfield(design, "vertexY") && ...
        isfield(design, "vertexGains") && isfield(design, "certificateDomain") && ...
        isfield(design.certificateDomain, "speedRateRange") && ...
        isequal(size(design.vertexLyapunovMatrices), [2, 2, 3]) && ...
        isequal(size(design.vertexY), [2, 2, 3]) && isequal(size(design.vertexGains), [2, 2, 3]) && ...
        isfield(design.model, "D1") && isequal(design.model.outputNames, ["speedTimesLateralAcceleration", "yawRate"]), ...
        "VehicleLocalization:ObsoleteLateralDesign", ...
        "Re-synthesize the virtual-output lateral design with scheduled P, Y and the speed-rate domain.");
    speedRange = double(design.polytope.speedRange(:).');
    assert(numel(speedRange) == 2 && all(isfinite(speedRange)) && speedRange(1) > 0.0 && ...
        speedRange(2) > speedRange(1), ...
        "design.polytope.speedRange must define a positive nonempty speed interval.");
    integrationMethod = lower(strtrim(string(cfg.observer.integrationMethod)));
    assert(any(integrationMethod == ["exponentialmidpoint", "rk4", "euler"]), ...
        "cfg.observer.integrationMethod must be exponentialMidpoint, rk4 or euler.");
    assert(isa(cfg.observer.nonlinearity, "function_handle"), ...
        "cfg.observer.nonlinearity must be a function handle.");
    assertFiniteVector(cfg.observer.initialState, 2, "cfg.observer.initialState");
    settings = struct("speedRange", speedRange, "speedRateRange", design.certificateDomain.speedRateRange);
end

function assertFiniteVector(value, expectedLength, name)
% assertFiniteVector Check a fixed-size finite vector.
    vector = double(value(:));
    assert(numel(vector) == expectedLength && all(isfinite(vector)), ...
        "%s must be a finite [%d x 1] vector.", name, expectedLength);
end
