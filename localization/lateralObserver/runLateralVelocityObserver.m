function estimate = runLateralVelocityObserver(measurements, design, cfg, outputVelocityMap)
% runLateralVelocityObserver Export the LPV lateral-velocity estimate directly.
% The only velocity state is the certified bicycle observer [vy; yawRate].
% Inside its positive-speed domain it uses y = Vx*ay and subtracts the
% known steering feedthrough, together with the measured yaw-rate innovation.
% Outside that domain, or when dynamicValid is false, its state is held and
% velocityValid is false. Holding is not a low-speed estimation guarantee.
% There is no inertial master, bias estimate, correction filter or shadow model.
%
% Output-point transport and sideslip rate are algebraic; there are no angle states.
% betaDot uses measured acceleration and estimated velocity/yaw rate.
% Longitudinal speed rate is reconstructed as ax + vyObserver*estimatedYawRate.
% Point transport uses estimated yaw rate and its LPV derivative. Time-varying
% output maps must supply rates from a model or filter state equation; neither
% map coefficients nor measured speeds are numerically differentiated here.
% Consumers must honor velocityValid for velocity/sideslip aiding and
% yawRateValid for yaw-rate aiding. The latter tracks dynamic-model validity.
% The default exponential-midpoint implementation freezes LPV coefficients
% at the interval midpoint and exponentiates that affine flow. Speed-domain
% boundary crossings split an interval; unavailable LPV flow remains held.
% Inputs are linearly interpolated between the two received endpoints,
% as for RK4. This is not exact integration of time-varying dynamics.
%
% Input:
%   measurements: aligned time, steeringAngle, longitudinalSpeed,
%       longitudinalAcceleration, lateralAcceleration, yawRate, and optional
%       dynamicValid series; design: current designLateralObserverGains result;
%       cfg: current lateralObserverConfig (optional).
% Output:
%   estimate: LPV state, output-point velocity, sideslip rate and validity.
% Optional outputVelocityMap is n-by-6: [a11 a12 a21 a22 b1 b2], applying
% vOutput=A*[vx;vyOutputPoint]+b before computing the sideslip rate. It changes
% only output coordinates, never the LPV dynamics, scheduling or innovation.
% A numeric map must be constant. For a time-varying map supply a struct with
% n-by-6 value and rate fields, or a function handle map(state,stateRate) that
% returns this struct after the LPV trajectory is evaluated. Its rate is an
% independently modeled estimate, not necessarily the derivative of noisy values.
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
    settings = validateRuntimeConfiguration(cfg, design);
    sampleCount = numel(measurements.time);
    state = double(cfg.observer.initialState(:));
    history = zeros(sampleCount, 2);
    innovationHistory = zeros(sampleCount, 2);
    gainNorm = zeros(sampleCount, 1);
    stateRate = zeros(sampleCount, 2);
    velocityValid = false(sampleCount, 1);
    for sampleIdx = 1:sampleCount
        sample = sampleAt(measurements, sampleIdx, 0.0);
        [derivative, innovation, gain, valid] = ...
            observerDerivative(state, sample, design, settings);
        history(sampleIdx, :) = state.';
        innovationHistory(sampleIdx, :) = innovation.';
        gainNorm(sampleIdx) = norm(gain);
        stateRate(sampleIdx,:) = derivative.';
        velocityValid(sampleIdx) = valid;
        if sampleIdx == sampleCount
            break;
        end
        dt = measurements.time(sampleIdx + 1) - measurements.time(sampleIdx);
        state = exponentialInterval(state, measurements, sampleIdx, dt, design, settings);
        assert(all(isfinite(state)), "The lateral observer diverged in interval %d.", sampleIdx);
    end
    % Reconstruct velocity rates from acceleration kinematics; no signal differences.
    observerValid = velocityValid;
    r = history(:,2);
    rDot = stateRate(:,2);
    vx = measurements.longitudinalSpeed;
    vy = history(:,1) - forwardOffset .* r;
    vxDot = measurements.longitudinalAcceleration + history(:,1) .* r;
    vyDot = measurements.lateralAcceleration - vx .* r - forwardOffset .* rDot;
    if isa(outputVelocityMap,'function_handle')
        outputVelocityMap = outputVelocityMap(history,stateRate);
    end
    [map,mapDot] = resolveOutputMap(outputVelocityMap,sampleCount);
    measurements.outputVelocityMap = map;
    measurements.outputVelocityMapRate = mapDot;
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
    betaDot = zeros(sampleCount,1);
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
    estimate.yawRateValid = observerValid;
    estimate.lateralVelocityRate = outputVyDot;
    estimate.longitudinalSpeedRate = outputVxDot;
    estimate.sideSlipAngleRate = betaDot;
    estimate.velocityValid = velocityValid;
    estimate.innovation = innovationHistory;
    estimate.gainNorm = gainNorm;
    estimate.minimumDesignSpeedMps = settings.speedRange(1);
    estimate.scheduledSpeed = measurements.longitudinalSpeed;
    estimate.scheduledSpeedRate = vxDot;
    % This physical rate estimate does not certify the derivative of noisy
    % scheduling measurements. The PDLF rate bound remains an assumption.
    rateKnown = false(sampleCount,1);
    rateWithinBounds = observerValid & vxDot >= settings.speedRateRange(1) & ...
        vxDot <= settings.speedRateRange(2);
    estimate.mode = mode;
    estimate.modeChanged = modeChanged;
    estimate.diagnostics = struct("dynamicModelEvaluated", observerValid, ...
        "sideSlipValid", velocityValid, "modeChanged", modeChanged, ...
        "sideSlipRateSource", "accelerationKinematics", ...
        "integrationMethod", "exponentialmidpoint", ...
        "schedulingRateSource", "kinematicEstimate", ...
        "schedulingRateKnown", rateKnown, "schedulingRateWithinBounds", rateWithinBounds);
    estimate.measurements = measurements;
end

function [derivative, innovation, gain, valid] = observerDerivative( ...
        state, sample, design, settings)
% observerDerivative Evaluate only the two-state LPV observer.
    valid = modelAvailable(sample, settings);
    if valid
        [lpvDerivative, innovation, gain] = ...
            dynamicObserverDerivative(state(1:2), sample, design);
    else
        lpvDerivative = zeros(2, 1);
        innovation = zeros(2, 1);
        gain = zeros(2);
    end
    derivative = lpvDerivative;
end

function state = exponentialInterval(state, measurements, sampleIdx, dt, design, settings)
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
            % The augmented exponential gives the flow and input integral
            % without inverting linear, including a singular linear part.
            transition = expm(h*[linear, eye(2); zeros(2,4)]);
            state(1:2) = transition(1:2,1:2)*state(1:2) + ...
                transition(1:2,3:4)*forcing;
        end
    end
end

function valid = modelAvailable(sample, settings)
% modelAvailable Gate reciprocal-speed dynamics independently of output geometry.
    valid = sample.dynamicValid && sample.longitudinalSpeed >= settings.speedRange(1) && ...
        sample.longitudinalSpeed <= settings.speedRange(2);
end

function [map,rate] = resolveOutputMap(input,n)
% Require a rate source instead of silently differentiating measured maps.
    if isstruct(input)
        assert(isscalar(input) && all(isfield(input,{'value','rate'})), ...
            'VehicleLocalization:InvalidOutputVelocityMap','Supply value and rate arrays.');
        map=input.value;rate=input.rate;
    else
        map=input;
        assert(isnumeric(map) && isequal(size(map),[n,6]), ...
            'VehicleLocalization:InvalidOutputVelocityMap','Require an n-by-6 affine velocity map.');
        assert(all(map==map(1,:),'all'), ...
            'VehicleLocalization:OutputMapRateRequired', ...
            'A varying output map requires rates from a model or filter dynamics.');
        rate=zeros(n,6);
    end
    assert(isnumeric(map) && isnumeric(rate) && isreal(map) && isreal(rate) && ...
        isequal(size(map),[n,6]) && isequal(size(rate),[n,6]) && ...
        all(isfinite(map),'all') && all(isfinite(rate),'all'), ...
        'VehicleLocalization:InvalidOutputVelocityMap','Require finite real n-by-6 values and rates.');
    map=double(map);rate=double(rate);
end

function [derivative, innovation, gain] = dynamicObserverDerivative( ...
        state, sample, design)
% dynamicObserverDerivative Evaluate the certified reciprocal-speed branch.
    rho = [sample.longitudinalSpeed; 1.0 ./ sample.longitudinalSpeed];
    [A, C, D] = lateralObserverSupport.evaluateLateralModel(design.model, rho);
    gain = lateralObserverSupport.scheduleLateralObserverGain(design, sample.longitudinalSpeed);
    innovation = [sample.longitudinalSpeed*sample.lateralAcceleration; sample.yawRate] - C*state - ...
        D*sample.steeringAngle;
    derivative = (A * state) + (design.model.B .* sample.steeringAngle) + ...
        (gain * innovation);
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
        string(design.certificateMethod) == "column-affine-expanded-iss-free-distributed-v11" && ...
        isfield(design, "vertexLyapunovMatrices") && isfield(design, "vertexY") && ...
        isfield(design, "vertexGains") && isfield(design, "certificateDomain") && ...
        isfield(design, "gainRecoveryMatrix") && isequal(size(design.gainRecoveryMatrix), [2, 2]) && ...
        isfield(design, "gainBasis") && ~isfield(design.gainBasis, "accelerationConstant") && ...
        isfield(design, "processDistribution") && isequal(size(design.processDistribution), [2,2]) && ...
        isfield(design.certificateDomain, "speedRateRange") && ...
        isequal(size(design.vertexLyapunovMatrices), [2, 2, 3]) && ...
        isequal(size(design.vertexY), [2, 2, 3]) && isequal(size(design.vertexGains), [2, 2, 3]) && ...
        isfield(design.model, "D1") && isequal(design.model.outputNames, ["speedTimesLateralAcceleration", "yawRate"]), ...
        "VehicleLocalization:ObsoleteLateralDesign", ...
        "Re-synthesize the column-affine design with common X, scheduled P and the expanded certificate.");
    assert(isfield(design.gainBasis, 'accelerationReciprocal') && ...
        isequal(size(design.gainBasis.accelerationReciprocal), [2,1]) && ...
        isreal(design.gainBasis.accelerationReciprocal) && ...
        all(isfinite(design.gainBasis.accelerationReciprocal)), ...
        'VehicleLocalization:InvalidLateralGainBasis', ...
        'The acceleration basis must be a finite real 2-by-1 vector.');
    speedRange = double(design.polytope.speedRange(:).');
    assert(numel(speedRange) == 2 && all(isfinite(speedRange)) && speedRange(1) > 0.0 && ...
        speedRange(2) > speedRange(1), ...
        "design.polytope.speedRange must define a positive nonempty speed interval.");
    assert(strcmpi(strtrim(string(cfg.observer.integrationMethod)),"exponentialMidpoint"), ...
        'VehicleLocalization:RetiredLateralIntegrator','Lateral integration uses exponentialMidpoint.');
    assert(~isfield(cfg.observer, 'nonlinearity') && ...
        ~isfield(cfg.synthesis, 'lipschitzConstant') && ~isfield(cfg.synthesis, 'accelerationGainConstant') && ...
        ~isfield(cfg.synthesis, 'issGainRelaxation') && ~isfield(cfg.synthesis, 'initialErrorWeight'), ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Use the current linear LPV configuration; nonlinear lateral dynamics are no longer supported.');
    assertFiniteVector(cfg.observer.initialState, 2, "cfg.observer.initialState");
    settings = struct("speedRange", speedRange, "speedRateRange", design.certificateDomain.speedRateRange);
end

function assertFiniteVector(value, expectedLength, name)
% assertFiniteVector Check a fixed-size finite vector.
    vector = double(value(:));
    assert(numel(vector) == expectedLength && all(isfinite(vector)), ...
        "%s must be a finite [%d x 1] vector.", name, expectedLength);
end
