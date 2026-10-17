function estimate = runLateralVelocityObserver(measurements, design, cfg)
% runLateralVelocityObserver Estimate lateral motion with bumpless fusion.
% A division-free master state [vy; lateral-accelerometer bias] runs at every
% speed. The certified LPV bicycle observer remains a persistent information
% source, but its reciprocal-speed model is evaluated only inside the design
% speed interval. Its lateral-velocity estimate is the only correction source
% for the master, acting through one persistent injection state:
%
%   xhatKDot = fK(xhatK,u) + etaDynamic
%   tau etaDynamicDot = -etaDynamic + nuDynamicTarget.
%
% Outside the domain, or when dynamicValid is false, the correction target is
% zero and the stored injection decays. Inertial continuation has no low-speed
% drift guarantee; no stationary or kinematic velocity constraint is applied.
%
% A mode change alters only nuDynamicTarget. It never resets the master state, an
% injection state, or an exported quantity, so xhatK and xhatKDot have no
% mode-induced jump for continuous physical inputs. A persistent second-order
% side-slip interface state similarly exports continuous beta and betaDot.
%
% Input:
%   measurements: struct with aligned column series time, steeringAngle,
%       longitudinalSpeed, longitudinalAcceleration, lateralAcceleration,
%       and yawRate. An optional logical dynamicValid series can immediately
%       withdraw the LPV information channel without resetting any state.
%   design: struct from designLateralObserverGains
%   cfg: optional current struct from lateralObserverConfig. Designs
%       must include the current hybrid runtime configuration.
%
% Output:
%   estimate: struct containing the master and hidden dynamic states, smooth
%       side-slip interface, persistent LPV correction, operating-mode
%       diagnostics, and the normalized input measurements
    if nargin < 3 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(isfield(cfg,"hybrid"),"VehicleLocalization:MissingHybridConfiguration", ...
        "Use the current lateralObserverConfig.");
    assert(isfield(cfg,"outputPoint") && isfield(cfg.outputPoint,"forwardOffsetM") && ...
        isscalar(cfg.outputPoint.forwardOffsetM) && isfinite(double(cfg.outputPoint.forwardOffsetM)), ...
        "VehicleLocalization:MissingOutputPointConfiguration", ...
        "Use the current lateralObserverConfig with an outputPoint.forwardOffsetM entry.");
    forwardOffset = double(cfg.outputPoint.forwardOffsetM);
    lateralObserverSupport.assertLateralVehicleMatches(design,cfg);
    measurements = normalizeMeasurements(measurements);
    settings = validateRuntimeConfiguration(cfg, design);
    layout = stateLayout();
    sampleCount = numel(measurements.time);

    augmentedState = initialAugmentedState(measurements, cfg, settings, layout);
    masterHistory = zeros(sampleCount, 2);
    dynamicHistory = zeros(sampleCount, 2);
    sideSlipHistory = zeros(sampleCount, 2);
    correctionHistory = zeros(sampleCount, 2);
    correctionTargetHistory = zeros(sampleCount, 2);
    innovationHistory = zeros(sampleCount, 2);
    gainNorm = zeros(sampleCount, 1);
    lateralVelocityRate = zeros(sampleCount, 1);
    longitudinalSpeedRate = zeros(sampleCount, 1);
    sideSlipCommand = zeros(sampleCount, 1);
    dynamicParticipation = zeros(sampleCount, 1);
    dynamicModelEvaluated = false(sampleCount, 1);
    sideSlipCommandValid = false(sampleCount, 1);
    operatingMode = strings(sampleCount, 1);
    modeChanged = false(sampleCount, 1);

    integrationMethod = lower(strtrim(string(cfg.observer.integrationMethod)));

    for sampleIdx = 1:sampleCount
        sample = sampleAt(measurements, sampleIdx, 0.0);
        [derivative, diagnostics] = hybridDerivative( ...
            augmentedState, sample, design, cfg, settings, layout);
        masterHistory(sampleIdx, :) = augmentedState(layout.master).';
        dynamicHistory(sampleIdx, :) = augmentedState(layout.dynamic).';
        sideSlipHistory(sampleIdx, :) = augmentedState(layout.sideSlip).';
        correctionHistory(sampleIdx, :) = augmentedState(layout.corrections).';
        correctionTargetHistory(sampleIdx, :) = diagnostics.correctionTarget(:).';
        innovationHistory(sampleIdx, :) = diagnostics.dynamicInnovation.';
        gainNorm(sampleIdx) = diagnostics.gainNorm;
        lateralVelocityRate(sampleIdx) = derivative(layout.master(1));
        longitudinalSpeedRate(sampleIdx) = diagnostics.longitudinalSpeedRate;
        sideSlipCommand(sampleIdx) = diagnostics.sideSlipCommand;
        dynamicParticipation(sampleIdx) = diagnostics.dynamicParticipation;
        dynamicModelEvaluated(sampleIdx) = diagnostics.dynamicModelEvaluated;
        sideSlipCommandValid(sampleIdx) = diagnostics.sideSlipCommandValid;
        if diagnostics.dynamicModelEvaluated
            operatingMode(sampleIdx) = "dynamic";
        else
            operatingMode(sampleIdx) = "inertial";
        end
        if sampleIdx > 1
            modeChanged(sampleIdx) = operatingMode(sampleIdx) ~= operatingMode(sampleIdx - 1);
        end

        if sampleIdx == sampleCount
            break;
        end
        stepTime = measurements.time(sampleIdx + 1) - measurements.time(sampleIdx);
        if integrationMethod == "euler"
            augmentedState = augmentedState + (stepTime .* derivative);
        else
            midSample = sampleAt(measurements, sampleIdx, 0.5);
            nextSample = sampleAt(measurements, sampleIdx, 1.0);
            k1 = derivative;
            k2 = hybridDerivative(augmentedState + (0.5 .* stepTime .* k1), ...
                midSample, design, cfg, settings, layout);
            k3 = hybridDerivative(augmentedState + (0.5 .* stepTime .* k2), ...
                midSample, design, cfg, settings, layout);
            k4 = hybridDerivative(augmentedState + (stepTime .* k3), ...
                nextSample, design, cfg, settings, layout);
            augmentedState = augmentedState + ((stepTime ./ 6.0) .* ...
                (k1 + (2.0 .* k2) + (2.0 .* k3) + k4));
        end
        assert(all(isfinite(augmentedState)), ...
            "The hybrid lateral observer diverged in interval %d.", sampleIdx);
    end

    estimate = struct();
    estimate.time = measurements.time;
    estimate.state = masterHistory;
    estimate.stateNames = ["lateralVelocity", "lateralAccelerationBias"];
    estimate.dynamicState = dynamicHistory;
    estimate.dynamicStateNames = ["lateralVelocity", "yawRate"];
    % The master state is the lateral velocity at the observer (IMU) point.
    % Rigid-body transport exports it at the declared output point.
    estimate.observerPointLateralVelocity = masterHistory(:, 1);
    estimate.lateralVelocity = masterHistory(:, 1) - forwardOffset .* measurements.yawRate;
    estimate.outputPoint = cfg.outputPoint;
    estimate.lateralAccelerationBias = masterHistory(:, 2);
    estimate.yawRate = dynamicHistory(:, 2);
    estimate.lateralVelocityRate = lateralVelocityRate;
    estimate.longitudinalSpeedRate = longitudinalSpeedRate;
    estimate.sideSlipAngle = sideSlipHistory(:, 1);
    estimate.sideSlipAngleRate = sideSlipHistory(:, 2);
    estimate.sideSlipCommand = sideSlipCommand;
    estimate.innovation = innovationHistory;
    estimate.gainNorm = gainNorm;
    estimate.scheduledSpeed = measurements.longitudinalSpeed;
    estimate.mode = operatingMode;
    estimate.modeChanged = modeChanged;
    estimate.correctionInjection = struct("dynamic", correctionHistory, ...
        "total", correctionHistory);
    estimate.correctionTarget = struct("dynamic", correctionTargetHistory);
    estimate.diagnostics = struct("dynamicModelEvaluated", dynamicModelEvaluated, ...
        "dynamicParticipation", dynamicParticipation, ...
        "sideSlipCommandValid", sideSlipCommandValid, ...
        "modeChanged", modeChanged);
    estimate.measurements = measurements;
end

function [derivative, diagnostics] = hybridDerivative(augmentedState, sample, ...
        design, cfg, settings, layout)
% hybridDerivative Evaluate inertial propagation and the LPV correction target.
    masterState = augmentedState(layout.master);
    dynamicState = augmentedState(layout.dynamic);
    correctionState = augmentedState(layout.corrections);

    masterPrediction = [sample.lateralAcceleration - masterState(2) - ...
        (sample.yawRate .* sample.longitudinalSpeed); ...
        -masterState(2) ./ double(cfg.hybrid.accelerometerBiasTimeConstant)];
    longitudinalSpeedRate = sample.longitudinalAcceleration + ...
        (masterState(1) .* sample.yawRate);

    dynamicModelSafe = sample.dynamicValid && ...
        sample.longitudinalSpeed >= settings.speedRange(1) && ...
        sample.longitudinalSpeed <= settings.speedRange(2);
    if dynamicModelSafe
        [dynamicDerivative, dynamicInnovation, dynamicGain] = ...
            dynamicObserverDerivative(dynamicState, sample, design, cfg.observer.nonlinearity);
    else
        dynamicDerivative = [double(cfg.hybrid.shadow.lateralVelocityTrackingGain) .* ...
            (masterState(1) - dynamicState(1)); ...
            double(cfg.hybrid.shadow.yawRateTrackingGain) .* ...
            (sample.yawRate - dynamicState(2))];
        dynamicInnovation = zeros(2, 1);
        dynamicGain = zeros(2, 2);
    end

    lowerSpeedParticipation = quinticStep(sample.longitudinalSpeed, ...
        settings.speedRange(1), double(cfg.hybrid.dynamic.fullParticipationSpeed));
    upperSpeedParticipation = 1.0 - quinticStep(sample.longitudinalSpeed, ...
        double(cfg.hybrid.dynamic.upperFullParticipationSpeed), settings.speedRange(2));
    lateralAccelerationParticipation = upperParticipation(abs(sample.lateralAcceleration), ...
        cfg.hybrid.dynamic.fullLateralAcceleration, cfg.hybrid.dynamic.zeroLateralAcceleration);
    lateralInnovationParticipation = upperParticipation(abs(dynamicInnovation(1)), ...
        cfg.hybrid.dynamic.fullLateralInnovation, cfg.hybrid.dynamic.zeroLateralInnovation);
    yawInnovationParticipation = upperParticipation(abs(dynamicInnovation(2)), ...
        cfg.hybrid.dynamic.fullYawInnovation, cfg.hybrid.dynamic.zeroYawInnovation);
    dynamicParticipation = double(dynamicModelSafe) .* lowerSpeedParticipation .* ...
        upperSpeedParticipation .* lateralAccelerationParticipation .* ...
        lateralInnovationParticipation .* yawInnovationParticipation;
    dynamicTarget = correctionTarget(dynamicState(1) - masterState(1), ...
        cfg.hybrid.correction.dynamic, dynamicParticipation);
    correctionDerivative = (dynamicTarget - correctionState) ./ ...
        double(cfg.hybrid.correction.dynamic.timeConstant);

    masterDerivative = masterPrediction + correctionState;
    betaDerivative = sideSlipDerivative(masterState, ...
        augmentedState(layout.sideSlip), sample, cfg.hybrid.sideSlip, cfg.outputPoint);
    derivative = [masterDerivative; dynamicDerivative; correctionDerivative(:); betaDerivative];

    if nargout > 1
        [betaCommand, betaCommandValid] = sideSlipCommandAt(masterState, sample, cfg.hybrid.sideSlip, cfg.outputPoint);
        diagnostics = struct();
        diagnostics.dynamicInnovation = dynamicInnovation;
        diagnostics.gainNorm = norm(dynamicGain);
        diagnostics.longitudinalSpeedRate = longitudinalSpeedRate;
        diagnostics.dynamicModelEvaluated = dynamicModelSafe;
        diagnostics.dynamicParticipation = dynamicParticipation;
        diagnostics.correctionTarget = dynamicTarget;
        diagnostics.sideSlipCommand = betaCommand;
        diagnostics.sideSlipCommandValid = betaCommandValid;
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

function derivative = sideSlipDerivative(masterState, sideSlipState, sample, sideSlipCfg, outputPoint)
% sideSlipDerivative Evaluate the persistent second-order interface filter.
    [command, ~] = sideSlipCommandAt(masterState, sample, sideSlipCfg, outputPoint);
    naturalFrequency = double(sideSlipCfg.naturalFrequency);
    dampingRatio = double(sideSlipCfg.dampingRatio);
    angleError = wrapAngleDifference(command - sideSlipState(1));
    derivative = [sideSlipState(2); ...
        (naturalFrequency.^2 .* angleError) - ...
        (2.0 .* dampingRatio .* naturalFrequency .* sideSlipState(2))];
end

function [command, valid] = sideSlipCommandAt(masterState, sample, sideSlipCfg, outputPoint)
% sideSlipCommandAt Form atan2 of the output-point velocity where its direction is valid.
    valid = sample.longitudinalSpeed >= double(sideSlipCfg.validSpeed);
    if valid
        participation = quinticStep(sample.longitudinalSpeed, ...
            double(sideSlipCfg.validSpeed), ...
            double(sideSlipCfg.fullParticipationSpeed));
        outputPointLateralVelocity = masterState(1) - ...
            double(outputPoint.forwardOffsetM) .* sample.yawRate;
        command = participation .* atan2(outputPointLateralVelocity, sample.longitudinalSpeed);
    else
        command = 0.0;
    end
end

function target = correctionTarget(residual, correctionCfg, participation)
% correctionTarget Convert a vy residual into a two-state correction target.
    target = participation .* [double(correctionCfg.velocityGain) .* residual; ...
        -double(correctionCfg.biasGain) .* residual];
end

function value = upperParticipation(magnitude, fullMagnitude, zeroMagnitude)
% upperParticipation Return one below fullMagnitude and zero above zeroMagnitude.
    value = 1.0 - quinticStep(magnitude, double(fullMagnitude), double(zeroMagnitude));
end

function value = quinticStep(argument, lowerBound, upperBound)
% quinticStep C2 transition with zero first and second endpoint derivatives.
    fraction = min(max((double(argument) - double(lowerBound)) ./ ...
        (double(upperBound) - double(lowerBound)), 0.0), 1.0);
    value = (6.0 .* fraction.^5) - (15.0 .* fraction.^4) + (10.0 .* fraction.^3);
end

function augmentedState = initialAugmentedState(measurements, cfg, settings, layout)
% initialAugmentedState Build all persistent states without a mode reset.
    dynamicState = double(cfg.observer.initialState(:));
    masterState = double(cfg.hybrid.initialMasterState(:));
    if isempty(masterState)
        masterState = [dynamicState(1); 0.0];
    end
    correctionState = double(cfg.hybrid.initialCorrectionState);
    firstSample = sampleAt(measurements, 1, 0.0);
    sideSlipState = double(cfg.hybrid.sideSlip.initialState(:));
    if isempty(sideSlipState)
        [initialCommand, ~] = sideSlipCommandAt(masterState, firstSample, cfg.hybrid.sideSlip, cfg.outputPoint);
        sideSlipState = [initialCommand; 0.0];
    end
    augmentedState = zeros(layout.stateCount, 1);
    augmentedState(layout.master) = masterState;
    augmentedState(layout.dynamic) = dynamicState;
    augmentedState(layout.corrections) = correctionState(:);
    augmentedState(layout.sideSlip) = sideSlipState;
    assert(all(isfinite(augmentedState)), "Every hybrid observer initial state must be finite.");
    assert(settings.speedRange(1) > 0.0, "The dynamic speed certificate must exclude zero speed.");
end

function layout = stateLayout()
% stateLayout Define the private augmented-state slices.
    layout = struct();
    layout.master = 1:2;
    layout.dynamic = 3:4;
    layout.corrections = 5:6;
    layout.sideSlip = 7:8;
    layout.stateCount = 8;
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
% validateRuntimeConfiguration Check dimensions and smooth-transition bounds.
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
    if ~isempty(cfg.hybrid.initialMasterState)
        assertFiniteVector(cfg.hybrid.initialMasterState, 2, "cfg.hybrid.initialMasterState");
    end
    correctionInitialState = double(cfg.hybrid.initialCorrectionState);
    assert(isequal(size(correctionInitialState), [2, 1]) && all(isfinite(correctionInitialState), "all"), ...
        "cfg.hybrid.initialCorrectionState must be a finite [2 x 1] vector.");
    if ~isempty(cfg.hybrid.sideSlip.initialState)
        assertFiniteVector(cfg.hybrid.sideSlip.initialState, 2, "cfg.hybrid.sideSlip.initialState");
    end

    assertPositiveScalar(cfg.hybrid.accelerometerBiasTimeConstant, ...
        "cfg.hybrid.accelerometerBiasTimeConstant");
    validateCorrectionConfiguration(cfg.hybrid.correction.dynamic, ...
        "cfg.hybrid.correction.dynamic");
    assert(double(cfg.hybrid.dynamic.fullParticipationSpeed) > speedRange(1) && ...
        double(cfg.hybrid.dynamic.fullParticipationSpeed) < speedRange(2), ...
        "cfg.hybrid.dynamic.fullParticipationSpeed must lie inside the dynamic certificate.");
    assert(double(cfg.hybrid.dynamic.upperFullParticipationSpeed) > ...
        double(cfg.hybrid.dynamic.fullParticipationSpeed) && ...
        double(cfg.hybrid.dynamic.upperFullParticipationSpeed) < speedRange(2), ...
        "cfg.hybrid.dynamic.upperFullParticipationSpeed must lie between the lower full-participation speed and the certificate maximum.");
    validateTransitionBounds(cfg.hybrid.dynamic.fullLateralAcceleration, ...
        cfg.hybrid.dynamic.zeroLateralAcceleration, "lateral-acceleration");
    validateTransitionBounds(cfg.hybrid.dynamic.fullLateralInnovation, ...
        cfg.hybrid.dynamic.zeroLateralInnovation, "lateral-innovation");
    validateTransitionBounds(cfg.hybrid.dynamic.fullYawInnovation, ...
        cfg.hybrid.dynamic.zeroYawInnovation, "yaw-innovation");
    assertPositiveScalar(cfg.hybrid.shadow.lateralVelocityTrackingGain, ...
        "cfg.hybrid.shadow.lateralVelocityTrackingGain");
    assertPositiveScalar(cfg.hybrid.shadow.yawRateTrackingGain, ...
        "cfg.hybrid.shadow.yawRateTrackingGain");
    assertPositiveScalar(cfg.hybrid.sideSlip.validSpeed, "cfg.hybrid.sideSlip.validSpeed");
    assert(double(cfg.hybrid.sideSlip.fullParticipationSpeed) > ...
        double(cfg.hybrid.sideSlip.validSpeed), ...
        "cfg.hybrid.sideSlip.fullParticipationSpeed must exceed validSpeed.");
    assertPositiveScalar(cfg.hybrid.sideSlip.naturalFrequency, ...
        "cfg.hybrid.sideSlip.naturalFrequency");
    assertPositiveScalar(cfg.hybrid.sideSlip.dampingRatio, ...
        "cfg.hybrid.sideSlip.dampingRatio");
    settings = struct("speedRange", speedRange);
end

function validateCorrectionConfiguration(correctionCfg, name)
% validateCorrectionConfiguration Check one persistent injection channel.
    assertPositiveScalar(correctionCfg.timeConstant, name + ".timeConstant");
    assertNonnegativeScalar(correctionCfg.velocityGain, name + ".velocityGain");
    assertNonnegativeScalar(correctionCfg.biasGain, name + ".biasGain");
end

function validateTransitionBounds(fullValue, zeroValue, name)
% validateTransitionBounds Require a nonempty C2 fade interval.
    assertNonnegativeScalar(fullValue, "full " + name + " threshold");
    assert(double(zeroValue) > double(fullValue) && isfinite(double(zeroValue)), ...
        "The zero-participation %s threshold must exceed its full-participation threshold.", name);
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

function assertNonnegativeScalar(value, name)
% assertNonnegativeScalar Check a nonnegative finite scalar.
    numericValue = double(value);
    assert(isscalar(numericValue) && isfinite(numericValue) && numericValue >= 0.0, ...
        "%s must be a nonnegative finite scalar.", name);
end

function wrapped = wrapAngleDifference(angle)
% wrapAngleDifference Wrap an angular error to [-pi, pi).
    wrapped = mod(angle + pi, 2.0 .* pi) - pi;
end
