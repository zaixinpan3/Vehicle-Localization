function estimate = runLateralVelocityObserver(measurements, design, cfg)
% runLateralVelocityObserver: Estimate the ego lateral velocity and yaw rate
% from steering, wheel speed, and inertial measurements with the LPV
% observer synthesized by designLateralObserverGains:
%
%   xhat_dot = A(rho) xhat + B u + f(xhat) + L(rho) (y - C(rho) xhat - D u),
%
% where rho = [Vx; 1/Vx] is scheduled by the measured longitudinal speed and
% the gain is looked up from the design grid. The yaw rate is measured
% directly, so the estimator effectively reconstructs the lateral velocity
% and, with it, the vehicle side-slip angle.
%
% This block supplies the side-slip interface that the ego-state observer
% consumes: the side-slip angle and, crucially, its rate, which enters the
% track-angle rate as thetaDot = yawRate + sideSlipRate. The rate is taken
% analytically from the first row of the observer right-hand side, never by
% differentiating the estimate numerically. Below cfg.observer.minimumSpeed
% the side-slip outputs are held at zero, because atan2(vy, Vx) and its rate
% lose meaning as the speed vanishes.
%
% Input:
%   measurements: struct with column series time, steeringAngle,
%       longitudinalSpeed, longitudinalAcceleration, lateralAcceleration,
%       and yawRate. longitudinalAcceleration is the inertial longitudinal
%       specific force ax measured by the IMU, not the derivative of the
%       speed: the speed rate is reconstructed internally as
%       VxDot = ax + vy r, and it is that rate, not ax, that schedules the
%       gain and enters the side-slip rate.
%   design: struct from designLateralObserverGains
%   cfg: optional struct from lateralObserverConfig
%
% Output:
%   estimate: struct with time, state [N x 2] of lateral velocity and yaw
%       rate, lateralVelocity, yawRate, lateralVelocityRate, sideSlipAngle,
%       sideSlipAngleRate, longitudinalSpeedRate, lowSpeedHold, innovation
%       [N x 2], gainNorm, and the scheduling series actually used
    if nargin < 3 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    measurements = normalizeMeasurements(measurements);
    numSamples = numel(measurements.time);

    nonlinearity = cfg.observer.nonlinearity;
    assert(isa(nonlinearity, "function_handle"), "cfg.observer.nonlinearity must be a function handle.");
    minimumSpeed = double(cfg.observer.minimumSpeed);
    assert(isscalar(minimumSpeed) && isfinite(minimumSpeed) && minimumSpeed > 0, ...
        "cfg.observer.minimumSpeed must be a positive finite scalar.");
    integrationMethod = lower(strtrim(string(cfg.observer.integrationMethod)));
    assert(integrationMethod == "rk4" || integrationMethod == "euler", ...
        "cfg.observer.integrationMethod must be rk4 or euler.");

    state = double(cfg.observer.initialState(:));
    assert(numel(state) == 2 && all(isfinite(state)), "cfg.observer.initialState must be a finite [2 x 1] state.");

    stateHistory = zeros(numSamples, 2);
    innovation = zeros(numSamples, 2);
    gainNorm = zeros(numSamples, 1);
    scheduledSpeed = zeros(numSamples, 1);
    lateralVelocityRate = zeros(numSamples, 1);
    longitudinalSpeedRate = zeros(numSamples, 1);
    lowSpeedHold = false(numSamples, 1);
    stateHistory(1, :) = state.';

    for sampleIdx = 1:numSamples
        sample = sampleAt(measurements, sampleIdx, 0.0, minimumSpeed);
        [derivative, sampleInnovation, L, speedRate] = observerDerivative(state, sample, design, nonlinearity);
        innovation(sampleIdx, :) = sampleInnovation.';
        gainNorm(sampleIdx) = norm(L);
        scheduledSpeed(sampleIdx) = sample.longitudinalSpeed;
        lateralVelocityRate(sampleIdx) = derivative(1);
        longitudinalSpeedRate(sampleIdx) = speedRate;
        lowSpeedHold(sampleIdx) = sample.belowMinimumSpeed;
        stateHistory(sampleIdx, :) = state.';
        if sampleIdx == numSamples
            break;
        end

        stepTime = measurements.time(sampleIdx + 1) - measurements.time(sampleIdx);
        assert(stepTime > 0, "measurements.time must be strictly increasing.");
        if integrationMethod == "euler"
            state = state + (stepTime .* derivative);
        else
            midSample = sampleAt(measurements, sampleIdx, 0.5, minimumSpeed);
            nextSample = sampleAt(measurements, sampleIdx, 1.0, minimumSpeed);
            k1 = derivative;
            k2 = observerDerivative(state + (0.5 .* stepTime .* k1), midSample, design, nonlinearity);
            k3 = observerDerivative(state + (0.5 .* stepTime .* k2), midSample, design, nonlinearity);
            k4 = observerDerivative(state + (stepTime .* k3), nextSample, design, nonlinearity);
            state = state + ((stepTime ./ 6.0) .* (k1 + (2.0 .* k2) + (2.0 .* k3) + k4));
        end
        assert(all(isfinite(state)), "The lateral observer diverged at sample %d.", sampleIdx);
    end

    % Side-slip angle and its analytic rate, both held at zero below the
    % minimum scheduling speed where they lose meaning
    lateralVelocity = stateHistory(:, 1);
    sideSlipAngle = atan2(lateralVelocity, scheduledSpeed);
    sideSlipDenominator = (scheduledSpeed.^2) + (lateralVelocity.^2);
    sideSlipAngleRate = ((lateralVelocityRate .* scheduledSpeed) - ...
        (lateralVelocity .* longitudinalSpeedRate)) ./ sideSlipDenominator;
    sideSlipAngle(lowSpeedHold) = 0.0;
    sideSlipAngleRate(lowSpeedHold) = 0.0;

    estimate = struct();
    estimate.time = measurements.time;
    estimate.state = stateHistory;
    estimate.lateralVelocity = lateralVelocity;
    estimate.yawRate = stateHistory(:, 2);
    estimate.lateralVelocityRate = lateralVelocityRate;
    estimate.sideSlipAngle = sideSlipAngle;
    estimate.sideSlipAngleRate = sideSlipAngleRate;
    estimate.longitudinalSpeedRate = longitudinalSpeedRate;
    estimate.lowSpeedHold = lowSpeedHold;
    estimate.innovation = innovation;
    estimate.gainNorm = gainNorm;
    estimate.scheduledSpeed = scheduledSpeed;
    estimate.measurements = measurements;
end

function [derivative, innovation, L, speedRate] = observerDerivative(state, sample, design, nonlinearity)
% observerDerivative: Evaluate the observer vector field at one operating
% point: the LPV model prediction, the copied nonlinearity, and the
% gain-weighted output innovation. The scheduling rate is the speed
% derivative VxDot = ax + vy r reconstructed from the measured longitudinal
% specific force and the current estimate, not the specific force itself.
%
% Input:
%   state: [2 x 1] current estimate of [lateral velocity; yaw rate]
%   sample: struct with the interpolated measurements of one instant
%   design: struct from designLateralObserverGains
%   nonlinearity: function handle f(x)
%
% Output:
%   derivative: [2 x 1] state derivative
%   innovation: [2 x 1] output innovation
%   L: [2 x 2] scheduled observer gain
%   speedRate: scalar reconstructed longitudinal speed derivative
    model = design.model;
    rho = [sample.longitudinalSpeed; 1.0 ./ sample.longitudinalSpeed];
    [A, C] = evaluateLateralModel(model, rho);
    speedRate = sample.longitudinalAcceleration + (state(1) .* sample.yawRate);
    L = scheduleLateralObserverGain(design, sample.longitudinalSpeed, speedRate);

    measuredOutput = [sample.lateralAcceleration; sample.yawRate];
    predictedOutput = (C * state) + (model.D .* sample.steeringAngle);
    innovation = measuredOutput - predictedOutput;

    nonlinearTerm = nonlinearity(state);
    nonlinearTerm = double(nonlinearTerm(:));
    assert(numel(nonlinearTerm) == 2 && all(isfinite(nonlinearTerm)), ...
        "cfg.observer.nonlinearity must return a finite [2 x 1] vector.");

    derivative = (A * state) + (model.B .* sample.steeringAngle) + nonlinearTerm + (L * innovation);
end

function sample = sampleAt(measurements, sampleIdx, stepFraction, minimumSpeed)
% sampleAt: Interpolate the measurement series inside one integration step,
% and floor the scheduling speed so the reciprocal stays finite.
%
% Input:
%   measurements: normalized measurement struct
%   sampleIdx: index of the interval start
%   stepFraction: position inside the interval, in [0, 1]
%   minimumSpeed: lower bound applied to the scheduling speed
%
% Output:
%   sample: struct with the interpolated scalar measurements
    fields = ["steeringAngle", "longitudinalSpeed", "longitudinalAcceleration", "lateralAcceleration", "yawRate"];
    nextIdx = min(sampleIdx + 1, numel(measurements.time));
    sample = struct();
    for fieldName = fields
        series = measurements.(fieldName);
        sample.(fieldName) = ((1.0 - stepFraction) .* series(sampleIdx)) + (stepFraction .* series(nextIdx));
    end
    sample.belowMinimumSpeed = sample.longitudinalSpeed < minimumSpeed;
    sample.longitudinalSpeed = max(sample.longitudinalSpeed, minimumSpeed);
end

function measurements = normalizeMeasurements(measurements)
% normalizeMeasurements: Validate the measurement series and return them as
% aligned finite column vectors.
%
% Input:
%   measurements: struct with the required measurement series
%
% Output:
%   measurements: struct with validated [N x 1] double series
    requiredFields = ["time", "steeringAngle", "longitudinalSpeed", "longitudinalAcceleration", ...
        "lateralAcceleration", "yawRate"];
    assert(isstruct(measurements), "measurements must be a struct of measurement series.");
    for fieldName = requiredFields
        assert(isfield(measurements, fieldName), "measurements is missing required series: %s.", fieldName);
    end
    numSamples = numel(measurements.time);
    assert(numSamples >= 2, "measurements must contain at least two samples.");
    for fieldName = requiredFields
        series = double(measurements.(fieldName)(:));
        assert(numel(series) == numSamples && all(isfinite(series)), ...
            "measurements.%s must be a finite series aligned with measurements.time.", fieldName);
        measurements.(fieldName) = series;
    end
    assert(all(diff(measurements.time) > 0), "measurements.time must be strictly increasing.");
    assert(all(measurements.longitudinalSpeed > 0), "measurements.longitudinalSpeed must be strictly positive.");
end
