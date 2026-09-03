function result = simulateLateralObserverScenario(design, cfg)
% simulateLateralObserverScenario: Exercise the LPV lateral observer on a
% synthetic drive. The vehicle follows a raised-cosine steering profile
% while the longitudinal speed varies sinusoidally, so both the scheduling
% parameter and its rate are excited; the lateral accelerometer and the
% gyroscope are corrupted by white noise; and the observer starts from a
% deliberately wrong lateral velocity. The truth comes from integrating the
% same LPV model the observer uses, so the residual error measures the
% observer, not a model mismatch.
%
% Input:
%   design: struct from designLateralObserverGains
%   cfg: optional struct from lateralObserverConfig
%
% Output:
%   result: struct with truth, measurements, estimate, and metrics
    if nargin < 2 || isempty(cfg)
        cfg = design.cfg;
    end
    truth = simulateTruth(design, cfg);
    measurements = buildMeasurements(truth, design, cfg);

    observerCfg = cfg;
    observerCfg.observer.initialState = truth.state(1, :).' + double(cfg.simulation.initialStateError(:));
    estimate = runLateralVelocityObserver(measurements, design, observerCfg);
    metrics = computeMetrics(truth, estimate, cfg);

    result = struct();
    result.truth = truth;
    result.measurements = measurements;
    result.estimate = estimate;
    result.metrics = metrics;
end

function truth = simulateTruth(design, cfg)
% simulateTruth: Integrate the true lateral dynamics along the scheduled
% speed profile with fourth-order Runge-Kutta.
%
% Input:
%   design: struct from designLateralObserverGains
%   cfg: struct from lateralObserverConfig
%
% Output:
%   truth: struct with time, speed and acceleration profiles, steering,
%       state, lateral acceleration, and side-slip angle
    sampleTime = double(cfg.simulation.sampleTime);
    numSteps = round(double(cfg.simulation.tFinal) ./ sampleTime);
    time = (0:numSteps).' .* sampleTime;
    [longitudinalSpeed, longitudinalAcceleration] = speedProfile(time, cfg);
    steeringAngle = steeringProfile(time, cfg);
    nonlinearity = cfg.observer.nonlinearity;
    model = design.model;

    state = zeros(numel(time), 2);
    for sampleIdx = 1:(numel(time) - 1)
        stepTime = time(sampleIdx + 1) - time(sampleIdx);
        current = state(sampleIdx, :).';
        k1 = plantDerivative(current, sampleIdx, 0.0, longitudinalSpeed, steeringAngle, model, nonlinearity);
        k2 = plantDerivative(current + (0.5 .* stepTime .* k1), sampleIdx, 0.5, longitudinalSpeed, steeringAngle, model, nonlinearity);
        k3 = plantDerivative(current + (0.5 .* stepTime .* k2), sampleIdx, 0.5, longitudinalSpeed, steeringAngle, model, nonlinearity);
        k4 = plantDerivative(current + (stepTime .* k3), sampleIdx, 1.0, longitudinalSpeed, steeringAngle, model, nonlinearity);
        state(sampleIdx + 1, :) = (current + ((stepTime ./ 6.0) .* (k1 + (2.0 .* k2) + (2.0 .* k3) + k4))).';
    end

    lateralAcceleration = zeros(numel(time), 1);
    for sampleIdx = 1:numel(time)
        [~, C] = evaluateLateralModel(model, [longitudinalSpeed(sampleIdx); 1.0 ./ longitudinalSpeed(sampleIdx)]);
        lateralAcceleration(sampleIdx) = (C(1, :) * state(sampleIdx, :).') + (model.D(1) .* steeringAngle(sampleIdx));
    end

    truth = struct();
    truth.time = time;
    truth.longitudinalSpeed = longitudinalSpeed;
    truth.longitudinalAcceleration = longitudinalAcceleration;
    truth.steeringAngle = steeringAngle;
    truth.state = state;
    truth.lateralVelocity = state(:, 1);
    truth.yawRate = state(:, 2);
    truth.lateralAcceleration = lateralAcceleration;
    truth.sideSlipAngle = atan2(state(:, 1), longitudinalSpeed);
end

function derivative = plantDerivative(state, sampleIdx, stepFraction, longitudinalSpeed, steeringAngle, model, nonlinearity)
% plantDerivative: Evaluate the true lateral dynamics inside one step,
% interpolating the scheduling speed and the steering input.
%
% Input:
%   state: [2 x 1] true state
%   sampleIdx: index of the interval start
%   stepFraction: position inside the interval, in [0, 1]
%   longitudinalSpeed, steeringAngle: input series
%   model: struct from lateralBicycleModel
%   nonlinearity: function handle f(x)
%
% Output:
%   derivative: [2 x 1] true state derivative
    nextIdx = min(sampleIdx + 1, numel(longitudinalSpeed));
    speed = ((1.0 - stepFraction) .* longitudinalSpeed(sampleIdx)) + (stepFraction .* longitudinalSpeed(nextIdx));
    steering = ((1.0 - stepFraction) .* steeringAngle(sampleIdx)) + (stepFraction .* steeringAngle(nextIdx));
    A = evaluateLateralModel(model, [speed; 1.0 ./ speed]);
    nonlinearTerm = double(nonlinearity(state));
    derivative = (A * state) + (model.B .* steering) + nonlinearTerm(:);
end

function [longitudinalSpeed, longitudinalAcceleration] = speedProfile(time, cfg)
% speedProfile: Build a sinusoidal longitudinal acceleration and the speed
% obtained by integrating it exactly, so the scheduling parameter and its
% rate stay mutually consistent.
%
% Input:
%   time: [N x 1] time vector
%   cfg: struct from lateralObserverConfig
%
% Output:
%   longitudinalSpeed: [N x 1] speed series in meters per second
%   longitudinalAcceleration: [N x 1] acceleration series
    amplitude = double(cfg.simulation.accelerationAmplitude);
    period = double(cfg.simulation.accelerationPeriod);
    initialSpeed = double(cfg.simulation.initialSpeed);
    angularRate = (2.0 .* pi) ./ period;
    longitudinalAcceleration = amplitude .* sin(angularRate .* time);
    longitudinalSpeed = initialSpeed + ((amplitude ./ angularRate) .* (1.0 - cos(angularRate .* time)));
    speedRange = double(cfg.scheduling.speedRange);
    assert(all(longitudinalSpeed > speedRange(1)) && all(longitudinalSpeed < speedRange(2)), ...
        "The simulated speed profile leaves the designed scheduling range.");
end

function steeringAngle = steeringProfile(time, cfg)
% steeringProfile: Interpolate the configured steering waypoints with
% raised-cosine blends so the input is continuous and differentiable.
%
% Input:
%   time: [N x 1] time vector
%   cfg: struct from lateralObserverConfig
%
% Output:
%   steeringAngle: [N x 1] front steering angle in radians
    waypointTime = double(cfg.simulation.steeringWaypointTime(:));
    waypointAngle = deg2rad(double(cfg.simulation.steeringWaypointDeg(:)));
    assert(numel(waypointTime) == numel(waypointAngle) && numel(waypointTime) >= 2 && ...
        all(diff(waypointTime) > 0), ...
        "steeringWaypointTime must be strictly increasing and match steeringWaypointDeg.");
    steeringAngle = waypointAngle(1) .* ones(size(time));
    for segmentIdx = 1:(numel(waypointTime) - 1)
        segmentMask = time >= waypointTime(segmentIdx) & time <= waypointTime(segmentIdx + 1);
        segmentFraction = min(max((time(segmentMask) - waypointTime(segmentIdx)) ./ ...
            (waypointTime(segmentIdx + 1) - waypointTime(segmentIdx)), 0.0), 1.0);
        blend = 0.5 - (0.5 .* cos(pi .* segmentFraction));
        steeringAngle(segmentMask) = waypointAngle(segmentIdx) + ...
            (blend .* (waypointAngle(segmentIdx + 1) - waypointAngle(segmentIdx)));
    end
    steeringAngle(time >= waypointTime(end)) = waypointAngle(end);
end

function measurements = buildMeasurements(truth, design, cfg)
% buildMeasurements: Corrupt the true outputs with white measurement noise
% and assemble the measurement struct consumed by the observer.
%
% Input:
%   truth: struct from simulateTruth
%   design: struct from designLateralObserverGains
%   cfg: struct from lateralObserverConfig
%
% Output:
%   measurements: struct accepted by runLateralVelocityObserver
    rng(double(cfg.simulation.randomSeed), "twister");
    numSamples = numel(truth.time);
    measurements = struct();
    measurements.time = truth.time;
    measurements.steeringAngle = truth.steeringAngle;
    measurements.longitudinalSpeed = truth.longitudinalSpeed;
    measurements.longitudinalAcceleration = truth.longitudinalAcceleration;
    measurements.lateralAcceleration = truth.lateralAcceleration + ...
        (double(cfg.simulation.lateralAccelerationNoiseStd) .* randn(numSamples, 1));
    measurements.yawRate = truth.yawRate + ...
        (double(cfg.simulation.yawRateNoiseStd) .* randn(numSamples, 1));
    measurements.designH2Bound = design.h2Bound;
end

function metrics = computeMetrics(truth, estimate, cfg)
% computeMetrics: Summarize the estimation error over the whole run and over
% the settled part of it, after the initial transient has decayed.
%
% Input:
%   truth: struct from simulateTruth
%   estimate: struct from runLateralVelocityObserver
%   cfg: struct from lateralObserverConfig
%
% Output:
%   metrics: struct with lateral velocity, yaw rate, and side-slip errors
    settleTime = 0.25 .* double(cfg.simulation.tFinal);
    settledMask = truth.time >= settleTime;
    lateralVelocityError = estimate.lateralVelocity - truth.lateralVelocity;
    yawRateError = estimate.yawRate - truth.yawRate;
    sideSlipError = estimate.sideSlipAngle - truth.sideSlipAngle;

    metrics = struct();
    metrics.lateralVelocityRmse = sqrt(mean(lateralVelocityError.^2));
    metrics.yawRateRmse = sqrt(mean(yawRateError.^2));
    metrics.settledLateralVelocityRmse = sqrt(mean(lateralVelocityError(settledMask).^2));
    metrics.settledYawRateRmse = sqrt(mean(yawRateError(settledMask).^2));
    metrics.settledSideSlipRmse = sqrt(mean(sideSlipError(settledMask).^2));
    metrics.maxSettledLateralVelocityError = max(abs(lateralVelocityError(settledMask)));
    metrics.peakLateralVelocity = max(abs(truth.lateralVelocity));
    metrics.peakSideSlipAngle = max(abs(truth.sideSlipAngle));
    metrics.settleTime = settleTime;
end
