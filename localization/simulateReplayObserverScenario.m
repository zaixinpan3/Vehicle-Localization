function result = simulateReplayObserverScenario(cfg)
% simulateReplayObserverScenario: Exercise the replay observer on a
% synthetic drive. A kinematic bicycle follows a raised-cosine steering
% profile at constant speed; noisy 100 Hz speed, yaw-rate, steering, and
% constraint measurements and delayed, partly out-of-order 5 Hz [x, y, yaw]
% pose measurements are generated from the truth; the observer estimate is
% then compared with the truth.
%
% Input:
%   cfg: optional struct from replayObserverConfig with a designed replay.Kd
%
% Output:
%   result: struct with truth, highRateMeasurements, poseMeasurements,
%       estimate, and metrics
    if nargin < 1 || isempty(cfg)
        cfg = replayObserverConfig();
    end
    [truth, highRateMeasurements, poseMeasurements, observerCfg] = simulateReplayScenario(cfg);
    estimate = runReplayHighGainObserver(highRateMeasurements, poseMeasurements, observerCfg);
    metrics = computeReplayMetrics(truth, estimate);

    result = struct();
    result.truth = truth;
    result.highRateMeasurements = highRateMeasurements;
    result.poseMeasurements = poseMeasurements;
    result.estimate = estimate;
    result.metrics = metrics;
end

function [truth, highRateMeasurements, poseMeasurements, observerCfg] = simulateReplayScenario(cfg)
% simulateReplayScenario: Generate a multi-turn kinematic bicycle
% scenario, noisy 100 Hz high-rate measurements, delayed 5 Hz pose
% measurements, and a biased initial observer state.
%
% Input:
%   cfg: replay observer configuration struct
%
% Output:
%   truth: true vehicle trajectory and transformed state
%   highRateMeasurements: simulated high-rate measurement stream
%   poseMeasurements: simulated timestamped low-rate pose stream
%   observerCfg: cfg copy with scenario-specific initial state
    sampleTime = 1.0 ./ cfg.replay.highRateHz;
    numSteps = round(cfg.replaySimulation.tFinal ./ sampleTime);
    time = (0:numSteps).' .* sampleTime;
    speed = cfg.replaySimulation.speed .* ones(size(time));
    steeringAngle = steeringProfile(time, cfg);
    pose = integrateVehiclePose(time, speed, steeringAngle, cfg);
    beta = sideSlipAngle(steeringAngle, cfg);
    yawRate = speed ./ cfg.vehicle.wheelbase .* cos(beta) .* tan(steeringAngle);
    velocity = [speed .* cos(pose(:, 3) + beta), speed .* sin(pose(:, 3) + beta)];
    acceleration = [gradient(velocity(:, 1), sampleTime), gradient(velocity(:, 2), sampleTime)];
    transformedState = [pose(:, 1), velocity(:, 1), acceleration(:, 1), pose(:, 2), velocity(:, 2), acceleration(:, 2)];

    rng(cfg.replaySimulation.randomSeed, "twister");
    sensorNoiseScale = double(readReplaySimulationLogical(cfg, "enableSensorNoise", true));
    speedNoiseStd = sensorNoiseScale .* readReplaySimulationScalar(cfg, "speedNoiseStd", 0.0);
    yawRateNoiseStd = sensorNoiseScale .* readReplaySimulationScalar(cfg, "yawRateNoiseStd", 0.0);
    steeringNoiseStd = sensorNoiseScale .* readReplaySimulationScalar(cfg, "steeringAngleNoiseStd", 0.0);
    constraintNoiseStd = sensorNoiseScale .* readReplaySimulationScalar(cfg, "orthogonalityConstraintNoiseStd", 0.0);
    positionNoiseStd = sensorNoiseScale .* readReplaySimulationScalar(cfg, "positionNoiseStd", 0.0);
    yawNoiseStd = sensorNoiseScale .* readReplaySimulationScalar(cfg, "yawNoiseStd", 0.0);

    highRateMeasurements = struct();
    highRateMeasurements.time = time;
    highRateMeasurements.speed = speed + speedNoiseStd .* randn(size(speed));
    highRateMeasurements.yawRate = yawRate + yawRateNoiseStd .* randn(size(yawRate));
    highRateMeasurements.steeringAngle = steeringAngle + steeringNoiseStd .* randn(size(steeringAngle));
    highRateMeasurements.orthogonalityConstraint = constraintNoiseStd .* randn(size(time));

    poseStep = max(1, round(cfg.replay.highRateHz ./ cfg.replay.poseRateHz));
    poseIdx = (1:poseStep:numel(time)).';
    poseTimestamp = time(poseIdx);
    poseArrivalTime = poseTimestamp + cfg.replaySimulation.poseDelay;
    if logical(cfg.replaySimulation.enableOutOfOrderArrival) && numel(poseArrivalTime) >= 6
        poseArrivalTime(5) = poseArrivalTime(5) + cfg.replaySimulation.outOfOrderExtraDelay;
    end
    poseNoise = [positionNoiseStd .* randn(numel(poseIdx), 2), yawNoiseStd .* randn(numel(poseIdx), 1)];

    poseMeasurements = struct();
    poseMeasurements.timestamp = poseTimestamp;
    poseMeasurements.arrivalTime = poseArrivalTime;
    poseMeasurements.pose = [pose(poseIdx, 1:2), wrapAngleToPi(pose(poseIdx, 3))] + poseNoise;
    poseMeasurements.pose(:, 3) = wrapAngleToPi(poseMeasurements.pose(:, 3));

    observerCfg = cfg;
    observerCfg.initialization.state = initialObserverState(cfg, steeringAngle(1));

    truth = struct();
    truth.time = time;
    truth.pose = pose;
    truth.position = pose(:, 1:2);
    truth.yaw = wrapAngleToPi(pose(:, 3));
    truth.beta = beta;
    truth.yawRate = yawRate;
    truth.speed = speed;
    truth.steeringAngle = steeringAngle;
    truth.velocity = velocity;
    truth.acceleration = acceleration;
    truth.z = transformedState;
end

function steeringAngle = steeringProfile(time, cfg)
% steeringProfile: Generate the smooth steering profile used for the
% replay observer simulation.
%
% Input:
%   time: [N x 1] simulation time vector
%   cfg: replay observer configuration struct
%
% Output:
%   steeringAngle: [N x 1] front steering angle in radians
    if isfield(cfg.replaySimulation, "steeringWaypointTime") && ...
            isfield(cfg.replaySimulation, "steeringWaypointDeg") && ...
            ~isempty(cfg.replaySimulation.steeringWaypointTime) && ...
            ~isempty(cfg.replaySimulation.steeringWaypointDeg)
        waypointTime = double(cfg.replaySimulation.steeringWaypointTime(:));
        waypointAngle = deg2rad(double(cfg.replaySimulation.steeringWaypointDeg(:)));
        assert(numel(waypointTime) == numel(waypointAngle) && numel(waypointTime) >= 2, ...
            "steeringWaypointTime and steeringWaypointDeg must have the same length of at least two.");
        assert(all(isfinite(waypointTime)) && all(isfinite(waypointAngle)) && all(diff(waypointTime) > 0.0), ...
            "steeringWaypointTime must be strictly increasing and steering waypoints must be finite.");
        steeringAngle = waypointAngle(1) .* ones(size(time));
        for segmentIdx = 1:(numel(waypointTime) - 1)
            segmentMask = time >= waypointTime(segmentIdx) & time <= waypointTime(segmentIdx + 1);
            segmentFraction = min(max((time(segmentMask) - waypointTime(segmentIdx)) ./ ...
                (waypointTime(segmentIdx + 1) - waypointTime(segmentIdx)), 0.0), 1.0);
            blend = 0.5 - 0.5 .* cos(pi .* segmentFraction);
            steeringAngle(segmentMask) = waypointAngle(segmentIdx) + ...
                blend .* (waypointAngle(segmentIdx + 1) - waypointAngle(segmentIdx));
        end
        steeringAngle(time >= waypointTime(end)) = waypointAngle(end);
    else
        finalAngle = deg2rad(cfg.replaySimulation.finalSteeringDeg);
        rampFraction = min(max(time ./ cfg.replaySimulation.rampDuration, 0.0), 1.0);
        blend = 0.5 - 0.5 .* cos(pi .* rampFraction);
        steeringAngle = finalAngle .* blend;
    end
end

function pose = integrateVehiclePose(time, speed, steeringAngle, cfg)
% integrateVehiclePose: Integrate the kinematic bicycle pose dynamics
% with fourth-order Runge-Kutta on the 100 Hz simulation grid.
%
% Input:
%   time: [N x 1] time vector
%   speed: [N x 1] speed profile
%   steeringAngle: [N x 1] steering profile
%   cfg: replay observer configuration struct
%
% Output:
%   pose: [N x 3] X, Y, and yaw trajectory
    pose = zeros(numel(time), 3);
    pose(1, 1:2) = cfg.replaySimulation.initialPosition(:).';
    pose(1, 3) = cfg.replaySimulation.initialYaw;
    for sampleIdx = 1:(numel(time) - 1)
        stepTime = time(sampleIdx + 1) - time(sampleIdx);
        currentPose = pose(sampleIdx, :).';
        currentInput = [speed(sampleIdx); steeringAngle(sampleIdx)];
        nextInput = [speed(sampleIdx + 1); steeringAngle(sampleIdx + 1)];
        midInput = 0.5 .* (currentInput + nextInput);
        k1 = vehicleDerivative(currentPose, currentInput, cfg);
        k2 = vehicleDerivative(currentPose + 0.5 .* stepTime .* k1, midInput, cfg);
        k3 = vehicleDerivative(currentPose + 0.5 .* stepTime .* k2, midInput, cfg);
        k4 = vehicleDerivative(currentPose + stepTime .* k3, nextInput, cfg);
        pose(sampleIdx + 1, :) = (currentPose + stepTime ./ 6.0 .* (k1 + 2.0 .* k2 + 2.0 .* k3 + k4)).';
    end
end

function derivative = vehicleDerivative(pose, input, cfg)
% vehicleDerivative: Evaluate the kinematic bicycle pose dynamics for
% one pose and one speed/steering input pair.
%
% Input:
%   pose: [3 x 1] X, Y, yaw state
%   input: [2 x 1] speed and steering angle
%   cfg: replay observer configuration struct
%
% Output:
%   derivative: [3 x 1] pose derivative
    speed = input(1);
    steeringAngle = input(2);
    beta = atan((cfg.vehicle.lr ./ cfg.vehicle.wheelbase) .* tan(steeringAngle));
    derivative = [speed .* cos(pose(3) + beta); ...
        speed .* sin(pose(3) + beta); ...
        speed ./ cfg.vehicle.wheelbase .* cos(beta) .* tan(steeringAngle)];
end

function initialState = initialObserverState(cfg, steeringAngle)
% initialObserverState: Convert the configured initial pose and speed
% bias into the transformed six-state coordinates used by the replay
% observer.
%
% Input:
%   cfg: replay observer configuration struct
%   steeringAngle: initial front steering angle in radians
%
% Output:
%   initialState: [6 x 1] transformed observer initial state
    initialPose = cfg.replaySimulation.initialEstimatePose(:);
    speedEstimate = cfg.replaySimulation.speed + cfg.replaySimulation.initialSpeedEstimateBias;
    beta = atan((cfg.vehicle.lr ./ cfg.vehicle.wheelbase) .* tan(steeringAngle));
    heading = initialPose(3) + beta;
    yawRate = speedEstimate ./ cfg.vehicle.wheelbase .* cos(beta) .* tan(steeringAngle);
    initialState = [initialPose(1); ...
        speedEstimate .* cos(heading); ...
        -speedEstimate .* sin(heading) .* yawRate; ...
        initialPose(2); ...
        speedEstimate .* sin(heading); ...
        speedEstimate .* cos(heading) .* yawRate];
end

function metrics = computeReplayMetrics(truth, estimate)
% computeReplayMetrics: Compute position, yaw, velocity, acceleration, and
% transformed-state error metrics for the replay observer simulation.
%
% Input:
%   truth: true simulated trajectory struct
%   estimate: replay observer estimate struct
%
% Output:
%   metrics: scalar error and certification metrics
    displayPosition = resolveDisplayPosition(estimate);
    displayAcceleration = resolveDisplayAcceleration(estimate);
    positionError = estimate.position - truth.position;
    displayPositionError = displayPosition - truth.position;
    yawError = wrapAngleToPi(estimate.yaw - truth.yaw);
    velocityError = estimate.velocity - truth.velocity;
    accelerationError = estimate.acceleration - truth.acceleration;
    displayAccelerationError = displayAcceleration - truth.acceleration;
    stateError = estimate.z - truth.z;

    metrics = struct();
    metrics.positionRmse = sqrt(mean(sum(positionError .* positionError, 2)));
    metrics.kinematicPositionRmse = sqrt(mean(sum(displayPositionError .* displayPositionError, 2)));
    metrics.yawRmse = sqrt(mean(yawError .* yawError));
    metrics.velocityRmse = sqrt(mean(sum(velocityError .* velocityError, 2)));
    metrics.accelerationRmse = sqrt(mean(sum(accelerationError .* accelerationError, 2)));
    metrics.filteredAccelerationRmse = sqrt(mean(sum(displayAccelerationError .* displayAccelerationError, 2)));
    metrics.stateRmse = sqrt(mean(sum(stateError .* stateError, 2)));
    metrics.finalPositionError = norm(positionError(end, :));
    metrics.finalYawError = yawError(end);
    metrics.maxInformationAge = max(estimate.diagnostics.informationAge, [], "omitnan");
    metrics.certifiedSampleFraction = mean(estimate.diagnostics.informationAgeCertified);
    metrics.numAcceptedPoseCorrections = numel(estimate.diagnostics.acceptedCorrections.timestamp);
    metrics.numReplayEvents = sum(estimate.diagnostics.replayCount);
end

function position = resolveDisplayPosition(estimate)
% resolveDisplayPosition: Select the continuous kinematic position output when
% available, falling back to the raw replay-observer position state for older
% estimate structs.
%
% Input:
%   estimate: replay-observer estimate struct
%
% Output:
%   position: [N x 2] position series used for vehicle trajectory plots
    if isfield(estimate, "displayPosition") && ~isempty(estimate.displayPosition)
        position = estimate.displayPosition;
    elseif isfield(estimate, "kinematicPosition") && ~isempty(estimate.kinematicPosition)
        position = estimate.kinematicPosition;
    else
        position = estimate.position;
    end
end

function acceleration = resolveDisplayAcceleration(estimate)
% resolveDisplayAcceleration: Select the filtered acceleration output when
% available, falling back to the raw replay-observer acceleration state for
% older estimate structs.
%
% Input:
%   estimate: replay-observer estimate struct
%
% Output:
%   acceleration: [N x 2] acceleration series used for vehicle state plots
    if isfield(estimate, "displayAcceleration") && ~isempty(estimate.displayAcceleration)
        acceleration = estimate.displayAcceleration;
    elseif isfield(estimate, "filteredAcceleration") && ~isempty(estimate.filteredAcceleration)
        acceleration = estimate.filteredAcceleration;
    else
        acceleration = estimate.acceleration;
    end
end

function value = readReplaySimulationScalar(cfg, fieldName, defaultValue)
% readReplaySimulationScalar: Read one scalar replay-simulation
% parameter with a fallback so older caller-provided configurations remain
% valid.
%
% Input:
%   cfg: replay observer configuration struct
%   fieldName: string scalar replaySimulation field name
%   defaultValue: scalar fallback value
%
% Output:
%   value: selected finite scalar value
    value = defaultValue;
    fieldName = char(string(fieldName));
    if isfield(cfg.replaySimulation, fieldName) && ~isempty(cfg.replaySimulation.(fieldName))
        candidate = double(cfg.replaySimulation.(fieldName));
        if isscalar(candidate) && isfinite(candidate)
            value = candidate;
        end
    end
end

function value = readReplaySimulationLogical(cfg, fieldName, defaultValue)
% readReplaySimulationLogical: Read one logical replay-simulation
% parameter with a fallback so older caller-provided configurations remain
% valid.
%
% Input:
%   cfg: replay observer configuration struct
%   fieldName: string scalar replaySimulation field name
%   defaultValue: logical fallback value
%
% Output:
%   value: selected logical scalar value
    value = logical(defaultValue);
    fieldName = char(string(fieldName));
    if isfield(cfg.replaySimulation, fieldName) && ~isempty(cfg.replaySimulation.(fieldName))
        value = logical(cfg.replaySimulation.(fieldName));
    end
end

function beta = sideSlipAngle(steeringAngle, cfg)
% sideSlipAngle: Compute the kinematic bicycle side-slip angle for a
% vector of steering angles.
%
% Input:
%   steeringAngle: numeric steering angle array in radians
%   cfg: replay observer configuration struct
%
% Output:
%   beta: numeric side-slip angle array in radians
    beta = atan((cfg.vehicle.lr ./ cfg.vehicle.wheelbase) .* tan(steeringAngle));
end

function wrappedAngle = wrapAngleToPi(angle)
% wrapAngleToPi: Wrap angle values to [-pi, pi] without requiring Mapping
% Toolbox.
%
% Input:
%   angle: numeric angle array in radians
%
% Output:
%   wrappedAngle: wrapped numeric angle array in radians
    wrappedAngle = mod(angle + pi, 2.0 .* pi) - pi;
end
