function result = simulateImprovedObserverScenario(observerDesign, lateralDesign, cfg)
% simulateImprovedObserverScenario Exercise the complete improved architecture.
% The deterministic scenario includes varying speed and steering, delayed and
% out-of-order GPS, fixed-delay LiDAR, a four-second GPS dropout, and a directionally weak, full-rank
% lidar information matrix while retaining a well-observed lidar heading.

    arguments
        observerDesign (1, 1) struct
        lateralDesign (1, 1) struct
        cfg (1, 1) struct = improvedObserverConfig()
    end

    truth = simulateTruth(lateralDesign, cfg);
    sensorData = buildSensorData(truth, cfg);
    runCfg = cfg;
    runCfg.observer.initialState = biasedInitialState(truth, cfg);
    runtimeLateralDesign = lateralDesign;
    runtimeLateralDesign.cfg.observer.initialState = [0.10; 0.02];
    estimate = runImprovedVehicleObserver(sensorData, runtimeLateralDesign, observerDesign, runCfg);
    metrics = computeMetrics(truth, estimate, cfg);

    result = struct();
    result.truth = truth;
    result.sensorData = sensorData;
    result.estimate = estimate;
    result.metrics = metrics;
    result.cfg = runCfg;
end

function truth = simulateTruth(lateralDesign, cfg)
% simulateTruth Generate a 2DOF body state and globally integrated trajectory.
    sampleTime = double(cfg.simulation.sampleTime);
    time = (0:sampleTime:double(cfg.simulation.finalTime)).';
    speedRate = (2.0 .* pi ./ double(cfg.simulation.longitudinalSpeedPeriod)) .* ...
        double(cfg.simulation.longitudinalSpeedAmplitude) .* ...
        cos(2.0 .* pi .* time ./ double(cfg.simulation.longitudinalSpeedPeriod));
    longitudinalSpeed = double(cfg.simulation.initialLongitudinalSpeed) + ...
        double(cfg.simulation.longitudinalSpeedAmplitude) .* ...
        sin(2.0 .* pi .* time ./ double(cfg.simulation.longitudinalSpeedPeriod));
    steeringAngle = double(cfg.simulation.steeringAmplitude) .* ...
        sin(2.0 .* pi .* time ./ double(cfg.simulation.steeringPeriod));

    lateralState = zeros(numel(time), 2);
    for sampleIdx = 1:(numel(time) - 1)
        step = time(sampleIdx + 1) - time(sampleIdx);
        k1 = lateralPlantDerivative(lateralState(sampleIdx, :).', sampleIdx, 0.0, ...
            longitudinalSpeed, steeringAngle, lateralDesign.model);
        k2 = lateralPlantDerivative(lateralState(sampleIdx, :).' + 0.5 .* step .* k1, ...
            sampleIdx, 0.5, longitudinalSpeed, steeringAngle, lateralDesign.model);
        k3 = lateralPlantDerivative(lateralState(sampleIdx, :).' + 0.5 .* step .* k2, ...
            sampleIdx, 0.5, longitudinalSpeed, steeringAngle, lateralDesign.model);
        k4 = lateralPlantDerivative(lateralState(sampleIdx, :).' + step .* k3, ...
            sampleIdx, 1.0, longitudinalSpeed, steeringAngle, lateralDesign.model);
        lateralState(sampleIdx + 1, :) = (lateralState(sampleIdx, :).' + ...
            (step ./ 6.0) .* (k1 + 2.0 .* k2 + 2.0 .* k3 + k4)).';
    end

    lateralVelocityRate = zeros(numel(time), 1);
    lateralAcceleration = zeros(numel(time), 1);
    for sampleIdx = 1:numel(time)
        speed = longitudinalSpeed(sampleIdx);
        [A, C] = evaluateLateralModel(lateralDesign.model, [speed; 1.0 ./ speed]);
        derivative = A * lateralState(sampleIdx, :).' + lateralDesign.model.B .* steeringAngle(sampleIdx);
        lateralVelocityRate(sampleIdx) = derivative(1);
        lateralAcceleration(sampleIdx) = C(1, :) * lateralState(sampleIdx, :).' + ...
            lateralDesign.model.D(1) .* steeringAngle(sampleIdx);
    end
    yawRate = lateralState(:, 2);
    lateralVelocity = lateralState(:, 1);
    longitudinalSpecificForce = speedRate - lateralVelocity .* yawRate;
    sideSlipAngle = atan2(lateralVelocity, longitudinalSpeed);
    sideSlipAngleRate = (lateralVelocityRate .* longitudinalSpeed - ...
        lateralVelocity .* speedRate) ./ (longitudinalSpeed.^2 + lateralVelocity.^2);
    heading = wrapAngleToPi(double(cfg.simulation.initialHeading) + cumtrapz(time, yawRate));

    globalVelocity = rotateRows([longitudinalSpeed, lateralVelocity], heading);
    globalAcceleration = rotateRows([longitudinalSpecificForce, lateralAcceleration], heading);
    position = double(cfg.simulation.initialPosition(:)).' + cumtrapz(time, globalVelocity);
    z = [position(:, 1), globalVelocity(:, 1), globalAcceleration(:, 1), ...
        position(:, 2), globalVelocity(:, 2), globalAcceleration(:, 2), heading];

    truth = struct();
    truth.time = time;
    truth.longitudinalSpeed = longitudinalSpeed;
    truth.longitudinalSpeedRate = speedRate;
    truth.steeringAngle = steeringAngle;
    truth.lateralState = lateralState;
    truth.lateralVelocity = lateralVelocity;
    truth.lateralVelocityRate = lateralVelocityRate;
    truth.yawRate = yawRate;
    truth.longitudinalAcceleration = longitudinalSpecificForce;
    truth.lateralAcceleration = lateralAcceleration;
    truth.sideSlipAngle = sideSlipAngle;
    truth.sideSlipAngleRate = sideSlipAngleRate;
    truth.trackAngleRate = yawRate + sideSlipAngleRate;
    truth.heading = heading;
    truth.position = position;
    truth.velocity = globalVelocity;
    truth.acceleration = globalAcceleration;
    truth.z = z;
end

function derivative = lateralPlantDerivative(state, sampleIdx, alpha, speed, steering, model)
% lateralPlantDerivative Evaluate the varying-speed linear bicycle plant.
    interpolatedSpeed = interpolateSeries(speed, sampleIdx, alpha);
    interpolatedSteering = interpolateSeries(steering, sampleIdx, alpha);
    [A, ~] = evaluateLateralModel(model, [interpolatedSpeed; 1.0 ./ interpolatedSpeed]);
    derivative = A * state + model.B .* interpolatedSteering;
end

function sensorData = buildSensorData(truth, cfg)
% buildSensorData Add deterministic noise and asynchronous pose metadata.
    previousRng = rng;
    cleanup = onCleanup(@() rng(previousRng));
    rng(double(cfg.simulation.randomSeed), "twister");
    sampleCount = numel(truth.time);

    highRate = struct();
    highRate.time = truth.time;
    highRate.steeringAngle = truth.steeringAngle + ...
        double(cfg.simulation.steeringNoiseStandardDeviation) .* randn(sampleCount, 1);
    highRate.longitudinalSpeed = max(0.0, truth.longitudinalSpeed + ...
        double(cfg.simulation.wheelSpeedNoiseStandardDeviation) .* randn(sampleCount, 1));
    highRate.longitudinalAcceleration = truth.longitudinalAcceleration + ...
        double(cfg.simulation.accelerationNoiseStandardDeviation) .* randn(sampleCount, 1);
    highRate.lateralAcceleration = truth.lateralAcceleration + ...
        double(cfg.simulation.accelerationNoiseStandardDeviation) .* randn(sampleCount, 1);
    highRate.yawRate = truth.yawRate + ...
        double(cfg.simulation.gyroNoiseStandardDeviation) .* randn(sampleCount, 1);

    gpsStep = max(1, round(1.0 ./ (double(cfg.simulation.gpsRateHz) .* cfg.simulation.sampleTime)));
    gpsIdx = (1:gpsStep:sampleCount).';
    dropout = truth.time(gpsIdx) >= cfg.simulation.gpsDropoutInterval(1) & ...
        truth.time(gpsIdx) <= cfg.simulation.gpsDropoutInterval(2);
    gpsIdx(dropout) = [];
    gps = struct();
    gps.timestamp = truth.time(gpsIdx);
    gps.arrivalTime = gps.timestamp + double(cfg.simulation.gpsDelay);
    gps.pose = truth.position(gpsIdx, :) + ...
        double(cfg.simulation.gpsPositionNoiseStandardDeviation) .* randn(numel(gpsIdx), 2);
    gps.arrivalTime = addOutOfOrderDelay(gps.arrivalTime, cfg);

    lidarStep = max(1, round(1.0 ./ (double(cfg.simulation.lidarRateHz) .* cfg.simulation.sampleTime)));
    lidarIdx = (1:lidarStep:sampleCount).';
    lidar = struct();
    lidar.timestamp = truth.time(lidarIdx);
    lidar.arrivalTime = lidar.timestamp + double(cfg.measurement.fixedLidarDelay);
    lidar.pose = [truth.position(lidarIdx, :) + ...
        double(cfg.simulation.lidarPositionNoiseStandardDeviation) .* randn(numel(lidarIdx), 2), ...
        wrapAngleToPi(truth.heading(lidarIdx) + ...
        double(cfg.simulation.lidarHeadingNoiseStandardDeviation) .* randn(numel(lidarIdx), 1))];
    lidar.information = lidarInformationSeries(truth.time(lidarIdx), truth.heading(lidarIdx), cfg);

    sensorData = struct("highRate", highRate, "gps", gps, "lidar", lidar);
    clear cleanup;
end

function arrivalTime = addOutOfOrderDelay(arrivalTime, cfg)
% addOutOfOrderDelay Delay one event beyond its successor when possible.
    if numel(arrivalTime) >= 8
        arrivalTime(7) = arrivalTime(7) + double(cfg.simulation.outOfOrderExtraDelay);
    end
end

function information = lidarInformationSeries(timestamp, heading, cfg)
% lidarInformationSeries Build full-rank nominal and weak-direction Hessians.
    information = zeros(3, 3, numel(timestamp));
    interval = double(cfg.simulation.lidarDegeneracyInterval(:));
    for eventIdx = 1:numel(timestamp)
        angle = 0.5 .* heading(eventIdx);
        direction = [cos(angle), -sin(angle); sin(angle), cos(angle)];
        if timestamp(eventIdx) >= interval(1) && timestamp(eventIdx) <= interval(2)
            translationEigenvalues = [450.0; 0.20];
        else
            translationEigenvalues = [450.0; 180.0];
        end
        information(1:2, 1:2, eventIdx) = direction * diag(translationEigenvalues) * direction.';
        information(3, 3, eventIdx) = 600.0;
    end
end

function initialState = biasedInitialState(truth, cfg)
% biasedInitialState Apply pose error while keeping body vectors consistent.
    errorVector = double(cfg.simulation.initialEstimateError(:));
    assert(numel(errorVector) == 3, "cfg.simulation.initialEstimateError must be [dx;dy;dheading].");
    heading = wrapAngleToPi(truth.heading(1) + errorVector(3));
    rotation = [cos(heading), -sin(heading); sin(heading), cos(heading)];
    velocity = rotation * [truth.longitudinalSpeed(1); truth.lateralVelocity(1)];
    acceleration = rotation * [truth.longitudinalAcceleration(1); truth.lateralAcceleration(1)];
    initialState = [truth.position(1, 1) + errorVector(1); velocity(1); acceleration(1); ...
        truth.position(1, 2) + errorVector(2); velocity(2); acceleration(2); heading];
end

function metrics = computeMetrics(truth, estimate, cfg)
% computeMetrics Quantify settled pose, state, and cascade-interface errors.
    settled = truth.time >= 0.25 .* double(cfg.simulation.finalTime);
    positionError = estimate.position - truth.position;
    headingError = wrapAngleToPi(estimate.heading - truth.heading);
    velocityError = estimate.velocity - truth.velocity;
    accelerationError = estimate.acceleration - truth.acceleration;
    trackRateError = estimate.trackAngleRate - truth.trackAngleRate;
    metrics = struct();
    metrics.positionRmse = sqrt(mean(sum(positionError(settled, :).^2, 2)));
    metrics.headingRmse = sqrt(mean(headingError(settled).^2));
    metrics.velocityRmse = sqrt(mean(sum(velocityError(settled, :).^2, 2)));
    metrics.accelerationRmse = sqrt(mean(sum(accelerationError(settled, :).^2, 2)));
    metrics.trackAngleRateRmse = sqrt(mean(trackRateError(settled).^2));
    metrics.finalPositionError = norm(positionError(end, :));
    metrics.finalHeadingError = abs(headingError(end));
    metrics.integrationStepCount = estimate.diagnostics.integrationStepCount;
    metrics.stateHistoryRecomputed = estimate.diagnostics.stateHistoryRecomputed;
    metrics.acceptedEventCount = estimate.diagnostics.acceptedEventCount(end);
    metrics.rejectedEventCount = estimate.diagnostics.rejectedEventCount(end);
    metrics.maximumStateMagnitude = max(abs(estimate.z), [], "all");
    metrics.degenerateLidarMinimumWeight = minimumTranslationWeight( ...
        estimate.diagnostics.translationWeight, truth.time, cfg.simulation.lidarDegeneracyInterval);
    metrics.nominalLidarMinimumWeight = minimumTranslationWeight( ...
        estimate.diagnostics.translationWeight, truth.time, [2.0, 6.0]);
end

function minimumWeight = minimumTranslationWeight(weights, time, interval)
% minimumTranslationWeight Return the smallest active eigenvalue in an interval.
    traceWeight = squeeze(weights(1, 1, :) + weights(2, 2, :));
    mask = time >= interval(1) & time <= interval(2) & traceWeight > 1.0e-12;
    values = NaN(nnz(mask), 1);
    sampleIndices = find(mask);
    for valueIdx = 1:numel(sampleIndices)
        eigenvalues = eig(0.5 .* (weights(:, :, sampleIndices(valueIdx)) + ...
            weights(:, :, sampleIndices(valueIdx)).'));
        values(valueIdx) = min(real(eigenvalues));
    end
    minimumWeight = min(values, [], "omitnan");
end

function rotated = rotateRows(bodyVectors, heading)
% rotateRows Rotate two-column body vectors into the global frame.
    rotated = [cos(heading) .* bodyVectors(:, 1) - sin(heading) .* bodyVectors(:, 2), ...
        sin(heading) .* bodyVectors(:, 1) + cos(heading) .* bodyVectors(:, 2)];
end

function value = interpolateSeries(series, sampleIdx, alpha)
% interpolateSeries Linearly interpolate one simulation input interval.
    nextIdx = min(sampleIdx + 1, numel(series));
    value = (1.0 - alpha) .* series(sampleIdx) + alpha .* series(nextIdx);
end

function wrappedAngle = wrapAngleToPi(angle)
% wrapAngleToPi Wrap radians to [-pi, pi).
    wrappedAngle = mod(angle + pi, 2.0 .* pi) - pi;
end
