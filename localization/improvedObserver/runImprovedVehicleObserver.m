function estimate = runImprovedVehicleObserver(sensorData, lateralDesign, observerDesign, cfg)
% runImprovedVehicleObserver Run the complete cascaded seven-state observer.
% The independent 2DOF observer first estimates lateral velocity, side slip,
% and side-slip rate. Its outputs then drive the nonsingular high-gain model.
% Delayed GPS and lidar events are inserted at their physical timestamps and
% the affected continuous-time state history is replayed to the present.

    arguments
        sensorData (1, 1) struct
        lateralDesign (1, 1) struct
        observerDesign (1, 1) struct
        cfg (1, 1) struct = improvedObserverConfig()
    end

    validateObserverDesign(observerDesign, cfg);
    [highRate, gps, lidar] = normalizeSensorData(sensorData, cfg);
    lateralCfg = lateralDesign.cfg;
    lateralCfg.observer.initialState = double(lateralCfg.observer.initialState(:));
    lateralEstimate = runLateralVelocityObserver(highRate, lateralDesign, lateralCfg);

    stateCount = 7;
    sampleCount = numel(highRate.time);
    stateHistory = NaN(sampleCount, stateCount);
    onlineState = NaN(sampleCount, stateCount);
    replayCount = zeros(sampleCount, 1);
    acceptedEventCount = zeros(sampleCount, 1);
    rejectedEventCount = zeros(sampleCount, 1);
    acceptedGps = false(numel(gps.timestamp), 1);
    rejectedGps = false(numel(gps.timestamp), 1);
    acceptedLidar = false(numel(lidar.timestamp), 1);
    rejectedLidar = false(numel(lidar.timestamp), 1);
    stateHistory(1, :) = buildInitialState(highRate, lateralEstimate, gps, lidar, cfg).';

    for sampleIdx = 1:sampleCount
        if sampleIdx > 1
            stateHistory(sampleIdx, :) = propagateState(stateHistory(sampleIdx - 1, :).', ...
                sampleIdx - 1, highRate, lateralEstimate, gps, lidar, acceptedGps, ...
                acceptedLidar, observerDesign, cfg).';
        end

        currentTime = highRate.time(sampleIdx);
        [acceptedGps, rejectedGps, newGps] = acceptArrivedEvents( ...
            gps, acceptedGps, rejectedGps, currentTime, cfg);
        [acceptedLidar, rejectedLidar, newLidar] = acceptArrivedEvents( ...
            lidar, acceptedLidar, rejectedLidar, currentTime, cfg);
        newTimestamps = [gps.timestamp(newGps); lidar.timestamp(newLidar)];
        if ~isempty(newTimestamps)
            replayAnchor = replayAnchorIndex(highRate.time, min(newTimestamps), cfg);
            for replayIdx = (replayAnchor + 1):sampleIdx
                stateHistory(replayIdx, :) = propagateState(stateHistory(replayIdx - 1, :).', ...
                    replayIdx - 1, highRate, lateralEstimate, gps, lidar, acceptedGps, ...
                    acceptedLidar, observerDesign, cfg).';
            end
            replayCount(sampleIdx) = 1;
        end

        onlineState(sampleIdx, :) = stateHistory(sampleIdx, :);
        acceptedEventCount(sampleIdx) = nnz(acceptedGps) + nnz(acceptedLidar);
        rejectedEventCount(sampleIdx) = nnz(rejectedGps) + nnz(rejectedLidar);
    end

    estimate = buildEstimate(stateHistory, onlineState, highRate, gps, lidar, ...
        acceptedGps, acceptedLidar, lateralEstimate, replayCount, acceptedEventCount, ...
        rejectedEventCount, observerDesign, cfg);
end

function stateNext = propagateState(state, intervalIdx, highRate, lateralEstimate, gps, lidar, ...
        acceptedGps, acceptedLidar, design, cfg)
% propagateState Integrate one high-rate interval with event-aware RK4.
    stepTime = highRate.time(intervalIdx + 1) - highRate.time(intervalIdx);
    method = lower(strtrim(string(cfg.observer.integrationMethod)));
    if method == "euler"
        derivative = observerDerivative(state, intervalIdx, 0.0, highRate, lateralEstimate, ...
            gps, lidar, acceptedGps, acceptedLidar, design, cfg);
        stateNext = state + stepTime .* derivative;
    elseif method == "rk4"
        k1 = observerDerivative(state, intervalIdx, 0.0, highRate, lateralEstimate, ...
            gps, lidar, acceptedGps, acceptedLidar, design, cfg);
        k2 = observerDerivative(state + 0.5 .* stepTime .* k1, intervalIdx, 0.5, ...
            highRate, lateralEstimate, gps, lidar, acceptedGps, acceptedLidar, design, cfg);
        k3 = observerDerivative(state + 0.5 .* stepTime .* k2, intervalIdx, 0.5, ...
            highRate, lateralEstimate, gps, lidar, acceptedGps, acceptedLidar, design, cfg);
        k4 = observerDerivative(state + stepTime .* k3, intervalIdx, 1.0, ...
            highRate, lateralEstimate, gps, lidar, acceptedGps, acceptedLidar, design, cfg);
        stateNext = state + (stepTime ./ 6.0) .* (k1 + 2.0 .* k2 + 2.0 .* k3 + k4);
    else
        error("Unsupported improved-observer integration method: %s.", method);
    end
    stateNext(7) = wrapAngleToPi(stateNext(7));
    assert(all(isfinite(stateNext)), ...
        "The improved observer produced a nonfinite state in interval %d.", intervalIdx);
end

function derivative = observerDerivative(state, intervalIdx, alpha, highRate, lateralEstimate, ...
        gps, lidar, acceptedGps, acceptedLidar, design, cfg)
% observerDerivative Evaluate prediction and all three innovation channels.
    queryTime = interpolateValue(highRate.time, intervalIdx, alpha, "linear");
    sample = measurementSample(highRate, lateralEstimate, intervalIdx, alpha, cfg);
    channels = evaluateImprovedObserverChannels(state, sample);
    fusion = fusionMeasurementAt(queryTime, gps, lidar, acceptedGps, acceptedLidar, cfg);
    [baseInnovation, lidarInnovation] = poseInnovations(state, fusion);

    theta = double(cfg.observer.theta);
    scaling = diag(theta.^double(cfg.observer.scalingExponents(:)));
    baseCorrection = scaling * design.K * (fusion.omega * baseInnovation);
    lidarGain = scaling * (design.P \ fusion.Cl.');
    lidarCorrection = lidarGain * fusion.translationWeight * lidarInnovation;
    invariantCorrection = (scaling * design.N ./ theta.^3.0) * channels.invariantInnovation;
    derivative = channels.modelDerivative + baseCorrection + lidarCorrection + invariantCorrection;
end

function sample = measurementSample(highRate, lateralEstimate, intervalIdx, alpha, cfg)
% measurementSample Interpolate the cascade inputs inside one HGO step.
    interpolation = string(cfg.measurement.inputInterpolation);
    sample = struct();
    sample.longitudinalSpeed = interpolateValue(highRate.longitudinalSpeed, intervalIdx, alpha, interpolation);
    sample.longitudinalAcceleration = interpolateValue( ...
        highRate.longitudinalAcceleration, intervalIdx, alpha, interpolation);
    sample.lateralAcceleration = interpolateValue(highRate.lateralAcceleration, intervalIdx, alpha, interpolation);
    sample.yawRate = interpolateValue(highRate.yawRate, intervalIdx, alpha, interpolation);
    sample.lateralVelocity = interpolateValue(lateralEstimate.lateralVelocity, intervalIdx, alpha, interpolation);
    sample.sideSlipAngle = interpolateAngle(lateralEstimate.sideSlipAngle, intervalIdx, alpha, interpolation);
    rawSideSlipRate = interpolateValue(lateralEstimate.sideSlipAngleRate, intervalIdx, alpha, interpolation);
    rawTrackRate = sample.yawRate + rawSideSlipRate;
    if logical(cfg.observer.clampTrackAngleRateToDesignEnvelope)
        maximumRate = double(cfg.operating.maximumTrackAngleRate);
        usedTrackRate = min(max(rawTrackRate, -maximumRate), maximumRate);
        sample.sideSlipAngleRate = usedTrackRate - sample.yawRate;
    else
        sample.sideSlipAngleRate = rawSideSlipRate;
    end
end

function [baseInnovation, lidarInnovation] = poseInnovations(state, fusion)
% poseInnovations Form linear position residuals and a wrapped yaw residual.
    baseInnovation = [fusion.gpsPosition(1) - state(1); ...
        fusion.gpsPosition(2) - state(4); ...
        wrapAngleToPi(fusion.lidarHeading - state(7))];
    lidarInnovation = fusion.lidarPosition - state([1, 4]);
end

function fusion = fusionMeasurementAt(queryTime, gps, lidar, acceptedGps, acceptedLidar, cfg)
% fusionMeasurementAt Return the latest physically available held samples.
    fusion = emptyFusionMeasurement();
    fusion.Cl = zeros(2, 7);
    fusion.Cl(1, 1) = 1.0;
    fusion.Cl(2, 4) = 1.0;

    gpsIdx = latestEligibleEvent(gps, acceptedGps, queryTime, cfg);
    if ~isempty(gpsIdx)
        gpsAge = max(0.0, queryTime - gps.timestamp(gpsIdx));
        if gpsAge <= double(cfg.measurement.gpsMaximumAge) && all(isfinite(gps.pose(gpsIdx, 1:2)))
            fusion.gpsPosition = gps.pose(gpsIdx, 1:2).';
            fusion.gpsValid = true;
            fusion.gpsAge = gpsAge;
        end
    end

    lidarIdx = latestEligibleEvent(lidar, acceptedLidar, queryTime, cfg);
    if ~isempty(lidarIdx)
        lidarAge = max(0.0, queryTime - lidar.timestamp(lidarIdx));
        if lidarAge <= double(cfg.measurement.lidarMaximumAge)
            information = lidar.information(:, :, lidarIdx);
            if any(~isfinite(information(:)))
                information = [];
            end
            [translationWeight, headingWeight] = computeLidarInformationWeights(information, cfg);
            if all(isfinite(lidar.pose(lidarIdx, 1:2)))
                fusion.lidarPosition = lidar.pose(lidarIdx, 1:2).';
                fusion.translationWeight = translationWeight;
                fusion.lidarPositionValid = true;
            end
            if isfinite(lidar.pose(lidarIdx, 3))
                fusion.lidarHeading = lidar.pose(lidarIdx, 3);
                fusion.headingWeight = headingWeight;
                fusion.lidarHeadingValid = true;
            end
            fusion.lidarAge = lidarAge;
        end
    end

    gpsWeight = double(fusion.gpsValid);
    fusion.omega = diag([gpsWeight, gpsWeight, fusion.headingWeight]);
end

function eventIdx = latestEligibleEvent(events, accepted, queryTime, cfg)
% latestEligibleEvent Select the most recent accepted physical timestamp.
    tolerance = double(cfg.measurement.timestampTolerance);
    eligible = accepted & events.timestamp <= queryTime + tolerance;
    if ~any(eligible)
        eventIdx = [];
        return;
    end
    latestTimestamp = max(events.timestamp(eligible));
    eventIdx = find(eligible & events.timestamp == latestTimestamp, 1, "last");
end

function fusion = emptyFusionMeasurement()
% emptyFusionMeasurement Return zero-weight residual placeholders.
    fusion = struct();
    fusion.gpsPosition = zeros(2, 1);
    fusion.lidarPosition = zeros(2, 1);
    fusion.lidarHeading = 0.0;
    fusion.translationWeight = zeros(2, 2);
    fusion.headingWeight = 0.0;
    fusion.omega = zeros(3, 3);
    fusion.Cl = zeros(2, 7);
    fusion.gpsValid = false;
    fusion.lidarPositionValid = false;
    fusion.lidarHeadingValid = false;
    fusion.gpsAge = NaN;
    fusion.lidarAge = NaN;
end

function [accepted, rejected, newlyAccepted] = acceptArrivedEvents(events, accepted, rejected, currentTime, cfg)
% acceptArrivedEvents Gate new arrivals by the bounded replay horizon.
    tolerance = double(cfg.measurement.timestampTolerance);
    arrived = ~accepted & ~rejected & events.arrivalTime <= currentTime + tolerance;
    candidateIdx = find(arrived);
    newlyAccepted = zeros(0, 1);
    for candidateOffset = 1:numel(candidateIdx)
        eventIdx = candidateIdx(candidateOffset);
        age = currentTime - events.timestamp(eventIdx);
        if age <= double(cfg.measurement.replayBufferDuration) + tolerance
            accepted(eventIdx) = true;
            newlyAccepted(end + 1, 1) = eventIdx; %#ok<AGROW>
        else
            rejected(eventIdx) = true;
        end
    end
end

function anchorIdx = replayAnchorIndex(time, timestamp, cfg)
% replayAnchorIndex Find the state just before a held event begins to act.
    tolerance = double(cfg.measurement.timestampTolerance);
    anchorIdx = find(time <= timestamp + tolerance, 1, "last");
    if isempty(anchorIdx)
        anchorIdx = 1;
    end
    anchorIdx = min(max(anchorIdx, 1), numel(time));
end

function initialState = buildInitialState(highRate, lateralEstimate, gps, lidar, cfg)
% buildInitialState Build [X,Vx,Ax,Y,Vy,Ay,heading] without zero-speed division.
    if ~isempty(cfg.observer.initialState)
        initialState = double(cfg.observer.initialState(:));
        assert(numel(initialState) == 7 && all(isfinite(initialState)), ...
            "cfg.observer.initialState must be empty or a finite [7 x 1] vector.");
        initialState(7) = wrapAngleToPi(initialState(7));
        return;
    end

    initialTime = highRate.time(1);
    initialGps = gps.arrivalTime <= initialTime & gps.timestamp <= initialTime;
    initialLidar = lidar.arrivalTime <= initialTime & lidar.timestamp <= initialTime;
    position = double(cfg.observer.fallbackPosition(:));
    heading = double(cfg.observer.fallbackHeading);
    if any(initialGps)
        eventIdx = find(initialGps, 1, "last");
        position = gps.pose(eventIdx, 1:2).';
    elseif any(initialLidar)
        eventIdx = find(initialLidar, 1, "last");
        position = lidar.pose(eventIdx, 1:2).';
    end
    if any(initialLidar)
        eventIdx = find(initialLidar & isfinite(lidar.pose(:, 3)), 1, "last");
        if ~isempty(eventIdx)
            heading = lidar.pose(eventIdx, 3);
        end
    end

    bodyVelocity = [highRate.longitudinalSpeed(1); lateralEstimate.lateralVelocity(1)];
    bodyAcceleration = [highRate.longitudinalAcceleration(1); highRate.lateralAcceleration(1)];
    rotation = [cos(heading), -sin(heading); sin(heading), cos(heading)];
    globalVelocity = rotation * bodyVelocity;
    globalAcceleration = rotation * bodyAcceleration;
    initialState = [position(1); globalVelocity(1); globalAcceleration(1); ...
        position(2); globalVelocity(2); globalAcceleration(2); wrapAngleToPi(heading)];
end

function estimate = buildEstimate(stateHistory, onlineState, highRate, gps, lidar, ...
        acceptedGps, acceptedLidar, lateralEstimate, replayCount, acceptedEventCount, ...
        rejectedEventCount, design, cfg)
% buildEstimate Assemble states, innovations, weights, and replay diagnostics.
    sampleCount = numel(highRate.time);
    baseInnovation = zeros(sampleCount, 3);
    lidarInnovation = zeros(sampleCount, 2);
    invariantInnovation = zeros(sampleCount, 4);
    translationWeight = zeros(2, 2, sampleCount);
    headingWeight = zeros(sampleCount, 1);
    gpsAge = NaN(sampleCount, 1);
    lidarAge = NaN(sampleCount, 1);
    trackAngleRate = zeros(sampleCount, 1);
    rawTrackAngleRate = highRate.yawRate + lateralEstimate.sideSlipAngleRate;
    outsideTrackRateEnvelope = abs(rawTrackAngleRate) > double(cfg.operating.maximumTrackAngleRate);
    estimatedVelocityOutsideEnvelope = any(abs(stateHistory(:, [2, 5])) > ...
        double(cfg.operating.maximumSpeed), 2);
    estimatedAccelerationOutsideEnvelope = any(abs(stateHistory(:, [3, 6])) > ...
        double(cfg.operating.maximumAcceleration), 2);

    for sampleIdx = 1:sampleCount
        sample = measurementSample(highRate, lateralEstimate, min(sampleIdx, sampleCount - 1), ...
            double(sampleIdx == sampleCount), cfg);
        channels = evaluateImprovedObserverChannels(stateHistory(sampleIdx, :).', sample);
        fusion = fusionMeasurementAt(highRate.time(sampleIdx), gps, lidar, acceptedGps, acceptedLidar, cfg);
        [baseInnovation(sampleIdx, :), lidarInnovation(sampleIdx, :)] = ...
            rowPoseInnovations(stateHistory(sampleIdx, :).', fusion);
        invariantInnovation(sampleIdx, :) = channels.invariantInnovation.';
        translationWeight(:, :, sampleIdx) = fusion.translationWeight;
        headingWeight(sampleIdx) = fusion.headingWeight;
        gpsAge(sampleIdx) = fusion.gpsAge;
        lidarAge(sampleIdx) = fusion.lidarAge;
        trackAngleRate(sampleIdx) = channels.trackAngleRate;
    end

    estimate = struct();
    estimate.time = highRate.time;
    estimate.z = stateHistory;
    estimate.onlineZ = onlineState;
    estimate.position = stateHistory(:, [1, 4]);
    estimate.velocity = stateHistory(:, [2, 5]);
    estimate.acceleration = stateHistory(:, [3, 6]);
    estimate.heading = stateHistory(:, 7);
    estimate.speed = hypot(stateHistory(:, 2), stateHistory(:, 5));
    estimate.sideSlipAngle = lateralEstimate.sideSlipAngle;
    estimate.sideSlipAngleRate = lateralEstimate.sideSlipAngleRate;
    estimate.trackAngleRate = trackAngleRate;
    estimate.lateral = lateralEstimate;
    estimate.measurements = struct("highRate", highRate, "gps", gps, "lidar", lidar);
    estimate.innovations = struct("base", baseInnovation, "lidarPosition", lidarInnovation, ...
        "invariant", invariantInnovation);
    estimate.diagnostics = struct();
    estimate.diagnostics.translationWeight = translationWeight;
    estimate.diagnostics.headingWeight = headingWeight;
    estimate.diagnostics.gpsAge = gpsAge;
    estimate.diagnostics.lidarAge = lidarAge;
    estimate.diagnostics.rawTrackAngleRate = rawTrackAngleRate;
    estimate.diagnostics.outsideTrackRateEnvelope = outsideTrackRateEnvelope;
    estimate.diagnostics.estimatedVelocityOutsideEnvelope = estimatedVelocityOutsideEnvelope;
    estimate.diagnostics.estimatedAccelerationOutsideEnvelope = estimatedAccelerationOutsideEnvelope;
    estimate.diagnostics.replayCount = replayCount;
    estimate.diagnostics.acceptedEventCount = acceptedEventCount;
    estimate.diagnostics.rejectedEventCount = rejectedEventCount;
    estimate.diagnostics.acceptedGps = acceptedGps;
    estimate.diagnostics.acceptedLidar = acceptedLidar;
    estimate.observer = struct("P", design.P, "K", design.K, "N", design.N, ...
        "theta", cfg.observer.theta, "sigma", cfg.observer.sigma, ...
        "certified", design.certified, "verification", design.verification);
end

function [baseRow, lidarRow] = rowPoseInnovations(state, fusion)
% rowPoseInnovations Return pose innovations as row vectors for storage.
    [base, lidarPosition] = poseInnovations(state, fusion);
    baseRow = base.';
    lidarRow = lidarPosition.';
end

function [highRate, gps, lidar] = normalizeSensorData(sensorData, cfg)
% normalizeSensorData Validate the common-rate stream and asynchronous events.
    assert(isfield(sensorData, "highRate") && isstruct(sensorData.highRate), ...
        "sensorData.highRate is required.");
    highRate = normalizeHighRate(sensorData.highRate);
    if isfield(sensorData, "gps")
        gps = normalizePoseEvents(sensorData.gps, false);
    else
        gps = emptyEvents(false);
    end
    if isfield(sensorData, "lidar")
        lidar = normalizePoseEvents(sensorData.lidar, true);
    else
        lidar = emptyEvents(true);
    end
    tolerance = double(cfg.measurement.timestampTolerance);
    assert(all(gps.arrivalTime + tolerance >= gps.timestamp), ...
        "GPS arrival times must not precede their physical timestamps.");
    assert(all(lidar.arrivalTime + tolerance >= lidar.timestamp), ...
        "Lidar arrival times must not precede their physical timestamps.");
end

function highRate = normalizeHighRate(highRate)
% normalizeHighRate Return finite aligned column vectors with nonnegative speed.
    requiredFields = ["time", "steeringAngle", "longitudinalSpeed", ...
        "longitudinalAcceleration", "lateralAcceleration", "yawRate"];
    sampleCount = numel(highRate.time);
    assert(sampleCount >= 2, "The high-rate stream must contain at least two samples.");
    for fieldName = requiredFields
        assert(isfield(highRate, fieldName), "sensorData.highRate.%s is required.", fieldName);
        values = double(highRate.(fieldName)(:));
        assert(numel(values) == sampleCount && all(isfinite(values)), ...
            "sensorData.highRate.%s must be a finite aligned series.", fieldName);
        highRate.(fieldName) = values;
    end
    if isfield(highRate, "dynamicValid")
        dynamicValid = highRate.dynamicValid(:);
        assert((islogical(dynamicValid) || isnumeric(dynamicValid)) && ...
            numel(dynamicValid) == sampleCount && all(isfinite(double(dynamicValid))), ...
            "sensorData.highRate.dynamicValid must be a finite aligned logical series.");
        highRate.dynamicValid = logical(dynamicValid);
    end
    assert(all(diff(highRate.time) > 0.0), "sensorData.highRate.time must be strictly increasing.");
    assert(all(highRate.longitudinalSpeed >= 0.0), ...
        "sensorData.highRate.longitudinalSpeed must be nonnegative.");
end

function events = normalizePoseEvents(rawEvents, includeInformation)
% normalizePoseEvents Normalize timestamped GPS or lidar event structures.
    if isempty(rawEvents) || (isstruct(rawEvents) && isempty(fieldnames(rawEvents)))
        events = emptyEvents(includeInformation);
        return;
    end
    assert(isstruct(rawEvents), "GPS and lidar event streams must be structs.");
    requiredFields = ["timestamp", "pose"];
    for fieldName = requiredFields
        assert(isfield(rawEvents, fieldName), "Event stream is missing %s.", fieldName);
    end
    timestamp = double(rawEvents.timestamp(:));
    eventCount = numel(timestamp);
    pose = double(rawEvents.pose);
    if size(pose, 1) ~= eventCount && size(pose, 2) == eventCount
        pose = pose.';
    end
    minimumWidth = 2 + double(includeInformation);
    assert(size(pose, 1) == eventCount && size(pose, 2) >= minimumWidth, ...
        "Event pose must have one row per timestamp and at least %d columns.", minimumWidth);
    pose = pose(:, 1:minimumWidth);
    if ~includeInformation
        pose(:, 3) = NaN;
    end
    if isfield(rawEvents, "arrivalTime")
        arrivalTime = double(rawEvents.arrivalTime(:));
    else
        arrivalTime = timestamp;
    end
    assert(numel(arrivalTime) == eventCount && all(isfinite(timestamp)) && all(isfinite(arrivalTime)), ...
        "Event timestamps and arrival times must be finite aligned vectors.");

    events = struct();
    events.timestamp = timestamp;
    events.arrivalTime = arrivalTime;
    events.pose = pose;
    if includeInformation
        events.information = normalizeInformation(rawEvents, eventCount);
    else
        events.information = NaN(3, 3, eventCount);
    end
    [~, order] = sortrows([events.arrivalTime, events.timestamp]);
    events.timestamp = events.timestamp(order);
    events.arrivalTime = events.arrivalTime(order);
    events.pose = events.pose(order, :);
    events.information = events.information(:, :, order);
end

function information = normalizeInformation(rawEvents, eventCount)
% normalizeInformation Accept [3 x 3 x N], [N x 9], or missing Hessians.
    information = NaN(3, 3, eventCount);
    if ~isfield(rawEvents, "information") || isempty(rawEvents.information)
        return;
    end
    rawInformation = double(rawEvents.information);
    if isequal(size(rawInformation), [3, 3]) && eventCount == 1
        information(:, :, 1) = rawInformation;
    elseif ndims(rawInformation) == 3 && isequal(size(rawInformation, 1), 3) && ...
            isequal(size(rawInformation, 2), 3) && size(rawInformation, 3) == eventCount
        information = rawInformation;
    elseif isequal(size(rawInformation), [eventCount, 9])
        for eventIdx = 1:eventCount
            information(:, :, eventIdx) = reshape(rawInformation(eventIdx, :), 3, 3);
        end
    else
        error("Lidar information must be [3 x 3 x N] or [N x 9].");
    end
end

function events = emptyEvents(includeInformation)
% emptyEvents Return a correctly shaped empty asynchronous stream.
    poseWidth = 2 + double(includeInformation);
    events = struct("timestamp", zeros(0, 1), "arrivalTime", zeros(0, 1), ...
        "pose", zeros(0, poseWidth), "information", NaN(3, 3, 0));
end

function value = interpolateValue(values, intervalIdx, alpha, interpolation)
% interpolateValue Apply linear interpolation or zero-order hold.
    nextIdx = min(intervalIdx + 1, numel(values));
    if lower(strtrim(string(interpolation))) == "zoh"
        value = values(intervalIdx);
    else
        value = (1.0 - alpha) .* values(intervalIdx) + alpha .* values(nextIdx);
    end
end

function angle = interpolateAngle(values, intervalIdx, alpha, interpolation)
% interpolateAngle Interpolate through the shortest wrapped angular increment.
    nextIdx = min(intervalIdx + 1, numel(values));
    if lower(strtrim(string(interpolation))) == "zoh"
        angle = values(intervalIdx);
    else
        increment = wrapAngleToPi(values(nextIdx) - values(intervalIdx));
        angle = wrapAngleToPi(values(intervalIdx) + alpha .* increment);
    end
end

function validateObserverDesign(design, cfg)
% validateObserverDesign Reject incomplete or mismatched gain artifacts.
    requiredFields = ["P", "K", "N", "sigma", "certified", "verification", ...
        "cfg", "knownInputIncludedExactly", "scalingExponents"];
    for fieldName = requiredFields
        assert(isfield(design, fieldName), "observerDesign.%s is required.", fieldName);
    end
    assert(logical(design.certified) && logical(design.verification.certified) && ...
        design.verification.checkedVertexCount == 65536, ...
        "observerDesign must carry a completed 65,536-vertex certificate.");
    assert(logical(design.knownInputIncludedExactly), ...
        "observerDesign must include the known-input model in its certificate.");
    assert(isequal(size(design.P), [7, 7]) && isequal(size(design.K), [7, 3]) && ...
        isequal(size(design.N), [7, 4]) && all(isfinite(design.P), "all") && ...
        all(isfinite(design.K), "all") && all(isfinite(design.N), "all"), ...
        "observerDesign matrices must have compatible dimensions and finite values.");
    assert(abs(double(design.sigma) - double(cfg.observer.sigma)) <= 1.0e-12, ...
        "observerDesign.sigma must match cfg.observer.sigma.");
    certificateParameters = [design.cfg.operating.maximumSpeed; ...
        design.cfg.operating.maximumAcceleration; ...
        design.cfg.operating.maximumTrackAngleRate; ...
        design.cfg.lidar.minimumHeadingWeight];
    runtimeParameters = [cfg.operating.maximumSpeed; cfg.operating.maximumAcceleration; ...
        cfg.operating.maximumTrackAngleRate; cfg.lidar.minimumHeadingWeight];
    assert(all(abs(double(certificateParameters) - double(runtimeParameters)) <= 1.0e-12), ...
        "The runtime operating envelope must match the certified design envelope.");
    assert(isequal(double(design.scalingExponents(:)), ...
        double(cfg.observer.scalingExponents(:))), ...
        "The runtime high-gain scaling exponents must match the certified design.");
    assert(max(abs(double(design.N(:)) - double(cfg.observer.invariantGain(:)))) <= 1.0e-12, ...
        "cfg.observer.invariantGain must match the certified design.");
    assert(double(cfg.observer.theta) > double(cfg.observer.sigma), ...
        "cfg.observer.theta must be strictly greater than sigma.");
end

function wrappedAngle = wrapAngleToPi(angle)
% wrapAngleToPi Wrap radians to [-pi, pi).
    wrappedAngle = mod(angle + pi, 2.0 .* pi) - pi;
end
