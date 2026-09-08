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
    scaling = diag(cfg.observer.theta.^cfg.observer.scalingExponents(:));
    observerDesign.poseGain = scaling*observerDesign.K;
    observerDesign.invariantPhysicalGain = scaling*observerDesign.N/cfg.observer.theta^3;
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
    left = highRate.time(intervalIdx);
    right = highRate.time(intervalIdx+1);
    % Split at every active pulse boundary. All RK stages within a piece use
    % its left-continuous terminal mode, so an endpoint cannot shorten a pulse.
    gpsTimes = gps.timestamp(acceptedGps);
    lidarTimes = lidar.timestamp(acceptedLidar);
    edges = [gpsTimes; gpsTimes+cfg.measurement.gpsMaximumAge; ...
        lidarTimes; lidarTimes+cfg.measurement.lidarMaximumAge; ...
        lidarTimes+cfg.measurement.maximumPoseInterval];
    edges = edges(edges>left & edges<right);
    pieces = ceil((right-left)/cfg.measurement.maximumIntegrationStep-1e-10);
    cuts = unique([linspace(left,right,max(1,pieces)+1).'; edges]);
    stateNext = state;
    for part = 1:numel(cuts)-1
        t0 = cuts(part); t1 = cuts(part+1); dt = t1-t0;
        alpha0 = (t0-left)/(right-left);
        alpha1 = (t1-left)/(right-left);
        middle = (alpha0+alpha1)/2;
        endQuery = max(t0,t1-32*eps(max(1,abs(t1))));
        k1 = observerDerivative(stateNext,intervalIdx,alpha0,t0,highRate,lateralEstimate, ...
            gps,lidar,acceptedGps,acceptedLidar,design,cfg);
        k2 = observerDerivative(stateNext+dt*k1/2,intervalIdx,middle,(t0+t1)/2, ...
            highRate,lateralEstimate,gps,lidar,acceptedGps,acceptedLidar,design,cfg);
        k3 = observerDerivative(stateNext+dt*k2/2,intervalIdx,middle,(t0+t1)/2, ...
            highRate,lateralEstimate,gps,lidar,acceptedGps,acceptedLidar,design,cfg);
        k4 = observerDerivative(stateNext+dt*k3,intervalIdx,alpha1,endQuery, ...
            highRate,lateralEstimate,gps,lidar,acceptedGps,acceptedLidar,design,cfg);
        stateNext = stateNext+dt*(k1+2*k2+2*k3+k4)/6;
    end
    stateNext(7) = wrapAngleToPi(stateNext(7));
    assert(all(isfinite(stateNext)), ...
        "The improved observer produced a nonfinite state in interval %d.", intervalIdx);
end

function derivative = observerDerivative(state, intervalIdx, alpha, queryTime, highRate, lateralEstimate, ...
        gps, lidar, acceptedGps, acceptedLidar, design, cfg)
% observerDerivative Use the certified full-pose gain in both source modes.
    sample = measurementSample(highRate, lateralEstimate, intervalIdx, alpha, cfg);
    channels = evaluateImprovedObserverChannels(state, sample, cfg.operating);
    fusion = fusionMeasurementAt(queryTime, gps, lidar, acceptedGps, acceptedLidar, cfg);
    lidarResidual=[fusion.lidarPosition-state([1,4]);wrapAngleToPi(fusion.lidarHeading-state(7))];
    gpsResidual=[fusion.gpsPosition-state([1,4]);0];
    correction=fusion.lidarWeight*lidarResidual+fusion.gpsWeight*gpsResidual;
    derivative = channels.modelDerivative + design.poseGain*correction ...
        + design.invariantPhysicalGain*channels.invariantInnovation;
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
    baseInnovation = [fusion.basePosition(1) - state(1); ...
        fusion.basePosition(2) - state(4); ...
        wrapAngleToPi(fusion.lidarHeading - state(7))];
    lidarInnovation = fusion.lidarPosition - state([1, 4]);
end

function fusion = fusionMeasurementAt(queryTime, gps, lidar, acceptedGps, acceptedLidar, cfg)
% fusionMeasurementAt Return the latest physically available held samples.
    fusion = emptyFusionMeasurement();
    gpsIdx = latestEligibleEvent(gps, acceptedGps, queryTime);
    if ~isempty(gpsIdx)
        age = queryTime-gps.timestamp(gpsIdx);
        if age < cfg.measurement.gpsMaximumAge && all(isfinite(gps.pose(gpsIdx,1:2)))
            fusion.gpsPosition = gps.pose(gpsIdx,1:2).';
            fusion.gpsValid = true;
            fusion.gpsAge = age;
        end
    end
    lidarIdx = latestEligibleEvent(lidar, acceptedLidar, queryTime);
    recentLidar = false;
    if ~isempty(lidarIdx)
        age = queryTime-lidar.timestamp(lidarIdx);
        recentLidar = age < cfg.measurement.maximumPoseInterval;
        if age < cfg.measurement.lidarMaximumAge
            fusion.lidarPosition = lidar.pose(lidarIdx,1:2).';
            fusion.lidarHeading = lidar.pose(lidarIdx,3);
            fusion.lidarWeight = lidar.poseWeight(:,:,lidarIdx);
            fusion.normalizedWeight = lidar.normalizedWeight(:,:,lidarIdx);
            fusion.lidarPositionValid = true;
            fusion.lidarHeadingValid = true;
            fusion.lidarAge = age;
            fusion.basePosition = fusion.lidarPosition;
            fusion.positionSource = "lidar";
            % Combine independent residuals in information form. Cross terms
            % remain active and the total normalized weight stays in [0,I].
            if fusion.gpsValid
                fusion.lidarWeight=lidar.gpsFusedLidarWeight(:,:,lidarIdx);
                fusion.gpsWeight=lidar.gpsFusedGpsWeight(:,:,lidarIdx);
                fusion.normalizedWeight=lidar.gpsFusedNormalizedWeight(:,:,lidarIdx);
                fusion.positionSource="informationFusion";
            end
            fusion.omega=fusion.lidarWeight+fusion.gpsWeight;
            fusion.translationWeight=fusion.lidarWeight(1:2,1:2);
            fusion.headingWeight=fusion.lidarWeight(3,3);
        end
    end
    if ~recentLidar && fusion.gpsValid
        % Position-only continuation when no recent full pose exists. The
        % missing-heading/gap conditions are reported as outside the full-pose
        % certificate; GPS availability is not falsely labeled a yaw proof.
        fusion.basePosition = fusion.gpsPosition;
        fusion.positionSource = "gpsOnly";
        fusion.gpsWeight=gps.poseWeight;
        fusion.omega=gps.poseWeight;
        fusion.normalizedWeight=gps.normalizedWeight;
    end
end

function eventIdx = latestEligibleEvent(events, accepted, queryTime)
% latestEligibleEvent Select the most recent accepted physical timestamp.
    eligible = accepted & events.timestamp <= queryTime;
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
    fusion.basePosition = zeros(2,1);
    fusion.positionSource = "none";
    fusion.gpsPosition = zeros(2, 1);
    fusion.lidarPosition = zeros(2, 1);
    fusion.lidarHeading = 0.0;
    fusion.translationWeight = zeros(2, 2);
    fusion.headingWeight = 0.0;
    fusion.omega = zeros(3, 3);
    fusion.lidarWeight=zeros(3);fusion.gpsWeight=zeros(3);fusion.normalizedWeight=zeros(3);
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
        if age <= double(cfg.measurement.replayBufferDuration) + tolerance && events.qualified(eventIdx)
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
    initialGps = gps.qualified & gps.arrivalTime <= initialTime & gps.timestamp <= initialTime;
    initialLidar = lidar.qualified & lidar.arrivalTime <= initialTime & lidar.timestamp <= initialTime;
    position = double(cfg.observer.fallbackPosition(:));
    heading = double(cfg.observer.fallbackHeading);
    gpsIdx = latestEligibleEvent(gps, initialGps, initialTime);
    lidarIdx = latestEligibleEvent(lidar, initialLidar, initialTime);
    if any(initialGps)
        position = gps.pose(gpsIdx, 1:2).';
    end
    if ~isempty(lidarIdx)
        if lidar.informationDiagnostics{lidarIdx}.rank == 3
            % Preserve full-pose initialization and GPS position precedence.
            if isempty(gpsIdx), position = lidar.pose(lidarIdx, 1:2).'; end
            heading = lidar.pose(lidarIdx, 3);
        else
            % An arbitrary representative in a LiDAR nullspace is not an
            % initial measurement. Apply its bounded directional weight to
            % the fallback/GPS prior, on the same local yaw branch as replay.
            residual = [lidar.pose(lidarIdx, 1:2).'-position; ...
                wrapAngleToPi(lidar.pose(lidarIdx, 3)-heading)];
            correction = lidar.poseWeight(:,:,lidarIdx)*residual;
            if isempty(gpsIdx), position = position+correction(1:2); end
            heading = heading+correction(3);
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
    motionHeadingSensitivity = zeros(sampleCount, 1);
    translationWeight = zeros(2, 2, sampleCount);
    headingWeight = zeros(sampleCount, 1);
    lidarPoseWeight=zeros(3,3,sampleCount);gpsPoseWeight=zeros(3,3,sampleCount);
    totalNormalizedPoseWeight=zeros(3,3,sampleCount);
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
        channels = evaluateImprovedObserverChannels(stateHistory(sampleIdx, :).', sample, cfg.operating);
        fusion = fusionMeasurementAt(highRate.time(sampleIdx), gps, lidar, acceptedGps, acceptedLidar, cfg);
        [baseInnovation(sampleIdx, :), lidarInnovation(sampleIdx, :)] = ...
            rowPoseInnovations(stateHistory(sampleIdx, :).', fusion);
        invariantInnovation(sampleIdx, :) = channels.invariantInnovation.';
        motionHeadingSensitivity(sampleIdx) = channels.motionHeadingSensitivity;
        translationWeight(:, :, sampleIdx) = fusion.translationWeight;
        headingWeight(sampleIdx) = fusion.headingWeight;
        lidarPoseWeight(:,:,sampleIdx)=fusion.lidarWeight;
        gpsPoseWeight(:,:,sampleIdx)=fusion.gpsWeight;
        totalNormalizedPoseWeight(:,:,sampleIdx)=fusion.normalizedWeight;
        gpsAge(sampleIdx) = fusion.gpsAge;
        lidarAge(sampleIdx) = fusion.lidarAge;
        trackAngleRate(sampleIdx) = channels.trackAngleRate;
    end

    estimate = struct();
    estimate.time = highRate.time;
    estimate.z = stateHistory; % Legacy revised history, not a causal output.
    estimate.revisedZ = stateHistory;
    estimate.pose = onlineState(:,[1,4,7]);
    estimate.onlineZ = onlineState;
    estimate.position = onlineState(:, [1, 4]);
    estimate.velocity = onlineState(:, [2, 5]);
    estimate.acceleration = onlineState(:, [3, 6]);
    estimate.heading = onlineState(:, 7);
    estimate.speed = hypot(onlineState(:, 2), onlineState(:, 5));
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
    estimate.diagnostics.lidarPoseWeight=lidarPoseWeight;
    estimate.diagnostics.gpsPoseWeight=gpsPoseWeight;
    estimate.diagnostics.totalNormalizedPoseWeight=totalNormalizedPoseWeight;
    estimate.diagnostics.lidarPoseGain=pagemtimes(design.poseGain,lidarPoseWeight);
    estimate.diagnostics.informationTimeBasis="revised measurement-time history";
    estimate.diagnostics.motionHeadingSensitivity = motionHeadingSensitivity;
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
    estimate.diagnostics.invariantExtensionActive = any(abs(onlineState(:,[2,5]))>cfg.operating.maximumSpeed,2) ...
        | any(abs(onlineState(:,[3,6]))>cfg.operating.maximumAcceleration,2);
    estimate.diagnostics.certificateConditions = observerCertificateConditions( ...
        highRate,gps,lidar,acceptedGps,acceptedLidar,onlineState,cfg);
    estimate.observer = struct("P", design.P, "K", design.K, "N", design.N, ...
        "theta", cfg.observer.theta, "sigma", cfg.observer.sigma, ...
        "certificateVerified", design.certified, "certified", false, ...
        "verification", design.verification, ...
        "scope", "conditional flow certificate; runtime error/domain and discretization bounds are separate");
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
    gps.qualified = all(isfinite(gps.pose(:,1:2)),2);
    lidar.qualified = false(numel(lidar.timestamp),1);
    lidar.headingWeight = zeros(numel(lidar.timestamp),1);
    lidar.poseWeight=zeros(3,3,numel(lidar.timestamp));
    lidar.normalizedWeight=lidar.poseWeight;
    lidar.gpsFusedLidarWeight=lidar.poseWeight;lidar.gpsFusedGpsWeight=lidar.poseWeight;
    lidar.gpsFusedNormalizedWeight=lidar.poseWeight;
    [~,gps.poseWeight,gps.normalizedWeight]=fusePoseInformationWeights(zeros(3),true,cfg);
    lidar.informationDiagnostics = cell(numel(lidar.timestamp),1);
    for eventIdx=1:numel(lidar.timestamp)
        [~,weight,info,W] = computeLidarInformationWeights(lidar.information(:,:,eventIdx),cfg);
        lidar.qualified(eventIdx) = info.qualified && all(isfinite(lidar.pose(eventIdx,:)));
        lidar.headingWeight(eventIdx) = weight;
        lidar.poseWeight(:,:,eventIdx)=W;lidar.normalizedWeight(:,:,eventIdx)=info.normalizedWeight;
        lidar.gpsFusedLidarWeight(:,:,eventIdx)=info.gpsFusedLidarWeight;
        lidar.gpsFusedGpsWeight(:,:,eventIdx)=info.gpsFusedGpsWeight;
        lidar.gpsFusedNormalizedWeight(:,:,eventIdx)=info.gpsFusedNormalizedWeight;
        lidar.informationDiagnostics{eventIdx} = info;
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
    assert(isfield(design,'kind') && string(design.kind)=="aperiodic-anisotropic-pose-v2", ...
        'VehicleLocalization:CertificateMismatch','Runtime requires the aperiodic pose certificate.');
    assert(design.certified && design.verification.certified, ...
        'VehicleLocalization:CertificateMismatch','The pose design is not verified.');
    assert(isfield(design.verification,'verifiedModel'), ...
        'VehicleLocalization:CertificateMismatch','The certificate lacks its verified model snapshot.');
    verified = design.verification.verifiedModel;
    assert(isequal(design.K,verified.K) && isequal(design.N,verified.N) ...
        && isequal(design.timer.P,verified.P) ...
        && isequal(design.timer.knots,verified.knots) ...
        && design.timer.rate==verified.rate && design.timer.alpha==verified.alpha ...
        && isequal(design.timer.multipliers,verified.multipliers) ...
        && isequal(cfg.lidar.poseScales(:),verified.poseScales(:)) ...
        && isequal(design.timer.poseScales(:),verified.poseScales(:)) ...
        && design.timer.onTime==verified.onTime && design.timer.tMin==verified.tMin ...
        && design.timer.tMax==verified.tMax ...
        && cfg.observer.theta==verified.theta ...
        && design.theta==verified.theta && design.timer.theta==verified.theta ...
        && isequal(cfg.operating,verified.operating) ...
        && isequal(cfg.observer.scalingExponents(:),verified.scalingExponents(:)), ...
        'VehicleLocalization:CertificateMismatch','Runtime matrices or envelope differ from verification.');
    expected = [verified.theta; verified.theta; verified.onTime; verified.onTime; ...
        verified.tMin; verified.tMax; verified.alpha];
    actual = [cfg.observer.theta; cfg.observer.sigma; cfg.measurement.gpsMaximumAge; ...
        cfg.measurement.lidarMaximumAge; cfg.measurement.minimumPoseInterval; ...
        cfg.measurement.maximumPoseInterval; cfg.lidar.certificateMinimumPoseWeight];
    assert(all(abs(expected-actual)<1e-12) ...
        && max(abs(design.N-cfg.observer.invariantGain),[],'all')<1e-12, ...
        'VehicleLocalization:CertificateMismatch','Runtime gains or pulse timing differ from verification.');
    assert(string(cfg.observer.integrationMethod)=="rk4" ...
        && cfg.measurement.maximumIntegrationStep>0 ...
        && cfg.measurement.maximumIntegrationStep<=.01 ...
        && cfg.measurement.timestampTolerance==0, ...
        'VehicleLocalization:CertificateMismatch','Use event-split RK4 with no future timestamp allowance.');
end

function wrappedAngle = wrapAngleToPi(angle)
% wrapAngleToPi Wrap radians to [-pi, pi).
    wrappedAngle = mod(angle + pi, 2.0 .* pi) - pi;
end

function conditions = observerCertificateConditions(highRate,gps,lidar,acceptedGps,acceptedLidar,onlineState,cfg)
% observerCertificateConditions Report observable hypotheses, not a theorem flag.
% Qualified-pose gaps are measured at sensor timestamps. Arrival delay is
% checked separately, so a stale delivery cannot masquerade as a fresh pose.
    time = highRate.time;
    stamps = sort(lidar.timestamp(acceptedLidar));
    eventWeights=lidar.normalizedWeight(:,:,acceptedLidar);
    minimumWeights=zeros(size(eventWeights,3),1);
    for k=1:numel(minimumWeights),minimumWeights(k)=min(eig(eventWeights(:,:,k)));end
    gaps = diff(stamps);
    if isempty(gaps), largestGap = Inf; else, largestGap = max(gaps); end
    fresh = false(size(time));
    for k=1:numel(stamps)
        fresh = fresh | (time>=stamps(k) & time<=stamps(k)+cfg.measurement.maximumPoseInterval);
    end
    delay = lidar.arrivalTime(acceptedLidar)-lidar.timestamp(acceptedLidar);
    if isempty(delay), delayMatches=false; else
        delayMatches=all(abs(delay-cfg.measurement.fixedLidarDelay)<1e-8);
    end
    tooShort = gaps<cfg.measurement.minimumPoseInterval-1e-9;
    tooLong = gaps>cfg.measurement.maximumPoseInterval+1e-9;
    if isempty(stamps), firstPose=Inf; else, firstPose=stamps(1); end
    % Only the fully received measurement-time horizon can be audited. The
    % causal prediction tail is checked separately through the delay model.
    settled = time>=firstPose & time<=time(end)-cfg.measurement.fixedLidarDelay;
    velocityOutside = any(abs(onlineState(:,[2 5]))>cfg.operating.maximumSpeed,2);
    accelerationOutside = any(abs(onlineState(:,[3 6]))>cfg.operating.maximumAcceleration,2);
    gpsDelays=gps.arrivalTime(acceptedGps)-gps.timestamp(acceptedGps);
    if isempty(gpsDelays), maximumGpsDelay=0; else, maximumGpsDelay=max(gpsDelays); end
    conditions = struct('maximumGpsDelaySeconds',maximumGpsDelay, ...
        'minimumLidarWeightEigenvalues',minimumWeights, ...
        'weightSectorViolationCount',nnz(minimumWeights<cfg.lidar.certificateMinimumPoseWeight-1e-9), ...
        'informationWithinCertificate',~isempty(minimumWeights) ...
            && all(minimumWeights>=cfg.lidar.certificateMinimumPoseWeight-1e-9), ...
        'fixedLagBoundCoversGpsDelay',maximumGpsDelay<=cfg.measurement.fixedLidarDelay+1e-8, ...
        'maximumQualifiedPoseGapSeconds',largestGap, ...
        'qualifiedPoseIntervals',gaps,'shortIntervalCount',nnz(tooShort), ...
        'longIntervalCount',nnz(tooLong),'fullPoseFreshAtMeasurementTime',fresh, ...
        'freshFractionAfterStartup',mean(fresh(settled)), ...
        'fixedDelayMatchesConfiguration',delayMatches, ...
        'configuredFixedDelaySeconds',cfg.measurement.fixedLidarDelay, ...
        'replayBufferCoversFixedDelay',cfg.measurement.replayBufferDuration ...
            >=cfg.measurement.fixedLidarDelay+cfg.measurement.maximumIntegrationStep, ...
        'informationRejectedCount',nnz(~lidar.qualified), ...
        'acceptedGpsEvents',nnz(acceptedGps),'availableGpsEvents',numel(gps.timestamp), ...
        'onlineVelocityOutsideEnvelope',velocityOutside, ...
        'onlineAccelerationOutsideEnvelope',accelerationOutside, ...
        'poseErrorBoundValidated',logical(cfg.lidar.errorBoundValidated), ...
        'upstreamDisturbanceBoundValidated',false,'numericalErrorBoundValidated',false, ...
        'unconditionalStabilityClaimed',false);
    conditions.timingWithinCertificate = ~isempty(gaps) && any(settled) && ~any(tooShort | tooLong) ...
        && all(fresh(settled));
    conditions.lidarInformationWithinCertificate = conditions.informationWithinCertificate;
    % Audit the matrix actually used throughout every settled LiDAR pulse.
    % GPS starts/expiry can fall between high-rate samples; split exactly at
    % those events so a short unanchored interval is not missed by sampling.
    [intervals, combinedMinimum] = pulseInformationIntervals(highRate,gps,lidar, ...
        acceptedGps,acceptedLidar,cfg);
    conditions.posePulseInformationIntervals = intervals;
    conditions.minimumCombinedWeightEigenvalues = combinedMinimum;
    conditions.combinedWeightSectorViolationCount = nnz( ...
        combinedMinimum<cfg.lidar.certificateMinimumPoseWeight-1e-9);
    conditions.informationWithinCertificate = ~isempty(combinedMinimum) ...
        && conditions.combinedWeightSectorViolationCount==0;
    conditions.informationAuditScope = ...
        "actual fused weights on event-split settled LiDAR pulses; prediction intervals excluded";
end

function [intervals, minimumWeights] = pulseInformationIntervals(highRate,gps,lidar, ...
        acceptedGps,acceptedLidar,cfg)
% pulseInformationIntervals Check every constant-weight piece, including expiry.
    stamps = unique(lidar.timestamp(acceptedLidar));
    gpsStamps = gps.timestamp(acceptedGps);
    auditEnd = highRate.time(end)-cfg.measurement.fixedLidarDelay;
    boundaries = unique([highRate.time(1); auditEnd; stamps; ...
        stamps+cfg.measurement.lidarMaximumAge; gpsStamps; ...
        gpsStamps+cfg.measurement.gpsMaximumAge]);
    boundaries = boundaries(boundaries>=highRate.time(1) & boundaries<=auditEnd);
    intervals = zeros(max(0,numel(boundaries)-1),2);
    minimumWeights = zeros(size(intervals,1),1);
    count = 0;
    for k=1:numel(boundaries)-1
        midpoint = boundaries(k)+(boundaries(k+1)-boundaries(k))/2;
        fusion = fusionMeasurementAt(midpoint,gps,lidar,acceptedGps,acceptedLidar,cfg);
        if ~fusion.lidarPositionValid, continue; end
        count = count+1;
        intervals(count,:) = boundaries(k:k+1).';
        minimumWeights(count) = min(eig(fusion.normalizedWeight));
    end
    intervals = intervals(1:count,:);
    minimumWeights = minimumWeights(1:count);
end
