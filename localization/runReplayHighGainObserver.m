function estimate = runReplayHighGainObserver(highRateMeasurements, poseMeasurements, cfg)
% runReplayHighGainObserver: Run an online retrodictive-replay high-gain observer for the Bessafa
% et al. transformed ego-vehicle state. The observer separates continuous
% high-rate flow from low-rate timestamped pose corrections: delayed pose
% measurements are inserted as discrete jumps at their physical timestamp,
% then the observer buffer is replayed through the high-rate dynamics to the
% current sample.
%
% Input:
%   highRateMeasurements: struct or table with 100 Hz time, speed, yawRate,
%       steeringAngle, and optional orthogonalityConstraint series
%   poseMeasurements: struct or table with 5 Hz timestamp, arrivalTime,
%       position or pose, and yaw/Psi series
%   cfg: struct from replayObserverConfig
%
% Output:
%   estimate: struct with 100 Hz transformed-state estimates, derived
%       vehicle states, replay diagnostics, and copied measurement data
    if nargin < 3 || isempty(cfg)
        cfg = replayObserverConfig();
    end

    assert(isstruct(highRateMeasurements) || istable(highRateMeasurements), ...
        "highRateMeasurements must be a struct or table.");
    assert(isstruct(poseMeasurements) || istable(poseMeasurements), ...
        "poseMeasurements must be a struct or table.");

    cfg = normalizeReplayConfig(cfg);
    highRate = normalizeHighRateMeasurements(highRateMeasurements, cfg);
    pose = normalizePoseMeasurements(poseMeasurements);
    observer = observerMatrices(cfg);
    initialState = buildInitialState(highRate, pose, cfg);

    numSamples = numel(highRate.time);
    stateHistory = NaN(numSamples, 6);
    onlineState = NaN(numSamples, 6);
    informationAge = NaN(numSamples, 1);
    replayCount = zeros(numSamples, 1);
    acceptedCount = zeros(numSamples, 1);
    replayStartIdx = NaN(numSamples, 1);
    bufferStartIdx = ones(numSamples, 1);
    stateHistory(1, :) = initialState.';
    preCorrectionState = createReplayWindow(initialState);
    stateBuffer = createReplayWindow(initialState);

    [~, poseArrivalOrder] = sortrows([pose.arrivalTime, pose.timestamp]);
    poseCursor = 1;
    acceptedCorrections = emptyAcceptedCorrections();
    sequenceNumber = 0;
    acceptedCorrectionCount = 0;
    latestAcceptedTimestamp = NaN;

    for sampleIdx = 1:numSamples
        if sampleIdx > 1
            preCorrectionRow = propagateState( ...
                replayWindowRow(stateBuffer, sampleIdx - 1).', highRate, sampleIdx - 1, cfg, observer).';
            preCorrectionState = setReplayWindowRow(preCorrectionState, sampleIdx, preCorrectionRow);
            stateBuffer = setReplayWindowRow(stateBuffer, sampleIdx, ...
                replayWindowRow(preCorrectionState, sampleIdx));
        end

        currentTime = highRate.time(sampleIdx);
        bufferStartIdx(sampleIdx) = replayBufferStartIdx(highRate.time, currentTime, ...
            cfg.replay.bufferDuration, cfg.replay.timestampTolerance);
        retainedStartIdx = retainedReplayStartIdx(bufferStartIdx(sampleIdx));
        preCorrectionState = trimReplayWindow(preCorrectionState, retainedStartIdx);
        stateBuffer = trimReplayWindow(stateBuffer, retainedStartIdx);
        earliestReplayIdx = Inf;
        acceptedThisSample = 0;
        while poseCursor <= numel(poseArrivalOrder) && ...
                pose.arrivalTime(poseArrivalOrder(poseCursor)) <= currentTime + cfg.replay.timestampTolerance
            poseIdx = poseArrivalOrder(poseCursor);
            timestampIdx = timestampIndex(highRate.time, pose.timestamp(poseIdx), cfg.replay.timestampTolerance);
            assert(timestampIdx <= sampleIdx, ...
                "Pose timestamp %.6f is later than the current replay sample time %.6f.", ...
                pose.timestamp(poseIdx), currentTime);
            assert(currentTime - pose.timestamp(poseIdx) <= cfg.replay.bufferDuration + cfg.replay.timestampTolerance, ...
                "Replay buffer is too short for a pose measurement with timestamp %.6f and arrival time %.6f.", ...
                pose.timestamp(poseIdx), currentTime);
            assert(timestampIdx >= bufferStartIdx(sampleIdx), ...
                "Replay buffer window starts at index %d, but pose correction requires index %d.", ...
                bufferStartIdx(sampleIdx), timestampIdx);
            sequenceNumber = sequenceNumber + 1;
            acceptedCorrections = appendAcceptedCorrection(acceptedCorrections, ...
                timestampIdx, pose.timestamp(poseIdx), pose.arrivalTime(poseIdx), pose.pose(poseIdx, :), sequenceNumber);
            acceptedCorrectionCount = acceptedCorrectionCount + 1;
            if ~isfinite(latestAcceptedTimestamp) || pose.timestamp(poseIdx) > latestAcceptedTimestamp
                latestAcceptedTimestamp = pose.timestamp(poseIdx);
            end
            earliestReplayIdx = min(earliestReplayIdx, timestampIdx);
            acceptedThisSample = acceptedThisSample + 1;
            poseCursor = poseCursor + 1;
        end

        if acceptedThisSample > 0
            [preCorrectionState, stateBuffer] = replayStateHistory(preCorrectionState, ...
                stateBuffer, highRate, acceptedCorrections, earliestReplayIdx, sampleIdx, cfg, observer);
            replayCount(sampleIdx) = replayCount(sampleIdx) + 1;
            replayStartIdx(sampleIdx) = earliestReplayIdx;
            stateHistory(earliestReplayIdx:sampleIdx, :) = ...
                replayWindowRows(stateBuffer, earliestReplayIdx, sampleIdx);
        else
            stateHistory(sampleIdx, :) = replayWindowRow(stateBuffer, sampleIdx);
        end

        onlineState(sampleIdx, :) = replayWindowRow(stateBuffer, sampleIdx);
        acceptedCount(sampleIdx) = acceptedCorrectionCount;
        informationAge(sampleIdx) = computeInformationAge(currentTime, latestAcceptedTimestamp);
        acceptedCorrections = trimAcceptedCorrections(acceptedCorrections, bufferStartIdx(sampleIdx));
    end

    estimate = buildEstimate(highRate, pose, stateHistory, onlineState, informationAge, ...
        replayCount, acceptedCount, acceptedCorrections, preCorrectionState, stateBuffer, ...
        replayStartIdx, bufferStartIdx, observer, cfg);
end

function cfg = normalizeReplayConfig(cfg)
% normalizeReplayConfig: Fill missing replay-specific fields with
% conservative defaults so the replay observer can run with either the
% replay config or the base Bessafa observer config.
%
% Input:
%   cfg: observer configuration struct
%
% Output:
%   cfg: configuration struct with required replay fields populated
    if ~isfield(cfg, "replay") || isempty(cfg.replay)
        cfg.replay = struct();
    end
    cfg.replay.sampleTime = readScalar(cfg.replay, "sampleTime", cfg.measurement.sampleTime);
    cfg.replay.bufferDuration = readScalar(cfg.replay, "bufferDuration", 1.0);
    cfg.replay.maxInformationAge = readScalar(cfg.replay, "maxInformationAge", cfg.replay.bufferDuration);
    cfg.replay.timestampTolerance = readScalar(cfg.replay, "timestampTolerance", 0.5 .* cfg.replay.sampleTime);
    cfg.replay.speedRegularization = readScalar(cfg.replay, "speedRegularization", 1.0e-6);
    cfg.replay.yawVelocityBlend = 0.0;
    cfg.replay.yawAccelerationBlend = 0.0;
    cfg.replay.outputPositionBlendTimeConstant = readScalar(cfg.replay, "outputPositionBlendTimeConstant", 0.35);
    cfg.replay.outputAccelerationFilterTimeConstant = readScalar(cfg.replay, "outputAccelerationFilterTimeConstant", 0.20);
    cfg.replay.inputInterpolation = readString(cfg.replay, "inputInterpolation", "zoh");
    cfg.replay.usePoseYawVelocityCorrection = false;
    assert(cfg.replay.bufferDuration >= cfg.replay.maxInformationAge, ...
        "cfg.replay.bufferDuration must be at least cfg.replay.maxInformationAge.");
end

function highRate = normalizeHighRateMeasurements(measurements, cfg)
% normalizeHighRateMeasurements: Convert the high-rate measurement
% stream into aligned numeric series used by the flow propagation. Missing
% steering values use the configured default, and the orthogonality
% constraint defaults to zero.
%
% Input:
%   measurements: raw high-rate struct or table
%   cfg: observer configuration struct
%
% Output:
%   highRate: normalized high-rate measurement struct
    numSamples = inferSeriesLength(measurements, ["time", "Time", "t", "timestamp", "Timestamp", ...
        cfg.measurement.speedFields, cfg.measurement.yawRateFields, cfg.measurement.steeringFields]);
    time = readScalarSeries(measurements, ["time", "Time", "t", "timestamp", "Timestamp"], numSamples);
    if all(~isfinite(time))
        time = (0:(numSamples - 1)).' .* cfg.replay.sampleTime;
    end

    steeringAngle = readScalarSeries(measurements, cfg.measurement.steeringFields, numSamples);
    steeringAngle(~isfinite(steeringAngle)) = cfg.measurement.defaultSteeringAngle;
    orthogonalityConstraint = readScalarSeries(measurements, ...
        ["orthogonalityConstraint", "constraint", "accelerationConstraint"], numSamples);
    orthogonalityConstraint(~isfinite(orthogonalityConstraint)) = 0.0;

    highRate = struct();
    highRate.time = time;
    highRate.speed = readScalarSeries(measurements, cfg.measurement.speedFields, numSamples);
    highRate.yawRate = readScalarSeries(measurements, cfg.measurement.yawRateFields, numSamples);
    highRate.steeringAngle = steeringAngle;
    highRate.orthogonalityConstraint = orthogonalityConstraint;
end

function pose = normalizePoseMeasurements(measurements)
% normalizePoseMeasurements: Convert timestamped low-rate pose
% measurements into physical timestamp, arrival time, and [X, Y, yaw] arrays.
% If arrivalTime is absent, each pose is treated as arriving at its physical
% timestamp.
%
% Input:
%   measurements: raw pose struct or table
%
% Output:
%   pose: normalized pose measurement struct
    numSamples = inferSeriesLength(measurements, ["timestamp", "Timestamp", "physicalTime", ...
        "sensorTime", "time", "Time", "arrivalTime", "ArrivalTime", "pose", "Pose", "position", "Position"]);
    timestamp = readScalarSeries(measurements, ["timestamp", "Timestamp", "physicalTime", ...
        "sensorTime", "time", "Time"], numSamples);
    assert(any(isfinite(timestamp)), "poseMeasurements must contain finite physical timestamps.");
    arrivalTime = readScalarSeries(measurements, ["arrivalTime", "ArrivalTime", ...
        "receivedTime", "receiveTime", "tArrival"], numSamples);
    arrivalTime(~isfinite(arrivalTime)) = timestamp(~isfinite(arrivalTime));

    poseMatrix = readMatrixSeries(measurements, ["pose", "Pose"], numSamples, 3);
    if isempty(poseMatrix)
        position = readMatrixSeries(measurements, ["position", "Position", "xy", "XY"], numSamples, 2);
        if isempty(position)
            xValue = readScalarSeries(measurements, ["x", "X"], numSamples);
            yValue = readScalarSeries(measurements, ["y", "Y"], numSamples);
            position = [xValue, yValue];
        end
        yaw = readScalarSeries(measurements, ["yaw", "Yaw", "psi", "Psi", "heading", "Heading"], numSamples);
        poseMatrix = [position(:, 1:2), yaw];
    end

    pose = struct();
    pose.timestamp = timestamp;
    pose.arrivalTime = arrivalTime;
    pose.pose = poseMatrix(:, 1:3);
end

function observer = observerMatrices(cfg)
% observerMatrices: Realize the high-gain scaling, high-rate
% extra-output gain, and timestamp correction gain used by the replay
% observer.
%
% Input:
%   cfg: observer configuration struct
%
% Output:
%   observer: struct with T, M, Ld, theta, and selected output names
    theta = double(cfg.observer.theta);
    scalingExponents = double(cfg.observer.scalingExponents(:));
    extraGainThetaPower = double(cfg.observer.extraGainThetaPower);
    extraOutputs = string(cfg.observer.extraOutputs(:));
    N = double(cfg.observer.N);
    assert(theta > 1.0 && numel(scalingExponents) == 6, ...
        "cfg.observer.theta and cfg.observer.scalingExponents must define a six-state scaling.");
    assert(size(N, 1) == 6 && size(N, 2) == numel(extraOutputs), ...
        "cfg.observer.N columns must match cfg.observer.extraOutputs.");

    observer = struct();
    observer.theta = theta;
    observer.T = diag(theta .^ scalingExponents);
    observer.M = (observer.T * N) ./ (theta .^ extraGainThetaPower);
    observer.extraOutputs = extraOutputs;

    if isfield(cfg.replay, "Kd") && ~isempty(cfg.replay.Kd)
        Kd = double(cfg.replay.Kd);
        assert(size(Kd, 1) == 6 && (size(Kd, 2) == 2 || size(Kd, 2) == 3), ...
            "cfg.replay.Kd must be [6 x 2] or [6 x 3].");
        observer.Ld = (observer.T * Kd) ./ theta;
        observer.Kd = Kd;
        observer.gainSource = "cfg.replay.Kd";
    elseif isfield(cfg.replay, "timestampCorrectionGain") && ~isempty(cfg.replay.timestampCorrectionGain)
        observer.Ld = double(cfg.replay.timestampCorrectionGain);
        assert(size(observer.Ld, 1) == 6 && (size(observer.Ld, 2) == 2 || size(observer.Ld, 2) == 3), ...
            "cfg.replay.timestampCorrectionGain must be [6 x 2] or [6 x 3].");
        observer.Kd = theta .* (observer.T \ observer.Ld);
        observer.gainSource = "cfg.replay.timestampCorrectionGain";
    else
        error("runReplayHighGainObserver requires cfg.replay.Kd from designReplayObserverGains or an explicit cfg.replay.timestampCorrectionGain; implicit fallback timestamp gains are not allowed.");
    end
    observer.poseCorrectionOutputs = onlinePoseCorrectionOutputs(cfg, size(observer.Ld, 2));
end

function outputs = onlinePoseCorrectionOutputs(cfg, numGainColumns)
% onlinePoseCorrectionOutputs: Read and validate the timestamped
% pose-correction channel order used by the online jump gain. The online
% residual vector is only accepted in the canonical [x; y] or [x; y; yaw]
% order so each gain column matches the offline LMI design column.
%
% Input:
%   cfg: observer configuration struct
%   numGainColumns: number of columns in the realized timestamp gain
%
% Output:
%   outputs: [p x 1] canonical pose-correction output names
    assert(isfield(cfg, "replayDesign") && isstruct(cfg.replayDesign) && ...
        isfield(cfg.replayDesign, "poseCorrectionOutputs") && ~isempty(cfg.replayDesign.poseCorrectionOutputs), ...
        "cfg.replayDesign.poseCorrectionOutputs is required for online pose-correction gain column binding.");
    outputs = lower(strtrim(string(cfg.replayDesign.poseCorrectionOutputs(:))));
    canonicalOutputs = ["x"; "y"; "yaw"];
    assert(numel(outputs) == numGainColumns, ...
        "cfg.replayDesign.poseCorrectionOutputs must have one entry per timestamp correction gain column.");
    assert(numGainColumns == 2 || numGainColumns == 3, ...
        "Timestamp correction gain must have two or three pose-correction columns.");
    assert(all(outputs == canonicalOutputs(1:numGainColumns)), ...
        "cfg.replayDesign.poseCorrectionOutputs must be ordered as [x; y] or [x; y; yaw].");
end

function initialState = buildInitialState(highRate, pose, cfg)
% buildInitialState: Initialize the transformed observer state from an
% explicit configured state when available, otherwise from the first pose,
% first speed measurement, steering-dependent side slip, and configured
% acceleration fallback.
%
% Input:
%   highRate: normalized high-rate measurement struct
%   pose: normalized pose measurement struct
%   cfg: observer configuration struct
%
% Output:
%   initialState: [6 x 1] transformed observer state
    if isfield(cfg, "initialization") && isfield(cfg.initialization, "state") && ~isempty(cfg.initialization.state)
        initialState = double(cfg.initialization.state(:));
        assert(numel(initialState) == 6 && all(isfinite(initialState)), ...
            "cfg.initialization.state must be a finite [6 x 1] vector.");
        return;
    end

    poseIdx = find(all(isfinite(pose.pose(:, 1:2)), 2), 1);
    if isempty(poseIdx)
        position = double(cfg.simulation.defaultInitialPosition(:));
        yaw = double(cfg.initialization.fallbackHeading);
    else
        position = pose.pose(poseIdx, 1:2).';
        yaw = pose.pose(poseIdx, 3);
        if ~isfinite(yaw)
            yaw = double(cfg.initialization.fallbackHeading);
        end
    end

    speed = firstFiniteScalar(highRate.speed, cfg.initialization.fallbackSpeed);
    steeringAngle = firstFiniteScalar(highRate.steeringAngle, cfg.measurement.defaultSteeringAngle);
    beta = sideSlipAngle(steeringAngle, cfg);
    heading = yaw + beta;
    yawRate = firstFiniteScalar(highRate.yawRate, 0.0);
    acceleration = double(cfg.initialization.acceleration(:));
    if numel(acceleration) ~= 2 || any(~isfinite(acceleration))
        acceleration = speed .* yawRate .* [-sin(heading); cos(heading)];
    end
    initialState = [position(1); speed .* cos(heading); acceleration(1); ...
        position(2); speed .* sin(heading); acceleration(2)];
end

function stateNext = propagateState(state, highRate, sampleIdx, cfg, observer)
% propagateState: Integrate one high-rate interval of the replay
% observer flow using either Euler or fourth-order Runge-Kutta integration
% and the configured high-rate measurement interpolation rule. The flow only
% contains the triangular vehicle dynamics and high-rate nonlinear output
% correction; timestamped pose corrections are applied separately as
% discrete jumps.
%
% Input:
%   state: [6 x 1] transformed observer state at high-rate sampleIdx
%   highRate: normalized high-rate measurement struct
%   sampleIdx: interval start index
%   cfg: observer configuration struct
%   observer: realized observer matrix struct
%
% Output:
%   stateNext: [6 x 1] propagated transformed observer state
    sampleTime = highRate.time(sampleIdx + 1) - highRate.time(sampleIdx);
    if ~isfinite(sampleTime) || sampleTime <= 0.0
        sampleTime = cfg.replay.sampleTime;
    end

    integrationMethod = lower(strtrim(string(cfg.observer.integrationMethod)));
    if integrationMethod == "euler"
        derivative = observerFlowDerivative(state, highRate, sampleIdx, 0.0, cfg, observer);
        stateNext = state + sampleTime .* derivative;
    elseif integrationMethod == "rk4"
        k1 = observerFlowDerivative(state, highRate, sampleIdx, 0.0, cfg, observer);
        k2 = observerFlowDerivative(state + 0.5 .* sampleTime .* k1, highRate, sampleIdx, 0.5, cfg, observer);
        k3 = observerFlowDerivative(state + 0.5 .* sampleTime .* k2, highRate, sampleIdx, 0.5, cfg, observer);
        k4 = observerFlowDerivative(state + sampleTime .* k3, highRate, sampleIdx, 1.0, cfg, observer);
        stateNext = state + sampleTime ./ 6.0 .* (k1 + 2.0 .* k2 + 2.0 .* k3 + k4);
    else
        error("Unsupported replay observer integration method: %s.", integrationMethod);
    end
end

function derivative = observerFlowDerivative(state, highRate, sampleIdx, alpha, cfg, observer)
% observerFlowDerivative: Evaluate the continuous replay-observer flow
% using only the triangular vehicle model and high-rate nonlinear output
% innovation. Low-rate pose measurements are not derivative inputs.
%
% Input:
%   state: [6 x 1] transformed observer state
%   highRate: normalized high-rate measurement struct
%   sampleIdx: interval start index
%   alpha: interpolation fraction in [0, 1]
%   cfg: observer configuration struct
%   observer: realized observer matrix struct
%
% Output:
%   derivative: [6 x 1] observer flow derivative
    highMeasurement = highRateMeasurementAt(highRate, sampleIdx, alpha, cfg);
    modelDerivative = triangularVehicleDerivative(state, cfg);
    extraPrediction = extraOutputPrediction(state, cfg);
    [extraMeasurement, extraValid] = extraMeasurementVector(highMeasurement, cfg);
    extraInnovation = extraMeasurement - extraPrediction;
    extraInnovation(~extraValid) = 0.0;
    derivative = modelDerivative + observer.M * extraInnovation;
end

function innovation = poseInnovation(state, steeringAngle, poseValue, cfg, observer)
% poseInnovation: Compute the timestamped low-rate pose residual against
% the transformed observer state using X, Y, and optional yaw channels.
%
% Input:
%   state: [6 x 1] transformed observer state
%   steeringAngle: scalar front steering angle in radians
%   poseValue: [3 x 1] propagated X, Y, yaw measurement vector
%   cfg: observer configuration struct
%   observer: realized observer matrix struct
%
% Output:
%   innovation: residual vector matching observer.Ld columns
    outputs = observer.poseCorrectionOutputs;
    innovation = zeros(numel(outputs), 1);
    for outputIdx = 1:numel(outputs)
        outputName = outputs(outputIdx);
        if outputName == "x" && isfinite(poseValue(1))
            innovation(outputIdx) = poseValue(1) - state(1);
        elseif outputName == "y" && isfinite(poseValue(2))
            innovation(outputIdx) = poseValue(2) - state(4);
        elseif outputName == "yaw" && isfinite(poseValue(3))
            innovation(outputIdx) = wrapAngleToPi(poseValue(3) - yawFromState(state, steeringAngle, cfg));
        end
    end
end

function measurement = highRateMeasurementAt(highRate, sampleIdx, alpha, cfg)
% highRateMeasurementAt: Evaluate the stored high-rate measurement
% stream at one replay substep using zero-order hold or linear interpolation.
%
% Input:
%   highRate: normalized high-rate measurement struct
%   sampleIdx: interval start index
%   alpha: interpolation fraction in [0, 1]
%   cfg: observer configuration struct
%
% Output:
%   measurement: scalar high-rate measurement struct
    measurement = struct();
    measurement.speed = interpolateRows(highRate.speed, sampleIdx, alpha, cfg.replay.inputInterpolation);
    measurement.yawRate = interpolateRows(highRate.yawRate, sampleIdx, alpha, cfg.replay.inputInterpolation);
    measurement.steeringAngle = interpolateRows(highRate.steeringAngle, sampleIdx, alpha, cfg.replay.inputInterpolation);
    measurement.orthogonalityConstraint = interpolateRows(highRate.orthogonalityConstraint, sampleIdx, alpha, cfg.replay.inputInterpolation);
end

function derivative = triangularVehicleDerivative(state, cfg)
% triangularVehicleDerivative: Evaluate the six-state triangular model
% used by the Bessafa observer. The yaw-rate term is reconstructed from the
% transformed velocity and acceleration states with denominator
% regularization for numerical safety.
%
% Input:
%   state: [6 x 1] transformed observer state
%   cfg: observer configuration struct
%
% Output:
%   derivative: [6 x 1] model derivative
    yawRate = yawRateFromState(state, cfg);
    yawRateSquared = yawRate .* yawRate;
    derivative = [state(2); state(3); -yawRateSquared .* state(2); ...
        state(5); state(6); -yawRateSquared .* state(5)];
end

function extraPrediction = extraOutputPrediction(state, cfg)
% extraOutputPrediction: Evaluate the configured nonlinear high-rate
% output map h(z), including speed, orthogonality constraint, and yaw rate.
%
% Input:
%   state: [6 x 1] transformed observer state
%   cfg: observer configuration struct
%
% Output:
%   extraPrediction: [m x 1] predicted high-rate output vector
    extraOutputs = string(cfg.observer.extraOutputs(:));
    extraPrediction = zeros(numel(extraOutputs), 1);
    for outputIdx = 1:numel(extraOutputs)
        outputName = lower(strtrim(extraOutputs(outputIdx)));
        if outputName == "speed"
            extraPrediction(outputIdx) = hypot(state(2), state(5));
        elseif outputName == "orthogonalityconstraint"
            extraPrediction(outputIdx) = state(2) .* state(3) + state(5) .* state(6);
        elseif outputName == "yawrate"
            extraPrediction(outputIdx) = yawRateFromState(state, cfg);
        else
            error("Unsupported Bessafa 2026 extra output: %s.", outputName);
        end
    end
end

function [extraMeasurement, extraValid] = extraMeasurementVector(measurement, cfg)
% extraMeasurementVector: Convert one scalar high-rate measurement
% struct into the ordered extra-output vector selected by cfg.observer.
%
% Input:
%   measurement: scalar high-rate measurement struct
%   cfg: observer configuration struct
%
% Output:
%   extraMeasurement: [m x 1] measured high-rate output vector
%   extraValid: [m x 1] finite-measurement validity mask
    extraOutputs = string(cfg.observer.extraOutputs(:));
    extraMeasurement = NaN(numel(extraOutputs), 1);
    extraValid = false(numel(extraOutputs), 1);
    for outputIdx = 1:numel(extraOutputs)
        outputName = lower(strtrim(extraOutputs(outputIdx)));
        if outputName == "speed"
            extraMeasurement(outputIdx) = measurement.speed;
        elseif outputName == "orthogonalityconstraint"
            extraMeasurement(outputIdx) = measurement.orthogonalityConstraint;
        elseif outputName == "yawrate"
            extraMeasurement(outputIdx) = measurement.yawRate;
        else
            error("Unsupported Bessafa 2026 extra output: %s.", outputName);
        end
        extraValid(outputIdx) = isfinite(extraMeasurement(outputIdx));
    end
end

function yawRate = yawRateFromState(state, cfg)
% yawRateFromState: Reconstruct yaw rate from transformed velocity and
% acceleration states using the Bessafa relation with a small denominator
% floor from the replay configuration.
%
% Input:
%   state: [6 x 1] transformed observer state
%   cfg: observer configuration struct
%
% Output:
%   yawRate: scalar yaw-rate estimate in radians per second
    speedSquared = state(2) .* state(2) + state(5) .* state(5);
    speedSquared = max(speedSquared, cfg.replay.speedRegularization);
    yawRate = (-state(5) .* state(3) + state(2) .* state(6)) ./ speedSquared;
end

function yaw = yawFromState(state, steeringAngle, cfg)
% yawFromState: Recover vehicle yaw from transformed velocity states
% by subtracting the steering-dependent side-slip angle from the velocity
% heading.
%
% Input:
%   state: [6 x 1] transformed observer state
%   steeringAngle: scalar front steering angle in radians
%   cfg: observer configuration struct
%
% Output:
%   yaw: scalar yaw angle in radians
    beta = sideSlipAngle(steeringAngle, cfg);
    yaw = wrapAngleToPi(atan2(state(5), state(2)) - beta);
end

function beta = sideSlipAngle(steeringAngle, cfg)
% sideSlipAngle: Compute the kinematic bicycle side-slip angle used by
% the Bessafa coordinate transformation.
%
% Input:
%   steeringAngle: numeric front steering angle in radians
%   cfg: observer configuration struct
%
% Output:
%   beta: numeric side-slip angle in radians
    beta = atan((cfg.vehicle.lr ./ cfg.vehicle.wheelbase) .* tan(steeringAngle));
end

function estimate = buildEstimate(highRate, pose, stateHistory, onlineState, informationAge, replayCount, acceptedCount, acceptedCorrections, preCorrectionState, stateBuffer, replayStartIdx, bufferStartIdx, observer, cfg)
% buildEstimate: Assemble the 100 Hz replay-observer state history,
% derived vehicle quantities, input measurements, pose measurements, and
% replay diagnostics into the public output struct.
%
% Input:
%   highRate: normalized high-rate measurement struct
%   pose: normalized pose measurement struct
%   stateHistory: [N x 6] replay-consistent transformed-state estimate
%   onlineState: [N x 6] causal wall-clock transformed-state estimate
%   informationAge: [N x 1] age of the newest accepted pose timestamp
%   replayCount: [N x 1] number of replays triggered at each high-rate time
%   acceptedCount: [N x 1] cumulative accepted correction count
%   acceptedCorrections: accepted timestamped pose-correction struct
%   preCorrectionState: active replay-window propagated pre-jump state buffer
%   stateBuffer: active replay-window posterior state buffer
%   replayStartIdx: [N x 1] earliest replayed index for each triggered replay
%   bufferStartIdx: [N x 1] earliest index retained by bounded replay window
%   observer: realized observer matrix struct
%   cfg: observer configuration struct
%
% Output:
%   estimate: public replay-observer estimate struct
    beta = sideSlipAngle(highRate.steeringAngle, cfg);
    yaw = wrapAngleToPi(atan2(stateHistory(:, 5), stateHistory(:, 2)) - beta);
    speedSquared = stateHistory(:, 2) .* stateHistory(:, 2) + stateHistory(:, 5) .* stateHistory(:, 5);
    speedSquared = max(speedSquared, cfg.replay.speedRegularization);
    yawRate = (-stateHistory(:, 5) .* stateHistory(:, 3) + stateHistory(:, 2) .* stateHistory(:, 6)) ./ speedSquared;
    kinematicPosition = kinematicPositionOutput(highRate.time, stateHistory(:, [1, 4]), stateHistory(:, [2, 5]), cfg);
    filteredAcceleration = lowPassRows(highRate.time, stateHistory(:, [3, 6]), ...
        cfg.replay.outputAccelerationFilterTimeConstant, cfg.replay.sampleTime);

    estimate = struct();
    estimate.time = highRate.time;
    estimate.z = stateHistory;
    estimate.onlineZ = onlineState;
    estimate.position = stateHistory(:, [1, 4]);
    estimate.velocity = stateHistory(:, [2, 5]);
    estimate.acceleration = stateHistory(:, [3, 6]);
    estimate.onlinePosition = onlineState(:, [1, 4]);
    estimate.kinematicPosition = kinematicPosition;
    estimate.filteredAcceleration = filteredAcceleration;
    estimate.displayPosition = kinematicPosition;
    estimate.displayAcceleration = filteredAcceleration;
    estimate.speed = hypot(stateHistory(:, 2), stateHistory(:, 5));
    estimate.yawRate = yawRate;
    estimate.yaw = yaw;
    estimate.beta = beta;

    estimate.measurements = struct();
    estimate.measurements.highRate = highRate;
    estimate.measurements.pose = pose;

    estimate.diagnostics = struct();
    estimate.diagnostics.informationAge = informationAge;
    estimate.diagnostics.replayCount = replayCount;
    estimate.diagnostics.acceptedCount = acceptedCount;
    estimate.diagnostics.acceptedCorrections = acceptedCorrections;
    estimate.diagnostics.preCorrectionState = preCorrectionState.values;
    estimate.diagnostics.preCorrectionStateStartIdx = preCorrectionState.startIdx;
    estimate.diagnostics.replayWindowState = stateBuffer.values;
    estimate.diagnostics.replayWindowStartIdx = stateBuffer.startIdx;
    estimate.diagnostics.replayStartIdx = replayStartIdx;
    estimate.diagnostics.bufferStartIdx = bufferStartIdx;
    estimate.diagnostics.acceptedCorrectionScope = "retainedReplayWindow";
    estimate.diagnostics.poseCorrectionMode = "timestampedDiscreteReplay";
    estimate.diagnostics.maxCertifiedInformationAge = cfg.replay.maxInformationAge;
    estimate.diagnostics.informationAgeCertified = informationAge <= cfg.replay.maxInformationAge | ~isfinite(informationAge);

    estimate.observer = struct();
    estimate.observer.gainSet = cfg.observer.gainSet;
    estimate.observer.theta = cfg.observer.theta;
    estimate.observer.N = cfg.observer.N;
    estimate.observer.M = observer.M;
    estimate.observer.Kd = observer.Kd;
    estimate.observer.Ld = observer.Ld;
    estimate.observer.gainSource = observer.gainSource;
    estimate.observer.poseCorrectionOutputs = observer.poseCorrectionOutputs;
end

function kinematicPosition = kinematicPositionOutput(time, rawPosition, velocity, cfg)
% kinematicPositionOutput: Build a continuous position trajectory for
% display and engineering consumption by integrating the 100 Hz velocity
% estimate and applying a causal low-bandwidth pull toward the raw hybrid
% replay position state.
%
% Input:
%   time: [N x 1] high-rate timestamp vector
%   rawPosition: [N x 2] raw replay-observer position state
%   velocity: [N x 2] replay-observer velocity state
%   cfg: observer configuration struct
%
% Output:
%   kinematicPosition: [N x 2] continuous kinematic position estimate
    kinematicPosition = rawPosition;
    blendTimeConstant = max(double(cfg.replay.outputPositionBlendTimeConstant), 0.0);
    for sampleIdx = 2:numel(time)
        sampleTime = time(sampleIdx) - time(sampleIdx - 1);
        if ~isfinite(sampleTime) || sampleTime <= 0.0
            sampleTime = cfg.replay.sampleTime;
        end
        predictedPosition = kinematicPosition(sampleIdx - 1, :) + ...
            0.5 .* sampleTime .* (velocity(sampleIdx - 1, :) + velocity(sampleIdx, :));
        if blendTimeConstant <= 0.0
            blend = 1.0;
        else
            blend = 1.0 - exp(-sampleTime ./ blendTimeConstant);
        end
        kinematicPosition(sampleIdx, :) = predictedPosition + blend .* (rawPosition(sampleIdx, :) - predictedPosition);
    end
end

function filteredValue = lowPassRows(time, rawValue, filterTimeConstant, defaultSampleTime)
% lowPassRows: Apply a causal first-order low-pass filter to each
% column of a high-rate signal using the supplied time constant.
%
% Input:
%   time: [N x 1] high-rate timestamp vector
%   rawValue: [N x M] signal to filter
%   filterTimeConstant: scalar filter time constant in seconds
%   defaultSampleTime: scalar fallback sample time in seconds
%
% Output:
%   filteredValue: [N x M] filtered signal
    filteredValue = rawValue;
    filterTimeConstant = max(double(filterTimeConstant), 0.0);
    for sampleIdx = 2:numel(time)
        sampleTime = time(sampleIdx) - time(sampleIdx - 1);
        if ~isfinite(sampleTime) || sampleTime <= 0.0
            sampleTime = defaultSampleTime;
        end
        if filterTimeConstant <= 0.0
            blend = 1.0;
        else
            blend = 1.0 - exp(-sampleTime ./ filterTimeConstant);
        end
        filteredValue(sampleIdx, :) = filteredValue(sampleIdx - 1, :) + ...
            blend .* (rawValue(sampleIdx, :) - filteredValue(sampleIdx - 1, :));
    end
end

function [preCorrectionState, stateBuffer] = replayStateHistory(preCorrectionState, stateBuffer, highRate, acceptedCorrections, startIdx, currentIdx, cfg, observer)
% replayStateHistory: Recompute the buffered observer history from the
% earliest newly affected timestamp through the current sample. Each sample
% first receives the high-rate flow result from the previous posterior state,
% then all accepted pose corrections scheduled at that physical timestamp are
% applied as deterministic discrete jumps.
%
% Input:
%   preCorrectionState: active replay-window pre-jump state buffer
%   stateBuffer: active replay-window posterior state buffer
%   highRate: normalized high-rate measurement struct
%   acceptedCorrections: timestamped correction list
%   startIdx: first high-rate sample whose posterior may change
%   currentIdx: current online high-rate sample
%   cfg: observer configuration struct
%   observer: realized observer matrix struct
%
% Output:
%   preCorrectionState: updated active pre-jump observer state buffer
%   stateBuffer: updated active posterior observer state buffer
    startIdx = min(max(startIdx, 1), currentIdx);
    if startIdx > 1
        preCorrectionRow = propagateState( ...
            replayWindowRow(stateBuffer, startIdx - 1).', highRate, startIdx - 1, cfg, observer).';
        preCorrectionState = setReplayWindowRow(preCorrectionState, startIdx, preCorrectionRow);
    end
    for replayIdx = startIdx:currentIdx
        if replayIdx > startIdx
            preCorrectionRow = propagateState( ...
                replayWindowRow(stateBuffer, replayIdx - 1).', highRate, replayIdx - 1, cfg, observer).';
            preCorrectionState = setReplayWindowRow(preCorrectionState, replayIdx, preCorrectionRow);
        end
        stateRow = applyScheduledPoseCorrections( ...
            replayWindowRow(preCorrectionState, replayIdx).', highRate, replayIdx, acceptedCorrections, cfg, observer).';
        stateBuffer = setReplayWindowRow(stateBuffer, replayIdx, stateRow);
    end
end

function state = applyScheduledPoseCorrections(state, highRate, sampleIdx, acceptedCorrections, cfg, observer)
% applyScheduledPoseCorrections: Apply all accepted low-rate pose
% corrections scheduled at one high-rate physical timestamp as discrete jump
% updates in arrival-sequence order.
%
% Input:
%   state: [6 x 1] pre-jump observer state at sampleIdx
%   highRate: normalized high-rate measurement struct
%   sampleIdx: high-rate sample index receiving corrections
%   acceptedCorrections: timestamped correction list
%   cfg: observer configuration struct
%   observer: realized observer matrix struct
%
% Output:
%   state: [6 x 1] posterior observer state after all timestamped jumps
    correctionIdx = find(acceptedCorrections.timestampIdx == sampleIdx);
    for correctionOffset = 1:numel(correctionIdx)
        poseValue = acceptedCorrections.pose(correctionIdx(correctionOffset), :).';
        innovation = poseInnovation(state, highRate.steeringAngle(sampleIdx), poseValue, cfg, observer);
        state = state + observer.Ld * innovation;
    end
end

function acceptedCorrections = emptyAcceptedCorrections()
% emptyAcceptedCorrections: Create an empty accepted-correction struct
% with array fields used by the replay buffer.
%
% Input:
%   none
%
% Output:
%   acceptedCorrections: empty accepted-correction struct
    acceptedCorrections = struct();
    acceptedCorrections.timestampIdx = zeros(0, 1);
    acceptedCorrections.timestamp = zeros(0, 1);
    acceptedCorrections.arrivalTime = zeros(0, 1);
    acceptedCorrections.pose = zeros(0, 3);
    acceptedCorrections.sequence = zeros(0, 1);
end

function acceptedCorrections = appendAcceptedCorrection(acceptedCorrections, timestampIdx, timestamp, arrivalTime, poseValue, sequenceNumber)
% appendAcceptedCorrection: Add one timestamped pose correction and
% keep all accepted corrections sorted by physical timestamp and arrival
% sequence for deterministic chronological replay.
%
% Input:
%   acceptedCorrections: existing accepted-correction struct
%   timestampIdx: high-rate sample index for the physical timestamp
%   timestamp: physical timestamp in seconds
%   arrivalTime: arrival time in seconds
%   poseValue: [1 x 3] measured X, Y, yaw row
%   sequenceNumber: accepted arrival sequence number
%
% Output:
%   acceptedCorrections: sorted accepted-correction struct
    acceptedCorrections.timestampIdx(end + 1, 1) = timestampIdx;
    acceptedCorrections.timestamp(end + 1, 1) = timestamp;
    acceptedCorrections.arrivalTime(end + 1, 1) = arrivalTime;
    acceptedCorrections.pose(end + 1, :) = poseValue;
    acceptedCorrections.sequence(end + 1, 1) = sequenceNumber;
    [~, order] = sortrows([acceptedCorrections.timestampIdx, acceptedCorrections.sequence]);
    acceptedCorrections.timestampIdx = acceptedCorrections.timestampIdx(order);
    acceptedCorrections.timestamp = acceptedCorrections.timestamp(order);
    acceptedCorrections.arrivalTime = acceptedCorrections.arrivalTime(order);
    acceptedCorrections.pose = acceptedCorrections.pose(order, :);
    acceptedCorrections.sequence = acceptedCorrections.sequence(order);
end

function acceptedCorrections = trimAcceptedCorrections(acceptedCorrections, bufferStartIdx)
% trimAcceptedCorrections: Retain only accepted corrections that remain
% inside the active replay window. Corrections older than the bounded window
% cannot be replayed again because any future measurement requiring them is
% rejected by the timestamp-window checks.
%
% Input:
%   acceptedCorrections: sorted accepted-correction struct
%   bufferStartIdx: earliest high-rate sample index retained by the window
%
% Output:
%   acceptedCorrections: accepted-correction struct cropped to the window
    keepMask = acceptedCorrections.timestampIdx >= bufferStartIdx;
    acceptedCorrections.timestampIdx = acceptedCorrections.timestampIdx(keepMask);
    acceptedCorrections.timestamp = acceptedCorrections.timestamp(keepMask);
    acceptedCorrections.arrivalTime = acceptedCorrections.arrivalTime(keepMask);
    acceptedCorrections.pose = acceptedCorrections.pose(keepMask, :);
    acceptedCorrections.sequence = acceptedCorrections.sequence(keepMask);
end

function informationAge = computeInformationAge(currentTime, latestAcceptedTimestamp)
% computeInformationAge: Compute the age of the newest physical timestamp that
% has been accepted and replayed into the current online estimate.
%
% Input:
%   currentTime: scalar current observer time
%   latestAcceptedTimestamp: newest accepted physical timestamp in seconds
%
% Output:
%   informationAge: scalar age in seconds or NaN before any correction
    if ~isfinite(latestAcceptedTimestamp)
        informationAge = NaN;
    else
        informationAge = currentTime - latestAcceptedTimestamp;
    end
end

function window = createReplayWindow(initialState)
% createReplayWindow: Create a bounded replay-buffer struct with global
% row indexing so active window rows can be cropped without changing the
% physical sample indices used by timestamped corrections.
%
% Input:
%   initialState: [6 x 1] initial posterior observer state
%
% Output:
%   window: replay-window struct with startIdx and values fields
    window = struct();
    window.startIdx = 1;
    window.values = initialState(:).';
end

function retainedStartIdx = retainedReplayStartIdx(bufferStartIdx)
% retainedReplayStartIdx: Return the state-buffer start index needed to
% support replay at bufferStartIdx, including the one-sample anchor before
% the earliest accepted correction timestamp when it exists.
%
% Input:
%   bufferStartIdx: earliest correction timestamp index retained by replay
%
% Output:
%   retainedStartIdx: earliest state row retained in the active buffer
    retainedStartIdx = max(1, bufferStartIdx - 1);
end

function window = trimReplayWindow(window, retainedStartIdx)
% trimReplayWindow: Drop replay-buffer rows older than the retained
% start index while preserving global sample indexing for the remaining
% active rows.
%
% Input:
%   window: replay-window struct with startIdx and values fields
%   retainedStartIdx: earliest global sample index to keep
%
% Output:
%   window: cropped replay-window struct
    retainedStartIdx = max(1, retainedStartIdx);
    dropCount = min(max(retainedStartIdx - window.startIdx, 0), size(window.values, 1));
    if dropCount > 0
        window.values = window.values((dropCount + 1):end, :);
        window.startIdx = window.startIdx + dropCount;
    end
end

function row = replayWindowRow(window, sampleIdx)
% replayWindowRow: Read one globally indexed replay-window row and fail
% if the requested sample has fallen outside the retained active window.
%
% Input:
%   window: replay-window struct with startIdx and values fields
%   sampleIdx: global high-rate sample index
%
% Output:
%   row: [1 x 6] replay-window state row
    localIdx = sampleIdx - window.startIdx + 1;
    assert(localIdx >= 1 && localIdx <= size(window.values, 1), ...
        "Replay window does not retain sample index %d.", sampleIdx);
    row = window.values(localIdx, :);
end

function rows = replayWindowRows(window, firstIdx, lastIdx)
% replayWindowRows: Read a contiguous global-index range from the
% retained replay-window state rows.
%
% Input:
%   window: replay-window struct with startIdx and values fields
%   firstIdx: first global high-rate sample index
%   lastIdx: last global high-rate sample index
%
% Output:
%   rows: [(lastIdx-firstIdx+1) x 6] replay-window state rows
    firstLocalIdx = firstIdx - window.startIdx + 1;
    lastLocalIdx = lastIdx - window.startIdx + 1;
    assert(firstLocalIdx >= 1 && lastLocalIdx <= size(window.values, 1), ...
        "Replay window does not retain sample range %d:%d.", firstIdx, lastIdx);
    rows = window.values(firstLocalIdx:lastLocalIdx, :);
end

function window = setReplayWindowRow(window, sampleIdx, row)
% setReplayWindowRow: Write one globally indexed replay-window row,
% extending the active buffer forward with NaN rows when the sequential
% online loop appends a new sample.
%
% Input:
%   window: replay-window struct with startIdx and values fields
%   sampleIdx: global high-rate sample index
%   row: [1 x 6] state row to store
%
% Output:
%   window: replay-window struct with the selected row updated
    localIdx = sampleIdx - window.startIdx + 1;
    assert(localIdx >= 1, "Replay window cannot write sample index %d before retained start %d.", ...
        sampleIdx, window.startIdx);
    if localIdx > size(window.values, 1)
        window.values((size(window.values, 1) + 1):localIdx, :) = NaN;
    end
    window.values(localIdx, :) = row;
end

function bufferStartIdx = replayBufferStartIdx(time, currentTime, bufferDuration, tolerance)
% replayBufferStartIdx: Return the earliest high-rate sample retained
% by the bounded replay window at the current online time. Accepted delayed
% corrections must target an index in this bounded window.
%
% Input:
%   time: [N x 1] high-rate timestamp vector
%   currentTime: scalar current online time
%   bufferDuration: scalar replay-buffer duration in seconds
%   tolerance: scalar timestamp tolerance in seconds
%
% Output:
%   bufferStartIdx: earliest retained high-rate sample index
    bufferStartIdx = find(time >= currentTime - bufferDuration - tolerance, 1);
    if isempty(bufferStartIdx)
        bufferStartIdx = 1;
    end
end

function timestampIdx = timestampIndex(time, timestamp, tolerance)
% timestampIndex: Map a physical timestamp to the nearest high-rate
% buffer sample and verify that the mismatch is within tolerance.
%
% Input:
%   time: [N x 1] high-rate time vector
%   timestamp: scalar physical measurement timestamp
%   tolerance: scalar allowed mismatch in seconds
%
% Output:
%   timestampIdx: nearest high-rate sample index
    [timeError, timestampIdx] = min(abs(time - timestamp));
    assert(timeError <= tolerance, ...
        "Pose timestamp %.6f is not aligned to the high-rate replay grid within %.6f seconds.", ...
        timestamp, tolerance);
end

function value = interpolateRows(values, sampleIdx, alpha, interpolationMode)
% interpolateRows: Read or interpolate one scalar high-rate series at a
% replay substep using zero-order hold or linear interpolation.
%
% Input:
%   values: [N x 1] numeric series
%   sampleIdx: interval start index
%   alpha: interpolation fraction in [0, 1]
%   interpolationMode: string scalar interpolation mode
%
% Output:
%   value: scalar interpolated value
    currentValue = values(sampleIdx, :);
    nextIdx = min(sampleIdx + 1, size(values, 1));
    nextValue = values(nextIdx, :);
    if lower(strtrim(string(interpolationMode))) == "linear" && isfinite(currentValue) && isfinite(nextValue)
        value = currentValue + alpha .* (nextValue - currentValue);
    elseif alpha >= 1.0
        value = nextValue;
    else
        value = currentValue;
    end
end

function numSamples = inferSeriesLength(source, names)
% inferSeriesLength: Infer the number of samples from the first
% available scalar or matrix field in a struct or table.
%
% Input:
%   source: struct or table
%   names: string vector of candidate field names
%
% Output:
%   numSamples: positive integer sample count
    rawValue = readFirstSeriesField(source, names);
    assert(~isempty(rawValue), "Input measurement data does not contain any expected sample series.");
    rawValue = double(rawValue);
    if isvector(rawValue)
        numSamples = numel(rawValue);
    else
        numSamples = size(rawValue, 1);
        if numSamples == 1 && size(rawValue, 2) > 1
            numSamples = size(rawValue, 2);
        end
    end
    assert(numSamples >= 1, "Input measurement data must contain at least one sample.");
end

function values = readScalarSeries(source, names, numSamples)
% readScalarSeries: Read a scalar numeric series from a struct field or
% table variable, returning NaNs when none of the candidate names exists.
%
% Input:
%   source: struct or table
%   names: string vector of candidate field names
%   numSamples: expected sample count
%
% Output:
%   values: [N x 1] numeric series
    rawValue = readFirstSeriesField(source, names);
    if isempty(rawValue)
        values = NaN(numSamples, 1);
        return;
    end
    rawValue = double(rawValue);
    if isvector(rawValue)
        values = rawValue(:);
    else
        values = rawValue(:, 1);
    end
    values = alignVectorRows(values, numSamples, char(string(names(1))));
end

function matrix = readMatrixSeries(source, names, numSamples, width)
% readMatrixSeries: Read a matrix-valued measurement series from a
% struct field or table variable, returning empty when no candidate exists.
%
% Input:
%   source: struct or table
%   names: string vector of candidate field names
%   numSamples: expected sample count
%   width: required number of columns
%
% Output:
%   matrix: [N x width] numeric matrix or empty array
    rawValue = readFirstSeriesField(source, names);
    if isempty(rawValue)
        matrix = [];
        return;
    end
    matrix = alignMatrixRows(rawValue, numSamples, width, char(string(names(1))));
end

function rawValue = readFirstSeriesField(source, names)
% readFirstSeriesField: Return the first matching struct field or table
% variable from a list of candidate names.
%
% Input:
%   source: struct or table
%   names: string vector of candidate field names
%
% Output:
%   rawValue: field value or empty array when absent
    rawValue = [];
    names = string(names(:));
    if istable(source)
        variableNames = string(source.Properties.VariableNames);
        for nameIdx = 1:numel(names)
            matchIdx = find(variableNames == names(nameIdx), 1);
            if ~isempty(matchIdx)
                rawValue = source.(char(variableNames(matchIdx)));
                return;
            end
        end
    else
        for nameIdx = 1:numel(names)
            fieldName = char(names(nameIdx));
            if isfield(source, fieldName)
                rawValue = source.(fieldName);
                return;
            end
        end
    end
end

function matrix = alignMatrixRows(rawValue, numSamples, width, label)
% alignMatrixRows: Validate and align a numeric vector or matrix into
% an [N x width] measurement array.
%
% Input:
%   rawValue: raw numeric vector or matrix
%   numSamples: expected sample count
%   width: minimum required column count
%   label: assertion label
%
% Output:
%   matrix: [N x width] numeric matrix
    rawValue = double(rawValue);
    if isvector(rawValue)
        rawValue = rawValue(:);
    end
    if size(rawValue, 1) == width && size(rawValue, 2) == numSamples
        rawValue = rawValue.';
    end
    assert(size(rawValue, 1) == numSamples && size(rawValue, 2) >= width, ...
        "%s must have %d rows and at least %d columns.", label, numSamples, width);
    matrix = rawValue(:, 1:width);
end

function values = alignVectorRows(values, numSamples, label)
% alignVectorRows: Validate that a scalar numeric series has the
% expected number of samples and return it as an [N x 1] vector.
%
% Input:
%   values: numeric vector
%   numSamples: expected sample count
%   label: assertion label
%
% Output:
%   values: [N x 1] numeric vector
    values = double(values(:));
    assert(numel(values) == numSamples, "%s must contain %d samples.", label, numSamples);
end

function value = firstFiniteScalar(values, defaultValue)
% firstFiniteScalar: Return the first finite scalar in a vector or the
% supplied default value when no finite value exists.
%
% Input:
%   values: numeric vector
%   defaultValue: scalar fallback
%
% Output:
%   value: selected scalar value
    finiteIdx = find(isfinite(values), 1);
    if isempty(finiteIdx)
        value = defaultValue;
    else
        value = values(finiteIdx);
    end
end

function value = readScalar(source, fieldName, defaultValue)
% readScalar: Read one scalar field from a struct, returning a default
% when the field is absent, empty, non-scalar, or nonfinite.
%
% Input:
%   source: struct to inspect
%   fieldName: string scalar field name
%   defaultValue: scalar fallback
%
% Output:
%   value: selected scalar value
    value = defaultValue;
    fieldName = char(string(fieldName));
    if isfield(source, fieldName) && ~isempty(source.(fieldName))
        candidate = double(source.(fieldName));
        if isscalar(candidate) && isfinite(candidate)
            value = candidate;
        end
    end
end

function value = readString(source, fieldName, defaultValue)
% readString: Read one string-valued field from a struct, returning a
% supplied default when the field is absent or empty.
%
% Input:
%   source: struct to inspect
%   fieldName: string scalar field name
%   defaultValue: string scalar fallback
%
% Output:
%   value: string scalar value
    value = string(defaultValue);
    fieldName = char(string(fieldName));
    if isfield(source, fieldName) && ~isempty(source.(fieldName))
        value = string(source.(fieldName));
    end
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
