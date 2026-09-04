function channels = evaluateImprovedObserverChannels(state, sample)
% evaluateImprovedObserverChannels Evaluate the seven-state model and h map.
% SAMPLE contains longitudinalSpeed, lateralVelocity, longitudinalAcceleration,
% lateralAcceleration, yawRate, sideSlipAngle, and sideSlipAngleRate.

    arguments
        state (7, 1) double {mustBeFinite}
        sample (1, 1) struct
    end

    requiredFields = ["longitudinalSpeed", "lateralVelocity", ...
        "longitudinalAcceleration", "lateralAcceleration", "yawRate", ...
        "sideSlipAngle", "sideSlipAngleRate"];
    for fieldName = requiredFields
        assert(isfield(sample, fieldName) && isscalar(sample.(fieldName)) && ...
            isfinite(sample.(fieldName)), "sample.%s must be a finite scalar.", fieldName);
    end

    trackAngleRate = sample.yawRate + sample.sideSlipAngleRate;
    trackAngleRateSquared = trackAngleRate.^2;
    modelDerivative = [state(2); state(3); ...
        trackAngleRateSquared .* state(2) - 2.0 .* trackAngleRate .* state(6); ...
        state(5); state(6); ...
        trackAngleRateSquared .* state(5) + 2.0 .* trackAngleRate .* state(3); ...
        sample.yawRate];

    velocitySquared = state(2).^2 + state(5).^2;
    velocityAccelerationDot = state(2) .* state(3) + state(5) .* state(6);
    velocityAccelerationCross = state(2) .* state(6) - state(5) .* state(3);
    coupledAngle = state(7) + sample.sideSlipAngle;
    coupling = state(5) .* cos(coupledAngle) - state(2) .* sin(coupledAngle);
    invariantPrediction = [velocitySquared; velocityAccelerationDot; ...
        velocityAccelerationCross; coupling];
    invariantMeasurement = [sample.longitudinalSpeed.^2 + sample.lateralVelocity.^2; ...
        sample.longitudinalSpeed .* sample.longitudinalAcceleration + ...
            sample.lateralVelocity .* sample.lateralAcceleration; ...
        sample.longitudinalSpeed .* sample.lateralAcceleration - ...
            sample.lateralVelocity .* sample.longitudinalAcceleration; ...
        0.0];

    channels = struct();
    channels.trackAngleRate = trackAngleRate;
    channels.modelDerivative = modelDerivative;
    channels.invariantPrediction = invariantPrediction;
    channels.invariantMeasurement = invariantMeasurement;
    channels.invariantInnovation = invariantMeasurement - invariantPrediction;
end
