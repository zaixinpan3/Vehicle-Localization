function channels = evaluateImprovedObserverChannels(state, sample, operating)
% evaluateImprovedObserverChannels Evaluate the seven-state model and h map.
% SAMPLE contains longitudinalSpeed, lateralVelocity, longitudinalAcceleration,
% lateralAcceleration, yawRate, sideSlipAngle, and sideSlipAngleRate.
% Body acceleration must be compensated inertial acceleration at the vehicle
% reference point. Prediction uses q^2*v+2*q*J*a with q=r_m+betaDot_m;
% omitted speed jerk, course angular acceleration and q error are additive
% model disturbances, not zero-motion assumptions (see the assimilation note).

    arguments
        state (7, 1) double {mustBeFinite}
        sample (1, 1) struct
        operating struct = struct()
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
    chain=[0,1,0;0,0,1;0,0,0];
    modelMatrix=blkdiag(chain,chain,0);
    modelMatrix(3,2)=trackAngleRateSquared;modelMatrix(6,5)=trackAngleRateSquared;
    modelMatrix(3,6)=-2*trackAngleRate;modelMatrix(6,3)=2*trackAngleRate;
    modelInput=zeros(7,1);modelInput(7)=sample.yawRate;
    modelDerivative=modelMatrix*state+modelInput;

    % Extend only h outside the physical operating box. Clipping its
    % arguments gives a globally bounded mean-value Jacobian already covered
    % by the certificate's interval vertices. The estimated state and linear
    % prediction are not clipped or reset.
    invariantState = state;
    if ~isempty(fieldnames(operating))
        invariantState([2,5]) = min(max(state([2,5]),-operating.maximumSpeed),operating.maximumSpeed);
        invariantState([3,6]) = min(max(state([3,6]),-operating.maximumAcceleration),operating.maximumAcceleration);
    end
    extensionActive = any(invariantState~=state);
    state = invariantState;
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
    channels.modelMatrix = modelMatrix;
    channels.modelInput = modelInput;
    channels.invariantPrediction = invariantPrediction;
    channels.invariantExtensionActive = extensionActive;
    channels.invariantMeasurement = invariantMeasurement;
    channels.invariantInnovation = invariantMeasurement - invariantPrediction;
    % Local sensitivity of h4 to yaw, not an absolute-heading observation.
    % It vanishes at zero velocity even if geometric heading is available.
    channels.motionHeadingSensitivity = -state(2)*cos(coupledAngle) ...
        -state(5)*sin(coupledAngle);
end
