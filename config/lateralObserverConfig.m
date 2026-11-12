function cfg = lateralObserverConfig(profile)
% lateralObserverConfig: Parameters of the LPV lateral-velocity observer.
% Two-state nominal bicycle dynamics with outputs y = [Vx*ay; r].
% P uses three speed vertices; Y=X*L uses a common recovery matrix X.
% Twelve expanded LMIs include Pdot and exact column-wise noise extensions.
% Sideslip angle and rate remain algebraic outputs of the two-state observer.
%
% Default vehicle parameters come from config/mncavVehicleParameters.json.
% MnCAV is the only supported vehicle profile.
%
% Input:
%   profile: optional "mncav" selector, retained for existing callers.
%       All parameters come from the adopted identification and public geometry.
%       Re-synthesize lateral gains after changing the vehicle configuration.
%
% Output:
%   cfg: struct with vehicle, outputPoint, scheduling, synthesis, observer, and
%       simulation groups
    arguments
        profile (1,1) string = "mncav"
    end
    assert(profile == "mncav", "VehicleLocalization:RemovedVehicleProfile", ...
        "Only the current MnCAV configuration is supported. Use lateralObserverConfig().");
    cfg = struct();

    % Single source for the adopted 2-DOF lateral bicycle model.
    parameters = mncavVehicleConfig();
    cfg.vehicle = parameters.vehicle;
    cfg.vehicleParameterProvenance = parameters;

    % The observer point is forwardOffsetM ahead of the exported output point:
    % vyOutput = vyObserver - forwardOffsetM*yawRate. The offset was calibrated
    % on the separate 12-11-24 drive by calibrateMncavMotionOutputPoint.
    cfg.outputPoint = jsondecode(fileread(fullfile(fileparts(mfilename("fullpath")), ...
        "mncavMotionOutputPoint.json")));

    % Speed-rate limits are certificate assumptions, not identified bounds.
    % This is d(Vx)/dt, not longitudinal accelerometer specific force ax.
    cfg.scheduling = struct('speedRange', [1.0, 30.0], ...
        'speedRateRange', [-4.0, 4.0]);

    % Scheduled P >= I for the nominal linear LPV model.
    % Minimize normalized disturbance gain in a single SDP.
    cfg.synthesis = struct();
    cfg.synthesis.decayRate = 2.0;
    % Reference units for [lateral-acceleration; yaw-acceleration] disturbances.
    % Design normalization only, not claimed measured uncertainty bounds.
    cfg.synthesis.processErrorScale = [1.0; 1.0];
    % E maps disturbance coordinates to [dot(vy); dot(r)] residuals.
    % Identity retains the existing acceleration-residual coordinates.
    cfg.synthesis.processDistribution = eye(2);
    % Fixed within each SDP; epsilon=0.02 s was evaluated in the column-gain study.
    cfg.synthesis.dilationTimeScale = 0.02;
    % Acceleration gain is fixed to [1;0]/speed for exact disturbance cancellation.
    % Only the affine yaw-gain coefficients are optimized.
    % A generous metric ceiling avoids poorly scaled SDP constraints. The
    % ceiling is a numerical safeguard, not an optimization objective.
    cfg.synthesis.pCeiling = 1.0e3;
    % Minimizing the ISS gain pushes the solution onto the constraint
    % boundary; strictnessEpsilon specifies the requested LMI margin.
    cfg.synthesis.strictnessEpsilon = 1.0e-6;
    cfg.synthesis.solver = "sdpt3";
    cfg.synthesis.verbose = 0;
    cfg.synthesis.dualize = 0;

    cfg.observer = struct();
    % Freeze LPV coefficients at each interval midpoint and exponentiate
    % the resulting affine flow. This is a numerical implementation,
    % not an extension of the continuous ISS certificate to sampled data.
    cfg.observer.integrationMethod = "exponentialMidpoint";
    cfg.observer.initialState = [0.0; 0.0];
    % Only the LPV [vy; r] state estimates velocity. Out-of-domain samples
    % hold that state and withdraw its exported velocity aiding.

    % Synthetic scenario: sinusoidal longitudinal acceleration so the
    % scheduling parameter and its rate are both exercised, with a
    % raised-cosine steering profile.
    cfg.simulation = struct();
    cfg.simulation.sampleTime = 0.01;
    cfg.simulation.tFinal = 24.0;
    cfg.simulation.initialSpeed = 15.0;
    cfg.simulation.accelerationAmplitude = 2.0;
    cfg.simulation.accelerationPeriod = 12.0;
    cfg.simulation.steeringWaypointTime = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0];
    % Kept small enough that the side-slip angle stays inside the linear
    % tire regime the 2-DOF model assumes
    cfg.simulation.steeringWaypointDeg = [0.0, 2.0, -2.0, 1.8, -1.8, 2.0, -1.4, 1.6, 0.0];
    cfg.simulation.initialStateError = [0.5; 0.05];
    % Explicit sensitivity override only; no stiffness-mismatch certificate.
    cfg.simulation.plantVehicle = [];
    cfg.simulation.randomSeed = 2026;
    sensors = mncavSensorConfig();
    cfg.sensorParameterProvenance = sensors;
    % Effective white-noise priors on the reconstructed simulation grid;
    % neither OEM IMU specifications nor a measured native covariance.
    cfg.simulation.yawRateNoiseStd = sensors.lateralSimulation.yawRateNoiseStdRadps;
    cfg.simulation.lateralAccelerationNoiseStd = sensors.lateralSimulation.lateralAccelerationNoiseStdMps2;

    % Normalize physical [ay; r] errors, scaling ay by speed in the model.
    % These are engineering references, not hard Gaussian-noise bounds.
    cfg.synthesis.measurementErrorScale = ...
        [max(abs(double(sensors.lateralSimulation.lateralAccelerationBiasStressMps2(:)))) + ...
        cfg.simulation.lateralAccelerationNoiseStd; ...
        max(abs(double(sensors.lateralSimulation.yawRateBiasStressRadps(:)))) + ...
        cfg.simulation.yawRateNoiseStd];
end
