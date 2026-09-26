function cfg = lateralObserverConfig(profile)
% lateralObserverConfig: Parameters of the LPV lateral-velocity observer.
% The vehicle group defines the 2-DOF lateral bicycle model; the scheduling
% group defines the speed range covered by the polytope and the speed grid
% on which the synthesis LMIs are imposed; the synthesis group holds the
% Lipschitz bound, the error weighting, and the LMI solver settings; the
% observer group configures the online integration; the hybrid group defines
% the division-free master estimator and its bumpless correction channels;
% the simulation group defines the synthetic scenario used to exercise the
% observer.
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
%   cfg: struct with vehicle, scheduling, synthesis, observer, and
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

    % Scheduling parameter rho = [Vx; 1/Vx] and the speed grid over it. The
    % gain is affine in the barycentric coordinates of rho on the triangle
    % over speedRange, so the synthesis returns three vertex gains and the
    % grid only fixes where the LMIs are imposed. The Lyapunov matrix is
    % constant, so the certificate holds for any longitudinal acceleration.
    cfg.scheduling = struct();
    cfg.scheduling.speedRange = [5.0, 30.0];
    cfg.scheduling.speedGridCount = 9;

    % H2 synthesis: Lipschitz bound of the unmodeled nonlinearity, the
    % error weighting Q of the performance output z = Q^(1/2) e, and the
    % fixed scalar tau of Young's inequality (searched over candidates).
    cfg.synthesis = struct();
    cfg.synthesis.lipschitzConstant = 0.5;
    cfg.synthesis.errorWeight = eye(2);
    cfg.synthesis.tauCandidates = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0];
    cfg.synthesis.pFloor = 1.0e-4;
    cfg.synthesis.pCeiling = 1.0e6;
    % Minimizing mu pushes the solution onto the constraint boundary, so
    % strictnessEpsilon is what buys a real margin in the certificate that
    % designLateralObserverGains re-checks afterwards.
    cfg.synthesis.strictnessEpsilon = 1.0e-6;
    cfg.synthesis.solver = "sedumi";
    cfg.synthesis.verbose = 0;
    cfg.synthesis.dualize = 0;

    % Online observer. The nonlinearity f(x) is the term the observer copies
    % from the plant; the default is zero, so lipschitzConstant acts purely
    % as a robustness margin against unmodeled lateral dynamics.
    cfg.observer = struct();
    cfg.observer.integrationMethod = "rk4";
    cfg.observer.nonlinearity = @(x) zeros(2, 1);
    cfg.observer.initialState = [0.0; 0.0];
    % Hybrid runtime. The common master state is [lateral velocity;
    % lateral-accelerometer bias]. The LPV state above remains a hidden,
    % continuously evolving information source and is evaluated only inside
    % its certified speed interval. Mode logic changes correction targets,
    % never the common state or an exported value.
    cfg.hybrid = struct();
    cfg.hybrid.initialMasterState = [];
    cfg.hybrid.initialCorrectionState = zeros(2, 3);
    cfg.hybrid.accelerometerBiasTimeConstant = 60.0;

    cfg.hybrid.stationary = struct();
    cfg.hybrid.stationary.entrySpeed = 0.10;
    cfg.hybrid.stationary.exitSpeed = 0.20;
    cfg.hybrid.stationary.maximumYawRate = 0.03;
    cfg.hybrid.stationary.maximumLongitudinalAcceleration = 0.15;
    cfg.hybrid.stationary.maximumLateralAcceleration = 0.15;
    cfg.hybrid.stationary.minimumDwellTime = 0.10;

    cfg.hybrid.crawl = struct();
    cfg.hybrid.crawl.enabled = true;
    cfg.hybrid.crawl.maximumSteeringAngle = 0.60;

    % Each pseudo-measurement acts through a persistent correction-injection
    % state. A positive biasGain is applied with a negative sign so a positive
    % lateral-velocity residual reduces the estimated accelerometer bias.
    cfg.hybrid.correction = struct();
    cfg.hybrid.correction.stationary = struct( ...
        "timeConstant", 0.08, "velocityGain", 12.0, "biasGain", 2.0);
    cfg.hybrid.correction.crawl = struct( ...
        "timeConstant", 0.15, "velocityGain", 2.0, "biasGain", 0.20);
    cfg.hybrid.correction.dynamic = struct( ...
        "timeConstant", 0.08, "velocityGain", 12.0, "biasGain", 0.50);

    % The dynamic correction reaches full participation above the lower
    % transition band and fades back to zero before either speed-certificate
    % boundary. Residual scores use quintic smooth steps, so their first two
    % derivatives vanish at the edges of every ordinary transition band.
    cfg.hybrid.dynamic = struct();
    cfg.hybrid.dynamic.fullParticipationSpeed = 6.0;
    cfg.hybrid.dynamic.upperFullParticipationSpeed = 29.0;
    cfg.hybrid.dynamic.fullLateralAcceleration = 4.0;
    cfg.hybrid.dynamic.zeroLateralAcceleration = 7.0;
    cfg.hybrid.dynamic.fullLateralInnovation = 2.0;
    cfg.hybrid.dynamic.zeroLateralInnovation = 8.0;
    cfg.hybrid.dynamic.fullYawInnovation = 0.05;
    cfg.hybrid.dynamic.zeroYawInnovation = 0.40;

    % Outside the LPV certificate the hidden state follows the common master
    % and measured yaw rate through a nonsingular shadow model. No reciprocal
    % speed is formed in this branch.
    cfg.hybrid.shadow = struct();
    cfg.hybrid.shadow.lateralVelocityTrackingGain = 8.0;
    cfg.hybrid.shadow.yawRateTrackingGain = 15.0;

    % A persistent second-order interface state exports beta and betaDot.
    % It tracks zero while the velocity direction is undefined and tracks the
    % wrapped atan2 command when it is valid; the state is never reset.
    cfg.hybrid.sideSlip = struct();
    cfg.hybrid.sideSlip.validSpeed = 1.0;
    cfg.hybrid.sideSlip.fullParticipationSpeed = ...
        cfg.hybrid.dynamic.fullParticipationSpeed;
    cfg.hybrid.sideSlip.naturalFrequency = 12.0;
    cfg.hybrid.sideSlip.dampingRatio = 1.0;
    cfg.hybrid.sideSlip.initialState = [];

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
    cfg.simulation.randomSeed = 2026;
    sensors = mncavSensorConfig();
    cfg.sensorParameterProvenance = sensors;
    % Effective white-noise priors on the reconstructed simulation grid;
    % neither OEM IMU specifications nor a measured native covariance.
    cfg.simulation.lateralAccelerationNoiseStd = ...
        sensors.lateralSimulation.lateralAccelerationNoiseStdMps2;
    cfg.simulation.yawRateNoiseStd = sensors.lateralSimulation.yawRateNoiseStdRadps;
end
