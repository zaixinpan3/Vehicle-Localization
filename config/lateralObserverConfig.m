function cfg = lateralObserverConfig()
% lateralObserverConfig: Parameters of the LPV lateral-velocity observer.
% The vehicle group defines the 2-DOF lateral bicycle model; the scheduling
% group defines the speed range covered by the polytope and the design grid
% over speed and longitudinal acceleration; the synthesis group holds the
% Lipschitz bound, the error weighting, and the LMI solver settings; the
% observer group configures the online integration; the simulation group
% defines the synthetic scenario used to exercise the observer.
%
% The vehicle parameters are the archived lateral-observer values of the
% original repository. They are a plausible mid-size sedan, not identified
% MnCAV parameters: replace mass, yaw inertia, and the cornering
% stiffnesses with identified values before drawing quantitative
% conclusions.
%
% Input:
%   none
%
% Output:
%   cfg: struct with vehicle, scheduling, synthesis, observer, and
%       simulation groups
    cfg = struct();

    % 2-DOF lateral bicycle model parameters
    cfg.vehicle = struct();
    cfg.vehicle.mass = 1575.0;
    cfg.vehicle.yawInertia = 2875.0;
    cfg.vehicle.lf = 1.20;
    cfg.vehicle.lr = 1.60;
    cfg.vehicle.frontCorneringStiffness = 75000.0;
    cfg.vehicle.rearCorneringStiffness = 56000.0;

    % Scheduling parameter rho = [Vx; 1/Vx] and the design grid over it.
    % The LMI is affine in the longitudinal acceleration through Pdot, so
    % the two extreme accelerations already cover the whole interval.
    cfg.scheduling = struct();
    cfg.scheduling.speedRange = [5.0, 30.0];
    cfg.scheduling.longitudinalAccelerationRange = [-3.0, 3.0];
    cfg.scheduling.speedGridCount = 9;
    cfg.scheduling.accelerationGridCount = 2;

    % H2 synthesis: Lipschitz bound of the unmodeled nonlinearity, the
    % error weighting Q of the performance output z = Q^(1/2) e, and the
    % fixed scalar tau of Young's inequality (searched over candidates).
    cfg.synthesis = struct();
    cfg.synthesis.lipschitzConstant = 0.5;
    cfg.synthesis.errorWeight = eye(2);
    cfg.synthesis.tauCandidates = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0];
    cfg.synthesis.pFloor = 1.0e-4;
    cfg.synthesis.pCeiling = 1.0e6;
    cfg.synthesis.etaFloor = 1.0e-6;
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
    cfg.observer.minimumSpeed = 1.0;

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
    cfg.simulation.lateralAccelerationNoiseStd = 0.05;
    cfg.simulation.yawRateNoiseStd = 0.002;
    cfg.simulation.initialStateError = [0.5; 0.05];
    cfg.simulation.randomSeed = 2026;
end
