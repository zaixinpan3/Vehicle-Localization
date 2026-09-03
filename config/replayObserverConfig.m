function cfg = replayObserverConfig()
% replayObserverConfig: Configuration of the retrodictive-replay high-gain
% ego-vehicle observer. The observer integrates the transformed six-state
% vehicle model at 100 Hz from speed, steering, yaw-rate, and constraint
% measurements, inserts delayed 5 Hz [x, y, yaw] pose measurements as
% discrete jumps at their physical timestamps, and replays the buffered
% history to the present. The gain set is the published speed, constraint,
% and yaw-rate observer; the pose jump gain Kd is synthesized offline by
% designReplayObserverGains.
%
% Input:
%   none
%
% Output:
%   cfg: struct with vehicle, design, observer, measurement, initialization,
%       simulation, replay, replayDesign, and replaySimulation groups
    cfg = struct();

    % Kinematic bicycle geometry used by the coordinate transformation
    cfg.vehicle = struct();
    cfg.vehicle.lf = 1.45;
    cfg.vehicle.lr = 1.55;
    cfg.vehicle.wheelbase = cfg.vehicle.lf + cfg.vehicle.lr;

    % Operating set certified by the gain design
    cfg.design = struct();
    cfg.design.speedBounds = [4.0, 8.0];
    cfg.design.maxAbsYawRate = 0.35;

    % Published high-gain observer parameters for speed, constraint, and yaw-rate outputs
    cfg.observer = struct();
    cfg.observer.gainSet = "speedConstraintYawRate";
    cfg.observer.theta = 3.0;
    cfg.observer.sigma = 3.0;
    cfg.observer.K = [0.2247, 0.0; ...
        0.4704, 0.0; ...
        0.1832, 0.0; ...
        0.0, 0.2247; ...
        0.0, 0.4704; ...
        0.0, 0.1832];
    cfg.observer.N = [0.4858, 0.7402, 0.4603; ...
        0.8858, 0.3402, 0.1603; ...
        0.4858, 0.7402, 0.4603; ...
        0.4858, 0.7402, 0.4603; ...
        0.8858, 0.3402, 0.1603; ...
        0.4858, 0.7402, 0.4603];
    cfg.observer.extraOutputs = ["speed"; "orthogonalityConstraint"; "yawRate"];
    cfg.observer.scalingExponents = [1; 2; 3; 1; 2; 3];
    cfg.observer.extraGainThetaPower = 6;
    cfg.observer.integrationMethod = "rk4";

    % High-rate measurement stream conventions
    cfg.measurement = struct();
    cfg.measurement.sampleTime = 0.01;
    cfg.measurement.defaultSteeringAngle = 0.0;
    cfg.measurement.speedFields = ["speed", "Speed", "v", "V"];
    cfg.measurement.yawRateFields = ["yawRate", "YawRate", "imuYawRate", "r"];
    cfg.measurement.steeringFields = ["steeringAngle", "SteeringAngle", "deltaF", "frontSteeringAngle"];

    % State initialization from an explicit state or the first measurements
    cfg.initialization = struct();
    cfg.initialization.state = [];
    cfg.initialization.velocity = [];
    cfg.initialization.acceleration = [0.0; 0.0];
    cfg.initialization.fallbackSpeed = 5.0;
    cfg.initialization.fallbackHeading = 0.0;
    cfg.simulation = struct();
    cfg.simulation.defaultInitialPosition = [0.0; 0.0];

    % Timestamped pose replay: buffer, tolerances, and output filtering
    cfg.replay = struct();
    cfg.replay.sampleTime = 0.01;
    cfg.replay.poseRateHz = 5.0;
    cfg.replay.highRateHz = 100.0;
    cfg.replay.bufferDuration = 1.0;
    cfg.replay.maxInformationAge = 0.50;
    cfg.replay.timestampTolerance = 0.0051;
    cfg.replay.inputInterpolation = "linear";
    cfg.replay.speedRegularization = 1.0e-6;
    cfg.replay.Kd = [];
    cfg.replay.timestampCorrectionGain = [];
    cfg.replay.usePoseYawVelocityCorrection = false;
    cfg.replay.yawVelocityBlend = 0.0;
    cfg.replay.yawAccelerationBlend = 0.0;
    cfg.replay.outputPositionBlendTimeConstant = 0.35;
    cfg.replay.outputAccelerationFilterTimeConstant = 0.20;

    % Offline LMI gain design (requires YALMIP and SeDuMi on the MATLAB path)
    cfg.replayDesign = struct();
    cfg.replayDesign.outputFolder = "";
    cfg.replayDesign.saveFileName = "replayObserverDesign.mat";
    cfg.replayDesign.solver = "sedumi";
    cfg.replayDesign.verbose = 0;
    cfg.replayDesign.dualize = 0;
    cfg.replayDesign.pFloor = 1.0e-4;
    cfg.replayDesign.pCeiling = 1.0e4;
    cfg.replayDesign.strictnessEpsilon = 1.0e-7;
    cfg.replayDesign.jumpStrictnessEpsilon = 0.0;
    cfg.replayDesign.flowAh = 2.0;
    cfg.replayDesign.flowObjectiveWeight = 1.0e-6;
    cfg.replayDesign.headingSamples = 49;
    cfg.replayDesign.speedSamples = 11;
    cfg.replayDesign.yawRateSamples = 11;
    cfg.replayDesign.derivativeInflation = 1.0e-3;
    cfg.replayDesign.transitionMinInterval = 1.0 ./ cfg.replay.poseRateHz;
    cfg.replayDesign.transitionMaxInterval = 1.0 ./ cfg.replay.poseRateHz;
    cfg.replayDesign.transitionTimeSamples = 1;
    cfg.replayDesign.transitionMaxHVertices = 96;
    cfg.replayDesign.transitionInflation = 0.0;
    cfg.replayDesign.transitionVerified = false;
    cfg.replayDesign.poseCorrectionOutputs = ["x"; "y"; "yaw"];
    cfg.replayDesign.speedSingularityMinSpeedEpsilon = 1.0e-3;
    cfg.replayDesign.poseYawMinSpeedEpsilon = cfg.replayDesign.speedSingularityMinSpeedEpsilon;
    cfg.replayDesign.rhoCandidates = [0.05, 0.10, 0.20, 0.30, 0.50, 0.80, 0.90, 0.95, 0.98, 0.995, 1.0];
    cfg.replayDesign.runSimulationAfterDesign = true;

    % Synthetic replay scenario: 100 Hz truth, noisy sensors, delayed 5 Hz poses
    cfg.replaySimulation = struct();
    cfg.replaySimulation.tFinal = 24.0;
    cfg.replaySimulation.speed = 5.0;
    cfg.replaySimulation.initialYaw = 0.0;
    cfg.replaySimulation.initialPosition = [0.0; 0.0];
    cfg.replaySimulation.finalSteeringDeg = 4.0;
    cfg.replaySimulation.rampDuration = 6.0;
    cfg.replaySimulation.steeringWaypointTime = [0.0, 3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0];
    cfg.replaySimulation.steeringWaypointDeg = [0.0, 5.0, -5.0, 4.5, -4.5, 5.0, -3.5, 4.0, 0.0];
    cfg.replaySimulation.poseDelay = 0.12;
    cfg.replaySimulation.enableOutOfOrderArrival = true;
    cfg.replaySimulation.outOfOrderExtraDelay = 0.18;
    cfg.replaySimulation.enableSensorNoise = true;
    cfg.replaySimulation.positionNoiseStd = 0.03;
    cfg.replaySimulation.yawNoiseStd = deg2rad(0.5);
    cfg.replaySimulation.speedNoiseStd = 0.03;
    cfg.replaySimulation.yawRateNoiseStd = 0.005;
    cfg.replaySimulation.steeringAngleNoiseStd = deg2rad(0.05);
    cfg.replaySimulation.orthogonalityConstraintNoiseStd = 0.02;
    cfg.replaySimulation.randomSeed = 2026;
    cfg.replaySimulation.initialEstimatePose = [0.20; -0.15; deg2rad(3.0)];
    cfg.replaySimulation.initialSpeedEstimateBias = -0.20;
end
