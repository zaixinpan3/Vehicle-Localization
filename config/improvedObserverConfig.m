function cfg = improvedObserverConfig()
% improvedObserverConfig Configure the seven-state improved vehicle observer.
% The configuration covers the operating envelope used by the robust LMI,
% the continuous high-gain observer, delayed GPS/lidar replay, lidar
% information shaping, and the deterministic synthetic validation scenario.

    cfg = struct();

    cfg.operating = struct();
    cfg.operating.maximumSpeed = 8.0;
    cfg.operating.maximumAcceleration = 5.0;
    cfg.operating.maximumTrackAngleRate = 0.60;

    cfg.observer = struct();
    cfg.observer.theta = 3.5;
    cfg.observer.sigma = 3.0;
    cfg.observer.scalingExponents = [1; 2; 3; 1; 2; 3; 1];
    cfg.observer.integrationMethod = "rk4";
    cfg.observer.initialState = [];
    cfg.observer.fallbackPosition = [0.0; 0.0];
    cfg.observer.fallbackHeading = 0.0;
    % Preserve q = r_m + betaDot exactly. Samples outside the certified
    % operating box are reported by the runtime rather than silently altered.
    cfg.observer.clampTrackAngleRateToDesignEnvelope = false;
    cfg.observer.invariantGain = zeros(7, 4);
    cfg.observer.invariantGain(2, 1) = 0.02;
    cfg.observer.invariantGain(5, 1) = 0.02;
    cfg.observer.invariantGain(3, 2) = 0.02;
    cfg.observer.invariantGain(6, 2) = 0.02;
    cfg.observer.invariantGain(3, 3) = 0.02;
    cfg.observer.invariantGain(6, 3) = -0.02;
    cfg.observer.invariantGain(7, 4) = -0.10;

    cfg.measurement = struct();
    cfg.measurement.sampleTime = 0.01;
    cfg.measurement.inputInterpolation = "linear";
    % Low-rate absolute poses act as short zero-order-held correction pulses.
    % Their physical timestamps, rather than their delayed arrival times, set
    % the pulse window during replay.
    cfg.measurement.gpsMaximumAge = 0.03;
    cfg.measurement.lidarMaximumAge = 0.03;
    cfg.measurement.replayBufferDuration = 1.0;
    cfg.measurement.timestampTolerance = 0.0051;

    cfg.lidar = struct();
    cfg.lidar.translationInformationScale = 25.0;
    cfg.lidar.headingInformationScale = 50.0;
    cfg.lidar.minimumHeadingWeight = 0.15;
    cfg.lidar.missingInformationTranslationWeight = 0.0;
    cfg.lidar.missingInformationHeadingWeight = 0.15;

    cfg.synthesis = struct();
    cfg.synthesis.solver = "sedumi";
    cfg.synthesis.verbose = 0;
    cfg.synthesis.dualize = 0;
    cfg.synthesis.normalizationTrace = 7.0;
    cfg.synthesis.minimumPEigenvalue = 0.14;
    cfg.synthesis.minimumLambda = 1.0e-5;
    cfg.synthesis.maximumLambda = 10.0;
    cfg.synthesis.strictnessEpsilon = 1.0e-7;
    cfg.synthesis.violationTolerance = 1.0e-7;
    cfg.synthesis.gainDecisionWeight = 2.0e-5;
    cfg.synthesis.lambdaSafetyFactor = 0.75;
    cfg.synthesis.schurSafetyFactor = 4.0;
    cfg.synthesis.maximumCuttingPlaneIterations = 40;
    cfg.synthesis.outputFolder = "";
    cfg.synthesis.saveFileName = "improvedObserverDesign.mat";

    cfg.simulation = struct();
    cfg.simulation.sampleTime = 0.01;
    cfg.simulation.finalTime = 24.0;
    cfg.simulation.initialPosition = [2.0; -1.0];
    cfg.simulation.initialHeading = deg2rad(8.0);
    cfg.simulation.initialLongitudinalSpeed = 6.0;
    cfg.simulation.longitudinalSpeedAmplitude = 0.8;
    cfg.simulation.longitudinalSpeedPeriod = 16.0;
    cfg.simulation.steeringAmplitude = deg2rad(2.2);
    cfg.simulation.steeringPeriod = 10.0;
    cfg.simulation.gpsRateHz = 5.0;
    cfg.simulation.lidarRateHz = 10.0;
    cfg.simulation.gpsDelay = 0.10;
    cfg.simulation.lidarDelay = 0.08;
    cfg.simulation.outOfOrderExtraDelay = 0.14;
    cfg.simulation.gpsDropoutInterval = [8.0, 12.0];
    cfg.simulation.lidarDegeneracyInterval = [14.0, 18.0];
    cfg.simulation.wheelSpeedNoiseStandardDeviation = 0.01;
    cfg.simulation.steeringNoiseStandardDeviation = deg2rad(0.02);
    cfg.simulation.gyroNoiseStandardDeviation = 0.0015;
    cfg.simulation.accelerationNoiseStandardDeviation = 0.025;
    cfg.simulation.gpsPositionNoiseStandardDeviation = 0.035;
    cfg.simulation.lidarPositionNoiseStandardDeviation = 0.020;
    cfg.simulation.lidarHeadingNoiseStandardDeviation = deg2rad(0.25);
    cfg.simulation.initialEstimateError = [0.30; -0.20; deg2rad(4.0)];
    cfg.simulation.randomSeed = 2026;
end
