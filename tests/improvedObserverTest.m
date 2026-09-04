classdef improvedObserverTest < matlab.unittest.TestCase
% improvedObserverTest Tests of the complete seven-state observer cascade.
% Algebraic tests cover the nonsingular model, invariant outputs, lidar
% information shaping, and robust-LMI vertex construction. Runtime tests use
% stored certified gains and therefore need no optimization toolbox.

    properties (Access = private)
        LateralDesign
        ObserverDesign
    end

    methods (TestClassSetup)
        function addProjectPathsAndLoadDesigns(testCase)
        % addProjectPathsAndLoadDesigns Add modules and load certified gains.
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            run(fullfile(projectFolder, "setupVehicleLocalization.m"));
            lateral = load(fullfile(projectFolder, "tests", "reference", ...
                "lateralObserverDesign.mat"));
            testCase.LateralDesign = lateral.design;
            testCase.ObserverDesign = improvedObserverReferenceDesign();
        end
    end

    methods (Test)
        function certificateDataHasEveryRequiredVertexFamily(testCase)
        % certificateDataHasEveryRequiredVertexFamily Check the complete box.
            cfg = improvedObserverConfig();
            data = buildImprovedObserverCertificateData(cfg);

            testCase.verifySize(data.A, [7, 7]);
            testCase.verifySize(data.B, [7, 2]);
            testCase.verifySize(data.Bu, [7, 1]);
            testCase.verifySize(data.Cb, [3, 7]);
            testCase.verifySize(data.Cl, [2, 7]);
            testCase.verifyEqual(data.outputVertexCount, 2^13);
            testCase.verifyEqual(data.omegaVertexCount, 2);
            testCase.verifyEqual(data.fVertexCount, 4);
            testCase.verifyEqual(data.totalVertexCount, 65536);
            testCase.verifyEqual(data.Tsigma, ...
                diag(cfg.observer.sigma.^cfg.observer.scalingExponents), AbsTol=0);
        end

        function channelsMatchTheNonsingularModelAndInvariantMap(testCase)
        % channelsMatchTheNonsingularModelAndInvariantMap Check all equations.
            state = [1.0; 2.0; 3.0; 4.0; 5.0; 6.0; 0.7];
            sample = struct("longitudinalSpeed", 2.2, "lateralVelocity", 0.3, ...
                "longitudinalAcceleration", 0.4, "lateralAcceleration", -0.2, ...
                "yawRate", 0.15, "sideSlipAngle", 0.1, ...
                "sideSlipAngleRate", -0.05);
            channels = evaluateImprovedObserverChannels(state, sample);
            trackRate = sample.yawRate + sample.sideSlipAngleRate;
            expectedDerivative = [state(2); state(3); ...
                trackRate.^2 .* state(2) - 2.0 .* trackRate .* state(6); ...
                state(5); state(6); ...
                trackRate.^2 .* state(5) + 2.0 .* trackRate .* state(3); ...
                sample.yawRate];
            expectedPrediction = [state(2).^2 + state(5).^2; ...
                state(2) .* state(3) + state(5) .* state(6); ...
                state(2) .* state(6) - state(5) .* state(3); ...
                state(5) .* cos(state(7) + sample.sideSlipAngle) - ...
                    state(2) .* sin(state(7) + sample.sideSlipAngle)];
            expectedMeasurement = [sample.longitudinalSpeed.^2 + sample.lateralVelocity.^2; ...
                sample.longitudinalSpeed .* sample.longitudinalAcceleration + ...
                    sample.lateralVelocity .* sample.lateralAcceleration; ...
                sample.longitudinalSpeed .* sample.lateralAcceleration - ...
                    sample.lateralVelocity .* sample.longitudinalAcceleration; 0.0];

            testCase.verifyEqual(channels.trackAngleRate, trackRate, AbsTol=1.0e-14);
            testCase.verifyEqual(channels.modelDerivative, expectedDerivative, AbsTol=1.0e-14);
            testCase.verifyEqual(channels.invariantPrediction, expectedPrediction, AbsTol=1.0e-14);
            testCase.verifyEqual(channels.invariantMeasurement, expectedMeasurement, AbsTol=1.0e-14);
            testCase.verifyEqual(channels.invariantInnovation, ...
                expectedMeasurement - expectedPrediction, AbsTol=1.0e-14);
        end

        function lidarWeightsPreserveObservedDirections(testCase)
        % lidarWeightsPreserveObservedDirections Check the Hessian mapping.
            cfg = improvedObserverConfig();
            angle = pi ./ 6.0;
            directions = [cos(angle), -sin(angle); sin(angle), cos(angle)];
            information = zeros(3, 3);
            information(1:2, 1:2) = directions * diag([225.0, 0.0]) * directions.';
            information(3, 3) = 100.0;
            [translationWeight, headingWeight, diagnostics] = ...
                computeLidarInformationWeights(information, cfg);

            testCase.verifyEqual(translationWeight * directions(:, 1), ...
                0.9 .* directions(:, 1), AbsTol=1.0e-12);
            testCase.verifyEqual(translationWeight * directions(:, 2), ...
                zeros(2, 1), AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(eig(translationWeight), -1.0e-12 .* ones(2, 1));
            testCase.verifyLessThanOrEqual(eig(translationWeight), ones(2, 1));
            testCase.verifyEqual(headingWeight, 2.0 ./ 3.0, AbsTol=1.0e-12);
            testCase.verifyTrue(diagnostics.hadInformation);
        end

        function missingLidarInformationUsesSafeConfiguredWeights(testCase)
        % missingLidarInformationUsesSafeConfiguredWeights Check fallback W.
            cfg = improvedObserverConfig();
            [translationWeight, headingWeight, diagnostics] = ...
                computeLidarInformationWeights([], cfg);

            testCase.verifyEqual(translationWeight, zeros(2), AbsTol=0);
            testCase.verifyEqual(headingWeight, cfg.lidar.missingInformationHeadingWeight, AbsTol=0);
            testCase.verifyFalse(diagnostics.hadInformation);
        end

        function storedDesignPassesTheExhaustiveCertificate(testCase)
        % storedDesignPassesTheExhaustiveCertificate Recheck all 65,536 LMIs.
            cfg = improvedObserverConfig();
            design = testCase.ObserverDesign;
            verification = verifyImprovedObserverDesign(design, cfg);

            testCase.verifyTrue(design.knownInputIncludedExactly);
            testCase.verifyGreaterThan(norm(design.N, "fro"), 0.0);
            testCase.verifyTrue(verification.certified);
            testCase.verifyEqual(verification.checkedVertexCount, 65536);
            testCase.verifyLessThan(verification.worstBlockMargin, 0.0);
            testCase.verifyLessThan(verification.worstSchurMargin, 0.0);
            testCase.verifyGreaterThan(verification.minimumPEigenvalue, 0.0);
            testCase.verifyGreaterThan(verification.minimumXEigenvalue, 0.0);
        end

        function completeCascadeTracksThroughDelayDropoutAndDegeneracy(testCase)
        % completeCascadeTracksThroughDelayDropoutAndDegeneracy Run the demo.
            result = simulateImprovedObserverScenario(testCase.ObserverDesign, ...
                testCase.LateralDesign, improvedObserverConfig());
            metrics = result.metrics;

            testCase.verifySize(result.estimate.z, [numel(result.truth.time), 7]);
            testCase.verifyTrue(all(isfinite(result.estimate.z), "all"));
            testCase.verifyLessThan(metrics.positionRmse, 0.25);
            testCase.verifyLessThan(metrics.headingRmse, 0.01);
            testCase.verifyLessThan(metrics.velocityRmse, 0.60);
            testCase.verifyLessThan(metrics.accelerationRmse, 1.10);
            testCase.verifyLessThan(metrics.trackAngleRateRmse, 0.01);
            testCase.verifyGreaterThan(metrics.replayCount, 0);
            testCase.verifyGreaterThan(metrics.acceptedEventCount, 0);
            testCase.verifyEqual(metrics.rejectedEventCount, 0);
            testCase.verifyLessThan(metrics.degenerateLidarMinimumWeight, 0.05);
            testCase.verifyGreaterThan(metrics.nominalLidarMinimumWeight, 0.80);
            testCase.verifyFalse(any(result.estimate.diagnostics.outsideTrackRateEnvelope));
            testCase.verifyFalse(any(result.estimate.diagnostics.estimatedVelocityOutsideEnvelope));
            testCase.verifyFalse(any(result.estimate.diagnostics.estimatedAccelerationOutsideEnvelope));
        end

        function trackAngleRateIsGyroPlusIndependentSideSlipRate(testCase)
        % trackAngleRateIsGyroPlusIndependentSideSlipRate Check the cascade.
            result = simulateImprovedObserverScenario(testCase.ObserverDesign, ...
                testCase.LateralDesign, improvedObserverConfig());
            estimate = result.estimate;
            expectedRate = estimate.measurements.highRate.yawRate + estimate.sideSlipAngleRate;

            testCase.verifyFalse(any(estimate.diagnostics.outsideTrackRateEnvelope));
            testCase.verifyEqual(estimate.trackAngleRate, expectedRate, AbsTol=1.0e-12);
        end

        function stationaryInputRemainsFiniteWithoutSpeedDivision(testCase)
        % stationaryInputRemainsFiniteWithoutSpeedDivision Check v=0 behavior.
            cfg = improvedObserverConfig();
            cfg.observer.initialState = zeros(7, 1);
            sensorData = testCase.zeroMotionSensorData(0.20);
            lateralDesign = testCase.LateralDesign;
            lateralDesign.cfg.observer.initialState = [1.0; 0.2];
            estimate = runImprovedVehicleObserver(sensorData, lateralDesign, ...
                testCase.ObserverDesign, cfg);

            testCase.verifyTrue(all(isfinite(estimate.z), "all"));
            testCase.verifyEqual(estimate.lateral.lateralVelocity, ...
                zeros(size(estimate.time)), AbsTol=0);
            testCase.verifyEqual(estimate.sideSlipAngle, zeros(size(estimate.time)), AbsTol=0);
            testCase.verifyEqual(estimate.sideSlipAngleRate, zeros(size(estimate.time)), AbsTol=0);
            testCase.verifyEqual(estimate.trackAngleRate, zeros(size(estimate.time)), AbsTol=0);
        end

        function wrappedHeadingInnovationUsesTheShortestArc(testCase)
        % wrappedHeadingInnovationUsesTheShortestArc Check the branch cut.
            cfg = improvedObserverConfig();
            cfg.observer.initialState = [0; 0; 0; 0; 0; 0; pi - 0.01];
            sensorData = testCase.zeroMotionSensorData(0.05);
            sensorData.lidar = struct("timestamp", 0.0, "arrivalTime", 0.0, ...
                "pose", [0.0, 0.0, -pi + 0.01], "information", eye(3));
            estimate = runImprovedVehicleObserver(sensorData, testCase.LateralDesign, ...
                testCase.ObserverDesign, cfg);

            testCase.verifyEqual(estimate.innovations.base(1, 3), 0.02, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(estimate.heading, -pi .* ones(size(estimate.heading)));
            testCase.verifyLessThan(estimate.heading, pi .* ones(size(estimate.heading)));
        end

        function eventOlderThanReplayBufferIsRejected(testCase)
        % eventOlderThanReplayBufferIsRejected Check bounded replay behavior.
            cfg = improvedObserverConfig();
            cfg.observer.initialState = zeros(7, 1);
            sensorData = testCase.zeroMotionSensorData(2.0);
            sensorData.gps = struct("timestamp", 0.0, "arrivalTime", 1.5, ...
                "pose", [1.0, 2.0]);
            estimate = runImprovedVehicleObserver(sensorData, testCase.LateralDesign, ...
                testCase.ObserverDesign, cfg);

            testCase.verifyEqual(estimate.diagnostics.acceptedEventCount(end), 0);
            testCase.verifyEqual(estimate.diagnostics.rejectedEventCount(end), 1);
            testCase.verifyFalse(estimate.diagnostics.acceptedGps);
        end

        function runtimeRejectsAMismatchedCertificateEnvelope(testCase)
        % runtimeRejectsAMismatchedCertificateEnvelope Prevent false claims.
            cfg = improvedObserverConfig();
            cfg.operating.maximumSpeed = cfg.operating.maximumSpeed + 1.0;
            sensorData = testCase.zeroMotionSensorData(0.05);

            testCase.verifyError(@() runImprovedVehicleObserver(sensorData, ...
                testCase.LateralDesign, testCase.ObserverDesign, cfg), "");
        end

        function synthesisProducesAnExhaustivelyCertifiedDesign(testCase)
        % synthesisProducesAnExhaustivelyCertifiedDesign Re-run the SDP.
            testCase.assumeTrue(exist("sdpvar", "file") == 2, ...
                "Improved-observer synthesis needs YALMIP and SeDuMi on the MATLAB path.");
            design = designImprovedObserverGains(improvedObserverConfig());

            testCase.verifyTrue(design.certified);
            testCase.verifyEqual(design.verification.checkedVertexCount, 65536);
            testCase.verifyLessThan(design.verification.worstBlockMargin, 0.0);
            testCase.verifyGreaterThan(norm(design.N, "fro"), 0.0);
        end
    end

    methods (Static, Access = private)
        function sensorData = zeroMotionSensorData(finalTime)
        % zeroMotionSensorData Build a finite stationary high-rate stream.
            time = (0:0.01:finalTime).';
            sampleCount = numel(time);
            highRate = struct("time", time, "steeringAngle", zeros(sampleCount, 1), ...
                "longitudinalSpeed", zeros(sampleCount, 1), ...
                "longitudinalAcceleration", zeros(sampleCount, 1), ...
                "lateralAcceleration", zeros(sampleCount, 1), ...
                "yawRate", zeros(sampleCount, 1));
            sensorData = struct("highRate", highRate);
        end
    end
end
