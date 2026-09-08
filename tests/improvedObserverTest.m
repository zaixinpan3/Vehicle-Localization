classdef improvedObserverTest < matlab.unittest.TestCase
% improvedObserverTest Tests of the complete seven-state observer cascade.
% Algebraic tests cover the nonsingular model, invariant outputs, lidar
% full-matrix admission, and robust timer-LMI construction. Runtime tests use
% stored certified gains; only the explicitly named synthesis test needs a solver.

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

        function deficientInformationDoesNotInventAPoseChannel(testCase)
            cfg = improvedObserverConfig();
            [translation,heading,info] = computeLidarInformationWeights(diag([225,0,100]),cfg);
            testCase.verifyEqual(translation,zeros(2),AbsTol=0);
            testCase.verifyEqual(heading,0,AbsTol=0);
            testCase.verifyFalse(info.qualified);
        end

        function fullPoseUsesUnitTranslationGain(testCase)
            cfg = improvedObserverConfig();
            [translation,heading,info] = computeLidarInformationWeights(diag([9,12,100]),cfg);
            testCase.verifyEqual(translation,eye(2),AbsTol=1e-12);
            testCase.verifyEqual(heading,2/3,AbsTol=1e-12);
            testCase.verifyTrue(info.qualified);
        end

        function fullInformationCrossTermsControlAdmission(testCase)
            cfg = improvedObserverConfig();
            cfg.lidar.informationLowerBound=.1*eye(3);
            [translation,heading,info] = computeLidarInformationWeights( ...
                [1,0,.99;0,1,0;.99,0,1],cfg);
            testCase.verifyEqual(translation,zeros(2),AbsTol=0);
            testCase.verifyEqual(heading,0,AbsTol=0);
            testCase.verifyFalse(info.qualified);
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
        % storedDesignPassesTheExhaustiveCertificate Check all flow and reset LMIs.
            cfg = improvedObserverConfig();
            design = testCase.ObserverDesign;
            verification = verifyImprovedObserverDesign(design, cfg);

            testCase.verifyTrue(design.knownInputIncludedExactly);
            testCase.verifyGreaterThan(norm(design.N, "fro"), 0.0);
            testCase.verifyTrue(verification.certified);
            testCase.verifyEqual(verification.checkedVertexCount, 1441792);
            testCase.verifyLessThan(verification.maximumFlowEigenvalue, 0.0);
            testCase.verifyLessThan(verification.maximumResetEigenvalue, 0.0);
            testCase.verifyGreaterThan(verification.minimumPEigenvalue, 0.0);

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
            testCase.verifyEqual(metrics.degenerateLidarMinimumWeight,1,AbsTol=1e-12);
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
            testCase.verifyFalse(any(estimate.lateral.diagnostics.dynamicModelEvaluated));
            testCase.verifyEqual(estimate.lateral.lateralVelocity(1), 1.0, AbsTol=0.0);
            testCase.verifyLessThan(abs(estimate.lateral.lateralVelocity(end)), ...
                abs(estimate.lateral.lateralVelocity(1)));
            testCase.verifyEqual(estimate.sideSlipAngle, zeros(size(estimate.time)), AbsTol=0);
            testCase.verifyEqual(estimate.sideSlipAngleRate, zeros(size(estimate.time)), AbsTol=0);
            testCase.verifyEqual(estimate.trackAngleRate, zeros(size(estimate.time)), AbsTol=0);
        end

        function completeCascadeRemainsFiniteThroughStopGoModes(testCase)
        % completeCascadeRemainsFiniteThroughStopGoModes Exercise stationary,
        % crawl, dynamic, and braking transitions through the public cascade.
            cfg = improvedObserverConfig();
            cfg.observer.initialState = zeros(7, 1);
            sensorData = testCase.stopGoSensorData();
            externallyInvalid = sensorData.highRate.time >= 2.50 & ...
                sensorData.highRate.time < 2.75;
            sensorData.highRate.dynamicValid = ~externallyInvalid;
            lateralDesign = testCase.LateralDesign;
            lateralDesign.cfg.observer.initialState = [0.80; 0.10];

            estimate = runImprovedVehicleObserver(sensorData, lateralDesign, ...
                testCase.ObserverDesign, cfg);

            testCase.verifyTrue(all(isfinite(estimate.z), "all"));
            testCase.verifyTrue(any(estimate.lateral.mode == "stationary"));
            testCase.verifyTrue(any(estimate.lateral.mode == "crawl"));
            testCase.verifyTrue(any(estimate.lateral.mode == "dynamic"));
            testCase.verifyGreaterThanOrEqual(nnz(estimate.lateral.modeChanged), 4);
            testCase.verifyFalse(any(estimate.lateral.diagnostics.dynamicModelEvaluated( ...
                externallyInvalid)));
            testCase.verifyFalse(any(estimate.diagnostics.outsideTrackRateEnvelope));
            testCase.verifyLessThan(max(abs(estimate.trackAngleRate)), ...
                cfg.operating.maximumTrackAngleRate);
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
                testCase.LateralDesign, testCase.ObserverDesign, cfg), "VehicleLocalization:CertificateMismatch");
        end

        function delayedReplayRecoversTheSameMeasurementTimeHistory(testCase)
            cfg = improvedObserverConfig(); cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.60);
            data.lidar=struct('timestamp',[0;.1;.2;.3], ...
                'arrivalTime',[0;.1;.2;.3], 'pose',repmat([.2,-.1,.01],4,1), ...
                'information',repmat(100*eye(3),1,1,4));
            immediate=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            data.lidar.arrivalTime=data.lidar.timestamp+.15;
            delayed=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(delayed.z,immediate.z,AbsTol=1e-11);
            testCase.verifyEqual(delayed.onlineZ(delayed.time<.15,:), ...
                zeros(nnz(delayed.time<.15),7),AbsTol=1e-12);
        end

        function irregularPulseEdgesAgreeWithAFinerInputGrid(testCase)
            cfg=improvedObserverConfig(); cfg.observer.initialState=zeros(7,1);
            coarse=testCase.zeroMotionSensorData(.10);
            coarse.lidar=struct('timestamp',.003,'arrivalTime',.003, ...
                'pose',[.2,-.1,.01],'information',100*eye(3));
            fine=coarse;
            fine.highRate=testCase.stationaryHighRate((0:.005:.1).');
            a=runImprovedVehicleObserver(coarse,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            b=runImprovedVehicleObserver(fine,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(a.z(end,:),b.z(end,:),AbsTol=1e-5);
        end

        function regularQualifiedScheduleSatisfiesTimingAudit(testCase)
            cfg=improvedObserverConfig(); cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.70);
            stamps=(0:.1:.5).';
            data.lidar=struct('timestamp',stamps,'arrivalTime',stamps+.15, ...
                'pose',zeros(6,3),'information',repmat(100*eye(3),1,1,6));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyTrue(e.diagnostics.certificateConditions.timingWithinCertificate);
            testCase.verifyTrue(e.diagnostics.certificateConditions.fixedDelayMatchesConfiguration);
            testCase.verifyTrue(e.observer.certificateVerified);
            testCase.verifyFalse(e.observer.certified);
        end

        function prolongedPoseGapIsReportedWithoutFalseCertification(testCase)
            cfg=improvedObserverConfig(); cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.70);
            data.lidar=struct('timestamp',[0;.5],'arrivalTime',[.15;.65], ...
                'pose',zeros(2,3),'information',repmat(100*eye(3),1,1,2));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyFalse(e.diagnostics.certificateConditions.timingWithinCertificate);
            testCase.verifyEqual(e.diagnostics.certificateConditions.longIntervalCount,1);
            testCase.verifyFalse(e.observer.certified);
        end

        function changedGainCannotReuseAVerificationFlag(testCase)
            cfg=improvedObserverConfig(); data=testCase.zeroMotionSensorData(.05);
            design=testCase.ObserverDesign; design.K(1,1)=2*design.K(1,1);
            testCase.verifyError(@() runImprovedVehicleObserver(data, ...
                testCase.LateralDesign,design,cfg),'VehicleLocalization:CertificateMismatch');
        end

        function runtimeRejectsMismatchedPulseDuration(testCase)
            cfg=improvedObserverConfig();cfg.measurement.lidarMaximumAge=.12;
            data=testCase.zeroMotionSensorData(.05);
            testCase.verifyError(@() runImprovedVehicleObserver(data, ...
                testCase.LateralDesign,testCase.ObserverDesign,cfg), ...
                'VehicleLocalization:CertificateMismatch');
        end

        function gpsInAPoseGapDoesNotAddAnUncertifiedGain(testCase)
            cfg=improvedObserverConfig(); cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.09);
            data.lidar=struct('timestamp',0,'arrivalTime',0, ...
                'pose',[0,0,0],'information',100*eye(3));
            data.gps=struct('timestamp',.05,'arrivalTime',.05,'pose',[1000,-1000]);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.z,zeros(size(e.z)),AbsTol=1e-12);
            testCase.verifyEqual(nnz(e.diagnostics.acceptedGps),1);
        end

        function gpsSubstitutesPositionWithinTheFullPosePulse(testCase)
            cfg=improvedObserverConfig(); cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.03);
            data.lidar=struct('timestamp',0,'arrivalTime',0, ...
                'pose',[0,0,0],'information',100*eye(3));
            data.gps=struct('timestamp',0,'arrivalTime',0,'pose',[1,0]);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.innovations.base(1,1),1,AbsTol=1e-12);
            testCase.verifyEqual(e.innovations.lidarPosition(1,:),[0,0],AbsTol=1e-12);
            testCase.verifyGreaterThan(e.onlineZ(end,1),0);
        end

        function invariantExtensionBoundsOnlyTheNonlinearPrediction(testCase)
            cfg=improvedObserverConfig();
            sample=struct('longitudinalSpeed',4,'lateralVelocity',0, ...
                'longitudinalAcceleration',0,'lateralAcceleration',0, ...
                'yawRate',0,'sideSlipAngle',0,'sideSlipAngleRate',0);
            z=[0;500;100;0;-500;-100;0];
            channels=evaluateImprovedObserverChannels(z,sample,cfg.operating);
            testCase.verifyTrue(channels.invariantExtensionActive);
            testCase.verifyEqual(channels.invariantPrediction(1),512,AbsTol=1e-12);
            testCase.verifyEqual(channels.modelDerivative([1,2,4,5]), ...
                [500;100;-500;-100],AbsTol=0);
        end

        function publicPoseFieldsAreCausal(testCase)
            cfg=improvedObserverConfig(); cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.30);
            data.lidar=struct('timestamp',0,'arrivalTime',.15, ...
                'pose',[1,0,.01],'information',100*eye(3));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.pose,e.onlineZ(:,[1,4,7]),AbsTol=0);
            testCase.verifyEqual(e.position,e.onlineZ(:,[1,4]),AbsTol=0);
            testCase.verifyEqual(e.heading,e.onlineZ(:,7),AbsTol=0);
            testCase.verifyEqual(e.pose(e.time<.15,:),zeros(nnz(e.time<.15),3),AbsTol=1e-12);
        end

        function lidarOnlyTracksOnTheCertifiedSchedule(testCase)
            cfg=improvedObserverConfig();
            cfg.simulation.gpsDropoutInterval=[0,cfg.simulation.finalTime+1];
            cfg.simulation.outOfOrderExtraDelay=0;
            result=simulateImprovedObserverScenario(testCase.ObserverDesign,testCase.LateralDesign,cfg);
            testCase.verifyEqual(nnz(result.estimate.diagnostics.acceptedGps),0);
            testCase.verifyTrue(result.estimate.diagnostics.certificateConditions.timingWithinCertificate);
            testCase.verifyLessThan(result.metrics.positionRmse,.25);
            testCase.verifyLessThan(result.metrics.headingRmse,.01);
        end

        function synthesisProducesAnExhaustivelyCertifiedDesign(testCase)
        % synthesisProducesAnExhaustivelyCertifiedDesign Re-run the SDP.
            testCase.assumeTrue(exist("sdpvar", "file") == 2, ...
                "Improved-observer synthesis needs YALMIP and SeDuMi on the MATLAB path.");
            design = designImprovedObserverGains(improvedObserverConfig());

            testCase.verifyTrue(design.certified);
            testCase.verifyEqual(design.verification.checkedVertexCount, 1441792);
            testCase.verifyLessThan(design.verification.maximumFlowEigenvalue, 0.0);
            testCase.verifyGreaterThan(norm(design.N, "fro"), 0.0);
        end
    end

    methods (Static, Access = private)
        function highRate=stationaryHighRate(time)
            n=numel(time);
            highRate=struct('time',time,'steeringAngle',zeros(n,1), ...
                'longitudinalSpeed',zeros(n,1),'longitudinalAcceleration',zeros(n,1), ...
                'lateralAcceleration',zeros(n,1),'yawRate',zeros(n,1));
        end

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

        function sensorData = stopGoSensorData()
        % stopGoSensorData Build a straight stop--launch--cruise--stop drive.
            sampleTime = 0.01;
            time = (0:sampleTime:6.0).';
            riseFraction = min(max((time - 0.50) ./ 1.50, 0.0), 1.0);
            fallFraction = min(max((time - 4.00) ./ 1.50, 0.0), 1.0);
            rise = 6.0 .* riseFraction.^5 - 15.0 .* riseFraction.^4 + ...
                10.0 .* riseFraction.^3;
            fall = 6.0 .* fallFraction.^5 - 15.0 .* fallFraction.^4 + ...
                10.0 .* fallFraction.^3;
            speed = 8.0 .* rise .* (1.0 - fall);
            sampleCount = numel(time);
            highRate = struct("time", time, ...
                "steeringAngle", zeros(sampleCount, 1), ...
                "longitudinalSpeed", speed, ...
                "longitudinalAcceleration", gradient(speed, sampleTime), ...
                "lateralAcceleration", zeros(sampleCount, 1), ...
                "yawRate", zeros(sampleCount, 1));
            sensorData = struct("highRate", highRate);
        end
    end
end
