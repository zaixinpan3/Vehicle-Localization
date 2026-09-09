classdef improvedObserverTest < matlab.unittest.TestCase
% improvedObserverTest Tests of the complete seven-state observer cascade.
% Algebraic tests cover the nonsingular model, invariant outputs, lidar
% full-matrix gain shaping, and robust timer-LMI construction. Runtime tests use
% stored reference gains; only the explicitly named synthesis test needs a solver.
% Runtime tests cover fixed-delay transport and immutable online state history.

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
            current=lateralObserverConfig();
            testCase.LateralDesign.cfg.hybrid=current.hybrid;
            testCase.ObserverDesign = improvedObserverReferenceDesign();
        end
    end

    methods (Test)
        function delayedYawTransportPreservesTheGyroAngleLift(testCase)
            cfg=improvedObserverConfig();yaw0=pi-.03;
            cfg.observer.initialState=[0;0;0;0;0;0;yaw0];
            data=testCase.zeroMotionSensorData(.5);data.highRate.yawRate(:)=.4;
            stamps=[0;.1;.2];headings=atan2(sin(yaw0+.4*stamps),cos(yaw0+.4*stamps));
            data.lidar=struct('timestamp',stamps,'pose',[zeros(3,2),headings], ...
                'information',repmat(100*eye(3),1,1,3));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            error=atan2(sin(e.heading-yaw0-.4*e.time),cos(e.heading-yaw0-.4*e.time));
            testCase.verifyEqual(error,zeros(size(error)),AbsTol=1e-11);
            testCase.verifyEqual(e.innovations.lidar,zeros(numel(e.time),3),AbsTol=1e-11);
        end

        function delayedCoupledNullspaceCannotChangeTheCorrection(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.4);row=[2,0,1];
            data.lidar=struct('timestamp',0,'pose',[.2,.1,.1], ...
                'information',100*(row.'*row+diag([0,1,0])));
            original=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            data.lidar.pose=data.lidar.pose+[.05,0,-.1];
            shifted=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(original.onlineZ,shifted.onlineZ,AbsTol=1e-11);
        end

        function currentGainIncludesTheNominalTransport(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.2);
            data.lidar=struct('timestamp',0,'pose',[.1,0,0],'information',100*eye(3));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            tau=.15;chain=[1,tau,tau^2/2;0,1,tau;0,0,1];
            expected=blkdiag(chain,chain,1)*diag(cfg.observer.theta.^cfg.observer.scalingExponents) ...
                *testCase.ObserverDesign.K*(100/105);
            sample=find(abs(e.time-.15)<1e-12,1);
            testCase.verifyEqual(e.diagnostics.lidarPoseGain(:,:,sample),expected,AbsTol=1e-10);
        end

        function inputHistoryIsBoundedAndTooShortHistoryIsRejected(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(3);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.diagnostics.integrationStepCount,300);
            testCase.verifyLessThanOrEqual(e.diagnostics.maximumInputHistorySegments,103);
            cfg.measurement.inputHistoryDuration=.14;
            testCase.verifyError(@() runImprovedVehicleObserver(data,testCase.LateralDesign, ...
                testCase.ObserverDesign,cfg),'VehicleLocalization:InputHistoryTooShort');
        end

        function registeredPartialCurbCorrectsStationaryYawEndToEnd(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=[0;0;0;0;0;0;.1];
            cloud=geometricRegistrationTest.parallelRoad();
            registration=registerSemanticProbabilityCloud(cloud,cloud,[.8 .2 .02]);
            event=registrationSupport.registrationPoseMeasurement(registration,0,.15);
            data=testCase.zeroMotionSensorData(.2);data.lidar=event;
            data.gps=struct('timestamp',0,'arrivalTime',.15,'pose',[0 0]);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyFalse(registration.accepted);
            testCase.verifyTrue(e.diagnostics.acceptedLidar);
            testCase.verifyLessThan(abs(e.heading(end)),.1);
            testCase.verifyEqual(event.information*[1;0;0],zeros(3,1),'AbsTol',1e-10);
            testCase.verifyFalse(e.observer.certified);
        end

        function registeredCurbWithoutGpsRetainsLongitudinalAmbiguity(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            cloud=geometricRegistrationTest.parallelRoad();
            registration=registerSemanticProbabilityCloud(cloud,cloud,[.8 0 0]);
            data=testCase.zeroMotionSensorData(.2);
            data.lidar=registrationSupport.registrationPoseMeasurement(registration,0,.15);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            data.lidar.pose(1)=data.lidar.pose(1)+100;
            shifted=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.onlineZ,shifted.onlineZ,'AbsTol',1e-10);
            testCase.verifyFalse(e.diagnostics.certificateConditions.informationWithinCertificate);
        end

        function fixedDelayArrivalDoesNotWaitForTheOutputGrid(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.3);
            data.lidar=struct('timestamp',.003,'arrivalTime',.153,'pose',[.2 0 0], ...
                'information',100*eye(3));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            c=e.diagnostics.certificateConditions;
            testCase.verifyEqual(c.lidarIncorporationTime,.153,'AbsTol',1e-12);
            testCase.verifyEqual(c.lidarArrivalToProcessingWaitSeconds,0,'AbsTol',1e-12);
            testCase.verifyEqual(c.lidarAssimilationDelaySeconds,.15,'AbsTol',1e-12);
            testCase.verifyEqual(c.configuredCausalLagBoundSeconds,.15,'AbsTol',1e-12);
            before=e.time<.153;
            testCase.verifyEqual(e.pose(before,:),zeros(nnz(before),3),'AbsTol',1e-12);
            testCase.verifyGreaterThan(norm(e.pose(find(e.time>.153,1),:)),0);
        end

        function omittedDeliveryUsesTheDeclaredFixedDelay(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            cloud=geometricRegistrationTest.parallelRoad();
            registration=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            data=testCase.zeroMotionSensorData(.2);
            data.lidar=registrationSupport.registrationPoseMeasurement(registration,0);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyFalse(e.diagnostics.certificateConditions.allAcceptedDeliveryTimesProvided);
            data.lidar.arrivalTime=.15;
            delivered=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyTrue(delivered.diagnostics.certificateConditions.allAcceptedDeliveryTimesProvided);
            testCase.verifyEqual(e.onlineZ,delivered.onlineZ,AbsTol=0);
            testCase.verifyEqual(e.measurements.lidar.arrivalTime,.15,AbsTol=1e-12);
        end

        function frameArrivingAfterTheRunRemainsUnincorporated(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.4);
            data.lidar=struct('timestamp',[0;.3],'arrivalTime',[.15;.45], ...
                'pose',zeros(2,3),'information',repmat(100*eye(3),1,1,2));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.diagnostics.acceptedLidar,[true;false]);
            testCase.verifyTrue(isnan(e.measurements.lidar.incorporationTime(2)));
            testCase.verifyFalse(e.diagnostics.stateHistoryRecomputed);
        end

        function transportedPosePreservesExactMovingInitialization(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=[0;10;0;0;0;0;0];
            data=testCase.zeroMotionSensorData(.4);data.highRate.longitudinalSpeed(:)=10;
            data.lidar=struct('timestamp',[0;.1;.2],'pose',[0,0,0;1,0,0;2,0,0], ...
                'information',repmat(100*eye(3),1,1,3));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.position(:,1),10*e.time,'AbsTol',1e-11);
            testCase.verifyEqual(e.innovations.lidar,zeros(numel(e.time),3),'AbsTol',1e-11);
            testCase.verifyEqual(nnz(e.diagnostics.acceptedLidar),3);
            testCase.verifyFalse(e.observer.certificateVerified);
        end

        function offGridTransportIsExactForConstantAcceleration(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=[0;2;1;0;0;0;0];
            data=testCase.zeroMotionSensorData(.6);t=data.highRate.time;
            data.highRate.longitudinalSpeed=2+t;data.highRate.longitudinalAcceleration(:)=1;
            stamps=[.003;.117;.219];
            data.lidar=struct('timestamp',stamps,'pose',[2*stamps+stamps.^2/2,zeros(3,2)], ...
                'information',repmat(100*eye(3),1,1,3));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.position(:,1),2*t+t.^2/2,'AbsTol',1e-11);
            testCase.verifyEqual(e.velocity(:,1),2+t,'AbsTol',1e-11);
            testCase.verifyEqual(e.innovations.lidar,zeros(numel(t),3),'AbsTol',1e-11);
        end

        function partialPoseInitializationIgnoresUnobservedCoordinates(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0;
            cfg.observer.fallbackPosition=[2;3];
            cfg.observer.fallbackHeading=.4;
            data=testCase.zeroMotionSensorData(.1);
            data.lidar=struct('timestamp',0,'arrivalTime',0,'pose',[1000,4,-2], ...
                'information',diag([0,100,0]));
            a=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            data.lidar.pose=[-9000,4,1];
            b=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(a.onlineZ,b.onlineZ,AbsTol=1e-12);
            testCase.verifyEqual(a.pose(1,[1,3]),[2,.4],AbsTol=1e-12);
            testCase.verifyGreaterThan(a.pose(1,2),3);
        end

        function coupledNullspaceDoesNotChangeAutomaticInitialization(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0;data=testCase.zeroMotionSensorData(.1);
            data.lidar=struct('timestamp',0,'arrivalTime',0,'pose',[.1,0,.1], ...
                'information',[100,0,100;0,100,0;100,0,100]);
            a=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            data.lidar.pose=data.lidar.pose+[.2,0,-.2];
            b=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(a.onlineZ,b.onlineZ,AbsTol=1e-11);
        end

        function fullPoseInitializationRetainsGpsPositionPrecedence(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0;data=testCase.zeroMotionSensorData(.1);
            data.lidar=struct('timestamp',0,'arrivalTime',0,'pose',[1,2,.3], ...
                'information',100*eye(3));
            data.gps=struct('timestamp',0,'arrivalTime',0,'pose',[4,5]);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.pose(1,:),[4,5,.3],AbsTol=1e-12);
        end

        function missingInputPrehistoryCannotInitializeFromAnOldPose(testCase)
            cfg=improvedObserverConfig();cfg.observer.fallbackPosition=[2;3];
            cfg.observer.fallbackHeading=.4;data=testCase.zeroMotionSensorData(.1);
            data.lidar=struct('timestamp',[-.2;-.15],'arrivalTime',[-.05;0], ...
                'pose',[1,2,.1;3,4,.2],'information',repmat(100*eye(3),1,1,2));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.pose(1,:),[2,3,.4],AbsTol=1e-12);
            testCase.verifyFalse(any(e.diagnostics.acceptedLidar));
            testCase.verifyEqual(e.measurements.lidar.rejectionReason, ...
                repmat("inputHistoryUnavailable",2,1));
        end

        function curbHeadingInformationRequiresSpatialExtent(testCase)
            cfg=improvedObserverConfig();
            extended=[0,0,0;0,3,0;0,0,2]; % lever arms [-1,0,1]
            point=[0,0,0;0,3,6;0,6,12]; % three lever arms equal to 2
            [~,~,a]=computeLidarInformationWeights(extended,cfg);
            [~,~,b]=computeLidarInformationWeights(point,cfg);
            testCase.verifyEqual(a.marginalizedHeadingInformation,2,AbsTol=1e-12);
            testCase.verifyEqual(b.marginalizedHeadingInformation,0,AbsTol=1e-12);
            testCase.verifyGreaterThan(b.headingInformation,0);
        end

        function tinyTranslationCurvatureDoesNotInventMarginalHeading(testCase)
            cfg=improvedObserverConfig();row=[1e-8,0,1];
            [~,~,info]=computeLidarInformationWeights(row.'*row,cfg);
            testCase.verifyEqual(info.marginalizedHeadingInformation,0,AbsTol=1e-12);
        end

        function stationaryYawNeedsGeometryDespiteGpsPosition(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0;cfg.observer.initialState=[0;0;0;0;0;0;.1];
            data=testCase.zeroMotionSensorData(.5);stamps=(0:.1:.4).';
            data.gps=struct('timestamp',stamps,'arrivalTime',stamps,'pose',zeros(5,2));
            gpsOnly=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            data.lidar=struct('timestamp',stamps,'arrivalTime',stamps,'pose',zeros(5,3), ...
                'information',repmat(diag([0,100,100]),1,1,5));
            geometric=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(gpsOnly.heading,.1*ones(size(gpsOnly.time)),AbsTol=1e-12);
            testCase.verifyEqual(gpsOnly.diagnostics.motionHeadingSensitivity, ...
                zeros(size(gpsOnly.time)),AbsTol=1e-12);
            testCase.verifyEqual(geometric.diagnostics.motionHeadingSensitivity(1),0,AbsTol=1e-12);
            testCase.verifyLessThan(abs(geometric.heading(end)),.1);
        end

        function gpsCompletesTheCurbSectorThroughoutEachPulse(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.7);stamps=(0:.1:.5).';
            data.gps=struct('timestamp',stamps,'arrivalTime',stamps+.15,'pose',zeros(6,2));
            data.lidar=struct('timestamp',stamps,'arrivalTime',stamps+.15,'pose',zeros(6,3), ...
                'information',repmat(diag([0,100,100]),1,1,6));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            c=e.diagnostics.certificateConditions;
            testCase.verifyFalse(c.lidarInformationWithinReferenceSector);
            testCase.verifyTrue(c.informationWithinReferenceSector);
            testCase.verifyTrue(c.timingWithinReferenceSchedule);
            testCase.verifyGreaterThanOrEqual(c.minimumCombinedWeightEigenvalues, ...
                cfg.lidar.certificateMinimumPoseWeight*ones(size(c.minimumCombinedWeightEigenvalues)));
            testCase.verifyFalse(e.observer.certified);
        end

        function gpsExpiryBetweenSamplesCannotHideMissingAnchoring(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.9);stamps=(.1:.1:.6).';
            data.gps=struct('timestamp',stamps-.004,'arrivalTime',stamps+.146,'pose',zeros(6,2));
            data.lidar=struct('timestamp',stamps,'arrivalTime',stamps+.15,'pose',zeros(6,3), ...
                'information',repmat(diag([0,100,100]),1,1,6));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            c=e.diagnostics.certificateConditions;
            bad=c.minimumCombinedWeightEigenvalues<cfg.lidar.certificateMinimumPoseWeight;
            testCase.verifyFalse(c.informationWithinReferenceSector);
            testCase.verifyEqual(sum(diff(c.posePulseInformationIntervals(bad,:),1,2)),.024,AbsTol=1e-12);
        end

        function variableLidarDelayIsRejectedByTheFixedDelayContract(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.7);stamps=[.003;.107;.211;.315];
            data.lidar=struct('timestamp',stamps,'arrivalTime',stamps+[.23;.02;.18;.01], ...
                'pose',zeros(4,3),'information',repmat(100*eye(3),1,1,4));
            testCase.verifyError(@() runImprovedVehicleObserver(data,testCase.LateralDesign, ...
                testCase.ObserverDesign,cfg),'VehicleLocalization:FixedLidarDelayMismatch');
        end

        function varyingMotionLeavesExplicitJerkAndRateResiduals(testCase)
            cfg=improvedObserverConfig();s=4;sdot=-.7;sddot=.3;
            chi=.6;omega=.2;alpha=-.04;rateError=.03;
            t=[cos(chi);sin(chi)];n=[-sin(chi);cos(chi)];J=[0,-1;1,0];
            v=s*t;a=sdot*t+s*omega*n;
            jerk=(sddot-s*omega^2)*t+(2*sdot*omega+s*alpha)*n;
            sample=struct('longitudinalSpeed',s,'lateralVelocity',0, ...
                'longitudinalAcceleration',sdot,'lateralAcceleration',s*omega, ...
                'yawRate',omega-rateError,'sideSlipAngle',0,'sideSlipAngleRate',0);
            channels=evaluateImprovedObserverChannels([0;v(1);a(1);0;v(2);a(2);chi],sample,cfg.operating);
            q=omega-rateError;
            residual=sddot*t+alpha*J*v+(omega^2-q^2)*v+2*(omega-q)*J*a;
            testCase.verifyEqual(jerk-channels.modelDerivative([3,6]),residual,AbsTol=1e-12);
            testCase.verifyEqual(channels.invariantMeasurement(2),s*sdot,AbsTol=1e-12);
            testCase.verifyEqual(channels.invariantMeasurement(3),s^2*omega,AbsTol=1e-12);
        end

        function exactIncrementAcrossClippingFitsTheScaledCoefficientBox(testCase)
            cfg=improvedObserverConfig();data=buildImprovedObserverCertificateData(cfg);
            sample=struct('longitudinalSpeed',2,'lateralVelocity',.1, ...
                'longitudinalAcceleration',.3,'lateralAcceleration',.4, ...
                'yawRate',.1,'sideSlipAngle',.2,'sideSlipAngleRate',0);
            a=[1;30;-20;2;-40;12;3.1];b=[-2;-5;3;4;7;-2;3.2];
            H=testCase.incrementalOutputMatrix(a,b,sample,cfg.operating);
            ha=evaluateImprovedObserverChannels(a,sample,cfg.operating);
            hb=evaluateImprovedObserverChannels(b,sample,cfg.operating);
            scaled=H*data.Tsigma/cfg.observer.sigma^4;
            testCase.verifyEqual(ha.invariantPrediction-hb.invariantPrediction,H*(a-b),AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(abs(scaled),max(abs(data.outputVertices),[],3)+1e-12);
        end

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

        function deficientInformationRetainsObservedDirections(testCase)
            cfg=improvedObserverConfig();
            [translation,heading,info]=computeLidarInformationWeights(diag([225,0,100]),cfg);
            testCase.verifyGreaterThan(translation(1,1),.9);
            testCase.verifyEqual(translation(2,2),0,AbsTol=1e-12);
            testCase.verifyGreaterThan(heading,.9);
            testCase.verifyTrue(info.qualified);
            testCase.verifyEqual(info.rank,2);
        end

        function weakAndStrongDirectionsReceiveDifferentGains(testCase)
            cfg=improvedObserverConfig();
            [translation,heading,info]=computeLidarInformationWeights(diag([.01,100,100]),cfg);
            testCase.verifyLessThan(translation(1,1),.01);
            testCase.verifyGreaterThan(translation(2,2),.9);
            testCase.verifyGreaterThan(heading,.9);
            testCase.verifyTrue(info.qualified);
        end

        function fullInformationCrossTermsChangeTheCorrection(testCase)
            cfg=improvedObserverConfig();
            [~,~,info,W]=computeLidarInformationWeights([1,0,.99;0,1,0;.99,0,1],cfg);
            testCase.verifyTrue(info.qualified);
            testCase.verifyGreaterThan(W(1,3),.1);
            testCase.verifyEqual(W*[1;0;-1],(.01/5.01)*[1;0;-1],AbsTol=1e-12);
        end

        function gainRotatesWithTheInformationPrincipalDirections(testCase)
            cfg=improvedObserverConfig();a=.7;
            U=[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];
            [~,~,~,W]=computeLidarInformationWeights(U*diag([100,.01,50])*U.',cfg);
            testCase.verifyEqual(W*U(:,1),(100/105)*U(:,1),AbsTol=1e-12);
            testCase.verifyEqual(W*U(:,2),(.01/5.01)*U(:,2),AbsTol=1e-12);
        end

        function rankOneInformationCanInjectWithoutFullPoseQualification(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0;cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.1);
            data.lidar=struct('timestamp',0,'arrivalTime',0,'pose',[1,0,0], ...
                'information',diag([100,0,0]));
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(nnz(e.diagnostics.acceptedLidar),1);
            testCase.verifyGreaterThan(e.position(end,1),0);
            testCase.verifyEqual(e.diagnostics.lidarPoseWeight(:,:,1),diag([100/105,0,0]),AbsTol=1e-12);
            testCase.verifyFalse(e.diagnostics.certificateConditions.informationWithinCertificate);
        end

        function informationCrossTermsReachTheActualObserverGain(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0;cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.1);
            data.lidar=struct('timestamp',0,'arrivalTime',0,'pose',[0,0,.1], ...
                'information',[100,0,40;0,100,0;40,0,100]);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyGreaterThan(abs(e.diagnostics.lidarPoseGain(1,3,1)),.1);
            testCase.verifyGreaterThan(abs(e.position(end,1)),1e-4);
        end

        function zeroAndIndefiniteInformationDoNotCreateGain(testCase)
            cfg=improvedObserverConfig();
            [~,~,a,A]=computeLidarInformationWeights(zeros(3),cfg);
            [~,~,b,B]=computeLidarInformationWeights(diag([-1,10,10]),cfg);
            testCase.verifyFalse(a.qualified);testCase.verifyFalse(b.qualified);
            testCase.verifyEqual(A,zeros(3),AbsTol=0);testCase.verifyEqual(B,zeros(3),AbsTol=0);
        end

        function informationFusionPreservesTheHomogeneousSector(testCase)
            cfg=improvedObserverConfig();
            J=[100,0,40;0,20,0;40,0,100];
            [L,G,W]=fusePoseInformationWeights(J,true,cfg);
            testCase.verifyEqual(L+G,W,AbsTol=1e-12);
            testCase.verifyEqual(W,W.',AbsTol=1e-12);
            testCase.verifyGreaterThan(min(eig(W)),.8);
            testCase.verifyLessThan(max(eig(W)),1);
            testCase.verifyEqual(G(:,3),zeros(3,1),AbsTol=0);
        end

        function normalizationPreservesThePhysicalCorrection(testCase)
            cfg=improvedObserverConfig();J=[100,0,40;0,20,0;40,0,100];
            [~,~,~,A]=computeLidarInformationWeights(J,cfg);
            cfg.lidar.poseScales=[2;3;4];D=diag(cfg.lidar.poseScales);
            [~,~,~,B]=computeLidarInformationWeights(D\J/D,cfg);
            testCase.verifyEqual(B,D*A/D,AbsTol=1e-12);
        end

        function wideningTheWeightSectorRequiresNewVerification(testCase)
            cfg=improvedObserverConfig();cfg.lidar.certificateMinimumPoseWeight=.2;
            design=testCase.ObserverDesign;design.timer.alpha=.2;
            v=verifyImprovedObserverDesign(design,cfg);
            testCase.verifyFalse(v.certified);
            testCase.verifyGreaterThan(v.maximumFlowEigenvalue,0);
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
            testCase.verifyEqual(verification.checkedVertexCount, 720896);
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
            testCase.verifyFalse(metrics.stateHistoryRecomputed);
            testCase.verifyGreaterThan(metrics.acceptedEventCount, 0);
            testCase.verifyEqual(metrics.rejectedEventCount, 0);
            testCase.verifyLessThan(metrics.degenerateLidarMinimumWeight,.05);
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
            cfg = improvedObserverConfig();cfg.measurement.fixedLidarDelay=0;
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

        function eventOlderThanInputHistoryIsRejected(testCase)
        % eventOlderThanInputHistoryIsRejected Check the bounded input-map buffer.
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

        function fixedDelayNeverRewritesPastStateHistory(testCase)
            cfg=improvedObserverConfig();cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.6);
            data.lidar=struct('timestamp',[0;.1;.2;.3],'arrivalTime',[.15;.25;.35;.45], ...
                'pose',repmat([.2,-.1,.01],4,1),'information',repmat(100*eye(3),1,1,4));
            prefix=data;prefix.highRate=structfun(@(v) v(1:21,:),data.highRate,'UniformOutput',false);
            short=runImprovedVehicleObserver(prefix,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            full=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(full.z(1:21,:),short.z,AbsTol=0);
            testCase.verifyEqual(full.z,full.onlineZ,AbsTol=0);
            testCase.verifyFalse(isfield(full,'revisedZ'));
            testCase.verifyFalse(full.diagnostics.stateHistoryRecomputed);
        end

        function irregularPulseEdgesAgreeWithAFinerInputGrid(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0; cfg.observer.initialState=zeros(7,1);
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
            testCase.verifyTrue(e.diagnostics.certificateConditions.timingWithinReferenceSchedule);
            testCase.verifyTrue(e.diagnostics.certificateConditions.fixedDelayMatchesConfiguration);
            testCase.verifyFalse(e.observer.certificateVerified);
            testCase.verifyTrue(e.observer.referenceCertificateVerified);
            testCase.verifyFalse(e.observer.verification.certified);
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

        function changedTimerAndConfigCannotReuseAVerification(testCase)
            cfg=improvedObserverConfig();cfg.measurement.lidarMaximumAge=.02;
            cfg.measurement.gpsMaximumAge=.02;
            design=testCase.ObserverDesign;design.timer.onTime=.02;
            data=testCase.zeroMotionSensorData(.05);
            testCase.verifyError(@() runImprovedVehicleObserver(data, ...
                testCase.LateralDesign,design,cfg),'VehicleLocalization:CertificateMismatch');
        end

        function gpsInAPoseGapDoesNotAddAnUncertifiedGain(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0; cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.09);
            data.lidar=struct('timestamp',0,'arrivalTime',0, ...
                'pose',[0,0,0],'information',100*eye(3));
            data.gps=struct('timestamp',.05,'arrivalTime',.05,'pose',[1000,-1000]);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.z,zeros(size(e.z)),AbsTol=1e-12);
            testCase.verifyEqual(nnz(e.diagnostics.acceptedGps),1);
        end

        function gpsAndLidarHaveSeparateGainsWithinThePosePulse(testCase)
            cfg=improvedObserverConfig();cfg.measurement.fixedLidarDelay=0; cfg.observer.initialState=zeros(7,1);
            data=testCase.zeroMotionSensorData(.03);
            data.lidar=struct('timestamp',0,'arrivalTime',0, ...
                'pose',[0,0,0],'information',100*eye(3));
            data.gps=struct('timestamp',0,'arrivalTime',0,'pose',[1,0]);
            e=runImprovedVehicleObserver(data,testCase.LateralDesign,testCase.ObserverDesign,cfg);
            testCase.verifyEqual(e.diagnostics.gpsPoseWeight(1,1,1),25/130,AbsTol=1e-12);
            testCase.verifyEqual(e.diagnostics.lidarPoseWeight(1,1,1),100/130,AbsTol=1e-12);
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

        function lidarOnlyTracksOnTheReferenceSchedule(testCase)
            cfg=improvedObserverConfig();
            cfg.simulation.gpsDropoutInterval=[0,cfg.simulation.finalTime+1];
            cfg.simulation.outOfOrderExtraDelay=0;
            result=simulateImprovedObserverScenario(testCase.ObserverDesign,testCase.LateralDesign,cfg);
            testCase.verifyEqual(nnz(result.estimate.diagnostics.acceptedGps),0);
            testCase.verifyTrue(result.estimate.diagnostics.certificateConditions.timingWithinReferenceSchedule);
            testCase.verifyLessThan(result.metrics.positionRmse,.25);
            testCase.verifyLessThan(result.metrics.headingRmse,.01);
        end

        function synthesisProducesAnExhaustivelyCertifiedDesign(testCase)
        % synthesisProducesAnExhaustivelyCertifiedDesign Re-run the SDP.
            testCase.assumeTrue(exist("sdpvar", "file") == 2, ...
                "Improved-observer synthesis needs YALMIP and SeDuMi on the MATLAB path.");
            design = designImprovedObserverGains(improvedObserverConfig());

            testCase.verifyTrue(design.certified);
            testCase.verifyEqual(design.verification.checkedVertexCount, 720896);
            testCase.verifyLessThan(design.verification.maximumFlowEigenvalue, 0.0);
            testCase.verifyGreaterThan(norm(design.N, "fro"), 0.0);
        end
    end

    methods (Static, Access = private)
        function H=incrementalOutputMatrix(a,b,sample,operating)
        % incrementalOutputMatrix Form an exact coordinate telescoping secant.
            H=zeros(4,7);previous=a;
            for k=1:7
                next=previous;next(k)=b(k);
                hp=evaluateImprovedObserverChannels(previous,sample,operating);
                hn=evaluateImprovedObserverChannels(next,sample,operating);
                if a(k)~=b(k)
                    H(:,k)=(hp.invariantPrediction-hn.invariantPrediction)/(a(k)-b(k));
                end
                previous=next;
            end
        end

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
