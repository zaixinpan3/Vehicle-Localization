classdef improvedObserverTest < matlab.unittest.TestCase
% improvedObserverTest Behavioral checks of the continuous ODE/DDE contracts.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function continuousGnssPreservesMovingTruth(testCase)
            f=fixture("gnss",.5);
            result=runFixture(f);
            testCase.verifyEqual(result.z(:,1),8*f.data.highRate.time,AbsTol=1e-10);
            testCase.verifyLessThan(max(abs(result.innovations.pose),[],'all'),1e-10);
            testCase.verifyEqual(result.observer.mode,"gnss");
        end
        function continuousLidarPreservesDelayedMovingTruth(testCase)
            f=fixture("lidar",.6);
            result=runFixture(f);
            testCase.verifyEqual(result.position,[8*f.data.highRate.time,zeros(size(f.data.highRate.time))],AbsTol=1e-10);
            testCase.verifyLessThan(max(abs(result.innovations.pose),[],'all'),1e-10);
            testCase.verifyEqual(result.diagnostics.delayedState(:,1),8*(f.data.highRate.time-.15),AbsTol=1e-10);
        end
        function delayedFeedbackUsesTheActualInitialHistory(testCase)
            f=fixture("lidar",.1);f=stationary(f);
            f.cfg.observer.initialState(7)=1;
            f.history=@(t) [zeros(6,1);1+2*t];
            result=runFixture(f);t=result.time;weight=1e6/(1e6+5);
            expected=1-weight*((1-2*.15)*t+t.^2);
            testCase.verifyEqual(result.headingUnwrapped,expected,AbsTol=1e-11);
        end
        function delayedYawMatchesIndependentDde23(testCase)
            f=fixture("lidar",3);f=stationary(f);
            f.cfg.observer.initialState(7)=.2;f.history=@(~) [zeros(6,1);.2];
            result=runFixture(f);weight=1e6/(1e6+5);
            reference=dde23(@(~,~,past) -weight*past,.15,.2,[0,3], ...
                ddeset('RelTol',1e-10,'AbsTol',1e-12));
            testCase.verifyEqual(result.headingUnwrapped,deval(reference,result.time).',AbsTol=2e-7);
        end
        function gnssYawMatchesTheContinuousNonlinearSolution(testCase)
            f=fixture("gnss",1);f.cfg.observer.initialState(7)=.2;
            result=runFixture(f);
            expected=2*atan(tan(.1)*exp(-.5*8*result.time));
            testCase.verifyEqual(result.headingUnwrapped,expected,AbsTol=1e-9);
            testCase.verifyEqual(result.position(:,1),8*result.time,AbsTol=1e-10);
        end
        function standstillGnssDoesNotInventHeadingObservability(testCase)
            f=fixture("gnss",.5);f=stationary(f);f.cfg.observer.initialState(7)=.3;
            result=runFixture(f);
            testCase.verifyEqual(result.headingUnwrapped,.3*ones(size(result.time)),AbsTol=1e-12);
            testCase.verifyFalse(result.diagnostics.certificateConditions.conditionalCertificateApplicable);
            testCase.verifyFalse(result.observer.certified);
        end
        function lidarRequiresInitialHistory(testCase)
            f=fixture("lidar",.2);f.history=[];
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InitialHistoryRequired');
        end
        function lidarRejectsDiscontinuousInitialHistory(testCase)
            f=fixture("lidar",.2);f.history=@(t) straight(t)+ones(7,1);
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InitialHistoryMismatch');
        end
        function lidarRejectsInvalidInitialHistory(testCase)
            f=fixture("lidar",.2);f.history=@(~) NaN(7,1);
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InvalidInitialHistory');
        end
        function delayedLidarCannotInitializeFromAnOldPose(testCase)
            f=fixture("lidar",.2);f.cfg.observer.initialState=[];
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InitialStateRequired');
        end
        function gnssCanInitializeFromCurrentPositionAndDeclaredYaw(testCase)
            f=fixture("gnss",.2);f.cfg.observer.initialState=[];
            result=runFixture(f);
            testCase.verifyEqual(result.z(1,:),[0,8,0,0,0,0,0],AbsTol=1e-12);
        end
        function rejectArrivalMetadata(testCase)
            f=fixture("lidar",.2);f.data.lidar.arrivalTime=0;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:EventInputUnsupported');
        end
        function rejectMixedModes(testCase)
            f=fixture("gnss",.2);f.data.lidar=struct('evaluate',@lidarStraight);
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:MixedMeasurementModes');
        end
        function rejectUnsupportedMeasurementConfiguration(testCase)
            f=fixture("lidar",.2);f.cfg.measurement.unsupportedOption=1;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InvalidConfiguration');
        end
        function rejectMissingContinuousMeasurement(testCase)
            f=fixture("gnss",.2);f.data=rmfield(f.data,'gnss');
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:ContinuousMeasurementRequired');
        end
        function rejectWrongDeclaredDelay(testCase)
            f=fixture("lidar",.2);f.data.lidar.delay=.1;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:FixedLidarDelayMismatch');
        end
        function requireExplicitHeadingLift(testCase)
            f=fixture("lidar",.2);f.data.lidar=rmfield(f.data.lidar,'headingConvention');
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:HeadingLiftRequired');
        end
        function rejectDirectionalDegeneracy(testCase)
            f=fixture("lidar",.2);f.data.lidar.evaluate=@(~) struct('pose',zeros(3,1),'information',diag([1e6,0,1e6]));
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InsufficientLidarInformation');
        end
        function rejectFullRankBelowTheUniformBound(testCase)
            f=fixture("lidar",.2);f.data.lidar.evaluate=@(~) struct('pose',zeros(3,1),'information',eye(3));
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InsufficientLidarInformation');
        end
        function rejectMissingMeasurementDuringIntegration(testCase)
            f=fixture("gnss",.5);f.data.gnss.evaluate=@missingLater;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InvalidContinuousSignal');
        end
        function fullInformationCrossTermsAreRetained(testCase)
            cfg=improvedObserverConfig;rotation=[cos(.7),0,-sin(.7);0,1,0;sin(.7),0,cos(.7)];
            info=rotation*diag([1e4,2e4,1e6])*rotation.';
            [~,~,details,W]=computeLidarInformationWeights(info,cfg);
            expected=rotation*diag([1e4/10005,2e4/20005,1e6/1000005])*rotation.';
            testCase.verifyTrue(details.qualified);
            testCase.verifyEqual(W,expected,AbsTol=1e-12);
            testCase.verifyGreaterThan(abs(W(1,3)),1e-4);
        end
        function weightsUseNormalizedPoseUnits(testCase)
            cfg=improvedObserverConfig;cfg.lidar.poseScales=[2;.5;3];S=diag(cfg.lidar.poseScales);
            [~,~,~,W]=computeLidarInformationWeights(S\diag([1e4,2e4,3e4])/S,cfg);
            testCase.verifyEqual(W,diag([1e4/10005,2e4/20005,3e4/30005]),AbsTol=1e-12);
        end
        function asymmetricInformationIsRejected(testCase)
            cfg=improvedObserverConfig;
            [~,~,info]=computeLidarInformationWeights([1e6,100,0;0,1e6,0;0,0,1e6],cfg);
            testCase.verifyFalse(info.qualified);
            testCase.verifyEqual(info.reason,"asymmetricInformation");
        end
        function arrayReconstructionMustBeDeclared(testCase)
            f=fixture("gnss",.2);t=f.data.highRate.time;
            f.data.gnss=struct('time',t,'position',[8*t,zeros(size(t))]);
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InvalidContinuousSignal');
        end
        function linearArrayAndFunctionSignalsAgree(testCase)
            f=fixture("gnss",.5);t=f.data.highRate.time;expected=runFixture(f);
            f.data.gnss=struct('time',t,'position',[8*t,zeros(size(t))],'representation',"piecewiseLinear");
            actual=runFixture(f);
            testCase.verifyEqual(actual.z,expected.z,AbsTol=1e-10);
        end
        function sourceArrayGapsAreNotSilentlyBridged(testCase)
            f=fixture("gnss",.2);f.data.gnss=struct('time',[0;.2], ...
                'position',[0,0;1.6,0],'representation',"piecewiseLinear");
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InvalidContinuousSignal');
        end
        function historyMemoryRemainsBounded(testCase)
            f=fixture("lidar",2);result=runFixture(f);
            testCase.verifyLessThanOrEqual(result.diagnostics.maximumHistoryNodes,34);
            testCase.verifyFalse(result.diagnostics.stateHistoryRecomputed);
        end
        function resultsDoNotChangeWhenTheRunIsExtended(testCase)
            short=runFixture(fixture("lidar",.3));long=runFixture(fixture("lidar",.8));
            testCase.verifyEqual(short.z,long.z(1:numel(short.time),:),AbsTol=1e-12);
        end
        function nonGridDelayStillPreservesZeroError(testCase)
            f=fixture("lidar",.5);f.cfg.measurement.fixedLidarDelay=.137;
            f.design=improvedObserverReferenceDesign(f.cfg);f.data.lidar.delay=.137;
            f.data.lidar.evaluate=@(t) struct('pose',[8*(t-.137);0;0],'information',1e6*eye(3));
            result=runFixture(f);
            testCase.verifyEqual(result.z(:,1),8*result.time,AbsTol=1e-10);
        end
        function delaySmallerThanStepUsesMethodOfSteps(testCase)
            f=fixture("lidar",.05);f.cfg.measurement.fixedLidarDelay=.001;
            f.design=improvedObserverReferenceDesign(f.cfg);f.data.lidar.delay=.001;
            f.data.lidar.evaluate=@(t) struct('pose',[8*(t-.001);0;0],'information',1e6*eye(3));
            result=runFixture(f);
            testCase.verifyGreaterThanOrEqual(result.diagnostics.integrationStepCount,50);
            testCase.verifyEqual(result.z(:,1),8*result.time,AbsTol=1e-10);
        end
        function zeroDelayReducesToCurrentCorrection(testCase)
            f=fixture("lidar",.1);f.cfg.measurement.fixedLidarDelay=0;
            f.design=improvedObserverReferenceDesign(f.cfg);f.data.lidar.delay=0;f.history=[];
            f.data.lidar.evaluate=@(t) struct('pose',[8*t;0;0],'information',1e6*eye(3));
            result=runFixture(f);
            testCase.verifyEqual(result.z(:,1),8*result.time,AbsTol=1e-10);
            testCase.verifyEqual(result.diagnostics.maximumHistoryNodes,0);
        end
        function yawLiftRemainsContinuousAcrossPi(testCase)
            f=fixture("lidar",.5);f=stationary(f);f.data.highRate.yawRate(:)=.001;
            f.cfg.observer.initialState(7)=pi-.0002;
            f.history=@(t) [zeros(6,1);pi-.0002+.001*t];
            f.data.lidar.evaluate=@(t) struct('pose',[0;0;pi-.0002+.001*(t-.15)],'information',1e6*eye(3));
            result=runFixture(f);
            testCase.verifyEqual(result.headingUnwrapped,pi-.0002+.001*result.time,AbsTol=1e-11);
            testCase.verifyLessThan(result.heading(end),0);
            testCase.verifyGreaterThan(result.z(end,7),pi);
        end
        function changedMatricesCannotReuseACertifiedFlag(testCase)
            f=fixture("lidar",.2);f.design.P=-f.design.P;f.design.certified=true;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:CertificateMismatch');
        end
        function changedThetaRequiresACorrespondingCertificate(testCase)
            f=fixture("gnss",.2);f.cfg.observer.theta=21;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:CertificateMismatch');
        end
        function rateExcursionsAreReportedWithoutClamping(testCase)
            f=fixture("lidar",.2);f.data.highRate.yawRate(:)=.1;
            result=runFixture(f);
            testCase.verifyEqual(result.trackAngleRate,.1*ones(size(result.time)),AbsTol=1e-12);
            testCase.verifyFalse(result.diagnostics.certificateConditions.conditionalCertificateApplicable);
            testCase.verifyFalse(result.observer.certified);
        end
        function gnssGainCannotFeedHeadingBackIntoPosition(testCase)
            f=fixture("gnss",.2);f.design.N(1,4)=1;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:CertificateMismatch');
        end
        function providedLateralClockMustMatch(testCase)
            f=fixture("lidar",.2);f.lateral.time=f.lateral.time+.001;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InvalidLateralInputs');
        end
        function providedLateralValuesMustBeFinite(testCase)
            f=fixture("gnss",.2);f.lateral.sideSlipAngleRate(3)=NaN;
            testCase.verifyError(@() runFixture(f),'VehicleLocalization:InvalidLateralInputs');
        end
        function defaultLateralStageMatchesItsSuppliedOutput(testCase)
            f=fixture("gnss",.2);loaded=load(fullfile(fileparts(mfilename('fullpath')),'reference','lateralObserverDesign.mat'));
            cfg=lateralObserverConfig;loaded.design.cfg.hybrid=cfg.hybrid;
            actual=runImprovedVehicleObserver(f.data,loaded.design,f.design,f.cfg);
            f.lateral=actual.lateral;expected=runFixture(f);
            testCase.verifyEqual(actual.z,expected.z,AbsTol=1e-12);
        end
        function variableMotionRetainsItsExplicitModelResidual(testCase)
            v=4;vd=-.7;vdd=.3;angle=.6;q=.2;qd=-.04;used=.17;
            direction=[cos(angle);sin(angle)];J=[0,-1;1,0];velocity=v*direction;
            acceleration=vd*direction+v*q*J*direction;
            jerk=(vdd-v*q^2)*direction+(2*vd*q+v*qd)*J*direction;
            sample=struct('longitudinalSpeed',v,'lateralVelocity',0, ...
                'longitudinalAcceleration',vd,'lateralAcceleration',v*q,'yawRate',used, ...
                'sideSlipAngle',0,'sideSlipAngleRate',0);
            z=[0;velocity(1);acceleration(1);0;velocity(2);acceleration(2);angle];
            channels=evaluateImprovedObserverChannels(z,sample);
            residual=vdd*direction+qd*J*velocity+(q^2-used^2)*velocity+2*(q-used)*J*acceleration;
            testCase.verifyEqual(jerk-channels.modelDerivative([3,6]),residual,AbsTol=1e-12);
        end
        function reconstructedGnssSamplesFollowPhysicalTime(testCase)
            f=fixture("gnss",.2);t=f.data.highRate.time;
            [data,meta]=reconstructContinuousObserverSignals(f.data.highRate,t,[8*t,zeros(size(t))],[],f.cfg);
            f.data=data;result=runFixture(f);
            testCase.verifyEqual(result.position(:,1),8*result.time,AbsTol=1e-10);
            testCase.verifyFalse(meta.physicalContinuityVerified);
        end
        function reconstructedLidarAppliesExactlyOneFixedDelay(testCase)
            f=fixture("lidar",.4);t=(-.2:.01:.4).';
            [data,meta]=reconstructContinuousObserverSignals(f.data.highRate,t,[8*t,zeros(numel(t),2)], ...
                repmat(1e6*eye(3),1,1,numel(t)),f.cfg);
            f.data=data;result=runFixture(f);
            testCase.verifyEqual(result.position(:,1),8*result.time,AbsTol=1e-10);
            testCase.verifyEqual(meta.fixedDelaySeconds,.15,AbsTol=1e-12);
        end
        function reconstructionRejectsOriginalPoseOutages(testCase)
            f=fixture("gnss",.4);
            testCase.verifyError(@() reconstructContinuousObserverSignals(f.data.highRate,[0;.4], ...
                [0,0;3.2,0],[],f.cfg),'VehicleLocalization:ReconstructionGap');
        end
        function reconstructionRejectsInsufficientLidarInformation(testCase)
            f=fixture("lidar",.4);t=(-.2:.1:.4).';
            testCase.verifyError(@() reconstructContinuousObserverSignals(f.data.highRate,t, ...
                zeros(numel(t),3),repmat(eye(3),1,1,numel(t)),f.cfg), ...
                'VehicleLocalization:InsufficientLidarInformation');
        end
        function delayRefinementWithIncompatibleHistoryConverges(testCase)
            f=fixture("lidar",.5);f=stationary(f);f.cfg.measurement.fixedLidarDelay=.137;
            f.design=improvedObserverReferenceDesign(f.cfg);f.data.lidar.delay=.137;
            f.cfg.observer.initialState(7)=.2;f.history=@(~) [zeros(6,1);.2];
            coarse=runFixture(f);f.cfg.measurement.maximumIntegrationStep=.0025;fine=runFixture(f);
            testCase.verifyEqual(coarse.headingUnwrapped,fine.headingUnwrapped,AbsTol=1e-8);
        end
        function gnssSynthesisVerifiesItsActualMatrices(testCase)
            cfg=improvedObserverConfig("gnss");design=designImprovedObserverGains(cfg);
            verified=verifyImprovedObserverDesign(design,cfg);
            testCase.verifyTrue(verified.certified);
            testCase.verifyGreaterThan(verified.uniformMargin,0);
        end
    end
end

function f=fixture(mode,duration)
    f.cfg=improvedObserverConfig(mode);f.design=improvedObserverReferenceDesign(f.cfg);
    t=(0:.01:duration).';n=numel(t);
    f.data.highRate=struct('time',t,'steeringAngle',zeros(n,1),'longitudinalSpeed',8*ones(n,1), ...
        'longitudinalAcceleration',zeros(n,1),'lateralAcceleration',zeros(n,1),'yawRate',zeros(n,1));
    f.lateral=struct('time',t,'lateralVelocity',zeros(n,1),'sideSlipAngle',zeros(n,1),'sideSlipAngleRate',zeros(n,1));
    f.cfg.observer.initialState=straight(0);f.history=@straight;
    if mode=="gnss"
        f.data.gnss=struct('evaluate',@(s) [8*s;0]);
    else
        f.data.lidar=struct('evaluate',@lidarStraight,'delay',.15,'headingConvention',"unwrapped");
    end
end

function f=stationary(f)
    f.data.highRate.longitudinalSpeed(:)=0;f.cfg.observer.initialState=zeros(7,1);
    f.history=@(~) zeros(7,1);
    if f.cfg.mode=="gnss",f.data.gnss.evaluate=@(~) zeros(2,1);
    else,f.data.lidar.evaluate=@(~) struct('pose',zeros(3,1),'information',1e6*eye(3));end
end

function z=straight(t)
    z=[8*t;8;0;0;0;0;0];
end

function value=lidarStraight(t)
    value=struct('pose',[8*(t-.15);0;0],'information',1e6*eye(3));
end

function value=missingLater(t)
    value=[8*t;0];if t>.25,value(:)=NaN;end
end

function result=runFixture(f)
    result=runImprovedVehicleObserver(f.data,struct(),f.design,f.cfg, ...
        LateralInputs=f.lateral,InitialHistory=f.history);
end
