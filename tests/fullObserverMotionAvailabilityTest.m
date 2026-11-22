classdef (SharedTestFixtures={freshObserverGainsFixture}) fullObserverMotionAvailabilityTest < matlab.unittest.TestCase
% Missing motion outputs withdraw feedback rather than supply zero observations.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function rawGyroIsNotAGlobalInput(testCase)
            [d,l,c]=fixture();l.yawRate(:)=.12;
            a=execute(d,l,c);
            d.highRate.yawRate(:)=NaN;d.highRate.yawRateValid=false(size(l.time));
            b=execute(d,l,c);d.highRate=rmfield(d.highRate,{'yawRate','yawRateValid'});
            omitted=execute(d,l,c);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
            testCase.verifyEqual(a.internalState,omitted.internalState,AbsTol=0);
            testCase.verifyEqual(a.diagnostics.yawRateFeedbackSource,"upstreamMotionPacket");
        end
        function missingAndNonfiniteUpstreamRateHoldTheInitialRate(testCase)
            [d,l,c]=fixture();c.initialState(8)=.04;l.yawRate(:)=NaN;
            d.highRate.yawRate(:)=100;a=execute(d,l,c);
            l=rmfield(l,'yawRate');b=execute(d,l,c);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
            testCase.verifyFalse(isfield(a.diagnostics.motionOutputValidity,'yawRate'));
            testCase.verifyEqual(a.yawRate,.04*ones(size(l.time)),AbsTol=0);
            testCase.verifyFalse(any(a.diagnostics.upstreamYawRateValid));
        end
        function upstreamRateHasItsOwnAvailabilityAndResumesAfterAGap(testCase)
            [d,l,c]=fixture();
            t=[0;.03;.14;.23;.55;.81;1;1.05;1.2;1.6;2];
            d.highRate.time=t;l.time=t;l.yawRate(:)=.2;l.velocityValid=false(size(t));
            l.yawRateValid=true(size(t));l.yawRateValid(4:6)=false;l.yawRate(5)=1e6;
            l.yawRate(7)=NaN;
            a=execute(d,l,c);
            valid=l.yawRateValid & isfinite(l.yawRate);
            gain=c.invariantDesign.verification.yawRateGainPerSecond;
            expected=.2*(1-cumprod(1./(1+gain*[0;diff(t)].*valid)));
            testCase.verifyEqual(a.yawRate,expected,AbsTol=1e-12);
            testCase.verifyEqual(a.diagnostics.upstreamYawRateValid,valid);
            testCase.verifyEqual(a.diagnostics.packetCounts.upstreamYawRateValid,nnz(valid));
            testCase.verifyTrue(a.diagnostics.auxiliaryAvailabilityContractSatisfied);
        end
        function rateValiditySurvivesSynchronization(testCase)
            [d,l,c]=fixture();l.yawRate=.1+l.time;l.yawRateValid=true(size(l.time));l.yawRateValid(2)=false;
            d.lidar=struct('time',l.time,'pose',zeros(numel(l.time),3),'valid',true(size(l.time)), ...
                'information',repmat(eye(3),1,1,numel(l.time)),'delay',0);
            [~,out]=syntheticObserver.align(d,l,c);
            testCase.verifyEqual(out.yawRate,l.yawRate,AbsTol=0);
            testCase.verifyEqual(out.yawRateValid,l.yawRateValid);
            d.lidar.time=[0;.05;.1;.15;.2];
            c.synchronization.motionMaximumBracket=.11;
            d.lidar.pose=zeros(5,3);d.lidar.valid=true(5,1);d.lidar.information=repmat(eye(3),1,1,5);
            [~,out]=syntheticObserver.align(d,l,c);
            testCase.verifyEqual(out.yawRateValid,[true;false;false;false;true]);
        end
        function defaultMasksPreserveTheExistingRuntime(testCase)
            [d,l,c]=fixture();a=execute(d,l,c);
            d.highRate.bodyVelocityValid=true(size(l.time));
            d.highRate.bodyAccelerationValid=true(size(l.time));b=execute(d,l,c);
            testCase.verifyEqual(a,b);
        end
        function lowSpeedMotionUsesTheOrdinaryInnovations(testCase)
            [d,l,c]=fixture();n=numel(l.time);c.initialState=[0;1;0;0;0;0;0;0];
            d.highRate.longitudinalSpeed(:)=0;l.lateralVelocity(:)=0;l.velocityValid=true(n,1);
            l.yawRate(:)=.12;l.yawRateValid=true(n,1);
            e=execute(d,l,c);
            testCase.verifyTrue(all(e.diagnostics.upstreamYawRateValid));
            testCase.verifyTrue(all(e.diagnostics.allAuxiliaryOutputsAvailable));
            testCase.verifyGreaterThan(e.velocity(2,1),0);
            testCase.verifyGreaterThan(e.yawRate(2),0);
            testCase.verifyLessThan(e.yawRate(2),.12);
            expected=localizationSupport.predict(c.initialState,l.time(2)-l.time(1),c);
            testCase.verifyEqual(e.diagnostics.intrinsicPrediction(2,:).',expected,AbsTol=1e-12);
            d.highRate.rollingValid=true(n,1);d.highRate.rollingVelocity=zeros(n,2);
            testCase.verifyError(@()execute(d,l,c),'VehicleLocalization:RetiredRollingPrediction');
        end
        function lowSpeedPacketHonorsTheSpeedBoundaryAndSensorValidity(testCase)
            t=(0:5).';u=[0;-.5;1;2;.4;.1];r=[.1;.2;.3;.4;NaN;.8];
            dynamic=[false;false;true;true;false;false];
            l=struct('time',t,'longitudinalSpeed',abs(u),'lateralVelocity',ones(6,1), ...
                'yawRate',zeros(6,1),'velocityValid',dynamic,'yawRateValid',dynamic, ...
                'minimumDesignSpeedMps',1,'measurements',struct('dynamicValid',[true(5,1);false]));
            m=globalObserverMotionMeasurements(l,u,r);
            testCase.verifyEqual(m.lowSpeedMeasurement,logical([1;1;0;0;1;0]));
            testCase.verifyEqual(m.longitudinalSpeed(2),-.5);
            testCase.verifyEqual(m.lateralVelocity,[0;0;1;1;0;1]);
            testCase.verifyEqual(m.yawRateValid,logical([1;1;1;1;0;0]));
            testCase.verifyEqual(m.yawRate(1:4),[.1;.2;0;0]);
            testCase.verifyEqual(m.velocityValid,logical([1;1;1;1;1;0]));
            l.measurements.yawRateValid=[false;true;false;true;true;true];
            m=globalObserverMotionMeasurements(l,u,r);
            testCase.verifyEqual(m.yawRateValid,logical([0;1;1;1;0;0]));
        end
        function absentOutputsPreserveFreePrediction(testCase)
            [d,l,c]=fixture();n=numel(l.time);
            d.highRate.bodyVelocityValid=false(n,1);d.highRate.bodyAccelerationValid=false(n,1);
            d.highRate.longitudinalSpeed(:)=NaN;l.lateralVelocity(:)=NaN;
            d.highRate.longitudinalAcceleration(:)=NaN;d.highRate.lateralAcceleration(:)=NaN;
            a=execute(d,l,c);t=l.time;
            expectedVelocity=3+.2*t;expectedPosition=[0;cumsum(diff(t).*expectedVelocity(2:end))];
            testCase.verifyEqual(a.velocity(:,1),expectedVelocity,AbsTol=1e-12);
            testCase.verifyEqual(a.position(:,1),expectedPosition,AbsTol=1e-12);
            testCase.verifyEqual(a.diagnostics.packetCounts.bodyVelocityValid,0);
            testCase.verifyFalse(a.diagnostics.observedMotionEnvelopeSatisfied);
            testCase.verifyTrue(a.diagnostics.auxiliaryAvailabilityContractSatisfied);
            testCase.verifyFalse(any(a.diagnostics.allAuxiliaryOutputsAvailable));
        end
        function withdrawnPayloadCannotInfluenceAnyState(testCase)
            [d,l,c]=fixture();n=numel(l.time);
            d.highRate.bodyVelocityValid=false(n,1);d.highRate.bodyAccelerationValid=false(n,1);
            a=execute(d,l,c);
            d.highRate.longitudinalSpeed(:)=1e8;l.lateralVelocity(:)=NaN;
            d.highRate.longitudinalAcceleration(:)=NaN;d.highRate.lateralAcceleration(:)=-1e8;
            b=execute(d,l,c);testCase.verifyEqual(a.z,b.z);
        end
        function availableNanStillFailsValidation(testCase)
            [d,l,c]=fixture();d.highRate.longitudinalSpeed(2)=NaN;
            testCase.verifyError(@()execute(d,l,c),'VehicleLocalization:InvalidFullInput');
        end
        function invalidLpvOutputCannotReachPredictionOrInnovations(testCase)
            [d,l,c]=fixture();l.velocityValid=false(size(l.time));
            a=execute(d,l,c);
            l.lateralVelocity(:)=NaN;l.sideSlipAngleRate(:)=NaN;
            b=execute(d,l,c);
            testCase.verifyEqual(a.internalState,b.internalState);
            testCase.verifyEqual(b.diagnostics.packetCounts.bodyVelocityValid,0);
            for k=1:numel(l.time)
                testCase.verifyEqual(b.diagnostics.invariantOutputs{k}.valid, ...
                    false(4,1));
            end
        end
        function invalidLateralMaskIsRejected(testCase)
            [d,l,c]=fixture();l.velocityValid=NaN(size(l.time));
            testCase.verifyError(@()execute(d,l,c),'VehicleLocalization:InvalidMotionValidity');
        end
        function accelerationCannotActAloneWithoutVelocity(testCase)
            [d,l,c]=fixture();l.velocityValid=false(size(l.time));a=execute(d,l,c);
            d.highRate.longitudinalAcceleration(:)=20;d.highRate.lateralAcceleration(:)=-30;
            b=execute(d,l,c);
            testCase.verifyEqual(a.internalState,b.internalState,AbsTol=0);
            testCase.verifyFalse(any(b.diagnostics.motionOutputValidity.bodyAcceleration));
            testCase.verifyEqual(b.diagnostics.invariantOutputNames, ...
                ["speedSquared","velocityAccelerationDot","velocityAccelerationCross","bodyLateralVelocity"]);
        end
        function validityRequiresOneBooleanPerFrame(testCase)
            [d,l,c]=fixture();d.highRate.bodyVelocityValid=[true;false];
            testCase.verifyError(@()execute(d,l,c),'VehicleLocalization:InvalidMotionValidity');
        end
    end
end

function [d,l,c]=fixture()
    t=(0:.1:1).';zero=zeros(size(t));
    d=struct('highRate',struct('time',t,'longitudinalSpeed',8+zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero));
    l=struct('time',t,'lateralVelocity',zero,'yawRate',zero,'sideSlipAngleRate',zero);
    c=freshObserverTestConfig();
    c.initialState=[0;3;.2;0;0;0;0];
end

function r=execute(d,l,c)
    r=runFullLocalizationObserver(d,c,LateralInputs=l);
end
