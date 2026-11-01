classdef (SharedTestFixtures={freshObserverGainsFixture}) motionResponseTest < matlab.unittest.TestCase
% Performance and certificate regressions for bounded matched-gain scheduling.
    properties (TestParameter)
        dt={.05,.1}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function oneSolveMeetsFusionAndMotionConstraints(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());v=c.invariantDesign.verification;
            testCase.verifyTrue(v.certified && v.nominalFusionLmiVerified && v.motionResponse.verified);
            testCase.verifyEqual(v.nominalFusion.vertices,256);
            testCase.verifyFalse(v.motionResponse.longitudinalPoseFeedbackRequired);
            testCase.verifyFalse(v.motionResponse.fullNonlinearResponseCertified);
            testCase.verifyEqual(v.motionResponse.timeConstantS,.5,AbsTol=0);
        end
        function scheduleIsBoundedIncludingZeroAndLargeSpeed(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            a=localizationSupport.weights(d,0);b=localizationSupport.weights(d,1e100);
            testCase.verifyEqual(a(1:2),d.auxiliaryWeights(1:2)*30^2,RelTol=1e-14);
            testCase.verifyGreaterThanOrEqual(a,d.auxiliaryWeights);
            testCase.verifyLessThan(max(b(1:2)),1e-80);
            testCase.verifyEqual(b(3),d.auxiliaryWeights(3));
            testCase.verifyTrue(all(isfinite(a)));
        end
        function removedScheduleCannotRetainThePerformanceClaim(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=rmfield(c.invariantDesign,'motionGainSchedule');
            v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyFalse(v.certified);
            testCase.verifyFalse(v.motionResponse.scheduleVerified);
        end
        function weakenedMotionGainsFailIndependentPerformanceVerification(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            d.auxiliaryWeights(1:2)=d.auxiliaryWeights(1:2)/100;
            v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyFalse(v.motionResponse.verified);
            testCase.verifyFalse(v.certified);
        end
        function changingResponseTargetInvalidatesStoredGains(testCase)
            c=freshObserverTestConfig();c.invariantObserver.motionResponse.timeConstantS=2;
            testCase.verifyError(@()localizationSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
        end
        function brakeAndRestartAreTrackedWithoutPoseFeedback(testCase,dt)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());
            errors=braking(c,dt,0);
            testCase.verifyLessThan(max(abs(errors(:,1))),.6);
            testCase.verifyLessThan(norm(errors(end,:)),.02);
        end
        function sensorNoiseDoesNotBreakTheBrakingReplay(testCase,dt)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());
            errors=braking(c,dt,1);
            testCase.verifyTrue(all(isfinite(errors),'all'));
            testCase.verifyLessThan(rms(errors(:,1)),.6);
            testCase.verifyLessThan(rms(errors(:,2)),1);
        end
        function independentDensePhysicalModelsMeetTheMetricRate(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());
            violation=physicalResponseResidual(c);
            testCase.verifyLessThan(max(violation),1e-8);
        end
        function alteredScheduleDomainCannotRetainTheCertificate(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            d.motionGainSchedule.maximumSpeedMps=31;
            v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyFalse(v.certified);
            testCase.verifyFalse(v.motionResponse.scheduleVerified);
        end
    end
end

function errors=braking(c,dt,noiseScale)
    r=localizationSupport.runtime(c);
    % Allow two seconds after the final acceleration change. The nominal
    % exponential time constant is not a near-zero settling-time claim.
    x=[0;8;0;0;0;0;0;0];time=(0:dt:10).';errors=zeros(numel(time),2);
    stream=RandStream('mt19937ar','Seed',2910);
    for j=1:numel(time)
        now=time(j);
        if now<1,s=8;a=0;elseif now<4,s=8-2*(now-1);a=-2;elseif now<6,s=2;a=0;elseif now<8,s=2+(now-6);a=1;else,s=4;a=0;end
        sm=s+noiseScale*.256*randn(stream);am=a+noiseScale*.233*randn(stream);
        packet=localizationSupport.measurements(sm,0,am,0,0,0,true,true,true,c);
        x=localizationSupport.predict(x,dt,0);
        x=localizationSupport.correct(x,packet,dt,r,c);
        errors(j,:)=[x(2)-s,x(3)-a];
    end
end

function residual=physicalResponseResidual(c)
    d=c.invariantDesign;t=c.invariantObserver.theta;M=d.P\eye(8);R=M(2:3,2:3)\eye(2);
    T=diag([t^2,t^3]);Q=T'\R/T;rate=1/c.invariantObserver.motionResponse.timeConstantS;
    speeds=linspace(1,30,91);accels=linspace(-2,2,17);residual=zeros(numel(speeds)*numel(accels),1);index=0;
    D=localizationSupport.outputNormalization(c);
    for s=speeds
        for a=accels
            x=[0;s;a;0;0;0;0;0];[~,H]=localizationSupport.outputs(x,0,c);
            k=localizationSupport.weights(d,s^2);K=diag([k([1,2,2,3]);0]);
            B=diag(t.^[1,2,3,1,2,3,1,2]);C=D*H*B;
            feedback=t*B*(d.P\(C.'*K*D*H));F=[0,1;0,0]-feedback(2:3,2:3);
            W=Q*F+F.'*Q+2*rate*Q;index=index+1;residual(index)=max(eig((W+W.')/2))/norm(Q,2);
        end
    end
end
