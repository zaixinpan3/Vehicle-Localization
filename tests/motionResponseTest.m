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
        function freshDesignIncludesBothIndependentCertificates(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());v=c.invariantDesign.verification;
            testCase.verifyTrue(v.certified && v.jointSensorLmiVerified && v.motionResponse.verified);
            testCase.verifyEqual(v.jointSensor.vertices,133920);
            testCase.verifyFalse(v.motionResponse.poseFeedbackRequired);
            testCase.verifyFalse(v.motionResponse.fullNonlinearResponseCertified);
            testCase.verifyEqual(v.motionResponse.timeConstantS,1,AbsTol=0);
        end
        function scheduleIsBoundedIncludingZeroAndLargeSpeed(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            a=motionResponseSupport.weights(d,0);b=motionResponseSupport.weights(d,1e100);
            testCase.verifyEqual(a,motionResponseSupport.maximumWeights(d),RelTol=1e-14);
            testCase.verifyGreaterThanOrEqual(a,d.auxiliaryWeights);
            testCase.verifyEqual(b,d.auxiliaryWeights,RelTol=1e-13);
            testCase.verifyTrue(all(isfinite(a)));
        end
        function removedScheduleCannotRetainThePerformanceClaim(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=rmfield(c.invariantDesign,'motionGainSchedule');
            v=invariantObserverSupport.verify(d,c);
            testCase.verifyFalse(v.certified);
            testCase.verifyFalse(v.motionResponse.verified);
        end
        function weakenedAugmentationFailsIndependentPerformanceVerification(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            d.motionGainSchedule.incrementWeights=d.motionGainSchedule.incrementWeights/100;
            v=invariantObserverSupport.verify(d,c);
            testCase.verifyTrue(v.jointSensorLmiVerified);
            testCase.verifyFalse(v.motionResponse.verified);
            testCase.verifyFalse(v.certified);
        end
        function finalWeightsAreUsedInTheNonlinearRemainder(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            baseline=rmfield(d,'motionGainSchedule');j=d.verification.jointSensor;
            before=invariantIssSupport.regionalCertificate(baseline,c,j);
            after=invariantIssSupport.regionalCertificate(d,c,j);
            testCase.verifyGreaterThan(after.cubicRemainderBound,before.cubicRemainderBound);
            testCase.verifyLessThan(after.normalizedRadius,before.normalizedRadius);
            testCase.verifyTrue(after.certified);
        end
        function changingResponseTargetInvalidatesStoredGains(testCase)
            c=freshObserverTestConfig();c.invariantObserver.motionResponse.timeConstantS=2;
            testCase.verifyError(@()invariantObserverSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
        end
        function brakeAndRestartAreTrackedWithoutPoseFeedback(testCase,dt)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());
            [errors,baseline]=braking(c,dt,0);
            testCase.verifyLessThan(max(abs(errors(:,1))),.6);
            testCase.verifyLessThan(norm(errors(end,:)),.02);
            testCase.verifyLessThan(rms(errors(:,1)),.2*rms(baseline(:,1)));
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
        function responseNoiseCostMatchesFullPrimitiveColumnReconstruction(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            delta=objectiveDifference(c);
            testCase.verifyEqual(d.meanSquareBound-d.motionResponseDesign.seedNoiseObjective,delta,RelTol=1e-8);
        end
        function alteredScheduleDomainCannotRetainTheCertificate(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            d.motionGainSchedule.maximumSpeedMps=31;
            v=invariantObserverSupport.verify(d,c);
            testCase.verifyFalse(v.certified);
            testCase.verifyFalse(v.positiveGramAugmentationVerified);
        end
    end
end

function [errors,baseline]=braking(c,dt,noiseScale)
    r=invariantObserverSupport.runtime(c);r0=rmfield(r,'motionGainSchedule');
    % Allow two seconds after the final acceleration change. A one-second
    % exponential time constant is not a one-second near-zero settling claim.
    x=[0;8;0;0;0;0;0;0];y=x;time=(0:dt:10).';errors=zeros(numel(time),2);baseline=errors;
    stream=RandStream('mt19937ar','Seed',2910);
    for j=1:numel(time)
        now=time(j);
        if now<1,s=8;a=0;elseif now<4,s=8-2*(now-1);a=-2;elseif now<6,s=2;a=0;elseif now<8,s=2+(now-6);a=1;else,s=4;a=0;end
        sm=s+noiseScale*.256*randn(stream);am=a+noiseScale*.233*randn(stream);
        packet=invariantObserverSupport.measurements(sm,0,am,0,0,0,true,true,true,c);
        x=invariantObserverSupport.predict(x,dt,0);y=invariantObserverSupport.predict(y,dt,0);
        x=invariantObserverSupport.correct(x,packet,dt,r,c);y=invariantObserverSupport.correct(y,packet,dt,r0,c);
        errors(j,:)=[x(2)-s,x(3)-a];baseline(j,:)=[y(2)-s,y(3)-a];
    end
end

function delta=objectiveDifference(c)
    d=c.invariantDesign;z=c.invariantObserver;t=z.theta;n=z.noise;
    B=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),z.headingLengthM,z.headingLengthM]);
    D=invariantObserverSupport.outputNormalization(c);L=chol(d.P,'lower');delta=0;E=eye(8);
    speeds=[1,15.5,30];accels=2*[0,1,-1,0,0;0,0,0,1,-1];
    for s=speeds
        for a=accels
            x=[0;s;a(1);0;0;a(2);0;0];[~,H]=invariantObserverSupport.outputs(x,0,c);
            N=invariantIssSupport.measurementSecant([s;0],a,[s;0],a)* ...
                diag([n.velocityMps,n.velocityMps,n.accelerationMps2,n.accelerationMps2,n.gyroRadps]);
            N(:,6)=0;N(4,6)=n.velocityMps;C=D*H*B;
            k=motionResponseSupport.weights(d,s^2);k0=d.auxiliaryWeights;
            G=-t*C.'*diag([k([1,2,2,3]);0])*D*N;
            G0=-t*C.'*diag([k0([1,2,2,3]);0])*D*N;
            gyro=d.P*E(:,8)*(d.gyroGainNormalized*z.headingLengthM*n.gyroRadps/t);
            G(:,5)=G(:,5)-gyro;G0(:,5)=G0(:,5)-gyro;
            delta=delta+(norm(L\G,'fro')^2-norm(L\G0,'fro')^2)/15;
        end
    end
end

function residual=physicalResponseResidual(c)
    d=c.invariantDesign;t=c.invariantObserver.theta;M=d.P\eye(8);R=M(2:3,2:3)\eye(2);
    T=diag([t^2,t^3]);Q=T'\R/T;rate=1/c.invariantObserver.motionResponse.timeConstantS;
    speeds=linspace(1,30,91);accels=linspace(-2,2,17);residual=zeros(numel(speeds)*numel(accels),1);index=0;
    D=invariantObserverSupport.outputNormalization(c);
    for s=speeds
        for a=accels
            x=[0;s;a;0;0;0;0;0];[~,H]=invariantObserverSupport.outputs(x,0,c);
            k=motionResponseSupport.weights(d,s^2);K=diag([k([1,2,2,3]);0]);
            B=diag(t.^[1,2,3,1,2,3,1,2]);C=D*H*B;
            feedback=t*B*(d.P\(C.'*K*D*H));F=[0,1;0,0]-feedback(2:3,2:3);
            W=Q*F+F.'*Q+2*rate*Q;index=index+1;residual(index)=max(eig((W+W.')/2))/norm(Q,2);
        end
    end
end
