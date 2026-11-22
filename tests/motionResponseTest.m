classdef (SharedTestFixtures={freshObserverGainsFixture}) motionResponseTest < matlab.unittest.TestCase
% Matched auxiliary gains, certificate rejection and motion-only behavior.
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
        function motionWeightsMeetTheResponseBounds(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            testCase.verifyTrue(d.verification.motionResponseVerified);
            testCase.verifyTrue(d.verification.certified);
        end
        function physicalLinearizationsMeetCertifiedMarginInGlobalAxes(testCase)
            cfg=freshObserverTestConfig();
            testCase.verifyLessThan(globalAxisResidual(cfg),0);
        end
        function negativeAuxiliaryGainFailsIndependentVerification(testCase)
            c=freshObserverTestConfig();d=c.invariantDesign;d.auxiliaryWeights(3)=-1;
            v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyFalse(v.certified);
        end
        function withdrawnMotionKeepsTheVerifiedGains(testCase)
            c=freshObserverTestConfig();d=localizationSupport.runtime(c);
            x=[0;9;1;0;4;.2;.7;.1];
            packet=localizationSupport.measurements(NaN,NaN,NaN,NaN,false,false,c);
            testCase.verifyFalse(any(packet.valid));
            testCase.verifyEqual(localizationSupport.correct(x,packet,.1,d,c),x,AbsTol=0);
            testCase.verifyTrue(d.continuousCertificate.verification.certified);
            testCase.verifyFalse(d.continuousCertificate.verification.allAuxiliaryOutputsRequired);
        end
        function squaredSpeedCorrectionRotatesWithVelocity(testCase)
            c=freshObserverTestConfig();d=localizationSupport.runtime(c);
            a=[0;5;0;0;0;0;0;0];b=[0;0;0;0;5;0;pi/2;0];
            packet=struct('value',[36;NaN;NaN;NaN],'valid',logical([1;0;0;0]));
            xa=localizationSupport.correct(a,packet,.1,d,c)-a;
            xb=localizationSupport.correct(b,packet,.1,d,c)-b;
            testCase.verifyGreaterThan(xa(2),0);
            testCase.verifyEqual(xa(1:3),xb(4:6),AbsTol=1e-12);
            testCase.verifyEqual(xa(4:6),zeros(3,1),AbsTol=1e-12);
            testCase.verifyEqual(xb(1:3),zeros(3,1),AbsTol=1e-12);
        end
        function restoredImplicitStepMatchesThePreviousUpdateEquation(testCase)
            c=freshObserverTestConfig();d=localizationSupport.runtime(c);dt=.1;
            x=[0;-4;1;0;8;-.2;1.9;-.04];
            p=localizationSupport.measurements(8,.5,.8,-.3,true,true,c);
            [value,H]=localizationSupport.outputs(x,c);D=localizationSupport.outputNormalization(c);
            C=D*H*d.toPhysical;k=d.auxiliaryWeights;
            k(1:2)=k(1:2)*c.iss.maximumSpeed^2/(c.invariantObserver.fusion.minimumCourseSpeed^2+p.value(1));
            W=C.'*diag(k([1,2,2,3]));
            Q=d.theta*W*C;rhs=dt*d.theta*(d.P\(W*D*(p.value-value)));
            expected=x+d.toPhysical*((eye(8)+dt*(d.P\Q))\rhs);
            testCase.verifyEqual(localizationSupport.correct(x,p,dt,d,c),expected,AbsTol=1e-12);
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
        packet=localizationSupport.measurements(sm,0,am,0,true,true,c);
        x=localizationSupport.predict(x,dt,c);
        x=localizationSupport.correct(x,packet,dt,r,c);
        errors(j,:)=[x(2)-s,x(3)-a];
    end
end

function worst=globalAxisResidual(cfg)
    stream=RandStream('mt19937ar','Seed',20261121);
    r=localizationSupport.runtime(cfg);d=cfg.invariantDesign;
    T=r.toPhysical;t=r.theta;D=localizationSupport.outputNormalization(cfg);
    ell=cfg.invariantObserver.headingLengthM;E=eye(8);C=E([1,4,7],:);
    S=diag([cfg.iss.minimumPositionStrength*[1,1],cfg.iss.minimumHeadingStrength]) ...
        -(cfg.iss.maximumPositionHeadingCoupling+cfg.invariantObserver.fusion.lidarOperatorMismatch)*eye(3);
    worst=-Inf;
    for j=1:80
        course=2*pi*rand(stream);heading=2*pi*rand(stream);s=10+5*rand(stream);
        direction=[cos(course);sin(course)];a=2*sqrt(rand(stream))*[cos(heading);sin(heading)];
        x=zeros(8,1);x([2,5])=s*direction;x([3,6])=a;x(7)=2*pi*rand(stream);
        [F,H,G]=localizationSupport.secants(x,x,cfg);
        predictor=T\(F*T)/t;
        G=[G;ell*E(7,:)]*T/t;
        for mask=0:15
            active=logical(bitget(mask,1:4));
            k=d.auxiliaryWeights;
            measuredSquaredSpeed=double(active(1))*s^2;
            k(1:2)=k(1:2)*cfg.iss.maximumSpeed^2/(cfg.invariantObserver.fusion.minimumCourseSpeed^2+measuredSquaredSpeed);
            k=k([1,2,2,3]);Cmotion=D(active,active)*H(active,:)*T;
            motion=Cmotion.'*diag(k(active))*Cmotion;
            for gnss=[0,1]
                W=d.P*predictor-motion-gnss*d.P*d.K*G ...
                    -d.kappaL*C.'*S*C-d.kappaR*E(:,8)*E(8,:);
                worst=max(worst,max(eig(W+W.'+d.lambda*eye(8))));
            end
        end
    end
end
