classdef (SharedTestFixtures={freshObserverGainsFixture}) motionResponseTest < matlab.unittest.TestCase
% Constant auxiliary gains, certificate rejection and motion-only behavior.
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
        function constantColumnsSatisfyTheReservedMargin(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
            testCase.verifyEqual(2*norm(d.Z)*localizationSupport.motionSlopeBound(c),d.lambda,AbsTol=1e-12);
            testCase.verifyTrue(d.verification.certified);
        end
        function physicalLinearizationsMeetCertifiedMarginInGlobalAxes(testCase)
            cfg=freshObserverTestConfig();
            testCase.verifyLessThan(globalAxisResidual(cfg),0);
        end
        function excessiveAuxiliaryGainFailsIndependentVerification(testCase)
            c=freshObserverTestConfig();d=c.invariantDesign;d.Z=1e4*d.Z;
            v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyFalse(v.certified);
        end
        function withdrawnMotionColumnsStillHaveANominalCertificate(testCase)
            c=mncavFullObserverConfig();c.invariantObserver.velocityAvailable=false;
            c.invariantObserver.accelerationAvailable=false;
            ready=freshObserverTestConfig(c);v=ready.invariantDesign.verification;
            testCase.verifyTrue(v.certified);
            testCase.verifyFalse(v.allAuxiliaryOutputsRequired);
            testCase.verifyFalse(v.motionAvailabilityConfigured);
        end
        function fixedGainIsIndependentOfSpeedAndHeading(testCase)
            c=freshObserverTestConfig();d=localizationSupport.runtime(c);
            a=[0;5;1;0;0;0;0;0];b=[0;0;0;0;14;2;pi/2;0];
            u=[.2;1;-.4;.03];valid=true(4,1);
            pa=struct('value',localizationSupport.outputs(a,c)+u,'valid',valid);
            pb=struct('value',localizationSupport.outputs(b,c)+u,'valid',valid);
            xa=localizationSupport.correct(a,pa,.1,d,c);xb=localizationSupport.correct(b,pb,.1,d,c);
            testCase.verifyEqual(xa-a,.1*d.M*u,AbsTol=2e-14);
            testCase.verifyEqual(xb-b,xa-a,AbsTol=2e-14);
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
            motion=d.Z(:,active)*D(active,active)*H(active,:)*T;
            for gnss=[0,1]
                W=d.P*predictor-motion-gnss*d.P*d.K*G ...
                    -d.kappaL*C.'*S*C-d.kappaR*E(:,8)*E(8,:);
                worst=max(worst,max(eig(W+W.'+d.lambda*eye(8))));
            end
        end
    end
end
