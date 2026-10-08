classdef fullObserverContinuousIssTest < matlab.unittest.TestCase
% fullObserverContinuousIssTest Continuous synthesis precedes discretization.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function currentVehicleHasACompleteContinuousCertificate(testCase)
            cfg=mncavFullObserverConfig();d=fullObserverSupport.designFullObserverGains(cfg);
            v=fullObserverSupport.verifyFullObserverIssCertificate(d.continuousCertificate,cfg);
            testCase.verifyTrue(v.certified);
            testCase.verifyGreaterThan(v.verifiedDecayRate,cfg.iss.decayRate);
            testCase.verifySize(v.P,[7,7]);
            testCase.verifyFalse(v.sampledSystemCertified);
        end
        function timingIsNotAGainDesignSelector(testCase)
            cfg=fullObserverConfig();a=fullObserverSupport.designFullObserverGains(cfg);
            b=fullObserverSupport.designFullObserverGains(rmfield(cfg,'timing'));
            testCase.verifyEqual(a.continuousCertificate,b.continuousCertificate);
            testCase.verifyEqual(a.gains,b.gains,AbsTol=0);
            cfg.timing="historical_transport";
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(cfg),'VehicleLocalization:ObsoleteFullConfig');
        end
        function modifiedGainsInvalidatePreviousCertificate(testCase)
            cfg=fullObserverConfig();s=fullObserverSupport.solveFullObserverIssLmi(cfg);cfg.gains(1)=40;
            testCase.verifyError(@()fullObserverSupport.verifyFullObserverIssCertificate(s,cfg), ...
                'VehicleLocalization:StaleFullIssCertificate');
        end
        function modifiedInformationScaleInvalidatesPreviousCertificate(testCase)
            cfg=fullObserverConfig();s=fullObserverSupport.solveFullObserverIssLmi(cfg);cfg.lidar.gainInformationScale=64;
            testCase.verifyError(@()fullObserverSupport.verifyFullObserverIssCertificate(s,cfg), ...
                'VehicleLocalization:StaleFullIssCertificate');
        end
        function invalidGnssWeightCannotUseTheCertificate(testCase)
            cfg=fullObserverConfig();cfg.gnss.gainInformationScale=-1;
            testCase.verifyError(@()fullObserverSupport.solveFullObserverIssLmi(cfg), ...
                'VehicleLocalization:InvalidGnssInformationScale');
        end
        function solverFlagCannotReplaceNumericVerification(testCase)
            cfg=fullObserverConfig();s=fullObserverSupport.solveFullObserverIssLmi(cfg);
            s.weights=ones(3,1);s.certified=true;
            v=fullObserverSupport.verifyFullObserverIssCertificate(s,cfg);
            testCase.verifyFalse(v.certified);
        end
        function unstableContinuousCounterexampleIsRejected(testCase)
            cfg=counterexampleConfig(4);s=fullObserverSupport.solveFullObserverIssLmi(cfg);
            testCase.verifyFalse(s.certified);
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(cfg), ...
                'VehicleLocalization:InfeasibleFullCertificate');
        end
        function strongerPositionGainCertifiesSameContinuousDomain(testCase)
            cfg=counterexampleConfig(16);s=fullObserverSupport.solveFullObserverIssLmi(cfg);
            testCase.verifyTrue(s.certified);
            testCase.verifyGreaterThan(s.verification.verifiedDecayRate,cfg.iss.decayRate);
        end
        function widelySeparatedGainsStillRequireVerifiedMargins(testCase)
            cfg=mncavFullObserverConfig();cfg.gains([1,4])=[160,.125];
            s=fullObserverSupport.solveFullObserverIssLmi(cfg);v=fullObserverSupport.verifyFullObserverIssCertificate(s,cfg);
            testCase.verifyTrue(s.certified);
            testCase.verifyGreaterThan(min(v.normalizedMargins),cfg.iss.tolerance);
        end
        function completeNonlinearDerivativeObeysTheCertificate(testCase)
            violation=derivativeAudit();
            testCase.verifyLessThanOrEqual(violation,1e-8);
        end
        function discreteStepApproachesTheSameContinuousEquation(testCase)
            errors=consistencyAudit();
            testCase.verifyLessThan(errors(2)/errors(1),.4);
            testCase.verifyLessThan(errors(3)/errors(2),.4);
        end
        function insufficientGeometryIsReportedWithoutInventingInformation(testCase)
            [data,lateral,cfg]=oneStep(.1);data.lidar.information(:)=0;
            r=runFullLocalizationObserver(data,struct(),cfg,LateralInputs=lateral);
            testCase.verifyTrue(r.diagnostics.continuousLmiVerified);
            testCase.verifyFalse(any(r.diagnostics.continuousInformationQualified));
            testCase.verifyEqual(r.diagnostics.poseWeight,zeros(3,3,2),AbsTol=0);
            testCase.verifyFalse(r.diagnostics.allTheoremHypothesesVerified);
        end
    end
end

function cfg=counterexampleConfig(kp)
    cfg=fullObserverConfig();cfg.gains(1)=kp;cfg.gnss.positionGain=0;
    cfg.maximumTrackAngleRate=0;cfg.iss.maximumSpeed=8;cfg.iss.maximumAcceleration=0;
    cfg.iss.minimumPositionStrength=.5;cfg.iss.minimumHeadingStrength=.5;
    cfg.iss.maximumPositionHeadingCoupling=.25;
end

function violation=derivativeAudit()
    original=rng;cleanup=onCleanup(@()rng(original));rng(20261002,'twister');
    cfg=mncavFullObserverConfig();s=fullObserverSupport.solveFullObserverIssLmi(cfg);P=s.verification.P;
    g=cfg.gains;J=[0,-1;1,0];violation=-Inf;
    for k=1:2000
        e=randn(7,1);v=cfg.iss.maximumSpeed*rand*unit();a=cfg.iss.maximumAcceleration*rand*unit();
        q=cfg.maximumTrackAngleRate*(2*rand-1);R=rotation(-e(7));
        sp=.25+.3*rand;sy=.97+.01*rand;coupling=.04*rand*unit();
        S=[sp*eye(2),coupling;coupling.',sy];
        ep=e([1,4]);ev=e([2,5]);ea=e([3,6]);pose=[ep;e(7)];
        G=rotation(rand*2*pi);W=G*diag(rand(2,1))*G.';
        d=zeros(7,1);d([1,4])=ev-g(1)*S(1:2,:)*pose ...
            -cfg.gnss.positionGain*W*(ep+(eye(2)-R)*cfg.gnss.outputPoint.bodyOffset(:));
        d([2,5])=ea-g(2)*ev-g(2)*(R-eye(2))*v;
        d([3,6])=q^2*ev+2*q*J*ea-g(3)*ea-g(3)*(R-eye(2))*a;
        d(7)=-g(4)*S(3,:)*pose;
        violation=max(violation,2*e.'*P*d+cfg.iss.decayRate*(e.'*P*e));
    end
end

function u=unit()
    angle=2*pi*rand;u=[cos(angle);sin(angle)];
end

function errors=consistencyAudit()
    steps=[.02,.01,.005];errors=zeros(1,3);
    for k=1:3
        h=steps(k);[data,lateral,cfg,S]=oneStep(h);
        r=runFullLocalizationObserver(data,struct(),cfg,LateralInputs=lateral);
        options=odeset('RelTol',1e-12,'AbsTol',1e-13);
        [~,z]=ode45(@(~,x)derivative(x,cfg,S),[0,h],cfg.initialState,options);
        errors(k)=norm(r.z(end,:)-z(end,:));
    end
end

function [data,lateral,cfg,S]=oneStep(h)
    cfg=fullObserverConfig();cfg.initialState=[.2;8.1;.1;-.1;.1;-.1;.02];
    t=[0;h];zero=zeros(2,1);S=[.4,0,.005;0,.5,0;.005,0,.98];
    H=cfg.lidar.gainInformationScale*S/(eye(3)-S);H=(H+H.')/2;
    motion=struct('time',t,'longitudinalSpeed',8+zero,'longitudinalAcceleration',zero, ...
        'lateralAcceleration',zero,'yawRate',zero);
    lidar=struct('time',t,'pose',zeros(2,3),'information',repmat(H,1,1,2), ...
        'valid',true(2,1),'delay',0);
    data=struct('highRate',motion,'lidar',lidar);
    lateral=struct('time',t,'lateralVelocity',zero,'sideSlipAngleRate',zero);
end

function d=derivative(x,cfg,S)
    g=cfg.gains;pose=x([1,4,7]);d=zeros(7,1);
    d([1,4])=x([2,5])-g(1)*S(1:2,:)*pose;
    d([2,5])=x([3,6])+g(2)*(rotation(x(7))*[8;0]-x([2,5]));
    d([3,6])=-g(3)*x([3,6]);d(7)=-g(4)*S(3,:)*pose;
end

function R=rotation(a)
    R=[cos(a),-sin(a);sin(a),cos(a)];
end
