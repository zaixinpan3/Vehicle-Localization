classdef (SharedTestFixtures={freshObserverGainsFixture}) nominalFusionDesignTest < matlab.unittest.TestCase
% One nominal fusion solve and explicit limits of the certificate.
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
        function singleSolveReturnsIndependentlyVerifiedGains(testCase)
            [d,c]=selected();v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyTrue(v.certified && v.motionResponse.verified);
            testCase.verifyEqual(d.solveAudit.optimizationCalls,1);
            testCase.verifyEqual(d.kappaG,d.kappaL,AbsTol=1e-9);
            testCase.verifyEqual(v.vertices,256);
            testCase.verifyFalse(isfield(d,'gainSelection'));
            testCase.verifyFalse(isfield(d,'constraintGeneration'));
            testCase.verifyFalse(isfield(d,'accelerationMultipliers'));
        end
        function obsoleteSearchIsRejectedExplicitly(testCase)
            c=mncavSmallOffsetObserverConfig();c.gainSelection=struct('enabled',true);
            testCase.verifyError(@()localizationSupport.invariantInputs(c),'VehicleLocalization:ObsoleteFusionDesign');
        end
        function operatingSetAssumptionIsNotAnInvariantSetCertificate(testCase)
            [d,~]=selected();v=d.verification;
            testCase.verifyTrue(v.operatingSet.assumedInvariant);
            testCase.verifyFalse(v.operatingSet.invarianceVerified);
            testCase.verifyFalse(v.numericalInvariantRadiusCertified);
            testCase.verifyFalse(v.finiteInvariantSecantsCertified);
            testCase.verifyFalse(v.nominalFusion.coversGnssAbsent);
            testCase.verifyFalse(v.nominalFusion.coversZeroLidarInformation);
            testCase.verifyFalse(isfield(v,'regionalIss'));
        end
        function metricShearMakesMotionSchurComplementAffine(testCase)
            [d,~]=selected();P=d.P;inverse=P\eye(8);
            R=inverse(2:3,2:3)\eye(2);
            testCase.verifyEqual(R,P(2:3,2:3)-P(1,1)*[1,-1;-1,1],AbsTol=1e-10);
            testCase.verifyGreaterThan(abs(inverse(2,1)),0);
        end
        function informativeLidarStillCorrectsAStationaryPose(testCase,dt)
            [data,lateral,c]=constantMotionFixture(0,dt);
            lateral.velocityValid=false(size(lateral.time));
            c.initialState=[.2;.05;0;-.1;-.03;0;.03];
            e=runFullLocalizationObserver(data,struct(),c,LateralInputs=lateral);
            testCase.verifyTrue(all(isfinite(e.internalState),'all'));
            testCase.verifyLessThan(norm(e.position(end,:)),.02);
            testCase.verifyLessThan(abs(e.heading(end)),.001);
            testCase.verifyFalse(e.diagnostics.allTheoremHypothesesVerified);
        end
        function gnssOnlyRetainsAccuracyWithoutAnOutageCertificate(testCase,dt)
            [data,lateral,c]=constantMotionFixture(5,dt);
            data=rmfield(data,'lidar');n=numel(lateral.time);
            data.gnss=struct('time',lateral.time, ...
                'position',[5*lateral.time,zeros(n,1)]+c.gnss.outputPoint.bodyOffset(:).', ...
                'information',repmat(1e6*eye(2),1,1,n),'valid',true(n,1),'delay',0);
            c.initialState=[.2;5;0;-.1;0;0;.03];
            e=runFullLocalizationObserver(data,struct(),c,LateralInputs=lateral);
            testCase.verifyTrue(all(isfinite(e.internalState),'all'));
            testCase.verifyLessThan(norm(e.position(end,:)-[5*lateral.time(end),0]),.02);
            % Retain the historical behavioral target without an outage theorem.
            fprintf('GNSS-ONLY 8 s: dt=%g, heading=%.9g rad (behavioral target .005)\n',dt,abs(e.heading(end)));
            testCase.verifyLessThan(abs(e.heading(end)),.005);
            testCase.verifyTrue(e.diagnostics.nominalFusionLmiVerified);
            testCase.verifyFalse(e.diagnostics.allTheoremHypothesesVerified);
        end
    end
end

function [data,lateral,c]=constantMotionFixture(speed,dt)
    t=(0:dt:8).';n=numel(t);zero=zeros(n,1);
    data=struct('highRate',struct('time',t,'longitudinalSpeed',speed+zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero), ...
        'lidar',struct('time',t,'pose',[speed*t,zero,zero], ...
        'information',repmat(1e6*eye(3),1,1,n),'valid',true(n,1),'delay',0));
    lateral=struct('time',t,'lateralVelocity',zero,'sideSlipAngle',zero,'sideSlipAngleRate',zero);
    [~,c]=selected();
end

function [d,c]=selected()
    c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
end
