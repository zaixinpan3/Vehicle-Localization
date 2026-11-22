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
            c=freshObserverTestConfig();d=c.invariantDesign;v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyTrue(v.certified && v.constantAuxiliaryGain);
            testCase.verifyEqual(d.solveAudit.optimizationCalls,1);
            testCase.verifyEqual(d.K(1:3,1),[3;3;1]);
            testCase.verifyEqual(v.vertices,32);
            testCase.verifyTrue(v.independentYawRateCorrection && v.upstreamYawRateRequired);
            testCase.verifyEqual(d.P(8,1:7),zeros(1,7));
            testCase.verifyFalse(isfield(d,'yawRateGainNormalized'));
            testCase.verifyFalse(isfield(d,'gainSelection'));
            testCase.verifyFalse(isfield(d,'constraintGeneration'));
            testCase.verifyFalse(isfield(d,'accelerationMultipliers'));
        end
        function rateGainRespondsToNoiseAndRejectsPresetValues(testCase)
            c=freshObserverTestConfig();base=c.invariantDesign;
            testCase.verifyFalse(isfield(c.invariantObserver,'yawRateGainPerSecond'));
            downtown=downtownReferenceReplayConfig();
            testCase.verifyFalse(downtown.observer.invariantObserver.gnssPositionAvailable);
            testCase.verifyFalse(downtown.observer.invariantObserver.gnssHeadingAvailable);
            testCase.verifyTrue(base.verification.rateEnergyVerified);
            c.invariantObserver.noise.upstreamRateRadps=2*c.invariantObserver.noise.upstreamRateRadps;
            testCase.verifyError(@()localizationSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
            other=freshObserverTestConfig(c);
            testCase.verifyLessThan(other.invariantDesign.verification.yawRateGainPerSecond,base.verification.yawRateGainPerSecond);
            broken=base;broken.kappaR=0;
            testCase.verifyFalse(localizationSupport.verifyInvariantDesign(broken,mncavFullObserverConfig()).certified);
            testCase.verifyError(@() mncavFullObserverConfig(DesignOverride=struct('yawRateGainPerSecond',5)), ...
                'VehicleLocalization:PrescribedRateGain');
        end
        function obsoleteSearchIsRejectedExplicitly(testCase)
            c=mncavSmallOffsetObserverConfig();c.gainSelection=struct('enabled',true);
            testCase.verifyError(@()localizationSupport.invariantInputs(c),'VehicleLocalization:ObsoleteFusionDesign');
        end
        function operatingSetAssumptionIsNotAnInvariantSetCertificate(testCase)
            [d,~]=selected();v=d.verification;
            testCase.verifyTrue(v.operatingSet.assumedInvariant);
            testCase.verifyFalse(v.operatingSet.invarianceVerified);
            testCase.verifyTrue(v.nominalFusion.coversGnssAbsent);
            testCase.verifyFalse(v.nominalFusion.coversZeroLidarInformation);
            testCase.verifyFalse(isfield(v,'regionalIss'));
        end
        function independentGnssBandwidthChangesOnlyTheSpecifiedLinearDesign(testCase)
            c=freshObserverTestConfig();a=localizationSupport.runtime(c);
            c.invariantObserver.gnssBandwidthRatio=1.2;
            testCase.verifyError(@()localizationSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
            ready=freshObserverTestConfig(c);b=localizationSupport.runtime(ready);
            testCase.verifyEqual(b.LG(1,1)/a.LG(1,1),1.2,AbsTol=1e-12);
            testCase.verifyTrue(b.nominalFusionLmiVerified);
        end
        function informativeLidarStillCorrectsAStationaryPose(testCase,dt)
            [data,lateral,c]=constantMotionFixture(0,dt);
            lateral.velocityValid=false(size(lateral.time));
            c.initialState=[.2;.05;0;-.1;-.03;0;.03];
            e=syntheticObserver.run(data,c,LateralInputs=lateral);
            testCase.verifyTrue(all(isfinite(e.internalState),'all'));
            testCase.verifyLessThan(norm(e.position(end,:)),.02);
            testCase.verifyLessThan(abs(e.heading(end)),.001);
            testCase.verifyFalse(e.diagnostics.allTheoremHypothesesVerified);
        end
        function lidarHeadingInnovationLeavesTheUnavailableRateAlone(testCase)
            [data,lateral,c]=constantMotionFixture(0,.1,freshObserverTestConfig());
            lateral.velocityValid=false(size(lateral.time));lateral.yawRateValid=false(size(lateral.time));
            c.initialState=[0;0;0;0;0;0;.03;.05];
            estimate=syntheticObserver.run(data,c,LateralInputs=lateral);
            testCase.verifyEqual(estimate.yawRate,.05*ones(size(lateral.time)),AbsTol=0);
            testCase.verifyLessThan(abs(estimate.heading(end)),.03);
        end
        function gnssOnlyRetainsAccuracyWithoutAnOutageCertificate(testCase,dt)
            [data,lateral,c]=constantMotionFixture(5,dt);
            data=rmfield(data,'lidar');n=numel(lateral.time);
            data.gnss=struct('time',lateral.time, ...
                'position',[5*lateral.time,zeros(n,1)]+c.gnss.outputPoint.bodyOffset(:).', ...
                'information',repmat(1e6*eye(2),1,1,n),'valid',true(n,1),'delay',0);
            c.initialState=[.2;5;0;-.1;0;0;.03];
            e=runFullLocalizationObserver(data,c,LateralInputs=lateral);
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

function [data,lateral,c]=constantMotionFixture(speed,dt,c)
    t=(0:dt:8).';n=numel(t);zero=zeros(n,1);
    data=struct('highRate',struct('time',t,'longitudinalSpeed',speed+zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero), ...
        'lidar',struct('time',t,'pose',[speed*t,zero,zero], ...
        'information',repmat(1e6*eye(3),1,1,n),'valid',true(n,1),'delay',0));
    lateral=struct('time',t,'lateralVelocity',zero,'yawRate',zero,'sideSlipAngleRate',zero);
    if nargin<3,[~,c]=selected();end
end

function [d,c]=selected()
    c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());d=c.invariantDesign;
end
