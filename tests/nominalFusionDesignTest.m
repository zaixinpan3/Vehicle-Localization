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
        function separatePoseAndMotionSolvesReturnVerifiedGains(testCase)
            c=freshObserverTestConfig();d=c.invariantDesign;v=localizationSupport.verifyInvariantDesign(d,c);
            testCase.verifyTrue(v.certified && v.matchedAuxiliaryGain);
            testCase.verifyGreaterThanOrEqual(d.solveAudit.optimizationCalls,2);
            testCase.verifyEqual(d.K(1:3,1),[3;3;1]);
            testCase.verifyEqual([v.vertices,v.matrixInequalities],[4,4]);
            testCase.verifyLessThan(v.gnssDissipationDefect,1e-8);
            raw=localizationSupport.fusionBlocks(d,c,0);
            decay=-max(cellfun(@(b)max(eig(b)),raw));
            z=c.invariantObserver;
            floorBound=min([1,min(eig(d.P)),d.kappaR/z.margin,decay/(2*z.lambda+z.margin)]);
            testCase.verifyEqual(d.lambda/z.lambda,floorBound,AbsTol=1e-7);
            testCase.verifyTrue(v.independentYawRateCorrection && v.upstreamYawRateRequired);
            testCase.verifyEqual(d.P(8,1:7),zeros(1,7));
        end
        function rateGainRespondsToNoiseAndRejectsPresetValues(testCase)
            c=freshObserverTestConfig();base=c.invariantDesign;
            testCase.verifyTrue(base.verification.rateEnergyVerified);
            c.invariantObserver.noise.upstreamRateRadps=2*c.invariantObserver.noise.upstreamRateRadps;
            testCase.verifyError(@()localizationSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
            other=freshObserverTestConfig(c);
            testCase.verifyLessThan(other.invariantDesign.verification.yawRateGainPerSecond,base.verification.yawRateGainPerSecond);
            broken=base;broken.kappaR=0;
            testCase.verifyFalse(localizationSupport.verifyInvariantDesign(broken,mncavFullObserverConfig()).certified);
            testCase.verifyError(@() mncavFullObserverConfig(DesignOverride=struct('unknownOption',5)), ...
                'VehicleLocalization:UnknownDesignOption');
        end
        function downtownUsesTheNominalDesignDuringGnssWithdrawal(testCase)
            downtown=downtownReferenceReplayConfig();
            nominal=mncavFullObserverConfig();
            testCase.verifyEqual(localizationSupport.invariantInputs(downtown.observer), ...
                localizationSupport.invariantInputs(nominal));
            selectedDesign=freshObserverTestConfig(nominal);
            testCase.verifyTrue(selectedDesign.invariantDesign.verification.nominalFusion.coversGnssAbsent);
            testCase.verifyEqual(downtown.rmseTargets.positionM,.1435);
        end
        function operatingSetAssumptionIsNotAnInvariantSetCertificate(testCase)
            [d,~]=selected();v=d.verification;
            testCase.verifyTrue(v.operatingSet.assumedInvariant);
            testCase.verifyFalse(v.operatingSet.invarianceVerified);
            testCase.verifyTrue(v.nominalFusion.coversGnssAbsent);
            testCase.verifyFalse(v.nominalFusion.coversZeroLidarInformation);
            testCase.verifyFalse(isfield(v,'regionalIss'));
        end
        function receiverUncertaintyCannotChangeGnssFreeGains(testCase)
            c=freshObserverTestConfig();a=c.invariantDesign;
            c.invariantObserver.noise.gnssHeadingRad=.01;
            c.invariantObserver.noise.gnssPositionM=.10;
            c=freshObserverTestConfig(c);b=c.invariantDesign;
            testCase.verifyEqual(b.P,a.P,AbsTol=1e-10);
            testCase.verifyEqual([b.kappaR;b.auxiliaryWeights], ...
                [a.kappaR;a.auxiliaryWeights],AbsTol=1e-10);
            testCase.verifyGreaterThan(b.K(7,3),a.K(7,3));
            testCase.verifyEqual(b.gnssHeadingBandwidth.perSecond, ...
                max(b.gnssHeadingBandwidth.dissipationFloorPerSecond, ...
                c.invariantObserver.noise.upstreamRateRadps/.01),AbsTol=1e-12);
            c.invariantObserver.noise.gnssHeadingRad=1e5;
            [K,bound]=localizationSupport.linearGain(c,b.P);
            testCase.verifyEqual(bound.perSecond,bound.dissipationFloorPerSecond);
            gammaP=b.P(1,1:3)*K(1:3,1);gammaH=b.P(7,7)*K(7,3);
            testCase.verifyEqual(4*gammaH,gammaP*(norm(c.gnss.outputPoint.bodyOffset)/ ...
                c.invariantObserver.headingLengthM)^2,AbsTol=1e-12);
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
            sensor=mncavSensorConfig();synthetic=mncavFullObserverConfig();
            synthetic.invariantObserver.noise.gnssHeadingRad=sensor.referenceGnssIns.headingErrorSimulationPriorRad;
            [data,lateral,c]=constantMotionFixture(5,dt,freshObserverTestConfig(synthetic));
            data=rmfield(data,'lidar');n=numel(lateral.time);
            data.gnss=struct('time',lateral.time, ...
                'heading',zeros(n,1),'position',[5*lateral.time,zeros(n,1)]+c.gnss.outputPoint.bodyOffset(:).', ...
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
