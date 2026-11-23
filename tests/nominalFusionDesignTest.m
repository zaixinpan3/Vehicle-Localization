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
        function feasiblePairDeterminesEveryLinearColumn(testCase)
            c=freshObserverTestConfig();d=c.invariantDesign;r=localizationSupport.runtime(c);
            E=eye(8);theta=r.theta;
            testCase.verifyEqual(r.linearOutputGain(:,1:3),r.toPhysical*(d.P\E(:,[1,4,7])),AbsTol=1e-13);
            testCase.verifyEqual(r.linearOutputGain(:,4),theta^5*d.cr/d.P(8,8)*E(:,8),AbsTol=1e-13);
            testCase.verifyEqual(d.verification.matrixInequalities,4);
            testCase.verifyGreaterThan(theta^4*d.cr,d.lambda/2);
        end
        function removingRateFeedbackFailsTheCertificate(testCase)
            c=freshObserverTestConfig();d=c.invariantDesign;d.cr=0;
            testCase.verifyFalse(localizationSupport.verifyInvariantDesign(d,c).certified);
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
            synthetic=mncavFullObserverConfig();
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
