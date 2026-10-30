classdef (SharedTestFixtures={freshObserverGainsFixture}) observerGainSelectionTest < matlab.unittest.TestCase
% Engineering preferences must not become theorem or runtime requirements.
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
        function selectedDesignPassesOriginalCertificateAndBeatsTheSeedObjective(testCase)
            [d,c]=selected();a=d.gainSelection;
            v=invariantObserverSupport.verify(d,c);
            testCase.verifyTrue(v.certified && v.motionResponse.verified);
            testCase.verifyEqual(v.jointSensor.vertices,133920);
            testCase.verifyFalse(isfield(v,'motionPositionCoupling'));
            testCase.verifyFalse(a.certificateFeasibleSetChanged || a.referenceUsed);
            testCase.verifyEqual(d.meanSquareBound,min([a.candidates.noiseProxy]),RelTol=1e-10);
            testCase.verifyLessThan(d.meanSquareBound,a.candidates(1).noiseProxy);
        end
        function originalFeasibleDesignWithLargeCouplingIsStillAccepted(testCase)
            c=mncavSmallOffsetObserverConfig();c.gainSelection.enabled=false;
            c=freshObserverTestConfig(c);v=invariantObserverSupport.verify(c.invariantDesign,c);
            testCase.verifyTrue(v.certified);
            testCase.verifyGreaterThan(observerGainSelectionSupport.couplingRatio(c.invariantDesign.P,c),2);
        end
        function engineeringPreferencesAreNotCertificateInputs(testCase)
            [~,c]=selected();original=invariantObserverSupport.inputs(c);
            c.gainSelection.penaltyWeights=1e6;
            testCase.verifyEqual(invariantObserverSupport.inputs(c),original);
            r=invariantObserverSupport.runtime(c);
            testCase.verifyTrue(r.continuousCertificate.verification.certified);
        end
        function obsoleteHardBudgetIsRejectedExplicitly(testCase)
            c=mncavSmallOffsetObserverConfig();c.invariantObserver.maximumMotionPositionNoiseRatio=2;
            testCase.verifyError(@()invariantObserverSupport.inputs(c),'VehicleLocalization:ObsoleteCouplingConstraint');
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
        function gnssOnlyStillCorrectsAnInDomainCruise(testCase,dt)
            [data,lateral,c]=constantMotionFixture(5,dt);
            data=rmfield(data,'lidar');n=numel(lateral.time);
            data.gnss=struct('time',lateral.time, ...
                'position',[5*lateral.time,zeros(n,1)]+c.gnss.outputPoint.bodyOffset(:).', ...
                'information',repmat(1e6*eye(2),1,1,n),'valid',true(n,1),'delay',0);
            c.initialState=[.2;5;0;-.1;0;0;.03];
            e=runFullLocalizationObserver(data,struct(),c,LateralInputs=lateral);
            testCase.verifyTrue(all(isfinite(e.internalState),'all'));
            testCase.verifyLessThan(norm(e.position(end,:)-[5*lateral.time(end),0]),.02);
            testCase.verifyLessThan(abs(e.heading(end)),.005);
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
