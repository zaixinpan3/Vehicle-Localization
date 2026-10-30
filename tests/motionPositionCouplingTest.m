classdef (SharedTestFixtures={freshObserverGainsFixture}) motionPositionCouplingTest < matlab.unittest.TestCase
% Physical correction bounds and independent certificate rejection.
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
        function freshDesignMeetsCouplingResponseAndIssConstraints(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());v=c.invariantDesign.verification;
            testCase.verifyTrue(v.certified && v.motionResponse.verified && v.motionPositionCoupling.verified);
            testCase.verifyEqual(v.jointSensor.vertices,133920);
            testCase.verifyEqual(v.motionPositionCoupling.maximumScaledCouplingM,2*.0279,AbsTol=1e-14);
            testCase.verifyLessThanOrEqual(v.motionPositionCoupling.scaledCouplingM,2*.0279*(1+1e-7));
        end
        function actualImplicitCorrectionObeysThePhysicalBudget(testCase,dt)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());r=invariantObserverSupport.runtime(c);
            state=[0;5;.7;0;.5;-.1;.2;.1];
            packet=invariantObserverSupport.measurements(4.2,.2,-.5,.3,.1,.08,true,true,true,c);
            next=invariantObserverSupport.correct(state,packet,dt,r,c);delta=next-state;
            b=c.invariantDesign.verification.motionPositionCoupling.physicalCoupling;
            n=c.invariantObserver.noise;
            normalized=[delta([2,5])/n.velocityMps;delta([3,6])/n.accelerationMps2];
            bound=c.invariantObserver.maximumMotionPositionNoiseRatio*n.lidarPositionM*norm(normalized);
            testCase.verifyGreaterThan(norm(delta([2,3,5,6])),.01);
            testCase.verifyEqual(delta([1,4]),-b(1)*delta([2,5])-b(2)*delta([3,6]),AbsTol=1e-11);
            testCase.verifyLessThanOrEqual(norm(delta([1,4])),bound*(1+1e-7));
        end
        function stricterBudgetCannotReuseAnEarlierCertificate(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());
            c.invariantObserver.maximumMotionPositionNoiseRatio=.01;
            v=invariantObserverSupport.verify(c.invariantDesign,c);
            testCase.verifyTrue(v.jointSensorLmiVerified && v.motionResponse.verified);
            testCase.verifyFalse(v.motionPositionCoupling.verified);
            testCase.verifyFalse(v.certified || v.regionalIss.certified);
        end
        function changedCouplingBudgetInvalidatesStoredInputs(testCase)
            c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());
            c.invariantObserver.maximumMotionPositionNoiseRatio=3;
            testCase.verifyError(@()invariantObserverSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
        end
        function verifierUsesTheConfiguredPhysicalScaling(testCase)
            c=mncavFullObserverConfig(Theta=2);c.invariantObserver.maximumMotionPositionNoiseRatio=1;
            c.invariantObserver.noise.lidarPositionM=.5;
            c.invariantObserver.noise.velocityMps=1;c.invariantObserver.noise.accelerationMps2=2;
            Pc=[1,-.6,.8;-.6,2,0;.8,0,2];P=blkdiag(Pc,Pc,eye(2));
            v=motionPositionCouplingSupport.verify(P,c);
            testCase.verifyTrue(v.verified);
            testCase.verifyEqual(v.physicalCoupling,[-.3,.2],AbsTol=1e-14);
            testCase.verifyEqual(v.scaledCouplingM,.5,AbsTol=1e-14);
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
    c=freshObserverTestConfig(mncavSmallOffsetObserverConfig());
end
