classdef wheelLongitudinalSpeedTest < matlab.unittest.TestCase
% wheelLongitudinalSpeedTest Physical trajectories and missing-wheel behavior.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function constantSpeedIsExact(testCase)
            [d,c]=fixture(10,0,0);e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyEqual(e.longitudinalSpeed,10*ones(size(d.time)),AbsTol=1e-12);
            testCase.verifyTrue(all(e.valid));
        end
        function acceleratingMotionHasNoFilterLag(testCase)
            [d,c]=fixture(5,1,0);e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyEqual(e.longitudinalSpeed,5+d.time,AbsTol=1e-10);
        end
        function turningWheelDifferencesAreCompensated(testCase)
            [d,c]=fixture(10,0,.25);e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyGreaterThan(max(d.wheels.angularVelocity(1,:))-min(d.wheels.angularVelocity(1,:)),1);
            testCase.verifyEqual(e.longitudinalSpeed,10*ones(size(d.time)),AbsTol=1e-10);
        end
        function oneSlippingWheelIsRejected(testCase)
            [d,c]=fixture(10,0,.15);d.wheels.angularVelocity(:,1)=d.wheels.angularVelocity(:,1)+20;
            e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyEqual(e.longitudinalSpeed,10*ones(size(d.time)),AbsTol=1e-10);
            testCase.verifyTrue(all(e.rejectedWheels(:,1)));
            testCase.verifyEqual(e.acceptedWheelCount,3*ones(size(d.time)));
        end
        function absentWheelDoesNotPoisonOtherThree(testCase)
            [d,c]=fixture(10,0,0);d.wheels.angularVelocity(:,3)=NaN;
            e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyEqual(e.longitudinalSpeed,10*ones(size(d.time)),AbsTol=1e-12);
            testCase.verifyTrue(all(e.valid));
        end
        function absentPacketsExpire(testCase)
            [d,c]=fixture(10,0,0);d.wheels.time=d.wheels.time(1:6);d.wheels.angularVelocity=d.wheels.angularVelocity(1:6,:);
            e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyFalse(any(e.valid(d.time>.26)));
            testCase.verifyTrue(all(isfinite(e.longitudinalSpeed)));
        end
        function conflictingRemainingWheelsAreNotAveraged(testCase)
            [d,c]=fixture(10,0,0);d.wheels.angularVelocity(d.wheels.time>.5,1:2)=NaN;
            d.wheels.angularVelocity(d.wheels.time>.5,3)=80;
            e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyFalse(any(e.valid(d.time>.7)));
            testCase.verifyEqual(e.longitudinalSpeed,10*ones(size(d.time)),AbsTol=1e-12);
        end
        function stationaryWheelsSuppressInertialDrift(testCase)
            [d,c]=fixture(0,0,0);d.longitudinalAcceleration(:)=.08;
            e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyEqual(e.longitudinalSpeed,zeros(size(d.time)),AbsTol=1e-12);
        end
        function futureChangesDoNotAlterPast(testCase)
            [d,c]=fixture(10,0,0);a=estimateWheelLongitudinalSpeed(d,c);
            d.wheels.angularVelocity(d.wheels.time>1,:)=45;
            d.longitudinalAcceleration(d.time>1)=2;b=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyEqual(a.longitudinalSpeed(d.time<=1),b.longitudinalSpeed(d.time<=1),AbsTol=0);
        end
        function missingInitialWheelRequiresExplicitState(testCase)
            [d,c]=fixture(10,0,0);d.wheels.time=d.wheels.time+.02;
            testCase.verifyError(@()estimateWheelLongitudinalSpeed(d,c),'VehicleLocalization:WheelInitialization');
        end
        function initialStateDoesNotInventWheelValidity(testCase)
            [d,c]=fixture(10,0,0);d.wheels.time=d.wheels.time+.04;c.initialSpeed=10;
            e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyFalse(any(e.valid(d.time<.04)));
            testCase.verifyTrue(all(e.valid(d.time>=.04)));
        end
        function signedReverseWheelSpeedIsPreserved(testCase)
            [d,c]=fixture(-5,0,0);e=estimateWheelLongitudinalSpeed(d,c);
            testCase.verifyEqual(e.longitudinalSpeed,-5*ones(size(d.time)),AbsTol=1e-12);
        end
        function duplicatePacketTimesAreRejected(testCase)
            [d,c]=fixture(10,0,0);d.wheels.time(2)=d.wheels.time(1);
            testCase.verifyError(@()estimateWheelLongitudinalSpeed(d,c),'VehicleLocalization:InvalidWheelInput');
        end
    end
end

function [d,c]=fixture(speed,acceleration,rate)
    c=wheelSpeedObserverConfig();c.effectiveRadius=[.36,.36,.36,.36];c.lagCompensation=zeros(1,4);
    t=(0:.01:2).';wt=(0:.02:2).';delta=atan(c.wheelbase*rate/max(abs(speed),1));
    y=[c.frontTrack,-c.frontTrack,c.rearTrack,-c.rearTrack]/2;
    angle=[atan2(c.wheelbase*tan(delta),c.wheelbase-y(1:2)*tan(delta)),0,0];
    omega=((speed+acceleration*wt)-rate*y)./cos(angle)./c.effectiveRadius;
    d=struct('time',t,'steeringAngle',delta*ones(size(t)), ...
        'yawRate',rate*ones(size(t)),'longitudinalAcceleration',(acceleration-c.rearCgDistance*rate^2)*ones(size(t)), ...
        'wheels',struct('time',wt,'angularVelocity',omega));
end
