classdef lidarVelocityConsistencyTest < matlab.unittest.TestCase
% lidarVelocityConsistencyTest Bias recovery and information-flow controls.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function constantBiasRecoveredWithoutChangingRawState(testCase)
            [d,l,anchors]=fixture(30,0);original=l;
            [corrected,audit]=correctLateralVelocityFromLidar(d,l,anchors);
            testCase.verifyEqual(corrected.lateralVelocity(end),.3,AbsTol=.0004);
            testCase.verifyEqual(l,original);
            testCase.verifyGreaterThan(audit.acceptedWindows,200);
            testCase.verifyFalse(audit.referenceUsed);
        end
        function worldRotationDoesNotChangeBodyCorrection(testCase)
            [d,l,anchors]=fixture(15,0);a=correctLateralVelocityFromLidar(d,l,anchors);
            [d,l,anchors]=fixture(15,1.2);b=correctLateralVelocityFromLidar(d,l,anchors);
            testCase.verifyEqual(a.lateralVelocity,b.lateralVelocity,AbsTol=1e-11);
        end
        function turningSeparatesForwardAndLateralBias(testCase)
            [d,l,anchors]=fixture(30,0);t=d.highRate.time;yaw=.12*t;
            d.highRate.longitudinalSpeed(:)=10.1;
            velocity=[10*cos(yaw)-.3*sin(yaw),10*sin(yaw)+.3*cos(yaw)];
            d.lidar.pose=[cumtrapz(t,velocity),yaw];
            corrected=correctLateralVelocityFromLidar(d,l,anchors);
            testCase.verifyEqual(corrected.lateralVelocity(end),.3,AbsTol=.0004);
        end
        function unbiasedMotionRemainsUnchanged(testCase)
            [d,l,anchors]=fixture(20,0);l.lateralVelocity(:)=.3;
            corrected=correctLateralVelocityFromLidar(d,l,anchors);
            testCase.verifyEqual(corrected.lateralVelocity,l.lateralVelocity,AbsTol=1e-11);
        end
        function futureMeasurementsCannotChangePastCorrections(testCase)
            [d,l,anchors]=fixture(15,0);a=correctLateralVelocityFromLidar(d,l,anchors);
            changed=d.highRate.time>8;d.lidar.pose(changed,1:2)=d.lidar.pose(changed,1:2)+7;
            b=correctLateralVelocityFromLidar(d,l,anchors);
            testCase.verifyEqual(a.lateralVelocity(~changed),b.lateralVelocity(~changed));
        end
        function longGapsDoNotCreateBiasMeasurements(testCase)
            [d,l,~]=fixture(10,0);anchors=[0;10];
            [c,audit]=correctLateralVelocityFromLidar(d,l,anchors);
            testCase.verifyEqual(c.lateralVelocity,l.lateralVelocity);
            testCase.verifyEqual(audit.acceptedWindows,0);
        end
        function impossibleWindowsAreRejected(testCase)
            [d,l,anchors]=fixture(10,0);d.lidar.pose(:,2)=4*d.highRate.time;
            [c,audit]=correctLateralVelocityFromLidar(d,l,anchors);
            testCase.verifyEqual(c.lateralVelocity,l.lateralVelocity);
            testCase.verifyEqual(audit.acceptedWindows,0);
        end
    end
end
function [d,l,anchors]=fixture(duration,yaw)
    t=(0:.01:duration).';z=zeros(size(t));h=struct('time',t,'longitudinalSpeed',10+z);
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];xy=[10*t,.3*t]*R.';
    d=struct('highRate',h,'lidar',struct('pose',[xy,yaw+z],'delay',0,'headingConvention',"unwrapped"));
    l=struct('time',t,'lateralVelocity',z,'sideSlipAngle',z,'sideSlipAngleRate',z);
    anchors=t(1:10:end);
end
