classdef scanDeskewTest < matlab.unittest.TestCase
% scanDeskewTest Acquisition-time motion compensation and unchanged attributes.
    methods (TestClassSetup)
        function paths(testCase)
            root=setupVehicleLocalization();
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'research','root_cause_matching_20260929')));
        end
    end
    methods (Test)
        function constantTranslationRecoversOneStaticLandmark(testCase)
            frame=struct('x',[10;9],'y',[2;2],'z',[1;1],'intensity',[50;60]);
            actual=deskewStoredFrame(frame,[0;.1],[10 0 0],0,lidarFrameCalibrationConfig(),eye(3));
            testCase.verifyEqual(actual.x,[10;10],AbsTol=1e-12);
            testCase.verifyEqual(actual.y,frame.y,AbsTol=1e-12);
            testCase.verifyEqual(actual.intensity,frame.intensity);
        end
        function explicitMidScanEpochChangesTheReferencePose(testCase)
            frame=struct('x',[10;9],'y',[2;2],'z',[1;1]);
            actual=deskewStoredFrame(frame,[0;.1],[10 0 0],.05,lidarFrameCalibrationConfig(),eye(3));
            testCase.verifyEqual(actual.x,[9.5;9.5],AbsTol=1e-12);
        end
        function rotationAccountsForTheSensorLeverArm(testCase)
            [frame,times,cal,tilt,truth]=turningFixture();
            actual=deskewStoredFrame(frame,times,[0 0 1],0,cal,tilt);
            testCase.verifyEqual([actual.x actual.y actual.z],truth,AbsTol=1e-12);
        end
        function zeroMotionIsExactlyTheOriginalInput(testCase)
            [frame,times,cal,tilt]=turningFixture();
            actual=deskewStoredFrame(frame,times,[0 0 0],.05,cal,tilt);
            testCase.verifyEqual(actual,frame);
        end
        function missingPointTimeCannotBeSilentlyBroadcast(testCase)
            frame=struct('x',[10;9],'y',[2;2],'z',[1;1]);
            testCase.verifyError(@()deskewStoredFrame(frame,.1,[10 0 0],0,lidarFrameCalibrationConfig(),eye(3)), ...
                'VehicleLocalization:PointTimeShape');
        end
        function invalidZeroReturnsRemainInvalid(testCase)
            frame=struct('x',[0;9],'y',[0;2],'z',[0;1]);
            actual=deskewStoredFrame(frame,[.03;.1],[10 0 0],0,lidarFrameCalibrationConfig(),eye(3));
            testCase.verifyEqual([actual.x(1) actual.y(1) actual.z(1)],[0 0 0],AbsTol=0);
        end
    end
end
function [frame,times,cal,tilt,truth]=turningFixture()
    times=[0;.1];goal=[10 2 1];theta=.2;tilt=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];cal=lidarFrameCalibrationConfig();cal.translation=[2 .3 .5];translation=cal.translation*tilt.';
    ca=cos(times);sa=sin(times);local=[ca*goal(1)+sa*goal(2),-sa*goal(1)+ca*goal(2),repmat(goal(3),2,1)];
    xyz=(local-translation)*tilt;frame=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3));truth=repmat((goal-translation)*tilt,2,1);
end
