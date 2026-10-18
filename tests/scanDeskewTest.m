classdef scanDeskewTest < matlab.unittest.TestCase
% scanDeskewTest Current recorded-query deskew, including unavailable motion.
    methods (TestClassSetup)
        function paths(~)
            root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setupVehicleLocalization();
        end
    end
    methods (Test)
        function constantTranslationRecoversOneStaticLandmark(test)
            [frame,inputs]=fixture();
            actual=deskewDowntownMotionFrame(frame,inputs);
            test.verifyEqual(actual.x,single([10;10]));
            test.verifyEqual(actual.intensity,frame.intensity);
        end
        function midScanPointUsesTheDeclaredAcquisitionEpoch(test)
            [frame,inputs]=fixture();frame.timestamp=.05;frame.pointTimeSeconds=[-.05;.05];
            actual=deskewDowntownMotionFrame(frame,inputs);
            test.verifyEqual(actual.x,single([9.5;9.5]));
        end
        function rotationRecoversOneStaticLandmark(test)
            [frame,inputs]=fixture();inputs.motion=[0,0,0;0,0,.1;0,0,.2];
            frame.x=single([10;10*cos(.1)+2*sin(.1)]);frame.y=single([2;-10*sin(.1)+2*cos(.1)]);
            actual=deskewDowntownMotionFrame(frame,inputs);
            test.verifyEqual(actual.x,single([10;10]),AbsTol=single(1e-6));
            test.verifyEqual(actual.y,single([2;2]),AbsTol=single(1e-6));
        end
        function zeroMotionPreservesTheAcquiredScan(test)
            [frame,inputs]=fixture();inputs.motion(:)=0;
            actual=deskewDowntownMotionFrame(frame,inputs);
            test.verifyEqual(actual,frame);
        end
        function missingPointTimeCannotBeSilentlyBroadcast(test)
            [frame,inputs]=fixture();frame.pointTimeSeconds=.1;
            test.verifyError(@()deskewDowntownMotionFrame(frame,inputs),'VehicleLocalization:PointTimeShape');
        end
        function invalidReturnsRemainInvalid(test)
            [frame,inputs]=fixture();frame.x(1)=0;frame.y(1)=0;frame.z(1)=0;
            actual=deskewDowntownMotionFrame(frame,inputs);
            test.verifyTrue(isnan(actual.x(1)) && isnan(actual.y(1)));
        end
        function invalidLateralMotionWithdrawsDeskew(test)
            [frame,inputs]=fixture();inputs.motion(:)=NaN;inputs.motionSegment(:)=0;
            [actual,applied]=deskewDowntownMotionFrame(frame,inputs);
            test.verifyEqual(actual,frame);test.verifyFalse(applied);
        end
        function aScanAcrossReentryCannotUseTwoOdometryOrigins(test)
            [frame,inputs]=fixture();inputs.motionSegment=[1;2;2];
            [actual,applied]=deskewDowntownMotionFrame(frame,inputs);
            test.verifyEqual(actual,frame);test.verifyFalse(applied);
        end
    end
end
function [frame,inputs]=fixture()
    frame=struct('x',single([10;9]),'y',single([2;2]),'z',single([1;1]), ...
        'intensity',single([50;60]),'timestamp',0,'pointTimeSeconds',[0;.1]);
    inputs=struct('highRate',struct('time',[0;.1;.2]),'nativeOriginSeconds',0, ...
        'motion',[0,0,0;1,0,0;2,0,0],'motionSegment',ones(3,1));
end
