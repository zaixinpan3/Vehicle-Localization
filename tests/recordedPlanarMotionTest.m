classdef recordedPlanarMotionTest < matlab.unittest.TestCase
% recordedPlanarMotionTest Replay increments obey sensor time and body frame.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function straightMotionUsesPhysicalElapsedTime(testCase)
            poses=integrateRecordedPlanarMotion([0;1;2],repmat([3 0 0],3,1),[.25;.75;1.75]);
            testCase.verifyEqual(poses,[0 0 0;1.5 0 0;4.5 0 0],'AbsTol',1e-12);
        end
        function constantTurnMatchesCircularArc(testCase)
            poses=integrateRecordedPlanarMotion([0;2],repmat([2 0 1],2,1),[0;1;2]);
            testCase.verifyEqual(poses,[2*sin([0;1;2]),2*(1-cos([0;1;2])),[0;1;2]],'AbsTol',1e-12);
        end
        function futureSpeedDoesNotAffectEarlierIntervals(testCase)
            poses=integrateRecordedPlanarMotion([0;1;2],[1 0 0;3 0 0;99 0 0],[0;.5;1;1.5;2]);
            testCase.verifyEqual(poses(:,1),[0;.5;1;2.5;4],'AbsTol',1e-12);
        end
        function preservesLateralVelocityInBodyCoordinates(testCase)
            poses=integrateRecordedPlanarMotion([0;1],repmat([0 2 1],2,1),[0;1]);
            testCase.verifyEqual(poses(2,:),[-2*(1-cos(1)),2*sin(1),1],'AbsTol',1e-12);
        end
        function rejectsMissingCoverage(testCase)
            testCase.verifyError(@() integrateRecordedPlanarMotion([0;1],zeros(2,3),[-.1;.5]), ...
                'VehicleLocalization:MotionCoverage');
        end
    end
end
