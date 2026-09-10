classdef poleIsolationTest < matlab.unittest.TestCase
% poleIsolationTest: Separate a thin shaft from surrounding nonground clutter.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function aVerticalCoreDoesNotRescueAClutteredNeighborhood(testCase)
            [shaft,geometry,qualified]=scene();cfg=finePerceptionConfig();
            testCase.verifyTrue(validatePoleIsolation(shaft,[0 0 0],[0 0],qualified,geometry,cfg));
            clutter=shaft+[0.5 0 0];
            [isolated,detail]=validatePoleIsolation([shaft;clutter],[0 0 0],[0 0],qualified,geometry,cfg);
            testCase.verifyFalse(isolated);
            testCase.verifyEqual(detail.coreFraction,0.5);
            testCase.verifyEqual(detail.coreCount,size(shaft,1));
        end
        function remoteAndUnsupportedHeightReturnsDoNotInvalidateShaft(testCase)
            [shaft,geometry,qualified]=scene();cfg=finePerceptionConfig();
            extra=[shaft+[2 0 0];shaft+[0.5 0 4]];
            [isolated,detail]=validatePoleIsolation([shaft;extra],[0 0 0],[0 0],qualified,geometry,cfg);
            testCase.verifyTrue(isolated);
            testCase.verifyEqual(detail.coreFraction,1);
        end
        function spatialTransformAndPointOrderPreserveIsolation(testCase)
            [shaft,geometry,qualified]=scene();cfg=finePerceptionConfig();
            points=[shaft;shaft+[0.5 0 0]];
            slope=[0.05 -0.03];center=[4 -3 0];
            points(:,1:2)=points(:,1:2)+center(1:2)+points(:,3)*slope;
            angle=0.7;rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            points(:,1:2)=points(:,1:2)*rotation.';
            center(1:2)=center(1:2)*rotation.';slope=slope*rotation.';
            stream=RandStream('mt19937ar','Seed',9142593);points=points(randperm(stream,size(points,1)),:);
            [isolated,detail]=validatePoleIsolation(points,center,slope,qualified,geometry,cfg);
            testCase.verifyFalse(isolated);
            testCase.verifyEqual(detail.coreFraction,0.5);
        end
    end
end

function [points,geometry,qualified]=scene()
    z=(0.125:0.125:3.875).';
    points=[0.05*cos(4*z),0.05*sin(4*z),z];
    geometry=struct('minCorner',[0 0 0],'voxelSize',[0.3 0.3 0.5]);
    qualified=true(8,1);
end
