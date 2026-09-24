classdef fineHeightReferenceTest < matlab.unittest.TestCase
% fineHeightReferenceTest: Keep calibrated fine support bins after ground removal.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function groundRemovalKeepsTheSharedHistogramPhase(testCase)
            cfg=pillarGridConfig("offline");
            full=pillarizePointCloud([10 2 -2.13;10 2 -0.41;10 2 0.09;10 2 0.59],cfg);
            reference=min(full.points(:,3));
            allReturns=voxelizePillars(full,0.5,reference);
            nonground=pillarizePointCloud(full.points(2:end,:),cfg);
            fine=voxelizePillars(nonground,0.5,reference);
            testCase.verifyEqual(fine.pointVoxelSub,allReturns.pointVoxelSub(2:end,:));
            testCase.verifyEqual(fine.gridConfig.minCorner(3),-2.13);
            testCase.verifyEqual(fine.pointVoxelSub(:,3),int32([4;5;6]));
        end
        function rejectsReferenceAboveRetainedReturns(testCase)
            pillars=pillarizePointCloud([10 2 -1],pillarGridConfig("offline"));
            testCase.verifyError(@() voxelizePillars(pillars,0.5,0), ...
                'perception:InvalidHeightReference');
        end
        function exactTopEdgeRetainsItsOwnLayer(testCase)
            pillars=pillarizePointCloud([10 2 -0.75;10 2 0.25],pillarGridConfig("offline"));
            fine=voxelizePillars(pillars,0.5,-0.75);
            testCase.verifyEqual(fine.pointVoxelSub(:,3),int32([1;3]));
            testCase.verifyEqual(fine.gridConfig.dims(3),3);
        end
    end
end
