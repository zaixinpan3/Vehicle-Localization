classdef inspvaMappingPoseTest < matlab.unittest.TestCase
% inspvaMappingPoseTest Pose precedence and cached-feature reprojection.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function explicitPoseOverridesStaleOdom(testCase)
            row=inspvaMappingPoseTest.canonicalPose();
            row.odom_x_m=1e6;row.odom_y_m=-1e6;row.odom_z_m=100;
            row.odom_qw=1;row.odom_qx=0;row.odom_qy=0;row.odom_qz=0;
            actual=registerPointsToGlobalFrame([1,0,0;0,1,0],row);
            [pose,tilt,height]=poseRowToPlanarPose(row);
            testCase.verifyEqual(actual,[10,21,30;9,20,30],'AbsTol',1e-12);
            testCase.verifyEqual(pose,[10,20,pi/2],'AbsTol',1e-12);
            testCase.verifyEqual(tilt,eye(3),'AbsTol',1e-12);
            testCase.verifyEqual(height,30,'AbsTol',0);
        end
        function incompleteCanonicalPoseCannotFallBack(testCase)
            row=inspvaMappingPoseTest.canonicalPose();row.pose_qz=[];
            row.odom_x_m=0;row.odom_y_m=0;row.azimuth_deg=0;
            testCase.verifyError(@()poseRowToPlanarPose(row),'VehicleLocalization:IncompletePose');
        end
        function invalidCanonicalQuaternionIsRejected(testCase)
            row=inspvaMappingPoseTest.canonicalPose();row.pose_qw=0;row.pose_qz=0;
            testCase.verifyError(@()registerPointsToGlobalFrame([1,2,3],row),'VehicleLocalization:InvalidPose');
        end
        function reprojectionPreservesLocalFeatureGeometry(testCase)
            old=array2table([1,100,-20,4,1,0,0,0],VariableNames= ...
                {'frame_index','odom_x_m','odom_y_m','odom_z_m','odom_qw','odom_qx','odom_qy','odom_qz'});
            data=struct('frameIndices',1,'featureNames',"curb",'framePoseTable',old, ...
                'pointsByFeatureFrame',{{[101,-18,7;104,-21,6]}},'counts',2);
            replacement=inspvaMappingPoseTest.canonicalPose();replacement.frame_index=1;
            [actual,metadata]=reprojectSavedFeatureObservations(data,replacement);
            testCase.verifyEqual(actual.pointsByFeatureFrame{1},[8,21,33;11,24,32],'AbsTol',1e-12);
            testCase.verifyEqual(actual.counts,data.counts,'AbsTol',0);
            testCase.verifyLessThan(metadata.maximumLocalPointDifferenceM,1e-12);
        end
        function reprojectionRejectsMisorderedFrames(testCase)
            data=struct('frameIndices',[1,2]);replacement=table([2;1],VariableNames={'frame_index'});
            testCase.verifyError(@()reprojectSavedFeatureObservations(data,replacement),'VehicleLocalization:FrameMismatch');
        end
    end
    methods (Static,Access=private)
        function row=canonicalPose()
            row=array2table([10,20,30,sqrt(.5),0,0,sqrt(.5)],VariableNames= ...
                {'pose_x_m','pose_y_m','pose_z_m','pose_qw','pose_qx','pose_qy','pose_qz'});
        end
    end
end
