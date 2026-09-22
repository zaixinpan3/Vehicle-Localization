classdef lidarOriginCalibrationTest < matlab.unittest.TestCase
% lidarOriginCalibrationTest Preserve raw geometry across reference changes.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function offlineAndOnlineSelectTheSameProfile(testCase)
            offline=featureMapBuildConfig();online=perceptionConfig("Mississippi");
            testCase.verifyEqual(offline.frameCalibration,online.frameCalibration);
            testCase.verifyGreaterThan(norm(online.frameCalibration.translation(1:2)),.5);
            identity=lidarFrameCalibrationConfig();testCase.verifyEqual(identity.translation,[0,0,0]);
        end
        function recoversOffsetFromTurningSensorPoses(testCase)
            [q,f,m,b]=lidarOriginCalibrationTest.turningPairs();
            [calibration,report]=fitLidarTranslationCalibration(q,f,m);
            testCase.verifyEqual(calibration.translation,[b.',0],AbsTol=1e-10);
            testCase.verifyLessThan(report.afterRmseM,1e-10);
            testCase.verifyFalse(report.verticalTranslationEstimated);
        end
        function isolatedWrongPairDoesNotDominateFit(testCase)
            [q,f,m,b]=lidarOriginCalibrationTest.turningPairs();
            m(end,1:2)=m(end,1:2)+[4,-3];
            calibration=fitLidarTranslationCalibration(q,f,m);
            testCase.verifyLessThan(norm(calibration.translation(1:2).'-b),.06);
        end
        function straightMotionDoesNotIdentifyOffset(testCase)
            [q,f,m,~]=lidarOriginCalibrationTest.turningPairs();q(:,3)=f(:,3);
            testCase.verifyError(@()fitLidarTranslationCalibration(q,f,m), ...
                'VehicleLocalization:InsufficientCalibrationExcitation');
        end
        function recalibrationReversesOldRotationAndTranslation(testCase)
            [data,poses,calibration,expected]=lidarOriginCalibrationTest.reprojectionFixture();
            [actual,metadata]=reprojectSavedFeatureObservations(data,poses,FrameCalibration=calibration);
            testCase.verifyEqual(actual.pointsByFeatureFrame{1},expected,AbsTol=1e-10);
            testCase.verifyEqual(actual.frameCalibration,calibration);
            testCase.verifyLessThan(metadata.maximumLocalPointDifferenceM,1e-10);
        end
        function rebuildingCalibratedPointsDoesNotApplyOffsetTwice(testCase)
            [data,poses,calibration,expected]=lidarOriginCalibrationTest.reprojectionFixture();
            first=reprojectSavedFeatureObservations(data,poses,FrameCalibration=calibration);
            second=reprojectSavedFeatureObservations(first,poses,FrameCalibration=calibration);
            testCase.verifyEqual(second.pointsByFeatureFrame{1},expected,AbsTol=1e-10);
        end
        function changingUnknownSourceCalibrationIsRejected(testCase)
            [data,poses,calibration,~]=lidarOriginCalibrationTest.reprojectionFixture();data=rmfield(data,'frameCalibration');
            testCase.verifyError(@()reprojectSavedFeatureObservations(data,poses,FrameCalibration=calibration), ...
                'VehicleLocalization:MissingCalibration');
        end
    end
    methods (Static)
        function [q,f,m,b]=turningPairs()
            yaw=linspace(-1,1,20).';q=[(1:20).',zeros(20,1),yaw+.4];f=[zeros(20,2),yaw];b=[2.3;-.15];m=q;
            for k=1:20
                Rq=[cos(q(k,3)),-sin(q(k,3));sin(q(k,3)),cos(q(k,3))];
                Rf=[cos(f(k,3)),-sin(f(k,3));sin(f(k,3)),cos(f(k,3))];
                % Relative observed sensor motion, expressed in the fixed
                % frame and then interpreted as a reference-point pose.
                sensorQ=q(k,1:2).'+Rq*b;sensorF=f(k,1:2).'+Rf*b;
                relativeSensor=Rf.'*(sensorQ-sensorF);
                m(k,1:2)=(f(k,1:2).'+Rf*relativeSensor).';
            end
        end
        function [data,poses,calibration,expected]=reprojectionFixture()
            old=lidarFrameCalibrationConfig();old.rotation=[0,-1,0;1,0,0;0,0,1];old.translation=[.4,-.2,.1];
            raw=[1,2,3;-2,4,5;3,0,-1];oldPoses=pose(0,[10,20,30]);
            data=struct('frameIndices',1,'framePoseTable',oldPoses,'featureNames',"pole", ...
                'pointsByFeatureFrame',{{raw*old.rotation.'+old.translation+[10,20,30]}}, ...
                'counts',3,'frameCalibration',old);
            poses=pose(pi/2,[100,200,300]);calibration=lidarFrameCalibrationConfig();calibration.translation=[2.3,.15,0];
            [R,t]=poseRowToRigidTransform(poses);expected=(raw+calibration.translation)*R.'+t;
        end
    end
end

function row=pose(yaw,t)
    row=table(1,t(1),t(2),t(3),cos(yaw/2),0,0,sin(yaw/2), ...
        VariableNames={'frame_index','pose_x_m','pose_y_m','pose_z_m','pose_qw','pose_qx','pose_qy','pose_qz'});
end
