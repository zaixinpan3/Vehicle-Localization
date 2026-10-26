classdef geometricRegistrationTest < matlab.unittest.TestCase
% geometricRegistrationTest Calibration transforms and shared spatial fixtures.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function transformsGeometryConsistentlyBeforeRecordedAttitude(testCase)
            points=[1 2 3;2 1 5;3 4 4;5 3 8];
            r=[cos(.1) 0 sin(.1);0 1 0;-sin(.1) 0 cos(.1)];
            tilt=[1 0 0;0 cos(.2) -sin(.2);0 sin(.2) cos(.2)];
            translation=[.2 -.1 .4];
            expected=(points*r.'+translation)*tilt.';
            moments=aggregatePlanarCellMoments(points,ones(4,1),1,tilt*r,translation*tilt.');
            testCase.verifyEqual([moments.mean,moments.meanZ],mean(expected,1),'AbsTol',1e-12);
            testCase.verifyEqual(heightProbabilityCloudTest.unpack(moments),cov(expected,1),'AbsTol',1e-12);
        end
        function exportsCalibrationFromSelectedMapWindow(testCase)
            cfg=temporalStabilityMapConfig(); cfg.classes="curb";
            calibration=lidarFrameCalibrationConfig(); calibration.translation=[.1 .2 .3];
            cfg.frameCalibration=calibration;
            map=buildTemporalStabilityGmmMap(zeros(0,2),strings(0,1),zeros(0,1),cfg);
            cloud=temporalMapToProbabilityCloud(map);
            testCase.verifyEqual(cloud.frameCalibration,calibration);
        end
        function recoversOfflinePitchWithoutAddingPoseStates(testCase)
            [observations,rotation]=geometricRegistrationTest.calibrationFixture();
            [calibration,report]=lidarCalibrationSupport.fitLidarPitchCalibration(observations);
            testCase.verifyEqual(calibration.rotation,rotation,'AbsTol',1e-7);
            testCase.verifyEqual(calibration.translation,[0 0 0],'AbsTol',0);
            testCase.verifyLessThan(report.rmsFrameMedianAfterM,1e-7);
            testCase.verifyEqual(report.onlineState,"X,Y,psi");
            testCase.verifyFalse(report.translationEstimated);
        end
        function refusesStationaryCalibrationData(testCase)
            [observations,~]=geometricRegistrationTest.calibrationFixture();
            observations.framePoseTable.odom_x_m(:)=0;
            testCase.verifyError(@() lidarCalibrationSupport.fitLidarPitchCalibration(observations), ...
                'VehicleLocalization:InsufficientCalibrationExcitation');
        end
    end
    methods (Static)
        function cloud=parallelRoad()
            means=[(-4:2:4).',zeros(5,1);(-4:2:4).',4*ones(5,1)];
            c=struct('mean',means,'covariance',repmat(diag([2 .01]),1,1,10), ...
                'semanticName',repmat("curb",10,1),'mixtureWeight',ones(10,1)/10,'numComponents',10);
            c.repeatability=ones(c.numComponents,1);
            cloud=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());
        end
        function [observations,rotation]=calibrationFixture()
            angle=deg2rad(3);
            rotation=[cos(angle) 0 -sin(angle);0 1 0;sin(angle) 0 cos(angle)];
            world=[(0:2:38).',zeros(20,1),zeros(20,1)];
            positions=[0;2;4;6]; points=cell(1,4);
            for j=1:4
                position=[positions(j) 0 0];
                stored=(world-position)*rotation;
                points{j}=stored+position;
            end
            poses=table(positions,zeros(4,1),zeros(4,1),ones(4,1),zeros(4,1),zeros(4,1),zeros(4,1), ...
                'VariableNames',{'odom_x_m','odom_y_m','odom_z_m','odom_qw','odom_qx','odom_qy','odom_qz'});
            observations=struct('featureNames',"curb",'pointsByFeatureFrame',{points}, ...
                'frameIndices',1:4,'framePoseTable',poses);
        end
    end
end
