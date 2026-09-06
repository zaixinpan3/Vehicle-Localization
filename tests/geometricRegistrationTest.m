classdef geometricRegistrationTest < matlab.unittest.TestCase
% geometricRegistrationTest: Sampling invariance and honest pose observability.
    properties (TestParameter)
        repeatedComponents = {1,200}
    end
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function repeatedTargetsPreserveFirstTieAcrossBlocks(testCase,repeatedComponents)
            cloud=repeatedCloud(repeatedComponents);
            cfg=geometricRegistrationTest.configuration();
            result=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0],cfg);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,[0 0 0],'AbsTol',1e-12);
            testCase.verifyEqual(result.correspondences.target,repmat((1:6).',repeatedComponents,1));
        end
        function recoversGaussianPoseAtLargeCoordinates(testCase)
            moving=distributionRegistrationTest.exampleCloud();
            expected=[700001.2 4300000.7 0.16];
            fixed=distributionRegistrationTest.transform(moving,expected);
            cfg=geometricRegistrationTest.configuration();
            result=registerSemanticProbabilityCloud(fixed,moving,expected+[-.6 .3 -.09],cfg);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,expected,'AbsTol',3e-4);
            testCase.verifyEqual(result.observableRank,3);
        end
        function ignoresPositiveComponentSamplingMass(testCase)
            moving=distributionRegistrationTest.exampleCloud();
            fixed=distributionRegistrationTest.transform(moving,[1.2 -.7 .16]);
            cfg=geometricRegistrationTest.configuration();
            before=registerSemanticProbabilityCloud(fixed,moving,[1 -.6 .12],cfg);
            fixed.components.mixtureWeight=[1e-8;1;1e3;1e7;1e-5;10];
            moving.components.mixtureWeight=flipud(fixed.components.mixtureWeight);
            after=registerSemanticProbabilityCloud(fixed,moving,[1 -.6 .12],cfg);
            testCase.verifyEqual(after.poseXYTheta,before.poseXYTheta,'AbsTol',0);
            testCase.verifyEqual(after.scaledCurvature,before.scaledCurvature,'AbsTol',0);
        end
        function parallelRoadKeepsPredictedLongitudinalPosition(testCase)
            cloud=geometricRegistrationTest.parallelRoad();
            cfg=geometricRegistrationTest.configuration();
            result=registerSemanticProbabilityCloud(cloud,cloud,[.8 .3 .02],cfg);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.reason,"degenerateGeometry");
            testCase.verifyEqual(result.observableRank,2);
            testCase.verifyTrue(result.partialPoseAvailable);
            testCase.verifyEqual(result.poseXYTheta,[.8 0 0],'AbsTol',1e-4);
            testCase.verifyEqual(result.observableProjector*[1;0;0],zeros(3,1),'AbsTol',1e-10);
        end
        function shiftingRoadDensityCannotCreateLongitudinalConstraint(testCase)
            moving=geometricRegistrationTest.parallelRoad();
            fixed=moving; fixed.components.mean(:,1)=fixed.components.mean(:,1)+1;
            cfg=geometricRegistrationTest.configuration();
            result=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyEqual(result.observableRank,2);
            testCase.verifyEqual(result.poseXYTheta,[0 0 0],'AbsTol',1e-10);
            testCase.verifyFalse(result.accepted);
        end
        function conflictingClassesCannotProduceAcceptedPose(testCase)
            moving=distributionRegistrationTest.exampleCloud();
            fixed=moving;
            fixed.components.mean(fixed.components.semanticName=="pole",2)= ...
                fixed.components.mean(fixed.components.semanticName=="pole",2)+1.8;
            cfg=geometricRegistrationTest.configuration();
            result=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyFalse(result.accepted);
            testCase.verifyGreaterThan(max(result.classDiagnostics.observableCorrection),cfg.geometric.maximumClassCorrection);
        end
        function heightRejectsAnotherFloor(testCase)
            moving=heightProbabilityCloudTest.spatialCloud();
            fixed=moving; fixed.components.mean(:,3)=fixed.components.mean(:,3)+5;
            cfg=geometricRegistrationTest.configuration(); cfg.heightMode="xyz"; cfg.heightTranslation=0;
            result=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.reason,"insufficientOverlap");
            testCase.verifyEmpty(result.correspondences);
        end
        function compatibleHeightDoesNotPullPlanarPose(testCase)
            moving=heightProbabilityCloudTest.spatialCloud();
            fixed=heightProbabilityCloudTest.transform(moving,[1.2 -.7 .16],100);
            fixed.components.mean(:,3)=fixed.components.mean(:,3)+.2;
            cfg=geometricRegistrationTest.configuration(); cfg.heightTranslation=100;
            planar=registerSemanticProbabilityCloud(fixed,moving,[1 -.6 .12],cfg);
            cfg.heightMode="xyz";
            spatial=registerSemanticProbabilityCloud(fixed,moving,[1 -.6 .12],cfg);
            testCase.verifyTrue(spatial.accepted,spatial.reason);
            testCase.verifyEqual(spatial.poseXYTheta,planar.poseXYTheta,'AbsTol',0);
            testCase.verifyFalse(spatial.height.planarForce);
            testCase.verifyEqual(spatial.height.medianConditionalResidual,.2,'AbsTol',1e-5);
        end
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
        function rejectsMismatchedCalibrationAndExposesLegacyProvenance(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            other=cloud; other.frameCalibration=lidarFrameCalibrationConfig();
            cfg=geometricRegistrationTest.configuration();
            legacy=registerSemanticProbabilityCloud(cloud,other,[0 0 0],cfg);
            other.frameCalibration.translation=[.1 0 0];
            testCase.verifyEqual(legacy.height.calibrationStatus,"unverifiedLegacy");
            testCase.verifyError(@() registerSemanticProbabilityCloud(cloud,other,[0 0 0],cfg), ...
                'VehicleLocalization:CalibrationMismatch');
        end
        function exportsCalibrationFromSelectedMapWindow(testCase)
            layer=struct('classLabel',"curb",'priorScore',1,'componentMeans',[0 0], ...
                'componentCovariances',eye(2),'componentSupportAmplitudes',1);
            calibration=lidarFrameCalibrationConfig(); calibration.translation=[.1 .2 .3];
            map=struct('layers',layer,'frameCalibration',calibration);
            cloud=temporalMapToProbabilityCloud(struct('batchMaps',struct('gmmMap',map)));
            testCase.verifyEqual(cloud.frameCalibration,calibration);
        end
        function rejectsInvalidSemanticQuality(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            cloud.components.semanticProbability=nan(6,1);
            cfg=geometricRegistrationTest.configuration();
            testCase.verifyError(@() registerSemanticProbabilityCloud(cloud,cloud,[0 0 0],cfg), ...
                'VehicleLocalization:InvalidSemanticQuality');
        end
        function recoversOfflinePitchWithoutAddingPoseStates(testCase)
            [observations,rotation]=geometricRegistrationTest.calibrationFixture();
            [calibration,report]=fitLidarPitchCalibration(observations);
            testCase.verifyEqual(calibration.rotation,rotation,'AbsTol',1e-7);
            testCase.verifyEqual(calibration.translation,[0 0 0],'AbsTol',0);
            testCase.verifyLessThan(report.rmsFrameMedianAfterM,1e-7);
            testCase.verifyEqual(report.onlineState,"X,Y,psi");
            testCase.verifyFalse(report.translationEstimated);
        end
        function refusesStationaryCalibrationData(testCase)
            [observations,~]=geometricRegistrationTest.calibrationFixture();
            observations.framePoseTable.odom_x_m(:)=0;
            testCase.verifyError(@() fitLidarPitchCalibration(observations), ...
                'VehicleLocalization:InsufficientCalibrationExcitation');
        end
        function projectionPreservesCalibrationAndGeometricEvidence(testCase)
            cloud=heightProbabilityCloudTest.spatialCloud();
            cloud.frameCalibration=lidarFrameCalibrationConfig();
            cloud.frameCalibration.translation=[.1 0 0];
            cloud.components.semanticProbability=(1:6).'/6;
            projected=projectSemanticProbabilityCloud(cloud,2);
            testCase.verifyEqual(projected.frameCalibration,cloud.frameCalibration);
            testCase.verifyEqual(projected.components.semanticProbability,cloud.components.semanticProbability);
            identity=projected; identity.frameCalibration=lidarFrameCalibrationConfig();
            testCase.verifyError(@() registerSemanticProbabilityCloud(identity,projected,[0 0 0]), ...
                'VehicleLocalization:CalibrationMismatch');
        end
    end
    methods (Static)
        function cfg=configuration()
            cfg=distributionRegistrationConfig(); cfg.method="geometricD2D";
        end
        function cloud=parallelRoad()
            means=[(-4:2:4).',zeros(5,1);(-4:2:4).',4*ones(5,1)];
            c=struct('mean',means,'covariance',repmat(diag([2 .01]),1,1,10), ...
                'semanticName',repmat("curb",10,1),'mixtureWeight',ones(10,1)/10,'numComponents',10);
            cloud=struct('components',c);
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

function cloud=repeatedCloud(copies)
    cloud=distributionRegistrationTest.exampleCloud(); c=cloud.components;
    cloud.components=struct('mean',repmat(c.mean,copies,1), ...
        'covariance',repmat(c.covariance,1,1,copies), ...
        'mixtureWeight',repmat(c.mixtureWeight,copies,1), ...
        'semanticName',repmat(c.semanticName,copies,1),'numComponents',6*copies);
end
