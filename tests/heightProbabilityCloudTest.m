classdef heightProbabilityCloudTest < matlab.unittest.TestCase
% heightProbabilityCloudTest: Retained height, map lifting, and XYZ D2D.
    properties (TestParameter)
        poseAxis = struct('x',1,'y',2,'yaw',3)
    end
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function retainsFullEmpiricalCovarianceAfterTilt(testCase)
            points=[1 2 3;2 1 5;3 4 4;5 3 8];
            angle=0.2;
            tilt=[cos(angle) 0 sin(angle);0 1 0;-sin(angle) 0 cos(angle)];
            expected=points*tilt.';
            moments=aggregatePlanarCellMoments(points,ones(4,1),1,tilt);
            actual=heightProbabilityCloudTest.unpack(moments);
            testCase.verifyEqual([moments.mean,moments.meanZ],mean(expected,1),'AbsTol',1e-12);
            testCase.verifyEqual(actual,cov(expected,1),'AbsTol',1e-12);
        end
        function emptyAndSingletonHaveDefinedHeightMoments(testCase)
            empty=aggregatePlanarCellMoments(zeros(0,3),zeros(0,1),2);
            one=aggregatePlanarCellMoments([1 2 3],1,2);
            testCase.verifyEqual(empty.meanZ,zeros(2,1),'AbsTol',0);
            testCase.verifyEqual(one.meanZ,[3;0],'AbsTol',0);
            testCase.verifyEqual(one.heightCovariance,zeros(2,3),'AbsTol',0);
        end
        function heightReferenceMatchesOfflineCoordinateTransform(testCase)
            row=array2table([10 -20 123 cos(0.1) 0 sin(0.1) 0], ...
                'VariableNames',{'odom_x_m','odom_y_m','odom_z_m','odom_qw','odom_qx','odom_qy','odom_qz'});
            [pose,tilt,height]=poseRowToPlanarPose(row);
            points=[1 2 3;4 -1 2];
            yaw=[cos(pose(3)) -sin(pose(3)) 0;sin(pose(3)) cos(pose(3)) 0;0 0 1];
            actual=points*tilt.'*yaw.'+[pose(1:2),height];
            testCase.verifyEqual(actual,registerPointsToGlobalFrame(points,row),'AbsTol',1e-12);
            testCase.verifyEqual(height,123,'AbsTol',0);
        end
        function planarMapDoesNotInventHeight(testCase)
            [cfg,points,labels,frames]=heightProbabilityCloudTest.mapFixture();
            map=buildTemporalStabilityGmmMap(points(:,1:2),labels,frames,cfg);
            cloud=temporalMapToProbabilityCloud(map);
            testCase.verifyFalse(any(cloud.components.heightAvailable));
            testCase.verifyTrue(all(isnan(cloud.components.meanXYZ(:,3))));
            testCase.verifyError(@() registrationSupport.projectSemanticProbabilityCloud(cloud,3), ...
                'VehicleLocalization:HeightUnavailable');
        end
        function aggregatesWithinAndBetweenPillarHeight(testCase)
            [ground,offGround,cfg,points]=heightProbabilityCloudTest.groundFixture();
            cloud=buildCoarseSemanticProbabilityCloud(ground,offGround,cfg);
            spatial=registrationSupport.projectSemanticProbabilityCloud(cloud,3);
            testCase.verifyEqual(spatial.components.mean,mean(points,1),'AbsTol',1e-12);
            testCase.verifyEqual(spatial.components.covariance,cov(points,1),'AbsTol',1e-10);
            testCase.verifyEqual(spatial.components.covariance(1:2,1:2),cloud.components.covariance,'AbsTol',0);
            testCase.verifyEqual(spatial.components.mixtureWeight,cloud.components.mixtureWeight,'AbsTol',0);
        end
        function heightLiftPreservesTemporalMapXYAndMass(testCase)
            [cfg,points,labels,frames]=heightProbabilityCloudTest.mapFixture();
            planar=buildTemporalStabilityGmmMap(points(:,1:2),labels,frames,cfg);
            spatial=buildTemporalStabilityGmmMap(points,labels,frames,cfg);
            xyCloud=temporalMapToProbabilityCloud(planar);
            jointCloud=temporalMapToProbabilityCloud(spatial);
            full=registrationSupport.projectSemanticProbabilityCloud(jointCloud,3);
            expectedZ=30+0.2*full.components.mean(:,1)-0.1*full.components.mean(:,2);
            testCase.verifyEqual(jointCloud.components.mean,xyCloud.components.mean,'AbsTol',0);
            testCase.verifyEqual(jointCloud.components.covariance,xyCloud.components.covariance,'AbsTol',0);
            testCase.verifyEqual(jointCloud.components.mixtureWeight,xyCloud.components.mixtureWeight,'AbsTol',0);
            testCase.verifyEqual(full.components.mean(:,3),expectedZ,'AbsTol',1e-10);
            testCase.verifyEqual(queryTemporalStabilityGmmMap(spatial,[1 0],"curb"), ...
                queryTemporalStabilityGmmMap(planar,[1 0],"curb"),'AbsTol',0);
        end
        function spatialGradientIncludesCrossCovariance(testCase,poseAxis)
            moving=heightProbabilityCloudTest.spatialCloud();
            fixed=heightProbabilityCloudTest.transform(moving,[1.2 -0.7 0.16],5);
            cfg=heightProbabilityCloudTest.exactHeightConfig(5);
            pose=[1.1 -0.6 0.12];
            step=zeros(1,3); step(poseAxis)=1e-6;
            [~,details]=scoreSemanticProbabilityCloudAlignment(fixed,moving,pose,cfg);
            numerical=(scoreSemanticProbabilityCloudAlignment(fixed,moving,pose+step,cfg)- ...
                scoreSemanticProbabilityCloudAlignment(fixed,moving,pose-step,cfg))/(2e-6);
            testCase.verifyEqual(details.gradient(poseAxis),numerical,'AbsTol',1e-7);
        end
        function recoversPlanarPoseWithKnownGlobalHeight(testCase)
            moving=heightProbabilityCloudTest.spatialCloud();
            expected=[700001.2 4300000.7 0.16];
            fixed=heightProbabilityCloudTest.transform(moving,expected,500);
            cfg=heightProbabilityCloudTest.exactHeightConfig(500);
            result=registerSemanticProbabilityCloud(fixed,moving,expected+[-0.5 0.3 -0.05],cfg);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyTrue(result.height.heightUsed);
            testCase.verifyEqual(result.poseXYTheta,expected,'AbsTol',3e-4);
        end
        function missingVerticalReferenceUsesExactXYMarginal(testCase)
            moving=heightProbabilityCloudTest.spatialCloud();
            fixed=heightProbabilityCloudTest.transform(moving,[1 -2 0.1],100);
            cfg=distributionRegistrationConfig(); cfg.heightMode="auto";
            [score,details]=scoreSemanticProbabilityCloudAlignment(fixed,moving,[0.9 -1.9 0.08],cfg);
            expected=scoreSemanticProbabilityCloudAlignment(registrationSupport.projectSemanticProbabilityCloud(fixed,2), ...
                registrationSupport.projectSemanticProbabilityCloud(moving,2),[0.9 -1.9 0.08]);
            testCase.verifyEqual(score,expected,'AbsTol',0);
            testCase.verifyEqual(details.height.heightReason,"verticalReferenceUnavailable");
        end
        function explicitXYZRejectsMissingHeight(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            cfg=heightProbabilityCloudTest.exactHeightConfig(0);
            testCase.verifyError(@() registerSemanticProbabilityCloud(cloud,cloud,[0 0 0],cfg), ...
                'VehicleLocalization:HeightUnavailable');
        end
        function defaultKeepsHeightWithoutEnablingExperimentalMatcher(testCase)
            cloud=heightProbabilityCloudTest.spatialCloud();
            cfg=distributionRegistrationConfig(); cfg.heightTranslation=0;
            [~,details]=scoreSemanticProbabilityCloudAlignment(cloud,cloud,[0 0 0],cfg);
            testCase.verifyFalse(details.height.heightUsed);
            testCase.verifyEqual(details.height.heightReason,"explicitXY");
        end
        function heightDistinguishesIdenticalXYDistributions(testCase)
            cloud=heightProbabilityCloudTest.spatialCloud();
            wrong=cloud; wrong.components.mean(:,3)=wrong.components.mean(:,3)+2;
            cfg=heightProbabilityCloudTest.exactHeightConfig(0);
            xy=scoreSemanticProbabilityCloudAlignment(wrong,cloud,[0 0 0]);
            xyz=scoreSemanticProbabilityCloudAlignment(wrong,cloud,[0 0 0],cfg);
            testCase.verifyEqual(xy,1,'AbsTol',1e-12);
            testCase.verifyLessThan(xyz,0.01);
        end
        function uncertaintySoftensSmallVerticalMismatch(testCase)
            cloud=heightProbabilityCloudTest.spatialCloud();
            shifted=cloud; shifted.components.mean(:,3)=shifted.components.mean(:,3)+0.3;
            cfg=heightProbabilityCloudTest.exactHeightConfig(0);
            exact=scoreSemanticProbabilityCloudAlignment(shifted,cloud,[0 0 0],cfg);
            cfg.heightStandardDeviation=0.3;
            tolerant=scoreSemanticProbabilityCloudAlignment(shifted,cloud,[0 0 0],cfg);
            testCase.verifyGreaterThan(tolerant,exact);
        end
        function invalidRetainedCovarianceIsRejected(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            cloud.components.meanXYZ=[cloud.components.mean,zeros(6,1)];
            cloud.components.covarianceXYZ=repmat(diag([1 1 -1]),1,1,6);
            cloud.components.heightAvailable=true(6,1);
            testCase.verifyError(@() mappingSupport.validateSemanticProbabilityCloud(cloud),'VehicleLocalization:InvalidCovariance');
        end
    end
    methods (Static)
        function covariance=unpack(m)
            a=m.covariance; h=m.heightCovariance;
            covariance=[a(1) a(2) h(1);a(2) a(3) h(2);h(1) h(2) h(3)];
        end
        function cfg=exactHeightConfig(height)
            cfg=distributionRegistrationConfig(); cfg.heightMode="xyz";
            cfg.heightTranslation=height; cfg.heightStandardDeviation=0; cfg.tiltStandardDeviation=0;
        end
        function cloud=spatialCloud()
            cloud=distributionRegistrationTest.exampleCloud();
            c=cloud.components.covariance(:,:,1); b=c*[0.4;-0.25];
            covariance=[c b;b.' 0.12^2+[0.4 -0.25]*b];
            cloud.components.mean=[cloud.components.mean,[0;0.8;-0.1;0.2;2;0.1]];
            cloud.components.covariance=repmat(covariance,1,1,6);
        end
        function cloud=transform(cloud,pose,height)
            r=[cos(pose(3)) -sin(pose(3)) 0;sin(pose(3)) cos(pose(3)) 0;0 0 1];
            cloud.components.mean=cloud.components.mean*r.'+[pose(1:2),height];
            for k=1:cloud.components.numComponents
                cloud.components.covariance(:,:,k)=r*cloud.components.covariance(:,:,k)*r.';
            end
        end
        function [ground,offGround,cfg,points]=groundFixture()
            points=[0.1 0.1 1;0.2 0.2 1.1;0.4 0.1 1.3;0.5 0.2 1.7;0.2 0.15 1.2;0.5 0.16 1.4];
            moments=aggregatePlanarCellMoments(points,[1;1;2;2;1;2],2);
            ground=struct('curbCellMask',[true true],'curbProbability',[0.8 0.8], ...
                'cellOrigin',[0 0],'cellSize',[0.3 0.3],'stats',struct('countMap',[3 3]),'moments',moments);
            offGround=struct(); cfg=coarseSemanticProbabilityCloudConfig();
            cfg.semanticNames="curb"; cfg.xMin=0; cfg.yMin=0; cfg.xMax=1; cfg.yMax=1;
            cfg.minCovarianceEigenvalue=1e-10; cfg.regularizationVariance=0;
            cfg.minimumConditionalHeightVariance=1e-10;
        end
        function [cfg,points,labels,frames]=mapFixture()
            cfg=temporalStabilityMapConfig(); cfg.classes="curb"; cfg.classParams=cfg.classParams([]);
            cfg.defaultParams.maxComponentsPerTile=1;
            cfg.defaultParams.minComponentPoints=2;
            x=repmat((0:0.75:3).',3,1); y=[zeros(5,1);0.05*ones(5,1);-0.04*ones(5,1)];
            points=[x,y,30+0.2*x-0.1*y]; labels=repmat("curb",15,1); frames=repelem((1:3).',5);
        end
    end
end
