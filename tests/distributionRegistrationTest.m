classdef distributionRegistrationTest < matlab.unittest.TestCase
% distributionRegistrationTest: Analytic D2D geometry, map units and failure gates.
    properties (TestParameter)
        dimension = struct('xy',2,'xyz',3)
    end
    methods (TestClassSetup)
        function paths(~)
            run(fullfile(fileparts(fileparts(mfilename('fullpath'))),'setupVehicleLocalization.m'));
        end
    end
    methods (Test)
        function zeroMassComponentsPreserveScoreAndGradient(testCase,dimension)
            source=heightProbabilityCloudTest.spatialCloud();
            target=heightProbabilityCloudTest.transform(source,[1.2 -0.7 0.16],0);
            moving=registrationSupport.projectSemanticProbabilityCloud(source,dimension);
            fixed=registrationSupport.projectSemanticProbabilityCloud(target,dimension);
            cfg=distributionRegistrationConfig(); cfg.heightMode="auto"; cfg.heightTranslation=0;
            pose=[1.1 -0.6 0.12];
            [expected,reference]=scoreSemanticProbabilityCloudAlignment(fixed,moving,pose,cfg);
            [actual,details]=scoreSemanticProbabilityCloudAlignment( ...
                testCase.withZeroMassComponents(fixed),testCase.withZeroMassComponents(moving),pose,cfg);
            testCase.verifyEqual(actual,expected,'AbsTol',1e-12);
            testCase.verifyEqual(details.gradient,reference.gradient,'AbsTol',1e-12);
        end
        function energyIsIndependentOfRequestedGradient(testCase,dimension)
            cloud=registrationSupport.projectSemanticProbabilityCloud(heightProbabilityCloudTest.spatialCloud(),dimension);
            energy=registrationSupport.semanticGaussianOverlap(cloud.components,cloud.components,[0.1 -0.2 0.03]);
            [withGradient,gradient]=registrationSupport.semanticGaussianOverlap(cloud.components,cloud.components,[0.1 -0.2 0.03]);
            testCase.verifyEqual(energy,withGradient,'AbsTol',0);
            testCase.verifyTrue(all(isfinite(gradient)));
        end
        function analyticGradientIncludesCovarianceRotation(testCase)
            moving = distributionRegistrationTest.exampleCloud();
            fixed = distributionRegistrationTest.transform(moving,[1.2 -0.7 0.16]);
            pose = [1.1 -0.6 0.12];
            [~,details] = scoreSemanticProbabilityCloudAlignment(fixed,moving,pose);
            numerical = zeros(1,3);
            for k=1:3
                step=zeros(1,3); step(k)=1e-6;
                numerical(k)=(scoreSemanticProbabilityCloudAlignment(fixed,moving,pose+step)- ...
                    scoreSemanticProbabilityCloudAlignment(fixed,moving,pose-step))/(2e-6);
            end
            testCase.verifyEqual(details.gradient,numerical,'AbsTol',1e-7);
        end
        function recoversKnownPoseAtLargeMapCoordinates(testCase)
            moving = distributionRegistrationTest.exampleCloud();
            expected = [700001.2 4300000.7 0.16];
            fixed = distributionRegistrationTest.transform(moving,expected);
            result = registerSemanticProbabilityCloud(fixed,moving,expected+[-0.6 0.3 -0.09]);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,expected,'AbsTol',3e-4);
            testCase.verifyGreaterThan(result.similarity,0.99999);
        end
        function classBalanceIsInvariantToSemanticMassScale(testCase)
            moving=distributionRegistrationTest.exampleCloud();
            fixed=distributionRegistrationTest.transform(moving,[1.2 -0.7 0.16]);
            pose=[1.0 -0.6 0.12];
            before=scoreSemanticProbabilityCloudAlignment(fixed,moving,pose);
            fixed.components.mixtureWeight(fixed.components.semanticName=="curb")=1e6;
            moving.components.mixtureWeight(moving.components.semanticName=="pole")=1e-6;
            after=scoreSemanticProbabilityCloudAlignment(fixed,moving,pose);
            testCase.verifyEqual(after,before,'AbsTol',1e-12);
        end
        function rejectsUnobservableYaw(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            cloud.components.mean(:)=0;
            cloud.components.covariance=repmat(eye(2),1,1,6);
            result=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.reason,"degenerateGeometry");
        end
        function rejectsNoOverlapAndIncompatibleSemantics(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            result=registerSemanticProbabilityCloud(cloud,cloud,[100 100 0]);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.reason,"insufficientOverlap");
            other=cloud; other.components.semanticName(:)="unmatched";
            result=registerSemanticProbabilityCloud(other,cloud,[0 0 0]);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.reason,"insufficientComponents");
        end
        function supportsSingletonAndRejectsInvalidCovariance(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            c=cloud.components;
            cloud.components=struct('mean',c.mean(1,:),'covariance',c.covariance(:,:,1), ...
                'semanticName',c.semanticName(1),'mixtureWeight',1,'numComponents',1);
            testCase.verifyEqual(scoreSemanticProbabilityCloudAlignment(cloud,cloud,[0 0 0]),1,'AbsTol',1e-12);
            cloud.components.covariance=[1 2;2 1];
            testCase.verifyError(@() mappingSupport.validateSemanticProbabilityCloud(cloud),'VehicleLocalization:InvalidCovariance');
        end
        function temporalExportUsesIntegratedSupportAndOneWindow(testCase)
            layer=struct('classLabel',"curb",'priorScore',0.5, ...
                'componentMeans',[0 0;2 0],'componentCovariances',cat(3,eye(2),4*eye(2)), ...
                'componentSupportAmplitudes',[0.8;0.4],'componentMixtureWeights',[0.99;0.01]);
            map=struct('layers',layer);
            batches=struct('batchMaps',repmat(struct('gmmMap',map),2,1));
            cloud=temporalMapToProbabilityCloud(batches,2);
            testCase.verifyEqual(cloud.components.numComponents,2);
            testCase.verifyEqual(cloud.components.mixtureWeight,[1;2]/3,'AbsTol',1e-12);
            testCase.verifyEqual(cloud.sourceBatchIndex,2);
            testCase.verifyEqual(map.layers.componentMixtureWeights,[0.99;0.01]);
        end
        function emptyMapProducesRejectedResult(testCase)
            empty=temporalMapToProbabilityCloud([]);
            testCase.verifyEqual(scoreSemanticProbabilityCloudAlignment(empty,empty,[0 0 0]),0);
            result=registerSemanticProbabilityCloud(empty,empty,[0 0 0]);
            testCase.verifyFalse(result.accepted);
        end
    end
    methods (Static)
        function cloud=withZeroMassComponents(cloud)
            c=cloud.components;
            names=c.semanticName; names(end)="zeroMassOnly";
            cloud.components=struct('mean',[c.mean;c.mean+7], ...
                'covariance',cat(3,c.covariance,c.covariance), ...
                'semanticName',[c.semanticName;names], ...
                'mixtureWeight',[c.mixtureWeight;zeros(c.numComponents,1)], ...
                'numComponents',2*c.numComponents);
        end
        function cloud=exampleCloud()
            means=[-2 -1;0 0;2 1;4 -1;0 4;5 3];
            covariance=repmat([0.3 0.07;0.07 0.04],1,1,6);
            c=struct('mean',means,'covariance',covariance, ...
                'semanticName',["curb";"pole";"roadMarking";"curb";"pole";"roadMarking"], ...
                'mixtureWeight',ones(6,1)/6,'numComponents',6);
            cloud=struct('components',c);
        end
        function cloud=transform(cloud,pose)
            r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
            cloud.components.mean=cloud.components.mean*r.'+pose(1:2);
            for k=1:cloud.components.numComponents
                cloud.components.covariance(:,:,k)=r*cloud.components.covariance(:,:,k)*r.';
            end
        end
    end
end
