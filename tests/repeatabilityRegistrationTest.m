classdef repeatabilityRegistrationTest < matlab.unittest.TestCase
% repeatabilityRegistrationTest Stored map mass is the sole stability prior.
    properties (TestParameter)
        dimension = {2,3}
        invalidRepeatability = struct('negative',[-.1;ones(5,1)], ...
            'aboveOne',[1.1;ones(5,1)],'nan',[NaN;ones(5,1)], ...
            'infinite',[Inf;ones(5,1)],'complex',[1i;ones(5,1)], ...
            'wrongLength',ones(5,1),'matrix',ones(2,3))
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function projectionRetainsStabilityWithoutChangingMass(testCase,dimension)
            cloud=heightProbabilityCloudTest.spatialCloud();
            cloud.components.repeatability=(1:6)/6;
            projected=registrationSupport.projectSemanticProbabilityCloud(cloud,dimension);
            testCase.verifyEqual(projected.components.repeatability,(1:6).'/6,'AbsTol',0);
            testCase.verifyEqual(projected.components.mixtureWeight,cloud.components.mixtureWeight,'AbsTol',0);
        end
        function rejectsMalformedStabilityMetadata(testCase,invalidRepeatability)
            cloud=distributionRegistrationTest.exampleCloud();
            cloud.components.repeatability=invalidRepeatability;
            testCase.verifyError(@() registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]), ...
                'VehicleLocalization:InvalidRepeatability');
        end
        function storedWeightsDoNotRequireRedundantRepeatability(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            fixed=cloud;fixed.components=rmfield(fixed.components,'repeatability');
            result=registerSemanticProbabilityCloud(fixed,cloud,[.1 .1 .01]);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,[0 0 0],AbsTol=1e-4);
        end
        function changingMetadataCannotDoubleDiscountMapMass(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            unit=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            fixed=cloud;fixed.components.repeatability=zeros(6,1);
            actual=registerSemanticProbabilityCloud(fixed,cloud,[0 0 0]);
            testCase.verifyEqual(actual.information,unit.information,AbsTol=0);
            testCase.verifyEqual(actual.similarity,unit.similarity,AbsTol=0);
            testCase.verifyEqual(actual.correspondences.weight,unit.correspondences.weight,AbsTol=0);
        end
        function existingPriorResolvesAmbiguousLandmarks(testCase)
            [moving,fixed]=ambiguousPoles();
            first=registerSemanticProbabilityCloud(fixed,moving,[.3 0 0]);
            fixed.components.mixtureWeight=flipud(fixed.components.mixtureWeight);
            second=registerSemanticProbabilityCloud(fixed,moving,[.3 0 0]);
            testCase.verifyTrue(first.accepted,first.reason);
            testCase.verifyTrue(second.accepted,second.reason);
            testCase.verifyEqual(first.poseXYTheta,[0 0 0],AbsTol=1e-4);
            testCase.verifyEqual(second.poseXYTheta,[.6 0 0],AbsTol=1e-4);
            testCase.verifyEqual(first.correspondences.target,(1:4).');
            testCase.verifyEqual(second.correspondences.target,(5:8).');
        end
        function commonPriorScaleCannotChangeTheSolutionOrInformation(testCase)
            [moving,fixed]=ambiguousPoles();
            first=registerSemanticProbabilityCloud(fixed,moving,[.3 0 0]);
            fixed.components.mixtureWeight=1e-200*fixed.components.mixtureWeight;
            second=registerSemanticProbabilityCloud(fixed,moving,[.3 0 0]);
            testCase.verifyEqual(second.poseXYTheta,first.poseXYTheta,AbsTol=1e-12);
            testCase.verifyEqual(second.information,first.information,AbsTol=1e-10);
        end
        function zeroStoredWeightDisablesTargets(testCase)
            [moving,fixed]=ambiguousPoles();
            fixed.components.mixtureWeight(1:4)=0;
            result=registerSemanticProbabilityCloud(fixed,moving,[.3 0 0]);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.correspondences.target,(5:8).');
            testCase.verifyEqual(result.poseXYTheta,[.6 0 0],AbsTol=1e-4);
        end
        function rejectsMapWithoutAnyPositiveStoredMass(testCase)
            cloud=distributionRegistrationTest.exampleCloud();fixed=cloud;
            fixed.components.mixtureWeight=zeros(6,1);
            testCase.verifyError(@()registerSemanticProbabilityCloud(fixed,cloud,[0 0 0]), ...
                'VehicleLocalization:ZeroMixtureMass');
        end
        function sourceRepeatabilityIsNotInventedForASingleScan(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            before=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            moving=cloud;moving.components.repeatability=zeros(6,1);
            after=registerSemanticProbabilityCloud(cloud,moving,[0 0 0]);
            testCase.verifyEqual(after.information,before.information,AbsTol=0);
        end
    end
end

function [moving,fixed]=ambiguousPoles()
    c=struct('mean',[-8 -8;-8 8;8 -8;8 8], ...
        'covariance',repmat(.2*eye(2),1,1,4),'semanticName',repmat("pole",4,1), ...
        'mixtureWeight',ones(4,1)/4,'numComponents',4);
    moving=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());fixed=moving;
    fixed.components=struct('mean',[c.mean;c.mean+[.6 0]], ...
        'covariance',repmat(c.covariance,1,1,2),'semanticName',repmat("pole",8,1), ...
        'mixtureWeight',[.9*ones(4,1);.1*ones(4,1)]/4,'numComponents',8);
end
