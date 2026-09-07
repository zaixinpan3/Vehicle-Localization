classdef repeatabilityRegistrationTest < matlab.unittest.TestCase
% repeatabilityRegistrationTest Map stability affects geometric pose evidence.
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
            projected=projectSemanticProbabilityCloud(cloud,dimension);
            testCase.verifyEqual(projected.components.repeatability,(1:6).'/6,'AbsTol',0);
            testCase.verifyEqual(projected.components.mixtureWeight,cloud.components.mixtureWeight,'AbsTol',0);
        end
        function rejectsMalformedStability(testCase,invalidRepeatability)
            cloud=distributionRegistrationTest.exampleCloud();
            cloud.components.repeatability=invalidRepeatability;
            testCase.verifyError(@() registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]), ...
                'VehicleLocalization:InvalidRepeatability');
        end
        function absentStabilityPreservesUnitWeightBehavior(testCase)
            moving=distributionRegistrationTest.exampleCloud();
            fixed=distributionRegistrationTest.transform(moving,[1.2 -.7 .16]);
            legacy=registerSemanticProbabilityCloud(fixed,moving,[1 -.6 .12]);
            fixed.components.repeatability=ones(6,1);
            current=registerSemanticProbabilityCloud(fixed,moving,[1 -.6 .12]);
            testCase.verifyTrue(current.accepted,current.reason);
            testCase.verifyEqual(current.poseXYTheta,legacy.poseXYTheta,'AbsTol',0);
            testCase.verifyEqual(current.scaledCurvature,legacy.scaledCurvature,'AbsTol',0);
            testCase.verifyEqual(current.similarity,legacy.similarity,'AbsTol',0);
            testCase.verifyEqual(legacy.repeatabilitySource,"legacyUnitWeight");
            testCase.verifyEqual(current.repeatabilitySource,"mapPosterior");
        end
        function classBalancingDoesNotEraseUniformLowStability(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            unit=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            fixed=cloud; fixed.components.repeatability=.25*ones(6,1);
            low=registerSemanticProbabilityCloud(fixed,cloud,[0 0 0]);
            testCase.verifyTrue(low.accepted,low.reason);
            testCase.verifyEqual(low.scaledCurvature,.25*unit.scaledCurvature,'AbsTol',1e-12);
            testCase.verifyEqual(low.similarity,.25*unit.similarity,'AbsTol',1e-12);
            testCase.verifyEqual(low.correspondences.weight,.25*unit.correspondences.weight,'AbsTol',1e-12);
        end
        function preservesAbsoluteReliabilityBetweenClasses(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            fixed=cloud; fixed.components.repeatability=ones(6,1);
            fixed.components.repeatability(fixed.components.semanticName=="pole")=.25;
            result=registerSemanticProbabilityCloud(fixed,cloud,[0 0 0]);
            pairs=result.correspondences;
            testCase.verifyEqual(sum(pairs.weight(pairs.semanticName=="pole")),1/12,'AbsTol',1e-12);
            testCase.verifyEqual(sum(pairs.weight(pairs.semanticName=="curb")),1/3,'AbsTol',1e-12);
            testCase.verifyEqual(result.similarity,.75,'AbsTol',1e-12);
        end
        function stableLandmarksDominateConflictingSameClassLandmarks(testCase)
            [moving,fixed]=conflictingPoles();
            equal=registerSemanticProbabilityCloud(fixed,moving,[0 0 0]);
            fixed.components.repeatability=[ones(4,1);.1*ones(4,1)];
            stable=registerSemanticProbabilityCloud(fixed,moving,[0 0 0]);
            fixed.components.repeatability=flipud(fixed.components.repeatability);
            swapped=registerSemanticProbabilityCloud(fixed,moving,[0 0 0]);
            testCase.verifyTrue(stable.accepted,stable.reason);
            testCase.verifyTrue(swapped.accepted,swapped.reason);
            testCase.verifyEqual(equal.poseXYTheta,[.3 0 0],'AbsTol',2e-4);
            testCase.verifyLessThan(abs(stable.poseXYTheta(1)),.08);
            testCase.verifyLessThan(abs(swapped.poseXYTheta(1)-.6),.08);
            testCase.verifyEqual(stable.poseXYTheta(2:3),[0 0],'AbsTol',1e-8);
            testCase.verifyEqual(stable.correspondences.weight(1)/stable.correspondences.weight(5),10,'AbsTol',1e-12);
            testCase.verifyEqual(stable.correspondences.robustWeight,stable.correspondences.weight./ ...
                (1+stable.correspondences.squaredStandardizedResidual/2.5^2),'AbsTol',1e-12);
        end
        function positiveSamplingMassCannotReplaceStability(testCase)
            [moving,fixed]=conflictingPoles();
            fixed.components.repeatability=[ones(4,1);.1*ones(4,1)];
            before=registerSemanticProbabilityCloud(fixed,moving,[0 0 0]);
            fixed.components.mixtureWeight=10.^(-4:3).';
            after=registerSemanticProbabilityCloud(fixed,moving,[0 0 0]);
            testCase.verifyEqual(after.poseXYTheta,before.poseXYTheta,'AbsTol',0);
            testCase.verifyEqual(after.scaledCurvature,before.scaledCurvature,'AbsTol',0);
        end
        function zeroStabilityDisablesTargets(testCase)
            [moving,fixed]=conflictingPoles();
            fixed.components.repeatability=[ones(4,1);zeros(4,1)];
            result=registerSemanticProbabilityCloud(fixed,moving,[0 0 0]);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.correspondences.target,(1:4).');
            testCase.verifyEqual(result.poseXYTheta,[0 0 0],'AbsTol',1e-12);
        end
        function allZeroStabilityRejectsMeasurement(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            fixed=cloud; fixed.components.repeatability=zeros(6,1);
            result=registerSemanticProbabilityCloud(fixed,cloud,[0 0 0]);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.reason,"insufficientOverlap");
            testCase.verifyEmpty(result.correspondences);
        end
        function onlyMapStabilityDiscountsSingleFrameEvidence(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            before=registerSemanticProbabilityCloud(cloud,cloud,[0 0 0]);
            moving=cloud; moving.components.repeatability=zeros(6,1);
            after=registerSemanticProbabilityCloud(cloud,moving,[0 0 0]);
            testCase.verifyEqual(after.scaledCurvature,before.scaledCurvature,'AbsTol',0);
            testCase.verifyEqual(after.similarity,before.similarity,'AbsTol',0);
        end
    end
end

function [moving,fixed]=conflictingPoles()
% Two symmetric, separated groups constrain the same translation differently.
    means=[-8 -8;-8 8;8 -8;8 8;-16 -16;-16 16;16 -16;16 16];
    c=struct('mean',means,'covariance',repmat(.2*eye(2),1,1,8), ...
        'semanticName',repmat("pole",8,1),'mixtureWeight',ones(8,1)/8,'numComponents',8);
    moving=struct('components',c); fixed=moving;
    fixed.components.mean(5:8,1)=fixed.components.mean(5:8,1)+.6;
end
