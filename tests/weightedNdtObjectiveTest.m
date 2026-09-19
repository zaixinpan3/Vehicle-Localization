classdef weightedNdtObjectiveTest < matlab.unittest.TestCase
% weightedNdtObjectiveTest Stored GMM weights, soft overlap and exact derivatives.
    properties (TestParameter)
        poseAxis={1,2,3}
        dimension={2,3}
    end
    methods (TestClassSetup)
        function paths(testCase)
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(fileparts(mfilename('fullpath')))));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function explicitConfigurationSelectsCandidateWithoutChangingDefault(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            candidate=registerSemanticProbabilityCloud(cloud,cloud,[.1 .1 .01],weightedNdtRegistrationConfig());
            current=registerSemanticProbabilityCloud(cloud,cloud,[.1 .1 .01]);
            testCase.verifyTrue(candidate.accepted,candidate.reason);
            testCase.verifyEqual(candidate.weightSemantics,"existingMapMixtureWeightOnce");
            testCase.verifyEqual(current.weightSemantics,"classBalancedQualityTimesMapRepeatability");
            testCase.verifyFalse(candidate.informationCalibrated);
        end
        function analyticDerivativeMatchesCostDifference(testCase,poseAxis,dimension)
            model=weightedNdtObjectiveTest.model(dimension);pose=[.08 -.04 .02];
            delta=zeros(1,3);delta(poseAxis)=1e-6;
            [~,gradient]=semanticNdtSupport.evaluate(model,pose);
            numerical=(semanticNdtSupport.evaluate(model,pose+delta)- ...
                semanticNdtSupport.evaluate(model,pose-delta))/(2e-6);
            testCase.verifyEqual(gradient(poseAxis),numerical,'AbsTol',1e-8);
        end
        function observedCurvatureMatchesSecondCostDifference(testCase,poseAxis)
            model=weightedNdtObjectiveTest.model(2);pose=[.08 -.04 .02];
            delta=zeros(1,3);delta(poseAxis)=1e-5;
            h=semanticNdtSupport.curvature(model,pose);
            numerical=(semanticNdtSupport.evaluate(model,pose+delta)- ...
                2*semanticNdtSupport.evaluate(model,pose)+semanticNdtSupport.evaluate(model,pose-delta))/1e-10;
            testCase.verifyEqual(h(poseAxis,poseAxis),numerical,'AbsTol',1e-6);
        end
        function existingMapWeightScalesObjectiveOnce(testCase)
            model=weightedNdtObjectiveTest.model(2);low=model;
            low.groups{1}.fw=.25*model.groups{1}.fw;
            low.groups{2}.fw=.25*model.groups{2}.fw;
            low.groups{3}.fw=.25*model.groups{3}.fw;
            [base,gradient]=semanticNdtSupport.evaluate(model,[.1 .2 .03]);
            [actual,lowGradient]=semanticNdtSupport.evaluate(low,[.1 .2 .03]);
            testCase.verifyEqual(actual,.25*base,'AbsTol',1e-12);
            testCase.verifyEqual(lowGradient,.25*gradient,'AbsTol',1e-12);
        end
        function metadataDoesNotMultiplyStoredWeightAgain(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            first=registerWeightedNdtProbabilityCloud(cloud,cloud,[.1 .1 .01]);
            cloud.components.repeatability(:)=.01;
            second=registerWeightedNdtProbabilityCloud(cloud,cloud,[.1 .1 .01]);
            testCase.verifyEqual(second.poseXYTheta,first.poseXYTheta,'AbsTol',0);
            testCase.verifyEqual(second.information,first.information,'AbsTol',0);
        end
        function acceptsMapWithWeightsAndNoRedundantRepeatability(testCase)
            cloud=distributionRegistrationTest.exampleCloud();
            fixed=cloud;fixed.components=rmfield(fixed.components,'repeatability');
            result=registerWeightedNdtProbabilityCloud(fixed,cloud,[.1 .1 .01]);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.weightSemantics,"existingMapMixtureWeightOnce");
        end
        function mapWeightChangesSelectedAlignment(testCase)
            [moving,fixed]=weightedNdtObjectiveTest.ambiguousPoles();
            stable=registerWeightedNdtProbabilityCloud(fixed,moving,[.3 0 0]);
            fixed.components.mixtureWeight=flipud(fixed.components.mixtureWeight);
            swapped=registerWeightedNdtProbabilityCloud(fixed,moving,[.3 0 0]);
            testCase.verifyTrue(stable.accepted,stable.reason);
            testCase.verifyTrue(swapped.accepted,swapped.reason);
            testCase.verifyLessThan(abs(stable.poseXYTheta(1)),.1);
            testCase.verifyLessThan(abs(swapped.poseXYTheta(1)-.6),.1);
        end
        function splitMapComponentsPreserveTheSameField(testCase)
            [moving,fixed]=weightedNdtObjectiveTest.ambiguousPoles();
            before=registerWeightedNdtProbabilityCloud(fixed,moving,[.3 0 0]);
            c=fixed.components;
            fixed.components=struct('mean',repmat(c.mean,2,1), ...
                'covariance',repmat(c.covariance,1,1,2), ...
                'semanticName',repmat(c.semanticName,2,1), ...
                'mixtureWeight',repmat(c.mixtureWeight/2,2,1),'numComponents',2*c.numComponents);
            after=registerWeightedNdtProbabilityCloud(fixed,moving,[.3 0 0]);
            testCase.verifyEqual(after.poseXYTheta,before.poseXYTheta,'AbsTol',1e-6);
            testCase.verifyEqual(after.information,before.information,'AbsTol',1e-7);
        end
    end
    methods (Static)
        function model=model(dimension)
            cloud=heightProbabilityCloudTest.spatialCloud();
            fixed=registrationSupport.projectSemanticProbabilityCloud(cloud,dimension).components;
            cfg=weightedNdtRegistrationConfig();
            model=semanticNdtSupport.prepare(fixed,fixed,cfg.ndt);
        end
        function [moving,fixed]=ambiguousPoles()
            c=struct('mean',[-8 -8;-8 8;8 -8;8 8], ...
                'covariance',repmat(.1*eye(2),1,1,4),'semanticName',repmat("pole",4,1), ...
                'mixtureWeight',ones(4,1)/4,'numComponents',4);
            moving=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());fixed=moving;
            fixed.components=struct('mean',[c.mean;c.mean+[.6 0]], ...
                'covariance',repmat(c.covariance,1,1,2),'semanticName',repmat("pole",8,1), ...
                'mixtureWeight',[.9*c.mixtureWeight;.1*c.mixtureWeight],'numComponents',8);
        end
    end
end
