classdef fineMatchingCloudTest < matlab.unittest.TestCase
% fineMatchingCloudTest Coordinate and support contracts of the test adapter.
    methods (TestClassSetup)
        function paths(testCase)
            here=fileparts(mfilename('fullpath'));root=fileparts(fileparts(here));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(here));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function excludesUnselectedOutlierAndUsesPopulationCovariance(testCase)
            [frame,masks,cfg]=fixture();
            result=buildFineMatchingCloud(frame,masks,cfg);
            testCase.verifyEqual(result.components.mean,[1 0],'AbsTol',1e-12);
            testCase.verifyEqual(result.components.count,2);
            testCase.verifyEqual(result.components.covariance,diag([1 .01]),'AbsTol',1e-12);
            testCase.verifyEqual(result.components.semanticProbability,1);
        end
        function appliesCalibrationThenTiltExactlyOnce(testCase)
            [frame,masks,cfg]=fixture();
            cfg.frameCalibration.rotation=[0 -1 0;1 0 0;0 0 1];
            cfg.frameCalibration.translation=[2 0 1];
            cfg.coarseProbabilityCloud.projectionRotation=[1 0 0;0 0 -1;0 1 0];
            cfg.coarseProbabilityCloud.projectionTranslation=[.2 .3 0];
            result=buildFineMatchingCloud(frame,masks,cfg);
            testCase.verifyEqual(result.components.mean,[2.2 -3.7],'AbsTol',1e-12);
            testCase.verifyEqual(result.components.count,2);
            testCase.verifyEqual(result.components.covariance,.01*eye(2),'AbsTol',1e-12);
        end
        function emptyFeaturesRemainEmpty(testCase)
            [frame,masks,cfg]=fixture();masks.pole(:)=false;
            result=buildFineMatchingCloud(frame,masks,cfg);
            testCase.verifyEqual(result.components.numComponents,0);
            testCase.verifySize(result.components.mean,[0 2]);
        end
        function unorganizedOrderDoesNotChangeStatistics(testCase)
            [frame,masks,cfg]=fixture();
            ordered=buildFineMatchingCloud(frame,masks,cfg);
            shuffled=struct('x',frame.x([3 2 1]),'y',frame.y([3 2 1]),'z',frame.z([3 2 1]));
            selected=struct('pole',masks.pole([3 2 1]));
            result=buildFineMatchingCloud(shuffled,selected,cfg);
            testCase.verifyEqual(result.components,ordered.components);
        end
    end
end

function [frame,masks,cfg]=fixture()
    frame=struct('x',[0;2;50],'y',[0;0;40],'z',[3;3;2]);masks=struct('pole',[true;true;false]);
    cfg=perceptionConfig();cfg.featureNames="pole";
    cfg.coarseProbabilityCloud.xMin=-10;cfg.coarseProbabilityCloud.xMax=10;
    cfg.coarseProbabilityCloud.yMin=-10;cfg.coarseProbabilityCloud.yMax=10;
    cfg.coarseProbabilityCloud.resolution=20;cfg.coarseProbabilityCloud.regularizationVariance=0;
end
