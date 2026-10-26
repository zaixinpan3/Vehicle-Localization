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
    end
end
