classdef boundedRegistrationTest < matlab.unittest.TestCase
% boundedRegistrationTest Loss-gradient consistency and class influence checks.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function boundedLossHasTheImplementedGradient(testCase)
            [model,~]=fixture();
            at=model.linearize([.1 -.1 .01],ones(3,1));
            numerical=frozenGradient(model,at,[.1 -.1 .01]);
            testCase.verifyEqual(numerical,2*at.gradient,AbsTol=1e-5);
        end
        function GrossResidualHasFiniteMaximumCost(testCase)
            [model,system,cfg]=fixture();
            cost=model.frozenCost(system,[1000 1000 1]);
            testCase.verifyLessThanOrEqual(cost,cfg.geometric.robustStandardizedDistance^2*sum(system.weights)+1e-10);
        end
        function globalClassGateAccountsForTheOtherClasses(testCase)
            fixed=distributionRegistrationTest.exampleCloud();moving=fixed;moving.components.mean(2,:)=[0 1.5];
            cfg=distributionRegistrationConfig();cfg.geometric.robustLoss="switchable";
            original=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            cfg.geometric.classGateNormalization="global";
            globalGate=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyLessThanOrEqual(globalGate.classDiagnostics.informationWeightedCorrection,original.classDiagnostics.informationWeightedCorrection+1e-10);
            testCase.verifyEqual(globalGate.poseXYTheta,original.poseXYTheta,AbsTol=1e-12);
        end
        function unknownLossIsRejected(testCase)
            fixed=distributionRegistrationTest.exampleCloud();cfg=distributionRegistrationConfig();cfg.geometric.robustLoss="unknown";
            testCase.verifyError(@()prepareSemanticRegistrationGeometry(fixed,fixed,[0 0 0],cfg), ...
                'VehicleLocalization:InvalidRobustLoss');
        end
    end
end
function [model,system,cfg]=fixture()
    fixed=distributionRegistrationTest.exampleCloud();moving=distributionRegistrationTest.transform(fixed,[.15 -.1 .02]);cfg=distributionRegistrationConfig();cfg.geometric.robustLoss="switchable";
    model=prepareSemanticRegistrationGeometry(fixed,moving,[0 0 0],cfg);system=model.linearize([0 0 0],ones(3,1));
end
function g=frozenGradient(model,system,pose)
    g=zeros(3,1);h=1e-6;
    for k=1:3
        step=zeros(1,3);step(k)=h;g(k)=(model.frozenCost(system,pose+step)-model.frozenCost(system,pose-step))/(2*h);
    end
end
