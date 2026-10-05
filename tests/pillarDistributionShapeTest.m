classdef pillarDistributionShapeTest < matlab.unittest.TestCase
% pillarDistributionShapeTest: Whole-distribution and rule-union contracts.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function translationAndPermutationPreserveStatistics(testCase)
            points=[0 0 0;.1 .05 1;.05 .1 2;0 .02 3;1 1 4;1.1 1.1 5];
            ids=int32([1;1;1;1;7;7]);order=[6 3 1 5 4 2];
            original=aggregatePillarDistributionShape(points,ids);
            changed=aggregatePillarDistributionShape(points(order,:)+[200 -300 50],ids(order));
            testCase.verifyEqual(original.pillarIndices,changed.pillarIndices);
            testCase.verifyEqual(original.values,changed.values,'AbsTol',1e-8);
        end
        function higherMomentsSeparateEqualVarianceDistributions(testCase)
            uniform=[-ones(5,1);ones(5,1)];clustered=[zeros(6,1);-sqrt(2.5)*ones(2,1);sqrt(2.5)*ones(2,1)];
            points=[zeros(20,2),[uniform;clustered]];
            summary=aggregatePillarDistributionShape(points,int32([ones(10,1);2*ones(10,1)]));
            testCase.verifyEqual(var(uniform,1),var(clustered,1),'AbsTol',1e-12);
            testCase.verifyEqual(summary.values(:,2),[1;2.5],'AbsTol',1e-12);
            testCase.verifyEqual(summary.values(:,1),[0;0],'AbsTol',1e-12);
        end
        function emptyAndDegeneratePopulationsRemainFinite(testCase)
            empty=aggregatePillarDistributionShape(zeros(0,3),int32([]));
            repeated=aggregatePillarDistributionShape(ones(5,3),int32(ones(5,1)));
            testCase.verifySize(empty.values,[0 8]);
            testCase.verifyEqual(repeated.values,zeros(1,8));
        end
        function alternativesRetainDistinctMorphologiesAndRejectMissingValues(testCase)
            first=struct('feature','height','operator','>=','threshold',2);
            second=struct('feature','fraction','operator','>','threshold',.8);
            rules=struct('alternatives',struct('predicates',{first,second}));
            accepted=evaluatePillarDistributionRules([3 0;1 .9;1 .8;NaN NaN], ...
                ["height","fraction"],rules);
            testCase.verifyEqual(accepted,[true;true;false;false]);
        end
        function historicalConjunctionRequiresEveryPredicate(testCase)
            rules=[struct('feature','height','operator','>=','threshold',2), ...
                struct('feature','fraction','operator','<=','threshold',.5)];
            accepted=evaluatePillarDistributionRules([3 .4;1 .4;3 .8], ...
                ["height","fraction"],rules);
            testCase.verifyEqual(accepted,[true;false;false]);
        end
    end
end
