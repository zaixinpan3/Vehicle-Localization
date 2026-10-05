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
        function intersectedMorphologyGroupsRequireBothStages(testCase)
            a=struct('alternatives',struct('predicates', ...
                struct('feature','height','operator','>=','threshold',2)));
            b=struct('alternatives',struct('predicates', ...
                struct('feature','fraction','operator','<=','threshold',.2)));
            rules=struct('allOf',{{a,b}});
            accepted=evaluatePillarDistributionRules([3 .1;1 .1;3 .5;3 NaN], ...
                ["height","fraction"],rules);
            testCase.verifyEqual(accepted,[true;false;false;false]);
        end
        function jsonIntersectionMatchesExpandedPredicateUnions(testCase)
            a=struct('alternatives',struct('predicates', ...
                struct('feature','height','operator','>=','threshold',2)));
            alternatives=struct('predicates',{ ...
                struct('feature','fraction','operator','<=','threshold',.2), ...
                struct('feature','fraction','operator','>=','threshold',.8)});
            b=struct('alternatives',alternatives);
            rules=jsondecode(jsonencode(struct('allOf',[a,b])));
            points=[3 .1;3 .9;3 .5;1 .1;NaN .9];
            expected=evaluatePillarDistributionRules(points,["height","fraction"],a) & ...
                evaluatePillarDistributionRules(points,["height","fraction"],b);
            testCase.verifyEqual(evaluatePillarDistributionRules(points,["height","fraction"],rules),expected);
        end
        function empiricalCdfDetectsClusteredHeightMass(testCase)
            z=[linspace(0,1,10).';zeros(5,1);ones(5,1)];
            summary=aggregatePillarDistributionShape([zeros(20,2),z],int32([ones(10,1);2*ones(10,1)]));
            column=summary.names=="heightMaximumGapFraction";
            testCase.verifyEqual(summary.values(:,column),[1/9;1],'AbsTol',1e-12);
            entropy=summary.names=="heightDecileEntropy";
            testCase.verifyGreaterThan(summary.values(1,entropy),summary.values(2,entropy));
        end
        function covarianceSeparatesSmoothGroundFromDiscontinuity(testCase)
            [x,y]=meshgrid(-2:2,-2:2);xy=[x(:),y(:)];
            smooth=[xy,.1*x(:)+.2*y(:)];stepped=[xy,.15*sign(x(:))];
            first=aggregatePillarStatistics(smooth,ones(25,1),struct(),false);
            second=aggregatePillarStatistics(stepped,ones(25,1),struct(),false);
            [a,names]=measurePillarMomentContext(first,[1 1],1);
            [b,~]=measurePillarMomentContext(second,[1 1],1);
            column=names=="r0_verticalResidualFraction";
            testCase.verifyLessThan(a(column),1e-7);
            testCase.verifyGreaterThan(b(column),.05);
        end
        function multiscaleMomentsIgnoreGlobalTranslationAndHandleRowRaster(testCase)
            points=[0 0 0;.1 .05 1;.05 .1 2;0 .02 3;1 1 4;1.1 1.1 5];
            ids=int32([1;1;1;1;3;3]);
            first=aggregatePillarStatistics(points,ids,struct(),false);
            second=aggregatePillarStatistics(points+[200 -300 50],ids,struct(),false);
            [a,names]=measurePillarMomentContext(first,[1 3],[1;3]);
            [b,otherNames]=measurePillarMomentContext(second,[1 3],[1;3]);
            testCase.verifySize(a,[2 numel(names)]);
            testCase.verifyEqual(names,otherNames);
            testCase.verifyEqual(a,b,'AbsTol',1e-7);
            testCase.verifyTrue(all(isfinite(a),'all'));
        end
        function neighborhoodCovarianceWeightsAllPopulationMembers(testCase)
            points=[0 0 0;.1 .2 1;.2 .1 2;1 0 4];ids=int32([1;1;1;2]);
            stats=aggregatePillarStatistics(points,ids,struct(),false);
            [a,names]=measurePillarMomentContext(stats,[1 2],1);
            covariance=cov(points,1);
            testCase.verifyEqual(a(names=="r1_count"),4);
            testCase.verifyEqual(a(names=="r1_centerCountFraction"),.75);
            testCase.verifyEqual(a(names=="r1_effectiveCells"),16/10,'AbsTol',1e-8);
            testCase.verifyEqual(a(names=="r1_verticalVariance"),covariance(3,3),'AbsTol',1e-8);
            testCase.verifyEqual(a(names=="r1_horizontalVariance"),trace(covariance(1:2,1:2)),'AbsTol',1e-8);
            testCase.verifyEqual(a(names=="r1_withinVerticalVariance"),.5,'AbsTol',1e-8);
        end
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
            testCase.verifySize(empty.values,[0 numel(empty.names)]);
            testCase.verifyEqual(repeated.values,zeros(1,numel(repeated.names)));
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
