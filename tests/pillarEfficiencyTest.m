classdef pillarEfficiencyTest < matlab.unittest.TestCase
% pillarEfficiencyTest: Exact boundary decisions and sufficient statistics.
    properties (TestParameter)
        shape = {[0 0],[0 5],[1 1],[1 13],[17 1],[31 27]}
        radius = {[0 0],[1 2],[50 50]}
    end
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function boxSumMatchesClippedIntegralReduction(testCase,shape,radius)
            testCase.assumeTrue(perceptionNativeAvailable);
            stream=RandStream('mt19937ar','Seed',1979);
            values=randn(stream,shape(1),shape(2));
            if numel(values)>1, values(1)=NaN;values(end)=Inf;end
            actual=perceptionKernelsMex('boxSum',values,radius);
            clean=values;clean(~isfinite(clean))=0;
            expected=zeros(shape);
            if ~isempty(clean)
                integral=zeros(shape+1);
                integral(2:end,2:end)=cumsum(cumsum(clean,1),2);
                for col=1:shape(2)
                    for row=1:shape(1)
                        lo=max([row col]-radius,1);hi=min([row col]+radius,shape);
                        expected(row,col)=integral(hi(1)+1,hi(2)+1)-integral(lo(1),hi(2)+1) ...
                            -integral(hi(1)+1,lo(2))+integral(lo(1),lo(2));
                    end
                end
            end
            testCase.verifyEqual(actual,expected);
        end
        function fusedStatisticsKeepBoundsRadiometryAndCrossCovariance(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            points=[700000 4300000 100;700000.002 4300000.004 100.006; ...
                700003 4300004 102;700003.02 4300004.04 102.06;7 8 9];
            ids=int32([91;91;5;5;102]);
            attributes=struct('intensity',[-5;NaN;-9;-3;Inf], ...
                'reflectivity',[NaN;Inf;0;2;7]);
            expected=aggregatePillarStatistics(points,ids,attributes,false);
            actual=aggregatePillarStatistics(points,ids,attributes,true);
            testCase.verifyEqual(actual,expected,'AbsTol',1e-15);
            testCase.verifyEqual(actual.minimumXYZ(1,:),points(3,:));
            testCase.verifyEqual(actual.maximumXYZ(1,:),points(4,:));
            testCase.verifyGreaterThan(actual.covarianceXYZ(2,4),0);
            testCase.verifyEqual(actual.intensity.count,[2;1;0]);
            testCase.verifyEqual(actual.intensity.maximum(1:2),[-3;-5]);
            testCase.verifyTrue(isnan(actual.intensity.maximum(3)));
        end
        function fusedStatisticsKeepEmptyShapes(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            attributes=struct('intensity',zeros(0,1));
            actual=aggregatePillarStatistics(zeros(0,3),int32(zeros(0,1)),attributes,true);
            expected=aggregatePillarStatistics(zeros(0,3),int32(zeros(0,1)),attributes,false);
            testCase.verifyEqual(actual,expected);
        end
        function boxSumRejectsInvalidRadius(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            testCase.verifyError(@() invalidBoxRadius(),'perception:native:InvalidInput');
        end
        function strictContextThresholdRetainsEqualityRejection(testCase)
            core=false(3);core(2,2)=true;evidence=ones(3);
            cfg=struct('minimumContextFraction',1/9);
            [~,context]=selectPillarFootprints(core,false(3),evidence,evidence,cfg,false(3));
            testCase.verifyFalse(any(context,'all'));
            cfg.minimumContextFraction=1/9-eps;
            [~,context]=selectPillarFootprints(core,false(3),evidence,evidence,cfg,false(3));
            testCase.verifyEqual(context,core);
        end
        function tiedPairsKeepUpDownLeftRightOrder(testCase)
            core=false(5);core(3,3)=true;support=false(5);support([2 4],3)=true;
            evidence=zeros(5);evidence(3,3)=4;evidence([2 4],3)=3;evidence(3,4)=4;
            cfg=struct('minimumContextFraction',0.45);
            [footprint,~,compact]=selectPillarFootprints(core,support,ones(5),evidence,cfg,false(5));
            expected=false(5);expected(2:3,3)=true;
            testCase.verifyEqual(footprint,expected);
            testCase.verifyEqual(compact,expected);
        end
        function rowAndColumnMapsKeepClippedPairSupport(testCase)
            cfg=struct('minimumContextFraction',0.5);
            for transposeMap=[false true]
                core=logical([0 1 0]);support=true(1,3);evidence=[4 4 4];
                expected=logical([1 1 0]);
                if transposeMap,core=core.';support=support.';evidence=evidence.';expected=expected.';end
                [footprint,~,compact]=selectPillarFootprints(core,support,evidence,evidence,cfg,false(size(core)));
                testCase.verifyEqual(footprint,expected);
                testCase.verifyEqual(compact,expected);
            end
        end
        function emptyCorePreservesAdditionalCandidate(testCase)
            core=false(3);extra=core;extra(1,1)=true;evidence=double(extra);
            [footprint,~,compact]=selectPillarFootprints(core,core,evidence,evidence, ...
                struct('minimumContextFraction',0.5),extra);
            testCase.verifyEqual(footprint,extra);
            testCase.verifyEqual(compact,extra);
        end
    end
end
function invalidBoxRadius()
    value=perceptionKernelsMex('boxSum',ones(2),[1 -1]); %#ok<NASGU>
end
