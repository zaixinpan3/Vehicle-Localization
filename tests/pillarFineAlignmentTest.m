classdef pillarFineAlignmentTest < matlab.unittest.TestCase
% pillarFineAlignmentTest: Original fine points define coarse coverage targets.
    properties (TestParameter)
        fineLabels=struct('indices',[1;2],'logicalMask',logical([1;1;0]));
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=setupVehicleLocalization();
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root,'research','pillar_fine_alignment_20260926')));
        end
    end
    methods (Test)
        function pointCoverageDoesNotCountPillarsAsPoints(testCase,fineLabels)
            frame=struct('x',[.1;.2;.7],'y',[.1;.2;.1]);
            geometry=pillarFineAlignmentTest.geometry();
            [metrics,detail]=measureFinePoleAlignment(frame,fineLabels,[1;2;2],geometry);
            testCase.verifyEqual(metrics.finePointCount,2);
            testCase.verifyEqual(metrics.coveredFinePointCount,2);
            testCase.verifyEqual(metrics.finePillarCount,1);
            testCase.verifyEqual(metrics.coveredFinePillarCount,1);
            testCase.verifyEqual(metrics.candidatePillarCount,2);
            testCase.verifyEqual(metrics.extraPillarCount,1);
            testCase.verifyEqual(metrics.pointCoverage,1,'AbsTol',0);
            testCase.verifyEqual(metrics.pillarPrecision,.5,'AbsTol',0);
            testCase.verifyEqual(detail.finePointPillarIds,[1;1]);
            testCase.verifyEqual(detail.extraPillarIds,2);
        end
        function filteringCannotShrinkTheFineReferenceDenominator(testCase)
            frame=struct('x',[.1;.7;.9],'y',[.1;.1;.2]);
            geometry=pillarFineAlignmentTest.geometry();
            eligible=logical([1;0;1]);
            [metrics,detail]=measureFinePoleAlignment(frame,[1;2],1,geometry,eligible);
            testCase.verifyEqual(metrics.finePointCountInRoi,2);
            testCase.verifyEqual(metrics.eligibleFinePointCount,1);
            testCase.verifyEqual(metrics.coveredEligibleFinePointCount,1);
            testCase.verifyEqual(metrics.coveredFinePointCount,1);
            testCase.verifyEqual(metrics.pointCoverage,.5,'AbsTol',0);
            testCase.verifyEqual(metrics.pointCoverageAllFine,.5,'AbsTol',0);
            testCase.verifyEqual(detail.finePointEligible,logical([1;0]));
            testCase.verifyEqual(detail.missedFinePillarIds,2);
        end
        function outsideRoiPointsRemainExplicitDiagnostics(testCase)
            frame=struct('x',[.1;1.3;NaN],'y',[.1;.1;.1]);
            geometry=pillarFineAlignmentTest.geometry();
            [metrics,detail]=measureFinePoleAlignment(frame,[1;2;3],1,geometry);
            testCase.verifyEqual(metrics.finePointCount,3);
            testCase.verifyEqual(metrics.finePointCountInRoi,1);
            testCase.verifyEqual(metrics.finePointCountOutsideRoi,2);
            testCase.verifyEqual(metrics.pointCoverage,1,'AbsTol',0);
            testCase.verifyEqual(metrics.pointCoverageAllFine,1/3,'AbsTol',1e-15);
            testCase.verifyEqual(detail.finePointInRoi,logical([1;0;0]));
            testCase.verifyEqual(detail.finePointPillarIds,[1;0;0]);
        end
        function emptyFineReferenceStillCountsExtraCandidates(testCase)
            frame=struct('x',[.1;.7],'y',[.1;.1]);
            geometry=pillarFineAlignmentTest.geometry();
            [metrics,detail]=measureFinePoleAlignment(frame,zeros(0,1),[1;2],geometry);
            testCase.verifyEqual(metrics.finePointCount,0);
            testCase.verifyEqual(metrics.finePillarCount,0);
            testCase.verifyEqual(metrics.extraPillarCount,2);
            testCase.verifyTrue(isnan(metrics.pointCoverage));
            testCase.verifyTrue(isnan(metrics.pointCoverageAllFine));
            testCase.verifyTrue(isnan(metrics.pillarRecall));
            testCase.verifyEqual(metrics.pillarPrecision,0,'AbsTol',0);
            testCase.verifyEqual(metrics.extraPillarFraction,1,'AbsTol',0);
            testCase.verifyEqual(detail.extraPillarIds,[1;2]);
        end
    end
    methods (Static,Access=private)
        function value=geometry()
            value=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 2]);
        end
    end
end
