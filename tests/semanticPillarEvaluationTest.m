classdef semanticPillarEvaluationTest < matlab.unittest.TestCase
% semanticPillarEvaluationTest: Offline pillar labels differ from point purity.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function originalPointIndexPermutationIsRespected(testCase)
            [frame,cfg,candidates]=fixture([.01 .01;.31 .01],2);
            frame.pointIndices=[2;1];
            a=evaluateSemanticPillarCandidates(frame,cfg,candidates,struct('curb',[true;false]));
            testCase.verifyEqual([a.tp a.fp a.fn],[1 0 0]);
        end
        function mixedPillarIsTruePositive(testCase)
            [frame,cfg,candidates]=fixture([.01 .01;.02 .02;.03 .03],1);
            masks=struct('curb',[true;false;false]);
            a=evaluateSemanticPillarCandidates(frame,cfg,candidates,masks);
            testCase.verifyEqual([a.tp a.fp a.fn],[1 0 0]);
            testCase.verifyEqual(a.precision,1);
        end
        function zeroTargetPillarCountsOnceRegardlessOfDensity(testCase)
            [frame,cfg,candidates]=fixture([repmat([.01 .01],100,1);.31 .01],[1;2]);
            target=false(101,1);target(end)=true;
            a=evaluateSemanticPillarCandidates(frame,cfg,candidates,struct('curb',target));
            testCase.verifyEqual([a.tp a.fp a.fn],[1 1 0]);
        end
        function targetLabelsAreIndependentOfHeightAndRangeFiltering(testCase)
            [frame,cfg,candidates]=fixture([.01 .01;.02 .02],1);
            frame.z=[-3;10];cfg.minRange=100;cfg.exclusionHalfSize=100;
            a=evaluateSemanticPillarCandidates(frame,cfg,candidates,struct('curb',[false;true]));
            testCase.verifyEqual([a.tp a.fp a.fn],[1 0 0]);
        end
        function classLabelsRemainIndependent(testCase)
            [frame,cfg,candidates]=fixture([.01 .01;.31 .01],1);
            candidates.semanticNames=["curb";"pole"];candidates.pillarIndices={int32(1);int32(1)};
            masks=struct('curb',[true;false],'pole',[false;true]);
            a=evaluateSemanticPillarCandidates(frame,cfg,candidates,masks);
            testCase.verifyEqual([a.tp a.fp a.fn],[1 0 0;0 1 1]);
        end
        function excludedTargetCellsRemainInRecallDenominator(testCase)
            [frame,cfg,candidates]=fixture([.01 .01;.31 .01;.91 .01],1);
            a=evaluateSemanticPillarCandidates(frame,cfg,candidates,struct('curb',true(3,1)));
            testCase.verifyEqual([a.tp a.fp a.fn a.outsideTargetPillars],[1 0 2 1]);
        end
        function emptySelectionAndInvalidLabelsAreExplicit(testCase)
            [frame,cfg,candidates]=fixture([.01 .01],[]);
            a=evaluateSemanticPillarCandidates(frame,cfg,candidates,struct('curb',true));
            testCase.verifyEqual([a.tp a.fp a.fn],[0 0 1]);
            frame.x=NaN;
            testCase.verifyError(@() evaluateSemanticPillarCandidates(frame,cfg,candidates,struct('curb',true)), ...
                'evaluation:InvalidReferenceXY');
        end
    end
end
function [frame,cfg,candidates]=fixture(xy,selected)
cfg=struct('gridDims',[3 1],'voxelSize',[.3 .3],'latticeOffset',[.45 .15]);
frame=struct('x',xy(:,1),'y',xy(:,2),'z',zeros(size(xy,1),1));
candidates=struct('geometry',struct('origin',[0 0],'cellSize',[.3 .3],'mapSize',[1 3]), ...
    'semanticNames',"curb",'pillarIndices',{{int32(selected(:))}});
end
