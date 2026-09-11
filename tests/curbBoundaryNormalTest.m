classdef curbBoundaryNormalTest < matlab.unittest.TestCase
% curbBoundaryNormalTest: Require height transitions across a spatial boundary.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function acceptsTransverseStepsAndRejectsLongitudinalSlopes(testCase)
            x=(0:.2:4).';xy=[x,zeros(size(x))];cfg=finePerceptionConfig();
            testCase.verifyTrue(hasConsistentCurbNormals(xy,[0*x,ones(size(x))],cfg));
            testCase.verifyFalse(hasConsistentCurbNormals(xy,[ones(size(x)),0*x],cfg));
        end
        function followsLocalNormalsAroundABend(testCase)
            angle=linspace(0,pi/2,40).';xy=4*[cos(angle),sin(angle)];
            cfg=finePerceptionConfig();
            testCase.verifyTrue(hasConsistentCurbNormals(xy,xy/4,cfg));
        end
        function leavesUnconstrainedSparseTangentsInconclusive(testCase)
            xy=[0 0;3 0;6 0];cfg=finePerceptionConfig();
            testCase.verifyTrue(hasConsistentCurbNormals(xy,[1 0;1 0;1 0],cfg));
        end
        function rejectsReportedBoundaryAndPreservesOtherReturns(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,384);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.curbMinimumBoundaryNormalFraction=0;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            reported=[21753 21628 20477 19007 17664 17600];
            testCase.verifyTrue(all(before.featureMasks.curb(reported)));
            testCase.verifyFalse(any(actual.featureMasks.curb(reported)));
            removed=sort([reported,19967,19710]).';
            testCase.verifyEqual(find(before.featureMasks.curb & ~actual.featureMasks.curb),removed);
            revisions=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbOrientationRecovery.json')));
            change=revisions.entries([revisions.entries.frameIndex]==384);
            ridge=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbRidgeContinuation.json')));
            changeRidge=ridge.entries([ridge.entries.frameIndex]==384);
            expectedAdded=union(setdiff(change.addedCurbIndices,changeRidge.removedCurbIndices),changeRidge.addedCurbIndices);
            testCase.verifyEqual(find(actual.featureMasks.curb & ~before.featureMasks.curb),expectedAdded);
            testCase.verifyEqual(nnz(actual.featureMasks.curb),127+numel(expectedAdded));
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',38421753);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="curb";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
        function recoversSparseObliqueCurbVicinity(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','curbBoundaryVicinity855.json')));
            frame=loadPointCloudFrame(file,855);cfg=perceptionConfig();cfg.executionMode="offline";
            actual=perceiveFrame(frame,cfg);xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            picks=xyz(annotation.leftVicinityIndices,1:2);curb=xyz(actual.featureMasks.curb,1:2);
            distance=min(hypot(picks(:,1)-curb(:,1).',picks(:,2)-curb(:,2).'),[],2);
            testCase.verifyGreaterThanOrEqual(nnz(distance<=0.35),105);
            testCase.verifyLessThan(median(distance),0.10);
            testCase.verifyTrue(all(actual.featureMasks.curb(annotation.baselineCurbIndices)));
            stream=RandStream('mt19937ar','Seed',85526534);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="curb";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
        function labelsAtLeastHalfOfReportedContinuation(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','curb855Continuation.json')));
            frame=loadPointCloudFrame(file,annotation.frameIndex);cfg=perceptionConfig();cfg.executionMode="offline";
            actual=perceiveFrame(frame,cfg);
            testCase.verifyGreaterThanOrEqual(nnz(actual.featureMasks.curb(annotation.reportedIndices)),21);
            testCase.verifyEqual(nnz(actual.featureMasks.pole),23);
            testCase.verifyEqual(nnz(actual.featureMasks.trafficSign),60);
        end
    end
end
