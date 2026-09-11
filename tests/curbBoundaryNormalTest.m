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
            testCase.verifyFalse(any(actual.featureMasks.curb & ~before.featureMasks.curb));
            testCase.verifyEqual(nnz(actual.featureMasks.curb),127);
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',38421753);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="curb";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
    end
end
