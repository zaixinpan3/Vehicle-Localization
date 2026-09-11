classdef poleVerticalContinuityTest < matlab.unittest.TestCase
% poleVerticalContinuityTest: Qualify real vertical runs, not summed heights.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function rejectsIsolatedTopFragment(testCase)
            cfg=finePerceptionConfig();shaft=(0:0.1:2).';fragment=(3.2:0.1:3.6).';
            actual=retainContinuousPoleSupport([shaft;fragment],cfg);
            testCase.verifyEqual(actual,[true(size(shaft));false(size(fragment))]);
        end
        function separatedFragmentsCannotPoolTheirHeight(testCase)
            cfg=finePerceptionConfig();z=[0:0.05:0.4,1.5:0.05:1.9,3:0.05:3.4];
            testCase.verifyFalse(any(retainContinuousPoleSupport(z,cfg)));
        end
        function independentlySupportedRunsRemainValid(testCase)
            cfg=finePerceptionConfig();z=[(0:0.1:2).';(3.2:0.1:4.4).'];
            testCase.verifyTrue(all(retainContinuousPoleSupport(z,cfg)));
        end
        function metricDecisionSurvivesPermutation(testCase)
            cfg=finePerceptionConfig();z=[0:0.1:2,3.2:0.1:3.6];
            expected=retainContinuousPoleSupport(z,cfg);
            stream=RandStream('mt19937ar','Seed',113754471);order=randperm(stream,numel(z));
            reordered=retainContinuousPoleSupport(z(order),cfg);
            restored=false(size(z));restored(order)=reordered;
            testCase.verifyEqual(restored,expected);
        end
        function rejectsReportedHighFragment(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,1137);cfg=perceptionConfig();cfg.executionMode="offline";
            % Isolate point-run trimming from the candidate-level continuity gate.
            cfg.fine.poleShortSupportHeight=0;
            beforeCfg=cfg;beforeCfg.fine.poleMaximumVerticalGapMeters=Inf;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            testCase.verifyTrue(before.featureMasks.pole(54471));
            testCase.verifyFalse(actual.featureMasks.pole(54471));
            testCase.verifyEqual(nnz(before.featureMasks.pole & ~actual.featureMasks.pole),5);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
        end
        function unqualifiedBridgeCannotSupportDetachedOutput(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','poleOutputContinuity615.json')));
            frame=loadPointCloudFrame(file,annotation.frameIndex);cfg=perceptionConfig();cfg.executionMode="offline";
            actual=perceiveFrame(frame,cfg);
            expected=false(numel(frame.x),1);expected(annotation.baselinePoleIndices)=true;
            expected(annotation.detachedFragmentIndices)=false;
            testCase.verifyEqual(actual.featureMasks.pole,expected);
            stream=RandStream('mt19937ar','Seed',61538985);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,expected);
        end
    end
end
