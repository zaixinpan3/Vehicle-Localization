classdef finePoleRejectionTest < matlab.unittest.TestCase
% finePoleRejectionTest: Reject short diffuse supports without ring metadata.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function rejectsReportedFalsePositive(testCase)
            frame=recordedFrame(testCase);
            cfg=perceptionConfig(); cfg.executionMode="offline";
            baselineCfg=cfg; baselineCfg.fine.poleShortSupportMaximumRadialRms=Inf;
            baseline=perceiveFrame(frame,baselineCfg);
            actual=perceiveFrame(frame,cfg);
            changes=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
                'reference','finePoleRejection.json')));
            entry=changes.entries([changes.entries.frameIndex]==91);
            expected=baseline.featureMasks.pole;
            expected(entry.removedPoleIndices)=false;
            testCase.verifyTrue(baseline.featureMasks.pole(47906));
            testCase.verifyFalse(actual.featureMasks.pole(47906));
            testCase.verifyEqual(actual.featureMasks.pole,expected);
            testCase.verifyTrue(baseline.featureMasks.pole(16728));
            testCase.verifyFalse(actual.featureMasks.pole(16728));
            testCase.verifyEqual(nnz(actual.featureMasks.pole),269);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'), ...
                rmfield(baseline.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,baseline.probabilityCloud);
        end
        function pointOrderDoesNotChangePoleDecisions(testCase)
            frame=recordedFrame(testCase);
            cfg=perceptionConfig(); cfg.executionMode="offline"; cfg.featureNames="pole";
            actual=perceiveFrame(frame,cfg);
            state=rng; cleanup=onCleanup(@() rng(state));
            rng(91047906,'twister'); order=randperm(numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1); restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,actual.featureMasks.pole);
            testCase.verifyFalse(restored(47906));
            testCase.verifyFalse(restored(16728));
        end
    end
end

function frame=recordedFrame(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
    testCase.assumeTrue(isfile(file),'Recorded source data are required.');
    frame=loadPointCloudFrame(file,91);
end
