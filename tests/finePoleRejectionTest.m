classdef finePoleRejectionTest < matlab.unittest.TestCase
% finePoleRejectionTest: Reject diffuse and cluttered supports without rings.
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
            baselineCfg.fine.poleLowContrastMaximumRadius=Inf;
            baselineCfg.fine.poleIsolationMinimumCoreFraction=0;
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
            testCase.verifyTrue(baseline.featureMasks.pole(43363));
            testCase.verifyFalse(actual.featureMasks.pole(43363));
            testCase.verifyTrue(baseline.featureMasks.pole(42593));
            testCase.verifyFalse(actual.featureMasks.pole(42593));
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            near=vecnorm(xyz(:,1:2)-xyz(42593,1:2),2,2)<=0.75;
            testCase.verifyFalse(any(actual.featureMasks.pole & near));
            testCase.verifyEqual(nnz(actual.featureMasks.pole),147);
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
            testCase.verifyFalse(restored(43363));
            testCase.verifyFalse(restored(42593));
        end
    end
end

function frame=recordedFrame(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
    testCase.assumeTrue(isfile(file),'Recorded source data are required.');
    frame=loadPointCloudFrame(file,91);
end
