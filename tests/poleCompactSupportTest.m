classdef poleCompactSupportTest < matlab.unittest.TestCase
% poleCompactSupportTest: Dense true shafts and continuous upright short support.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function recoversDenseCompactShaftBesideBackground(testCase)
            [frame,cfg]=scene(testCase);
            beforeCfg=cfg;beforeCfg.fine.poleDenseCoreMinimumPoints=Inf;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            picks=[50888 50122 49677 50383 50768 49681 50451 50006 50070 50391 ...
                50455 50840 50074 50523 50459 49693 50078 50399 50784 49954 ...
                50018 50724 50403 49573 50279 50664 49513 49898];
            testCase.verifyFalse(any(before.featureMasks.pole(picks)));
            testCase.verifyTrue(all(actual.featureMasks.pole(picks)));
            added=actual.featureMasks.pole & ~before.featureMasks.pole;
            testCase.verifyEqual(nnz(added),195);
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            testCase.verifyLessThan(max(vecnorm(xyz(added,1:2)-[-0.4 -4.4],2,2)),0.5);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',113750888);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,actual.featureMasks.pole);
        end
        function recoversSplitShaftWithoutImportingHighFragments(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,384);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.poleIndependentMaximumNeighborAxisRms=0;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            picks=[63136 62049 62053 62374 62759 63144 62378 62763 63148 62382 62767];
            testCase.verifyFalse(any(before.featureMasks.pole(picks)));
            testCase.verifyTrue(all(actual.featureMasks.pole(picks)));
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            added=actual.featureMasks.pole & ~before.featureMasks.pole;
            testCase.verifyEqual(nnz(added),58);
            testCase.verifyLessThan(max(vecnorm(xyz(added,1:2)-[-12.8 -3.8],2,2)),0.5);
            testCase.verifyLessThan(max(xyz(added,3)),2);
            testCase.verifyGreaterThan(max(xyz(added,3))-min(xyz(added,3)),3);
            testCase.verifyFalse(any(before.featureMasks.pole & ~actual.featureMasks.pole));
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',38463136);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,actual.featureMasks.pole);
        end
        function rejectsIntermittentAndTiltedShortCandidates(testCase)
            [frame,cfg]=scene(testCase);
            beforeCfg=cfg;beforeCfg.fine.poleShortSupportHeight=0;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            falsePoints=[54355 53525 54232 53081 53402 53723 52829 54044 52893 54744 54179];
            testCase.verifyGreaterThanOrEqual(nnz(before.featureMasks.pole(falsePoints)),10);
            testCase.verifyFalse(any(actual.featureMasks.pole(falsePoints)));
            testCase.verifyFalse(actual.featureMasks.pole(54471));
            testCase.verifyEqual(nnz(actual.featureMasks.pole),234);
        end
    end
end

function [frame,cfg]=scene(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));
    file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
    testCase.assumeTrue(isfile(file),'Recorded source data are required.');
    frame=loadPointCloudFrame(file,1137);cfg=perceptionConfig();cfg.executionMode="offline";
end
