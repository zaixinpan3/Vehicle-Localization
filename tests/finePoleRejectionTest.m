classdef finePoleRejectionTest < matlab.unittest.TestCase
% finePoleRejectionTest: Reject diffuse and cluttered supports without rings.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function rejectsShortElongatedSupportWithoutPointOrder(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,28);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.poleShortSupportWideSurfaceMinimumAxisStd=0.12;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            testCase.verifyTrue(before.featureMasks.pole(37328));
            testCase.verifyFalse(actual.featureMasks.pole(37328));
            removed=before.featureMasks.pole & ~actual.featureMasks.pole;
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            testCase.verifyEqual(nnz(removed),14);
            testCase.verifyLessThan(max(vecnorm(xyz(removed,1:2)-xyz(37328,1:2),2,2)),0.3);
            testCase.verifyFalse(any(actual.featureMasks.pole & ~before.featureMasks.pole));
            testCase.verifyEqual(nnz(actual.featureMasks.pole),127);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',2837328);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,actual.featureMasks.pole);
        end
        function rejectsReportedFalsePositive(testCase)
            frame=recordedFrame(testCase);
            cfg=perceptionConfig(); cfg.executionMode="offline";
            baselineCfg=cfg; baselineCfg.fine.poleShortSupportMaximumRadialRms=Inf;
            baselineCfg.fine.poleLowContrastMaximumRadius=Inf;
            baselineCfg.fine.poleIsolationMinimumCoreFraction=0;
            baselineCfg.fine.poleMaximumVerticalGapMeters=Inf;
            baselineCfg.fine.poleShortSupportHeight=0;
            baseline=perceiveFrame(frame,baselineCfg);
            actual=perceiveFrame(frame,cfg);
            root=fileparts(mfilename('fullpath'));
            changes=jsondecode(fileread(fullfile(root,'reference','finePoleRejection.json')));
            entry=changes.entries([changes.entries.frameIndex]==91);
            expected=baseline.featureMasks.pole;expected(entry.removedPoleIndices)=false;
            changes=jsondecode(fileread(fullfile(root,'reference','finePoleBoundaryRecovery.json')));
            entry=changes.entries([changes.entries.frameIndex]==91);
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
            testCase.verifyEqual(nnz(actual.featureMasks.pole),143);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'), ...
                rmfield(baseline.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,baseline.probabilityCloud);
        end
        function rejectsFrame687BorderlineClutterWithoutPointOrder(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,687);
            cfg=perceptionConfig();cfg.executionMode="offline";
            previous=cfg;previous.fine.poleIsolationMinimumCoreFraction=0.75;
            before=perceiveFrame(frame,previous);actual=perceiveFrame(frame,cfg);
            testCase.verifyTrue(before.featureMasks.pole(47135));
            testCase.verifyFalse(actual.featureMasks.pole(47135));
            testCase.verifyEqual(nnz(actual.featureMasks.pole),124);
            testCase.verifyEqual(nnz(before.featureMasks.pole & ~actual.featureMasks.pole),12);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',68747135);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,actual.featureMasks.pole);
        end
        function rejectsDiffuseShaftAtShortSupportBoundary(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,901);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.poleShortSupportMaximumRadialRms=Inf;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            testCase.verifyTrue(before.featureMasks.pole(17708));
            testCase.verifyFalse(actual.featureMasks.pole(17708));
            removed=before.featureMasks.pole & ~actual.featureMasks.pole;
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            testCase.verifyEqual(nnz(removed),85);
            testCase.verifyLessThan(max(vecnorm(xyz(removed,1:2)-xyz(17708,1:2),2,2)),0.6);
            testCase.verifyFalse(any(actual.featureMasks.pole & ~before.featureMasks.pole));
            testCase.verifyEqual(nnz(actual.featureMasks.pole),155);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',90117708);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,actual.featureMasks.pole);
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
