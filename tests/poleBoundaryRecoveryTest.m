classdef poleBoundaryRecoveryTest < matlab.unittest.TestCase
% poleBoundaryRecoveryTest: Recover complete shafts without scan organization.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function completesReportedNearShaftAcrossCells(testCase)
            [frame,cfg]=recordedScene(testCase,746);
            beforeCfg=cfg;beforeCfg.fine.poleRecoveryMaximumNeighborAxisRms=0;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            picks=[55938 56708 55621 55749 56455 55941 55689 56074 56716 56780 ...
                55950 56720 56784 55890 56724 56279 56728 56792 56283 55517 ...
                55902 56287 56608 55842 55906 56291 56676 55589 55529 56299 ...
                56684 55918 56560 55473 55922 55413 56504 56632 55738 56572 56636];
            testCase.verifyFalse(any(before.featureMasks.pole(picks)));
            testCase.verifyGreaterThanOrEqual(nnz(actual.featureMasks.pole(picks)),40);
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            selected=actual.featureMasks.pole & vecnorm(xyz(:,1:2)-[-3.14 -4.1],2,2)<0.4;
            testCase.verifyGreaterThan(nnz(selected),250);
            testCase.verifyGreaterThan(max(xyz(selected,3))-min(xyz(selected,3)),3.0);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            verifyPermutation(testCase,frame,cfg,actual,74655938);
        end
        function rejectsBroadTrunkSurfaceWithoutLosingNarrowShaft(testCase)
            [frame,cfg]=recordedScene(testCase,746);
            beforeCfg=cfg;beforeCfg.fine.poleWideSurfaceMinimumAxisStd=Inf;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            testCase.verifyTrue(before.featureMasks.pole(19993));
            testCase.verifyFalse(actual.featureMasks.pole(19993));
            removed=before.featureMasks.pole & ~actual.featureMasks.pole;
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            testCase.verifyEqual(nnz(removed),131);
            testCase.verifyLessThan(max(vecnorm(xyz(removed,1:2)-xyz(19993,1:2),2,2)),0.75);
            shaft=vecnorm(xyz(:,1:2)-[-3.14 -4.1],2,2)<0.4;
            testCase.verifyEqual(actual.featureMasks.pole(shaft),before.featureMasks.pole(shaft));
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
        end
        function recoversSparseTallShaft(testCase)
            [frame,cfg]=recordedScene(testCase,214);
            beforeCfg=cfg;beforeCfg.fine.poleSparseRecoveryMinimumHeight=Inf;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            testCase.verifyFalse(before.featureMasks.pole(34450));
            testCase.verifyTrue(actual.featureMasks.pole(34450));
            added=actual.featureMasks.pole & ~before.featureMasks.pole;
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            testCase.verifyGreaterThanOrEqual(nnz(added),30);
            testCase.verifyLessThan(max(vecnorm(xyz(added,1:2)-xyz(34450,1:2),2,2)),0.75);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            verifyPermutation(testCase,frame,cfg,actual,21434450);
        end
        function rejectsReportedShortClutterSupport(testCase)
            [frame,cfg]=recordedScene(testCase,214);
            beforeCfg=cfg;beforeCfg.fine.poleIsolationMinimumCoreFraction=0;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            falsePoints=[12097 12225 12610 12482 12546];
            testCase.verifyTrue(all(before.featureMasks.pole(falsePoints)));
            testCase.verifyFalse(any(actual.featureMasks.pole(falsePoints)));
            testCase.verifyTrue(actual.featureMasks.pole(34450));
        end
    end
end

function [frame,cfg]=recordedScene(testCase,index)
    root=fileparts(fileparts(mfilename('fullpath')));
    file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
    testCase.assumeTrue(isfile(file),'Recorded source data are required.');
    frame=loadPointCloudFrame(file,index);cfg=perceptionConfig();cfg.executionMode="offline";
end

function verifyPermutation(testCase,frame,cfg,actual,seed)
    stream=RandStream('mt19937ar','Seed',seed);order=randperm(stream,numel(frame.x));
    shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
    cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
    restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
    testCase.verifyEqual(restored,actual.featureMasks.pole);
end
