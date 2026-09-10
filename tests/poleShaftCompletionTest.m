classdef poleShaftCompletionTest < matlab.unittest.TestCase
% poleShaftCompletionTest: Recover split shafts using unorganized XYZ moments.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function completesAdjacentSharedAxis(testCase)
            [stats,cfg]=scene();
            testCase.verifyEqual(completeRecoveredPoleShafts(stats,[8 8],10,cfg),[10;18]);
        end
        function rejectsSeparateParallelShaft(testCase)
            [stats,cfg]=scene();stats.meanXYZ(2,1)=0.3;
            testCase.verifyEqual(completeRecoveredPoleShafts(stats,[8 8],10,cfg),10);
        end
        function rejectsDiffuseReturnsBeforeTrimming(testCase)
            [stats,cfg]=scene();stats.covarianceXYZ(2,1)=0.04;
            testCase.verifyEqual(completeRecoveredPoleShafts(stats,[8 8],10,cfg),10);
        end
        function rejectsTiltedNeighbor(testCase)
            [stats,cfg]=scene();stats.covarianceXYZ(2,[1 4])=[0.0125 0.1];
            testCase.verifyEqual(completeRecoveredPoleShafts(stats,[8 8],10,cfg),10);
        end
        function rejectsDisjointHeightAndTransitiveGrowth(testCase)
            [stats,cfg]=scene();stats.minimumXYZ(2,3)=4;stats.maximumXYZ(2,3)=7;
            testCase.verifyEqual(completeRecoveredPoleShafts(stats,[8 8],10,cfg),10);
            [stats,cfg]=scene();names=fieldnames(stats);
            for k=1:numel(names),stats.(names{k})(3,:)=stats.(names{k})(2,:);end
            stats.pillarIndices(3)=26;
            testCase.verifyEqual(completeRecoveredPoleShafts(stats,[8 8],10,cfg),[10;18]);
        end
        function twoMeterSupportRejectsDiffuseShaft(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,1047);cfg=perceptionConfig();cfg.executionMode="offline";
            excludedBoundary=cfg;excludedBoundary.fine.poleShortSupportHeight=2-eps(2);
            before=perceiveFrame(frame,excludedBoundary);actual=perceiveFrame(frame,cfg);
            testCase.verifyTrue(before.featureMasks.pole(60330));
            testCase.verifyFalse(actual.featureMasks.pole(60330));
            testCase.verifyEqual(nnz(before.featureMasks.pole & ~actual.featureMasks.pole),9);
            testCase.verifyFalse(any(~before.featureMasks.pole & actual.featureMasks.pole));
        end
        function recordedShaftSurvivesUnorganizedPermutation(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,1047);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.poleRecoveryMaximumTiltDegrees=2;
            beforeCfg.fine.poleRecoveryMaximumNeighborAxisRms=0;
            beforeCfg.fine.poleShortSupportHeight=2-eps(2);
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            picks=[34897 35282 35667 36052 34901 35286 35671 35735 36056 34969 35354 35739 36124 34973];
            testCase.verifyFalse(any(before.featureMasks.pole(picks)));
            testCase.verifyTrue(all(actual.featureMasks.pole(picks)));
            testCase.verifyTrue(before.featureMasks.pole(60330));
            testCase.verifyFalse(actual.featureMasks.pole(60330));
            testCase.verifyEqual(nnz(actual.featureMasks.pole),88);
            testCase.verifyEqual(rmfield(actual.featureMasks,'pole'),rmfield(before.featureMasks,'pole'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',104734897);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="pole";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.pole;
            testCase.verifyEqual(restored,actual.featureMasks.pole);
        end
    end
end

function [stats,cfg]=scene()
    cfg=finePerceptionConfig();
    stats=struct('pillarIndices',[10;18],'count',[20;10], ...
        'meanXYZ',[0 0 0;0.10 0 0], ...
        'minimumXYZ',[-0.1 -0.1 -1.5;0 -0.1 -1.5], ...
        'maximumXYZ',[0.1 0.1 1.5;0.2 0.1 1.5], ...
        'covarianceXYZ',repmat([0.0025 0 0.0025 0 0 1],2,1));
end
