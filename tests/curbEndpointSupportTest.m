classdef curbEndpointSupportTest < matlab.unittest.TestCase
% curbEndpointSupportTest: Unsupported terminal responses must not form curbs.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function rejectsModeratelyMisalignedUnsupportedTip(testCase)
            cfg=finePerceptionConfig();xy=[(0:0.2:2).',zeros(11,1)];
            gradients=repmat([0 1],11,1);gradients(1,:)=[sind(21) cosd(21)];
            previous=cfg;previous.curbMaximumEndpointNormalAngleDegrees=25;
            testCase.verifyTrue(all(curbEndpointSupportMask(xy,gradients,previous)));
            expected=true(11,1);expected(1)=false;
            testCase.verifyEqual(curbEndpointSupportMask(xy,gradients,cfg),expected);
        end
        function trimsMismatchedTipWithoutErodingInterior(testCase)
            cfg=finePerceptionConfig();xy=[(0:0.2:2).',zeros(11,1)];
            gradients=repmat([0 1],11,1);gradients(1,:)=[1 1];gradients(6,:)=[1 1];
            expected=true(11,1);expected(1)=false;
            testCase.verifyEqual(curbEndpointSupportMask(xy,gradients,cfg),expected);
        end
        function trimsTerminalRunAndStopsAtSupportedInterior(testCase)
            cfg=finePerceptionConfig();xy=[(0:0.2:3).',zeros(16,1)];
            gradients=repmat([0 1],16,1);
            gradients([1 2 3 8 15 16],:)=repmat([1 1],6,1);
            expected=true(16,1);expected([1 2 3 15 16])=false;
            testCase.verifyEqual(curbEndpointSupportMask(xy,gradients,cfg),expected);
        end
        function preservesInconclusiveSparseTip(testCase)
            cfg=finePerceptionConfig();xy=[0 0;3 0;6 0];
            testCase.verifyTrue(all(curbEndpointSupportMask(xy,ones(3,2),cfg)));
        end
        function rotationAndPermutationPreserveDecision(testCase)
            cfg=finePerceptionConfig();xy=[(0:0.2:2).',zeros(11,1)];
            gradients=repmat([0 1],11,1);gradients(1,:)=[1 1];
            expected=curbEndpointSupportMask(xy,gradients,cfg);
            angle=0.71;rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            order=[8 2 9 4 7 3 1 11 6 10 5];
            actual=curbEndpointSupportMask(xy(order,:)*rotation,gradients(order,:)*rotation,cfg);
            restored=false(11,1);restored(order)=actual;
            testCase.verifyEqual(restored,expected);
        end
        function reportedFrame28TipIsRejectedWithoutScanOrder(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,28);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.curbMaximumEndpointNormalAngleDegrees=90;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            reported=[36671 36735 37180 37116 36474 36410];
            testCase.verifyTrue(all(before.featureMasks.curb(reported)));
            testCase.verifyFalse(any(actual.featureMasks.curb(reported)));
            expected=before.featureMasks.curb;expected(reported)=false;
            testCase.verifyEqual(actual.featureMasks.curb,expected);
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',2836671);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="curb";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
    end
end
