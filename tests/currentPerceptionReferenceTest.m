classdef currentPerceptionReferenceTest < matlab.unittest.TestCase
% currentPerceptionReferenceTest: Exact masks from immutable current-output evidence.
    properties (TestParameter)
        frameCase = recordedCases()
    end
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function preservesEveryRecordedPointDecision(testCase,frameCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw',frameCase.dataset+'PointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,frameCase.frameIndex);
            cfg=perceptionConfig(frameCase.dataset); cfg.executionMode="offline";
            expected=loadPerceptionMaskReference(frameCase.dataset,frameCase.frameIndex);
            actual=perceiveFrame(frame,cfg);
            testCase.verifyEqual(actual.featureMasks,expected.featureMasks);
        end
        function rejectsRemovedExecutionMode(testCase)
            frame=struct('x',10,'y',2,'z',1);
            cfg=perceptionConfig(); cfg.executionMode="legacyFull";
            testCase.verifyError(@() perceiveFrame(frame,cfg),'perception:InvalidExecutionMode');
        end
    end
end

function cases=recordedCases()
    root=fileparts(fileparts(mfilename('fullpath')));
    fixture=jsondecode(fileread(fullfile(root,'tests','reference','perceptionMasks.json')));
    cases=struct();
    for item=fixture.entries(:).'
        name=string(item.dataset)+string(item.frameIndex);
        cases.(name)=struct('dataset',string(item.dataset),'frameIndex',item.frameIndex);
    end
end
