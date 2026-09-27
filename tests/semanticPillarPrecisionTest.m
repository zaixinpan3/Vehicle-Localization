classdef semanticPillarPrecisionTest < matlab.unittest.TestCase
% semanticPillarPrecisionTest: Coarse precision gates and isolation contracts.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function precisionIsEnabledOnlyOnTheCoarseLattice(testCase)
            online=perceptionConfig('Downtown');offline=perceptionConfig('Downtown','offline');
            testCase.verifyTrue(online.semanticPrecision.enabled);
            testCase.verifyFalse(offline.semanticPrecision.enabled);
            testCase.verifyEqual(online.semanticPrecision.classes,["curb","trafficSign","facade"]);
        end
        function allModelSchemasHaveFiniteOperatingPoints(testCase)
            cfg=semanticPillarPrecisionConfig();
            for name=cfg.classes
                model=jsondecode(fileread(fullfile(cfg.modelDirectory,name+"PillarPrecisionModel.json")));
                testCase.verifyEqual(string(model.semantic),name);
                testCase.verifyGreaterThan(model.decisionThreshold,0);
                testCase.verifyLessThan(model.decisionThreshold,1);
                testCase.verifyEqual(numel(unique(string(model.featureNames))),numel(model.featureNames));
                testCase.verifyFalse(any(ismember(string(model.featureNames),["frame","pillar","finePointCount","dataset"])));
            end
        end
        function filtersOnlyPruneAndLeavePoleAndSourcePreprocessingIntact(testCase)
            frame=recordedFrame(testCase,'downTownPointClouds.mat',375);
            cfg=perceptionConfig('Downtown');before=cfg;before.semanticPrecision.enabled=false;
            old=perceiveFrame(frame,before);current=perceiveFrame(frame,cfg);
            testCase.verifyEqual(current.sourceSummary,old.sourceSummary);
            testCase.verifyEqual(current.candidates.geometry,old.candidates.geometry);
            for k=1:numel(current.featureNames)
                name=current.featureNames(k);ids=current.candidates.pillarIndices{k};baseline=old.candidates.pillarIndices{k};
                testCase.verifyTrue(all(ismember(ids,baseline)));
                if name=="pole",testCase.verifyEqual(ids,baseline);end
            end
        end
        function standaloneChannelsRetainTheSamePrecisionDecision(testCase)
            frame=recordedFrame(testCase,'downTownPointClouds.mat',200);
            cfg=perceptionConfig('Downtown');all=perceiveFrame(frame,cfg);
            for name=["curb","trafficSign","facade"]
                cfg.featureNames=name;single=perceiveFrame(frame,cfg);
                testCase.verifyEqual(single.candidates.pillarIndices{1}, ...
                    all.candidates.pillarIndices{all.candidates.semanticNames==name});
            end
        end
        function nativeAndMatlabBackendsSelectTheSameCells(testCase)
            frame=recordedFrame(testCase,'MissisipiPointClouds.mat',500);
            cfg=perceptionConfig();cfg.executionBackend="matlab";a=perceiveFrame(frame,cfg);
            cfg.executionBackend="auto";b=perceiveFrame(frame,cfg);
            testCase.verifyEqual(a.candidates,b.candidates);
        end
        function emptyInputsProduceNoCandidates(testCase)
            cfg=perceptionConfig('Downtown');frame=struct('x',NaN,'y',NaN,'z',NaN);
            p=perceiveFrame(frame,cfg);
            testCase.verifyTrue(all(cellfun(@isempty,p.candidates.pillarIndices)));
        end
    end
end

function frame=recordedFrame(testCase,filename,index)
    root=setupVehicleLocalization();path=fullfile(root,'data','raw',filename);
    testCase.assumeTrue(isfile(path),'Recorded-data fixture is unavailable.');
    frame=loadPointCloudFrame(path,index);
end
