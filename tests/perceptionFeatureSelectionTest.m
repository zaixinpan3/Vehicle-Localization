classdef perceptionFeatureSelectionTest < matlab.unittest.TestCase
% perceptionFeatureSelectionTest: One invocation list controls semantic work.
    properties (TestParameter)
        channels = channelSubsets()
    end
    properties
        Frame
    end
    methods (TestClassSetup)
        function pathsAndFrame(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
            file=fullfile(root,'data','raw','downTownPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            testCase.Frame=loadPointCloudFrame(file,200);
        end
    end
    methods (Test)
        function datasetDefaultsSelectExactlyTheRequiredChannels(testCase)
            suburban=perceptionConfig("Mississippi"); urban=perceptionConfig("Downtown");
            testCase.verifyEqual(suburban.featureNames,["curb","pole","trafficSign"]);
            testCase.verifyEqual(urban.featureNames,["curb","pole","facade","trafficSign"]);
            testCase.verifyFalse(isfield(urban.coarseProbabilityCloud,'semanticNames'));
            testCase.verifyFalse(isfield(urban.offGroundFeatures,'facadeDetectionEnabled'));
        end
        function everySubsetControlsCandidatesCloudAndFineDecisions(testCase,channels)
            cfg=perceptionConfig("Downtown"); cfg.featureNames=channels;
            online=perceiveFrame(testCase.Frame,cfg);
            cfg.executionMode="offline"; offline=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyEqual(online.featureNames,channels);
            testCase.verifyEqual(online.candidates.semanticNames,channels(:));
            testCase.verifyEqual(online.probabilityCloud.semanticNames,channels(:));
            testCase.verifyTrue(all(ismember(online.probabilityCloud.components.semanticName,channels)));
            testCase.verifyEqual(sort(string(fieldnames(offline.refinement))),sort(channels(:)));
            testCase.verifyEqual(offline.candidates,online.candidates);
            testCase.verifyEqual(offline.probabilityCloud,online.probabilityCloud);
            testCase.verifyTrue(unrequestedMasksAreEmpty(offline,channels));
        end
        function signOnlyDoesNotReadUnrelatedDetectorParameters(testCase)
            cfg=perceptionConfig("Downtown"); cfg.featureNames="trafficSign";
            cfg.groundFeatures.curb.weights=struct();
            cfg.offGroundFeatures.poleSeedRunLayerThreshold=struct();
            cfg.offGroundFeatures.thetaResolutionDeg=struct();
            p=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyGreaterThan(p.probabilityCloud.components.numComponents,0);
            testCase.verifyEqual(unique(p.probabilityCloud.components.semanticName),"trafficSign");
        end
        function groundOnlyDoesNotReadOffGroundDetectorParameters(testCase)
            cfg=perceptionConfig(); cfg.featureNames="curb";
            cfg.offGroundFeatures.trafficSignIntensityThreshold=struct();
            p=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyGreaterThan(p.probabilityCloud.components.numComponents,0);
            testCase.verifyTrue(all(ismember(p.probabilityCloud.components.semanticName,cfg.featureNames)));
        end
        function curbOnlyPreservesCurbPoints(testCase)
            cfg=perceptionConfig(); cfg.executionMode="offline";
            reference=perceiveFrame(testCase.Frame,cfg);
            cfg.featureNames="curb";
            actual=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyEqual(actual.featureMasks.curb,reference.featureMasks.curb);
            testCase.verifyFalse(isfield(actual.refinement,'roadMarking'));
        end
        function defaultOutputsContainOnlyRetainedChannels(testCase)
            cfg=perceptionConfig();cfg.executionMode="offline";
            cfg.coarseProbabilityCloud.storeDiagnostics=true;
            actual=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyFalse(isfield(actual.featureMasks,'roadMarking'));
            testCase.verifyFalse(isfield(actual.diagnostics.ground,'roadMarkingCellMask'));
            testCase.verifyEqual(actual.candidates.semanticNames,["curb";"pole";"trafficSign"]);
            testCase.verifyEqual(actual.probabilityCloud.semanticNames,["curb";"pole";"trafficSign"]);
        end
        function removedMarkingChannelIsRejected(testCase)
            cfg=perceptionConfig(); cfg.featureNames="roadMarking";
            testCase.verifyError(@() perceiveFrame(testCase.Frame,cfg),'perception:InvalidFeatureNames');
            testCase.verifyFalse(isfield(cfg.groundFeatures,'roadMarking'));
        end
        function preservesCallerOrderAndAcceptsCellArrays(testCase)
            cfg=perceptionConfig(); cfg.featureNames={'trafficSign';'curb'};
            p=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyEqual(p.featureNames,["trafficSign","curb"]);
            testCase.verifyEqual(p.candidates.semanticNames,["trafficSign";"curb"]);
        end
        function rejectsUnknownChannels(testCase)
            cfg=perceptionConfig(); cfg.featureNames="trafficSigns";
            testCase.verifyError(@() perceiveFrame(testCase.Frame,cfg),'perception:InvalidFeatureNames');
        end
        function rejectsDuplicateChannels(testCase)
            cfg=perceptionConfig(); cfg.featureNames=["curb","curb"];
            testCase.verifyError(@() perceiveFrame(testCase.Frame,cfg),'perception:DuplicateFeatureNames');
        end
        function rejectsObsoleteNestedSelector(testCase)
            cfg=perceptionConfig(); cfg.coarseProbabilityCloud.semanticNames="pole";
            testCase.verifyError(@() perceiveFrame(testCase.Frame,cfg),'perception:ObsoleteFeatureSelection');
        end
        function rejectsObsoleteFacadeSwitch(testCase)
            cfg=perceptionConfig(); cfg.offGroundFeatures.facadeDetectionEnabled=true;
            testCase.verifyError(@() perceiveFrame(testCase.Frame,cfg),'perception:ObsoleteFeatureSelection');
        end
    end
end

function cases=channelSubsets()
    names=["curb","pole","facade","trafficSign"];
    cases=struct();
    for bits=0:15
        selected=logical(bitget(bits,1:4));
        cases.(sprintf('subset%02d',bits))=names(selected);
    end
end

function valid=unrequestedMasksAreEmpty(perception,selected)
    names=["curb","pole","facade","trafficSign"];
    valid=true;
    for name=setdiff(names,selected)
        valid=valid && ~any(perception.featureMasks.(name));
    end
end
