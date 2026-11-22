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
        function transferDataUseTheirCalibratedVehicleFrames(testCase)
            for mode=["coarseProbabilityCloud","offline"]
                kitti=perceptionConfig("KITTI",mode);
                nuscenes=perceptionConfig("nuScenes",mode);
                for cfg={kitti,nuscenes}
                    profile=cfg{1};
                    testCase.verifyEqual(profile.featureNames,["curb","pole","facade"]);
                    testCase.verifyEqual(profile.frameCalibration,lidarFrameCalibrationConfig());
                    testCase.verifyEqual(profile.voxel.voxelSize,[.3 .3]);
                    testCase.verifyEqual(profile.downtownCandidates.curbExtensionMeters,16);
                    testCase.verifyEqual(profile.groundSegmentation.groundSeedMaxZ, ...
                        profile.groundSegmentation.slopeGridPriorHeight);
                end
                testCase.verifyEqual(kitti.groundSegmentation.slopeGridPriorHeight,-1.44);
                testCase.verifyEqual(nuscenes.groundSegmentation.slopeGridPriorHeight,.01);
                testCase.verifyEqual(kitti.groundFeatures.curb.extractionHeightStepMinMeters,.035);
                testCase.verifyEqual(nuscenes.groundFeatures.curb.extractionHeightStepMinMeters,.035);
            end
        end
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
            offlineCfg=perceptionConfig("Downtown","offline"); offlineCfg.featureNames=channels;
            offline=perceiveFrame(testCase.Frame,offlineCfg);
            testCase.verifyEqual(online.featureNames,channels);
            testCase.verifyEqual(online.candidates.semanticNames,channels(:));
            testCase.verifyEqual(online.probabilityCloud.semanticNames,channels(:));
            testCase.verifyTrue(all(ismember(online.probabilityCloud.components.semanticName,channels)));
            testCase.verifyEqual(sort(string(fieldnames(offline.refinement))),sort(channels(:)));
            testCase.verifyEqual(offline.candidates.semanticNames,online.candidates.semanticNames);
            testCase.verifyEqual(offline.probabilityCloud.semanticNames,online.probabilityCloud.semanticNames);
            testCase.verifyTrue(unrequestedMasksAreEmpty(offline,channels));
        end
        function signOnlyDoesNotReadUnrelatedDetectorParameters(testCase)
            cfg=perceptionConfig("Downtown","coarseProbabilityCloud"); cfg.featureNames="trafficSign";
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
            cfg=perceptionConfig("Mississippi","offline");
            reference=perceiveFrame(testCase.Frame,cfg);
            cfg.featureNames="curb";
            actual=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyEqual(actual.featureMasks.curb,reference.featureMasks.curb);
            testCase.verifyFalse(isfield(actual.refinement,'roadMarking'));
        end
        function defaultOutputsContainOnlyRetainedChannels(testCase)
            cfg=perceptionConfig("Mississippi","offline");
            cfg.coarseProbabilityCloud.storeDiagnostics=true;
            actual=perceiveFrame(testCase.Frame,cfg);
            testCase.verifyFalse(isfield(actual.featureMasks,'roadMarking'));
            testCase.verifyFalse(isfield(actual.diagnostics.ground,'roadMarkingCellMask'));
            testCase.verifyEqual(actual.candidates.semanticNames,["curb";"pole";"trafficSign"]);
            testCase.verifyEqual(actual.probabilityCloud.semanticNames,["curb";"pole";"trafficSign"]);
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
