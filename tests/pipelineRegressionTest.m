classdef pipelineRegressionTest < matlab.unittest.TestCase
% pipelineRegressionTest: Current perception and mapping on recorded scenarios.
% Point masks are compared with immutable current-output JSON. The original
% MAT reference supplies scenario indices and matched poses without executing
% its historical implementation. Set VEHICLE_LOCALIZATION_DATA_ROOT to the
% folder holding raw/MissisipiPointClouds.mat and raw/downTownPointClouds.mat.

    properties (Access = private)
        Reference
        DataRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
        % addProjectPaths: Put the vehicleLocalization modules on the path and
        % load the reference file.
        %
        % Input:
        %   testCase: matlab.unittest.TestCase instance
        %
        % Output:
        %   none
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            run(fullfile(projectFolder, "setupVehicleLocalization.m"));
            loaded = load(fullfile(projectFolder, "tests", "reference", "pipelineReference.mat"));
            testCase.Reference = loaded.reference;
            testCase.DataRoot = string(getenv("VEHICLE_LOCALIZATION_DATA_ROOT"));
        end
    end

    methods (Test)
        function perceptionReproducesReferenceChannels(testCase)
        % perceptionReproducesReferenceChannels: Every feature channel, refined
        % point mask matches the frozen current pipeline reference.
            testCase.assumeDataAvailable();
            reference = testCase.Reference;
            datasets = [reference.mississippi, reference.downtown];
            for datasetIdx = 1:numel(datasets)
                dataset = datasets(datasetIdx);
                matPath = fullfile(testCase.DataRoot, dataset.matFile);
                for frameEntry = dataset.frames.'
                    frame = loadPointCloudFrame(matPath, frameEntry.frameIdx);
                    profile="Mississippi";
                    if frameEntry.facadeDetectionEnabled, profile="Downtown"; end
                    cfg = perceptionConfig(profile); cfg.executionMode = "offline";
                    perception = perceiveFrame(frame, cfg);
                    expected=expectedFinePerception(profile,frameEntry.frameIdx);
                    for name = string(fieldnames(expected.featureMasks)).'
                        testCase.verifyEqual(perception.featureMasks.(name),expected.featureMasks.(name), ...
                            sprintf("frame %d channel %s",frameEntry.frameIdx,name));
                    end
                end
            end
        end

        function mappingBuildsRepeatableFieldFromReferenceObservations(testCase)
        % The archived noisy-OR scores are historical evidence, not the target
        % of the new model. Keep the original reference artifact unchanged.
            testCase.assumeDataAvailable();
            reference = testCase.Reference.map;
            matPath = fullfile(testCase.DataRoot, testCase.Reference.mississippi.matFile);
            cfg = featureMapBuildConfig();
            cfg.featureNames = ["curb", "pole", "trafficSign"];
            cfg.logEnabled = false;
            cfg.frameIndices = reference.frameIndices;
            perceptionCfg = perceptionConfig();
            perceptionCfg.executionMode = "offline";
            perceptionCfg.featureNames = cfg.featureNames;
            featureData = collectFeatureObservations(matPath, reference.frameIndices, reference.poseTable, perceptionCfg, cfg);
            probabilityCloudMap = buildSlidingWindowMap(featureData, cfg);
            gmmMap = probabilityCloudMap.canonicalMap;
            cloud = temporalMapToProbabilityCloud(probabilityCloudMap);
            [scores,details] = queryTemporalStabilityGmmMap(probabilityCloudMap,reference.queryPoints);
            testCase.verifyEqual(gmmMap.classLabels,cfg.featureNames(:));
            testCase.verifyEqual(probabilityCloudMap.frameIndices,reference.frameIndices(:).');
            testCase.verifyGreaterThan(cloud.totalMass,0);
            testCase.verifyGreaterThan(cloud.components.numComponents,0);
            testCase.verifyEqual(sum(cloud.components.mixtureWeight),1,'AbsTol',1e-12);
            testCase.verifyTrue(all(isfinite(scores(details.valid))));
            testCase.verifyTrue(all(isnan(scores(~details.valid))));
            testCase.verifyLessThanOrEqual(scores(details.valid),ones(nnz(details.valid),1));
            testCase.verifyGreaterThanOrEqual(scores(details.valid),zeros(nnz(details.valid),1));
            testCase.verifyEqual(cloud.queryRelationship,"exactIntensityNormalization");
        end
    end

    methods (Access = private)
        function assumeDataAvailable(testCase)
        % assumeDataAvailable: Skip data-dependent checks when the dataset root
        % is not configured or the recorded MAT files are missing.
            testCase.assumeTrue(strlength(testCase.DataRoot) > 0, ...
                "Set VEHICLE_LOCALIZATION_DATA_ROOT to run the data-dependent regression checks.");
            testCase.assumeTrue(isfile(fullfile(testCase.DataRoot, testCase.Reference.mississippi.matFile)) && ...
                isfile(fullfile(testCase.DataRoot, testCase.Reference.downtown.matFile)), ...
                "Recorded point-cloud MAT files were not found under the data root.");
        end
    end
end
