classdef pipelineRegressionTest < matlab.unittest.TestCase
% pipelineRegressionTest: Reproduce the reference outputs captured from the
% RobustVehicleLocalization code before the refactor. The perception and
% mapping checks need the recorded datasets and are skipped when the data
% root is unavailable. Set VEHICLE_LOCALIZATION_DATA_ROOT to the folder
% holding raw/MissisipiPointClouds.mat and raw/downTownPointClouds.mat.

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
        % semantic point list, and NDT component mean matches the reference.
            testCase.assumeDataAvailable();
            reference = testCase.Reference;
            datasets = [reference.mississippi, reference.downtown];
            for datasetIdx = 1:numel(datasets)
                dataset = datasets(datasetIdx);
                matPath = fullfile(testCase.DataRoot, dataset.matFile);
                for frameEntry = dataset.frames.'
                    frame = loadPointCloudFrame(matPath, frameEntry.frameIdx);
                    cfg = perceptionConfig();
                    cfg.executionMode = "legacyFull";
                    cfg.offGroundFeatures.facadeDetectionEnabled = frameEntry.facadeDetectionEnabled;
                    perception = perceiveFrame(frame, cfg);
                    for name = string(fieldnames(frameEntry.channels)).'
                        testCase.verifyEqual(uint32(find(perception.featureMasks.(name))), frameEntry.channels.(name), ...
                            sprintf("frame %d channel %s", frameEntry.frameIdx, name));
                    end
                    semanticGrid = buildSemanticVoxelGrid(perception);
                    pointProduct = refineSemanticPoints(semanticGrid, perception);
                    for name = string(fieldnames(frameEntry.refinedPointIdx)).'
                        testCase.verifyEqual(uint32(pointProduct.semanticPointIdx.(name)(:)), frameEntry.refinedPointIdx.(name), ...
                            sprintf("frame %d refined %s", frameEntry.frameIdx, name));
                    end
                    ndtMap = buildSemanticNdtGridMap(frame, semanticGrid, semanticNdtGridMapConfig());
                    testCase.verifyEqual(ndtMap.components.mean, frameEntry.ndtComponentMeans, AbsTol=1.0e-9);
                end
            end
        end

        function mappingReproducesReferenceMap(testCase)
        % mappingReproducesReferenceMap: The sliding-window temporal-stability
        % map built from the reference frame window reproduces the reference
        % components and query scores.
            testCase.assumeDataAvailable();
            reference = testCase.Reference.map;
            matPath = fullfile(testCase.DataRoot, testCase.Reference.mississippi.matFile);
            cfg = featureMapBuildConfig();
            cfg.featureNames = ["curb", "roadMarking", "pole", "trafficSign"];
            cfg.logEnabled = false;
            cfg.frameIndices = reference.frameIndices;
            perceptionCfg = perceptionConfig();
            perceptionCfg.executionMode = "legacyFull";
            perceptionCfg.offGroundFeatures.facadeDetectionEnabled = cfg.facadeDetectionEnabled;
            featureData = collectFeatureObservations(matPath, reference.frameIndices, reference.poseTable, perceptionCfg, cfg);
            probabilityCloudMap = buildSlidingWindowMap(featureData, cfg);
            gmmMap = probabilityCloudMap.batchMaps(1).gmmMap;
            testCase.verifyEqual(gmmMap.classLabels, reference.classLabels);
            for layerIdx = 1:numel(reference.layers)
                layer = gmmMap.layers(layerIdx);
                expected = reference.layers(layerIdx);
                testCase.verifyEqual(layer.componentMeans, expected.componentMeans, AbsTol=1.0e-8);
                testCase.verifyEqual(layer.componentCovariances, expected.componentCovariances, AbsTol=1.0e-8);
                testCase.verifyEqual(layer.componentSupportAmplitudes, expected.componentSupportAmplitudes, AbsTol=1.0e-8);
                testCase.verifyEqual(layer.componentMixtureWeights, expected.componentMixtureWeights, AbsTol=1.0e-8);
            end
            scores = queryTemporalStabilityGmmMap(probabilityCloudMap, reference.queryPoints);
            testCase.verifyEqual(scores, reference.queryScores, AbsTol=1.0e-8);
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
