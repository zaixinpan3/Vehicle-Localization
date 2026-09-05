classdef pipelineRegressionTest < matlab.unittest.TestCase
% pipelineRegressionTest: Reproduce the reference outputs captured from the
% RobustVehicleLocalization code before the refactor. The perception and
% mapping checks validate the redesigned field on recorded observations and are
% skipped when the data
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

        function mappingBuildsRepeatableFieldFromReferenceObservations(testCase)
        % The archived noisy-OR scores are a legacy baseline, not the target
        % of the new model. Keep the original reference artifact unchanged.
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
