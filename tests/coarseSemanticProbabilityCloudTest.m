classdef coarseSemanticProbabilityCloudTest < matlab.unittest.TestCase
% coarseSemanticProbabilityCloudTest: Verify the sparse voxel-only semantic
% NDT product, its D2D alignment score, and fidelity to the full perception
% baseline on recorded Mississippi frames.

    properties (Access = private)
        DataRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
        % addProjectPaths: Add repository modules and resolve optional data.
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            run(fullfile(projectFolder, "setupVehicleLocalization.m"));
            testCase.DataRoot = string(getenv("VEHICLE_LOCALIZATION_DATA_ROOT"));
        end
    end

    methods (Test)
        function builderReturnsSparseProbabilityNdtSchema(testCase)
        % builderReturnsSparseProbabilityNdtSchema: Source cells become one
        % compact Gaussian per semantic channel with valid probabilities.
            [ground, offGround, cfg] = ...
                coarseSemanticProbabilityCloudTest.syntheticVoxelFeatures();
            cloud = buildCoarseSemanticProbabilityCloud(ground, offGround, cfg);

            testCase.verifyEqual(cloud.mapType, "semanticNDTProbabilityCloud2D");
            testCase.verifyEqual(cloud.classificationStage, "voxelOnlyCoarseValidation");
            testCase.verifyEqual(cloud.components.numComponents, 3);
            testCase.verifyEqual(sort(cloud.components.semanticName), ...
                sort(["curb"; "roadMarking"; "pole"]));
            testCase.verifyEqual(sum(cloud.components.mixtureWeight), 1, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(cloud.components.semanticProbability, 0);
            testCase.verifyLessThanOrEqual(cloud.components.semanticProbability, 1);
            testCase.verifyGreaterThan(cloud.components.determinant, 0);
            testCase.verifyFalse(any(ismember(string(fieldnames(cloud)), ...
                ["primaryTagVolume", "semanticMaskVolumes", "countMap"])));
        end

        function d2dScorePeaksAtSelfAlignment(testCase)
        % d2dScorePeaksAtSelfAlignment: Identical probability fields attain
        % unit normalized overlap and exceed a translated alignment.
            [ground, offGround, cfg] = ...
                coarseSemanticProbabilityCloudTest.syntheticVoxelFeatures();
            cloud = buildCoarseSemanticProbabilityCloud(ground, offGround, cfg);
            [selfScore, selfDetails] = ...
                scoreSemanticProbabilityCloudAlignment(cloud, cloud, [0, 0, 0]);
            shiftedScore = ...
                scoreSemanticProbabilityCloudAlignment(cloud, cloud, [1.8, 0, 0]);

            testCase.verifyEqual(selfScore, 1, AbsTol=1.0e-12);
            testCase.verifyEqual(selfDetails.squaredL2Distance, 0, AbsTol=1.0e-12);
            testCase.verifyLessThan(shiftedScore, selfScore);
        end

        function emptyFrameProducesEmptyCloud(testCase)
        % emptyFrameProducesEmptyCloud: Missing scalar returns and no finite
        % points produce a valid empty sparse product.
            frame = struct("x", nan(4), "y", nan(4), "z", nan(4));
            cloud = perceiveCoarseProbabilityCloud(frame, perceptionConfig());

            testCase.verifyEqual(cloud.components.numComponents, 0);
            testCase.verifyEmpty(cloud.components.mean);
            testCase.verifyEqual(cloud.semanticNames, ...
                ["curb"; "roadMarking"; "pole"]);
        end

        function referenceFramesTrackFullPerception(testCase)
        % referenceFramesTrackFullPerception: Tuned frames retain the full
        % baseline support after projection to cells and NDT components.
            testCase.assumeMississippiData();
            metrics = coarseSemanticProbabilityCloudTest.compareFrames( ...
                testCase.DataRoot, [260, 300, 326]);

            testCase.verifyGreaterThanOrEqual(metrics.source.curb.precision, 0.69);
            testCase.verifyGreaterThanOrEqual(metrics.source.curb.recall, 0.97);
            testCase.verifyEqual(metrics.source.roadMarking.f1, 1, AbsTol=1.0e-12);
            testCase.verifyEqual(metrics.source.pole.f1, 1, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(metrics.ndt.curb.precision, 0.80);
            testCase.verifyGreaterThanOrEqual(metrics.ndt.curb.recall, 0.99);
            testCase.verifyEqual(metrics.ndt.roadMarking.f1, 1, AbsTol=1.0e-12);
            testCase.verifyEqual(metrics.ndt.pole.f1, 1, AbsTol=1.0e-12);
            testCase.verifyTrue(metrics.columnMapsExact);
        end

        function lockedValidationFramesPreserveHighRecall(testCase)
        % lockedValidationFramesPreserveHighRecall: Dispersed Mississippi
        % frames evaluated after parameter lock retain high feature support.
            testCase.assumeMississippiData();
            metrics = coarseSemanticProbabilityCloudTest.compareFrames( ...
                testCase.DataRoot, [370, 450, 550, 700, 850, 1000, 1150]);

            testCase.verifyGreaterThanOrEqual(metrics.source.curb.recall, 0.93);
            testCase.verifyGreaterThanOrEqual(metrics.source.roadMarking.precision, 0.99);
            testCase.verifyEqual(metrics.source.roadMarking.recall, 1, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(metrics.source.pole.precision, 0.58);
            testCase.verifyEqual(metrics.source.pole.recall, 1, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(metrics.ndt.curb.recall, 0.94);
            testCase.verifyGreaterThanOrEqual(metrics.ndt.roadMarking.precision, 0.98);
            testCase.verifyEqual(metrics.ndt.roadMarking.recall, 1, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(metrics.ndt.pole.precision, 0.65);
            testCase.verifyEqual(metrics.ndt.pole.recall, 1, AbsTol=1.0e-12);
            testCase.verifyTrue(metrics.columnMapsExact);
        end
    end

    methods (Access = private)
        function assumeMississippiData(testCase)
        % assumeMississippiData: Skip recorded-data checks when unconfigured.
            testCase.assumeTrue(strlength(testCase.DataRoot) > 0, ...
                "Set VEHICLE_LOCALIZATION_DATA_ROOT to run data regression.");
            testCase.assumeTrue(isfile(fullfile(testCase.DataRoot, ...
                "raw", "MissisipiPointClouds.mat")), ...
                "Mississippi point-cloud data are unavailable.");
        end
    end

    methods (Static, Access = private)
        function [ground, offGround, cfg] = syntheticVoxelFeatures()
        % syntheticVoxelFeatures: Three accepted source cells in one output
        % NDT cell, one for each supported semantic channel.
            cfg = coarseSemanticProbabilityCloudConfig();
            cfg.xMin = 0;
            cfg.xMax = 2;
            cfg.yMin = 0;
            cfg.yMax = 2;
            cfg.resolution = 0.9;

            ground = struct();
            ground.cellOrigin = [0, 0];
            ground.cellSize = [0.3, 0.3];
            ground.stats = struct("countMap", single([4, 0; 0, 3]));
            ground.curbCellMask = logical([1, 0; 0, 0]);
            ground.curbProbability = single([0.8, 0; 0, 0]);
            ground.roadMarkingCellMask = logical([0, 0; 0, 1]);
            ground.roadMarkingProbability = single([0, 0; 0, 0.9]);

            [xMap, yMap] = meshgrid(single([0.15, 0.45]), single([0.15, 0.45]));
            columnMaps = struct();
            columnMaps.xMap = xMap;
            columnMaps.yMap = yMap;
            columnMaps.pillarCounts = single([0, 5; 0, 0]);
            columnMaps.dx = 0.3;
            columnMaps.dy = 0.3;
            offGround = struct();
            offGround.columnMaps = columnMaps;
            offGround.poleCellMask = logical([0, 1; 0, 0]);
            offGround.poleProbability = single([0, 0.95; 0, 0]);
        end

        function metrics = compareFrames(dataRoot, frameIndices)
        % compareFrames: Aggregate source-grid and final-NDT support metrics
        % against the unchanged full perception result.
            matPath = fullfile(dataRoot, "raw", "MissisipiPointClouds.mat");
            cfg = perceptionConfig();
            semanticNames = ["curb", "roadMarking", "pole"];
            sourceCounts = repmat(struct("tp", 0, "fp", 0, "fn", 0), 3, 1);
            ndtCounts = sourceCounts;
            columnMapsExact = true;
            for frameIdx = frameIndices
                frame = loadPointCloudFrame(matPath, frameIdx);
                full = perceiveFrame(frame, cfg);
                [cloud, diagnostics] = perceiveCoarseProbabilityCloud(frame, cfg);
                referenceSourceMasks = ...
                    coarseSemanticProbabilityCloudTest.referenceSourceMasks(full, diagnostics);
                candidateSourceMasks = {diagnostics.ground.curbCellMask; ...
                    diagnostics.ground.roadMarkingCellMask; ...
                    diagnostics.offGround.poleCellMask};
                for semanticIdx = 1:numel(semanticNames)
                    sourceCounts(semanticIdx) = ...
                        coarseSemanticProbabilityCloudTest.addMaskCounts( ...
                        sourceCounts(semanticIdx), candidateSourceMasks{semanticIdx}, ...
                        referenceSourceMasks{semanticIdx});
                    candidateNdtCells = double(cloud.components.cellLinIdx( ...
                        cloud.components.semanticName == semanticNames(semanticIdx)));
                    referenceNdtCells = ...
                        coarseSemanticProbabilityCloudTest.projectPointMaskToNdt( ...
                        frame, full.featureMasks.(semanticNames(semanticIdx)), cloud.geometry);
                    ndtCounts(semanticIdx) = ...
                        coarseSemanticProbabilityCloudTest.addSetCounts( ...
                        ndtCounts(semanticIdx), candidateNdtCells, referenceNdtCells);
                end
                columnMapsExact = columnMapsExact && ...
                    coarseSemanticProbabilityCloudTest.sameColumnMaps( ...
                    diagnostics.offGround.columnMaps, full.offGround.columnMaps);
            end
            metrics = struct();
            metrics.source = coarseSemanticProbabilityCloudTest.finishMetrics( ...
                sourceCounts, semanticNames);
            metrics.ndt = coarseSemanticProbabilityCloudTest.finishMetrics( ...
                ndtCounts, semanticNames);
            metrics.columnMapsExact = columnMapsExact;
        end

        function masks = referenceSourceMasks(full, diagnostics)
        % referenceSourceMasks: Project full point semantics to their source
        % ground cells and retain the full pole support mask.
            mapSize = size(diagnostics.ground.curbCellMask);
            curb = coarseSemanticProbabilityCloudTest.projectGroundMask( ...
                full.ground.groundCellLinIdx, full.ground.curbPointMask, mapSize);
            roadMarking = coarseSemanticProbabilityCloudTest.projectGroundMask( ...
                full.ground.groundCellLinIdx, full.ground.roadMarkingPointMask, mapSize);
            masks = {curb; roadMarking; logical(full.offGround.pole.mask)};
        end

        function mask = projectGroundMask(pointCellLinIdx, pointMask, mapSize)
        % projectGroundMask: Convert internal [Nx Ny] point cells to a
        % displayed [Ny Nx] logical support mask.
            mask = false(mapSize);
            cellIdx = unique(double(pointCellLinIdx(logical(pointMask))));
            [xBin, yBin] = ind2sub([mapSize(2), mapSize(1)], cellIdx);
            mask(sub2ind(mapSize, yBin, xBin)) = true;
        end

        function cells = projectPointMaskToNdt(frame, pointMask, geometry)
        % projectPointMaskToNdt: Project full semantic points into the fixed
        % probability-cloud grid.
            x = double(frame.x(pointMask));
            y = double(frame.y(pointMask));
            xBin = floor((x - geometry.xMin) ./ geometry.resolution) + 1;
            yBin = floor((y - geometry.yMin) ./ geometry.resolution) + 1;
            valid = isfinite(xBin) & isfinite(yBin) & ...
                xBin >= 1 & xBin <= geometry.dims(2) & ...
                yBin >= 1 & yBin <= geometry.dims(1);
            cells = unique(yBin(valid) + ((xBin(valid) - 1) .* geometry.dims(1)));
        end

        function counts = addMaskCounts(counts, candidate, reference)
        % addMaskCounts: Accumulate binary-raster confusion counts.
            counts.tp = counts.tp + nnz(candidate & reference);
            counts.fp = counts.fp + nnz(candidate & ~reference);
            counts.fn = counts.fn + nnz(~candidate & reference);
        end

        function counts = addSetCounts(counts, candidate, reference)
        % addSetCounts: Accumulate set confusion counts.
            candidate = unique(double(candidate(:)));
            reference = unique(double(reference(:)));
            counts.tp = counts.tp + numel(intersect(candidate, reference));
            counts.fp = counts.fp + numel(setdiff(candidate, reference));
            counts.fn = counts.fn + numel(setdiff(reference, candidate));
        end

        function metrics = finishMetrics(counts, semanticNames)
        % finishMetrics: Convert confusion counts to named P/R/F1 structs.
            metrics = struct();
            for semanticIdx = 1:numel(semanticNames)
                count = counts(semanticIdx);
                precision = count.tp ./ max(count.tp + count.fp, 1);
                recall = count.tp ./ max(count.tp + count.fn, 1);
                f1 = (2 .* count.tp) ./ max((2 .* count.tp) + count.fp + count.fn, 1);
                metrics.(semanticNames(semanticIdx)) = struct( ...
                    "precision", precision, "recall", recall, "f1", f1, ...
                    "tp", count.tp, "fp", count.fp, "fn", count.fn);
            end
        end

        function isEqual = sameColumnMaps(first, second)
        % sameColumnMaps: Confirm sparse and dense column reductions match.
            fields = ["pillarCounts", "pillarZRange", "occupiedLayerCount", ...
                "maxRunLayerCount", "pointScore", "lineScore"];
            isEqual = true;
            for fieldName = fields
                isEqual = isEqual && isequal(first.(fieldName), second.(fieldName));
            end
        end
    end
end
